/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "math.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACCEL_I2C_CLK_SRC I2C0_CLK_SRC
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define FXOS8700_WHOAMI 0xC7U
#define MMA8451_WHOAMI 0x1AU
#define ACCEL_STATUS 0x00U
#define ACCEL_FFMT_CFG 0x15U
#define ACCEL_FFMT_SRC 0x16U
#define ACCEL_FFMT_THS 0x17U
#define ACCEL_FFMT_COUNT 0x18U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU
#define ACCEL_CTRL_REG2 0x2BU
#define ACCEL_CTRL_REG3 0x2CU
#define ACCEL_CTRL_REG4 0x2DU
#define ACCEL_CTRL_REG5 0x2EU
#define M_CTRL_REG_1 0x5BU

/* FXOS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x0DU
#define ACCEL_READ_TIMES 10U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
void freefall_INT_pinInit();
void MOTOR_Init();
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*  FXOS8700 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

i2c_master_handle_t g_m_handle;

uint8_t g_accel_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;
volatile bool bandera = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void PORTC_IRQHandler(void){
	if ((1<<6) & PORT_GetPinsInterruptFlags(PORTC))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTC, (1<<6));
		bandera = true;
	}
}

void PORTA_IRQHandler(void){
	if ((1<<4) & PORT_GetPinsInterruptFlags(PORTA))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTA, (1<<4));
		GPIO_ClearPinsOutput(GPIOC, (1<<10));

	}
}

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

static bool I2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;
    uint32_t sourceClock = 0;

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = ACCEL_I2C_CLK_FREQ;

    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = g_accel_address[0];
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == FXOS8700_WHOAMI)
            {
                PRINTF("Found an FXOS8700 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI)
            {
                PRINTF("Found an MMA8451 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    bool isThereAccel = false;
    BOARD_InitPins();
    freefall_INT_pinInit();


    BOARD_BootClockRUN();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();

    MOTOR_Init();



    PRINTF("\r\nI2C example -- Read Accelerometer Value\r\n");

    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    isThereAccel = I2C_ReadAccelWhoAmI();

    /*  read the accel xyz value if there is accel device on board */
    if (true == isThereAccel)
    {
        uint8_t databyte = 0;
        uint8_t write_reg = 0;
        uint8_t status0_value = 0;


        /** Step 1 */
        write_reg = ACCEL_CTRL_REG1;
        databyte = 0x0CU;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 2 */
        write_reg = ACCEL_FFMT_CFG;
        databyte = 0xB8U;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 3 */
		write_reg = ACCEL_FFMT_THS;
		databyte = 0x03U;
		I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 4 */
		write_reg = ACCEL_FFMT_COUNT;
		databyte = 0x06U;
		I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 5 */
        write_reg = ACCEL_CTRL_REG4;
        databyte = 0x04U;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 6 */
        write_reg = ACCEL_CTRL_REG5;
        databyte = 0x04U;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /** Step 7 */
        write_reg = ACCEL_CTRL_REG1;
        databyte = 0x0DU;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);


		for (;;) {

			/** Step 8 */
			if (bandera)
			{
				bandera = false;
				GPIO_SetPinsOutput(GPIOC, (1<<10));
				I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,
						0x0CU, &status0_value, 1);
				if ((status0_value & 0x04) == 0x04)
				{
					PRINTF("Me caigo\r\n");
					I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR,
							g_accel_addr_found, ACCEL_FFMT_SRC, &status0_value,
							1);
				}

			}

		}
	}

    for(;;)
    {
    }
}

void freefall_INT_pinInit()
{
	/* Input pin PORT configuration */
	port_pin_config_t config_switch = { kPORT_PullUp, kPORT_SlowSlewRate,
			kPORT_PassiveFilterEnable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };

	gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };

    CLOCK_EnableClock(kCLOCK_PortC);
    /* Port C Clock Gate Control: Clock enabled */
    PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);
    PORT_SetPinConfig(PORTC, 6, &config_switch);
    GPIO_PinInit(GPIOC, 6, &switch_config);

	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn, 6);
}


void MOTOR_Init()
{
	/*Habilitar el reloj SCG*/
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);

    /*Configurar el pin del puerto C para encender el Motor*/
    /* Input pin PORT configuration */
	port_pin_config_t config_mot = {
	  		kPORT_PullDisable,				/*Resistencias deshabilitadas*/
	   		kPORT_SlowSlewRate,				/*SlewRate menor velocidad*/
	   		kPORT_PassiveFilterEnable,		/*Filtro habilitado*/
	   		kPORT_OpenDrainDisable,			/**/
	   		kPORT_LowDriveStrength,			/**/
	   		kPORT_MuxAsGpio,				/*Modo GPIO*/
			kPORT_UnlockRegister };			/**/

    /* Input pin PORT configuration */
    port_pin_config_t config_switch = {
    		kPORT_PullDisable,
    		kPORT_SlowSlewRate,
    		kPORT_PassiveFilterEnable,
    		kPORT_OpenDrainDisable,
    		kPORT_LowDriveStrength,
    		kPORT_MuxAsGpio,
    		kPORT_UnlockRegister};

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	/* Sets the configuration */
	PORT_SetPinConfig(PORTC, 10, &config_mot);
	PORT_SetPinConfig(PORTA, 4, &config_switch);

	NVIC_EnableIRQ(PORTA_IRQn);

	gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };
	gpio_pin_config_t mot_config = { kGPIO_DigitalOutput, 0 };

	/* Sets the configuration */
	GPIO_PinInit(GPIOA, 4, &switch_config);
	GPIO_PinInit(GPIOC, 10, &mot_config);

}
