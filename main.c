/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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

/**
 * @file    imu_read.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define ACC_CONST_PER_LSB (0.000061)

#define turn_red_led_on GPIO_ClearPinsOutput(GPIOB, 1 << 22);
#define turn_red_led_off GPIO_SetPinsOutput(GPIOB, 1 << 22);

volatile bool g_MasterCompletionFlag = false;

typedef enum
{
	x_axis = 0, y_axis, z_axis
} axis_definition;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

/*
 * @brief   Application entry point.
 */
int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	/*Habilitar el reloj SCG*/
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_I2c0);

	/*Configurar el puerto para encender un LED*/
	/* Input pin PORT configuration */
	port_pin_config_t config_led =
	{ kPORT_PullDisable, /*Resistencias deshabilitadas*/
	kPORT_SlowSlewRate, /*SlewRate menor velocidad*/
	kPORT_PassiveFilterEnable, /*Filtro habilitado*/
	kPORT_OpenDrainDisable, /**/
	kPORT_LowDriveStrength, /**/
	kPORT_MuxAsGpio, /*Modo GPIO*/
	kPORT_UnlockRegister }; /**/

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt5,
	        kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 22, &config_led);
	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 22, &led_config);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle;

	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
	NULL);

	i2c_master_transfer_t masterXfer;

	uint8_t write_data = 0x01;

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x2A;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &write_data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	while (!g_MasterCompletionFlag)
	{
	}
	g_MasterCompletionFlag = false;

	/* Force the counter to be placed into memory. */
	uint8_t data_buffer[6];
	int16_t accelerometer[3];
	float gs[3];
	/* Enter an infinite loop, just incrementing a counter. */
	while (1)
	{
		masterXfer.slaveAddress = 0x1D;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = 0x01;
		masterXfer.subaddressSize = 1;
		masterXfer.data = data_buffer;
		masterXfer.dataSize = 6;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
		while (!g_MasterCompletionFlag)
		{
		}
		g_MasterCompletionFlag = false;

		accelerometer[x_axis] = data_buffer[0] << 8 | data_buffer[1];
		accelerometer[y_axis] = data_buffer[2] << 8 | data_buffer[3];
		accelerometer[z_axis] = data_buffer[4] << 8 | data_buffer[5];

		gs[x_axis] = ((float) accelerometer[x_axis]) * (ACC_CONST_PER_LSB);
		gs[y_axis] = ((float) accelerometer[y_axis]) * (ACC_CONST_PER_LSB);
		gs[z_axis] = ((float) accelerometer[z_axis]) * (ACC_CONST_PER_LSB);

		if( ( gs[x_axis] < 0.2 ) && ( gs[y_axis] < 0.2 ) && ( gs[z_axis] < 0.2 ) )
		{
			turn_red_led_on;
		}
		else
		{
			turn_red_led_off;
		}

	}
	return 0;
}
