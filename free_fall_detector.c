#include "free_fall_detector.h"

static i2c_master_transfer_t masterXfer;
static i2c_master_handle_t g_m_handle;

static uint8_t data_buffer[REGISTER_AXIS_ACCELEROMETER];
static int16_t accelerometer[NUMBER_OF_AXIS];

static volatile bool g_MasterCompletionFlag = false;

static const int16_t equivalent_five_percent_error = 1450;

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 100; i++)
    {
        __NOP();
    }
}

void i2c_ReleaseBus()
{
	uint8_t i = 0;
	gpio_pin_config_t pin_config;
	port_pin_config_t i2c_pin_config =
	{ 0 };

	/* Config pin mux as gpio */
	i2c_pin_config.pullSelect = kPORT_PullUp;
	i2c_pin_config.mux = kPORT_MuxAsGpio;

	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 1U;
	CLOCK_EnableClock(kCLOCK_PortE);
	PORT_SetPinConfig(PORTE, 24, &i2c_pin_config);
	PORT_SetPinConfig(PORTE, 25, &i2c_pin_config);

	GPIO_PinInit(GPIOE, 24, &pin_config);
	GPIO_PinInit(GPIOE, 25, &pin_config);

	GPIO_PinWrite(GPIOE, 25, 0U);
	i2c_release_bus_delay();

	for (i = 0; i < 9; i++)
	{
		GPIO_PinWrite(GPIOE, 24, 0U);
		i2c_release_bus_delay();

		GPIO_PinWrite(GPIOE, 25, 1U);
		i2c_release_bus_delay();

		GPIO_PinWrite(GPIOE, 24, 1U);
		i2c_release_bus_delay();
		i2c_release_bus_delay();
	}

	GPIO_PinWrite(GPIOE, 24, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 25, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 24, 1U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 25, 1U);
	i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

void config_fall_detector_led()
{
	/*Habilitar el reloj SCG*/
	CLOCK_EnableClock(kCLOCK_PortB);

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

	PORT_SetPinConfig(PORTB, 22, &config_led);
	gpio_pin_config_t led_config =
	{ kGPIO_DigitalOutput, 1 };
	GPIO_PinInit(GPIOB, 22, &led_config);

}

void config_fall_detector_i2c()
{
	/*Habilitar el reloj SCG*/
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt5,
	        kPORT_UnlockRegister };

	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
	        NULL);

}

void enable_i2c_accelerometer()
{
	static uint8_t write_data = 0x01;

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

}

void fall_detector()
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

	accelerometer[x_axis] = data_buffer[X_ACC_MSB] << MSB_ACC_AXIS_MASK
	        | data_buffer[X_ACC_LSB];
	accelerometer[y_axis] = data_buffer[Y_ACC_MSB] << MSB_ACC_AXIS_MASK
	        | data_buffer[Y_ACC_LSB];
	accelerometer[z_axis] = data_buffer[Z_ACC_MSB] << MSB_ACC_AXIS_MASK
	        | data_buffer[Z_ACC_LSB];

	if ((equivalent_five_percent_error >= accelerometer[x_axis])
	        && (-equivalent_five_percent_error <= accelerometer[x_axis])
	        && (equivalent_five_percent_error >= accelerometer[y_axis])
	        && (-equivalent_five_percent_error <= accelerometer[y_axis])
	        && (equivalent_five_percent_error >= accelerometer[z_axis])
	        && (-equivalent_five_percent_error <= accelerometer[z_axis]))
	{
		turn_red_led_on;
	}

	else
	{
		turn_red_led_off;
	}

}

