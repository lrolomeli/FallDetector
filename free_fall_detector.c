#include "free_fall_detector.h"

static i2c_master_transfer_t masterXfer;
static i2c_master_handle_t g_m_handle;

static uint8_t data_buffer[REGISTER_AXIS_ACCELEROMETER];
static int16_t accelerometer[NUMBER_OF_AXIS];

static volatile bool g_MasterCompletionFlag = false;

static const int16_t equivalent_five_percent_error = 820;

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

