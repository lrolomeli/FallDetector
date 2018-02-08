/*
 * free_fall_detector.h
 *
 *  Created on: Feb 6, 2018
 *      Author: lrolo
 */

#ifndef FREE_FALL_DETECTOR_H_
#define FREE_FALL_DETECTOR_H_

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"

#define NUMBER_OF_AXIS 3
#define REGISTER_AXIS_ACCELEROMETER 6
#define MSB_ACC_AXIS_MASK 8

#define RED_LED_PIN 0x400000
#define turn_red_led_on GPIO_ClearPinsOutput(GPIOB, RED_LED_PIN);
#define turn_red_led_off GPIO_SetPinsOutput(GPIOB, RED_LED_PIN);

typedef enum
{
	x_axis = 0, y_axis, z_axis

} axis_definition;

typedef enum
{
	X_ACC_MSB = 0, X_ACC_LSB, Y_ACC_MSB, Y_ACC_LSB, Z_ACC_MSB, Z_ACC_LSB
} acc_register_axis_definition;

void i2c_ReleaseBus();

void config_fall_detector_led();

void config_fall_detector_i2c();

void enable_i2c_accelerometer();

void fall_detector();

#endif /* FREE_FALL_DETECTOR_H_ */
