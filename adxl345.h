/*
 * Copyright (c) Wolfgang Neue
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* ------------------------------------------------------------------------------------- */

/*
 * PLEASE READ THIS BEFORE FIRST USE
 * ESPECIALLY IF YOU DON'T KNOW, WHAT LSB MEANS
 * 
 * In this context, 1 LSB represents the smallest possible step.
 * If you have 1 unsigned Byte, you have 255 possible, non-zero values.
 * A value of 1 stands for 1 LSB, a value of 2 for 2 LSB.
 * If you read something like "15.6 mg / LSB", it means the following:
 *
 * 		If you want to set 1 g (gravitational force, not weight),
 * 		you have to divide 1 g by 0.0156 g (15.6 mg).
 *			1 / 0.0156 = 64.102564...
 *		Now you round the result of this division (64.102564... -> 64) and you got the amount
 *		of LSBs you have to give the function.
 */

/* ------------------------------------------------------------------------------------- */

// Flags for adxl345_act_inact_settings()
#define ADXL345_ACT_RELATIVE		1<<7
#define ADXL345_ACT_ABSOLUTE		0
#define ADXL345_ACT_ENABLE_X		1<<6
#define ADXL345_ACT_ENABLE_Y		1<<5
#define ADXL345_ACT_ENABLE_Z		1<<4
#define ADXL345_ACT_ENABLE_ALL		(ADXL345_ACT_ENABLE_X | ADXL345_ACT_ENABLE_Y | ADXL345_ACT_ENABLE_Z)
#define ADXL345_INACT_RELATIVE		1<<3
#define ADXL345_INACT_ABSOLUTE		0
#define ADXL345_INACT_ENABLE_X		1<<2
#define ADXL345_INACT_ENABLE_Y		1<<1
#define ADXL345_INACT_ENABLE_Z		1
#define ADXL345_INACT_ENABLE_ALL	(ADXL345_INACT_ENABLE_X | ADXL345_INACT_ENABLE_Y | ADXL345_INACT_ENABLE_Z)
#define ADXL345_ACT_INACT_DEFAULT	0

// Flags for adxl345_tap_settings()
#define ADXL345_TAP_SUPRESS			1<<3
#define ADXL345_TAP_ENABLE_X		1<<2
#define ADXL345_TAP_ENABLE_Y		1<<1
#define ADXL345_TAP_ENABLE_Z		1
#define ADXL345_TAP_DEFAULT			0

// Internal flags for adxl345_..._is_..._involved()
#define _ADXL345_ACT_X_INV			1<<6
#define _ADXL345_ACT_Y_INV			1<<5
#define _ADXL345_ACT_Z_INV			1<<4
#define _ADXL345_TAP_X_INV			1<<2
#define _ADXL345_TAP_Y_INV			1<<1
#define _ADXL345_TAP_Z_INV			0

// Flags for adxl345_set_bandwidth()
#define ADXL345_BW_1600				15
#define ADXL345_BW_800				14
#define ADXL345_BW_400				13
#define ADXL345_BW_200				12
#define ADXL345_BW_100				11
#define ADXL345_BW_50				10
#define ADXL345_BW_25				9
#define ADXL345_BW_12_5				1<<3
#define ADXL345_BW_6_25				7
#define ADXL345_BW_3_13				6
#define ADXL345_BW_1_56				5
#define ADXL345_BW_0_78				1<<2
#define ADXL345_BW_0_39				3
#define ADXL345_BW_0_20				1<<1
#define ADXL345_BW_0_10				1
#define ADXL345_BW_0_05				0
#define ADXL345_BW_DEFAULT			ADXL345_BW_50
#define ADXL345_BW_SPEED_IS_KEY		ADXL345_BW_1600

// Flags for adxl345_power_settings()
#define ADXL345_POWER_LINK			1<<5
#define ADXL345_POWER_AUTO_SLEEP	1<<4
#define ADXL345_POWER_MEASURE		1<<3
#define ADXL345_POWER_SLEEP			1<<2
#define ADXL345_POWER_S_1HZ			3
#define ADXL345_POWER_S_2HZ			1<<1
#define ADXL345_POWER_S_4HZ			1
#define ADXL345_POWER_S_8HZ			0
#define ADXL345_POWER_DEFAULT		0

// Flags for adxl345_enable_interrupts() and adxl345_map_interrupts()
#define ADXL345_INT_DATA_READY		1<<7
#define ADXL345_INT_SINGLE_TAP		1<<6
#define ADXL345_INT_DOUBLE_TAP		1<<5
#define ADXL345_INT_ACTIVITY		1<<4
#define ADXL345_INT_INACTIVITY		1<<3
#define ADXL345_INT_FREE_FALL		1<<2
#define	ADXL345_INT_WATERMARK		1<<1
#define ADXL345_INT_OVERRUN			1
#define ADXL345_INT_DEFAULT			0

// Flags for adxl345_data_settings()
#define ADXL345_DATA_SELF_TEST		1<<7
#define ADXL345_DATA_SPI_3W			1<<6
#define ADXL345_DATA_INT_INVERT		1<<5
#define ADXL345_DATA_FULL_RES		1<<3
#define ADXL345_DATA_MSB			1<<2
#define ADXL345_DATA_RANGE_16G		3
#define ADXL345_DATA_RANGE_8G		1<<1
#define ADXL345_DATA_RANGE_4G		1
#define ADXL345_DATA_RANGE_2G		0
#define ADXL345_DATA_DEFAULT		0

// Flags for adxl345_fifo_settings()
#define ADXL345_FIFO_MODE_TRIGGER	192
#define ADXL345_FIFO_MODE_STREAM	1<<7
#define ADXL345_FIFO_MODE_FIFO		1<<6
#define ADXL345_FIFO_MODE_BYPASS	0
#define ADXL345_FIFO_TRIGGER_INT1	0
#define ADXL345_FIFO_TRIGGER_INT2	1<<5
#define ADXL345_FIFO_DEFAULT		0

// Indices for adxl345_axis_get_data_raw() and adxl345_axis_get_data_raw_timeout()
#define ADXL345_INDEX_X 0
#define ADXL345_INDEX_Y 1
#define ADXL345_INDEX_Z 2

/* ------------------------------------------------------------------------------------- */
/* - Data types - */

/*
 * Represents a single adxl345 sensor on the I²C bus
 */
typedef struct
{
	i2c_inst_t *i2c_port;
	uint8_t i2c_addr;
} adxl345_sensor;

/*
 * Represents a single set of values read from a sensor
 * (x-, y- and z-axis in this order)
 */
typedef union
{
	uint16_t axis[3];
	uint8_t raw[6];
} adxl345_axis_data;

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Basic usage - */

/*
 * Initializes the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be initialized
 * i2c_port	-	I²C instance specifier, either i2c0 or i2c1
 * i2c_addr	-	I²C address of the sensor
 */
bool adxl345_init (adxl345_sensor *sensor, i2c_inst_t *i2c_port, uint8_t i2c_addr);

#define adxl345_start(sensor)	adxl345_power_settings (sensor, ADXL345_POWER_MEASURE)

/*
 * Reads only the acceleration of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be read from
 * data		-	Buffer the data will be saved in
 */
int adxl345_axis_get_data (adxl345_sensor *sensor, adxl345_axis_data *data);

/*
 * Reads only the acceleration of the given adxl345 sensor with a timeout (in seconds)
 *
 * sensor		-	adxl345 instance to be read from
 * data			-	Buffer the data will be saved in
 * timeout_s	-	Amount of time (in seconds) the function tries to read
 */
int adxl345_axis_get_data_timeout (adxl345_sensor *sensor, adxl345_axis_data *data, unsigned int timeout_s);

/*
 * Reads only the acceleration of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be read from
 * data		-	Buffer the data will be saved in
 */
int adxl345_axis_get_data_raw (adxl345_sensor *sensor, uint8_t axis, uint8_t *data);

/*
 * Reads only the acceleration of the given adxl345 sensor with a timeout (in seconds)
 *
 * sensor		-	adxl345 instance to be read from
 * data			-	Buffer the data will be saved in
 * timeout_s	-	Amount of time (in seconds) the function tries to read
 */
int adxl345_axis_get_data_raw_timeout (adxl345_sensor *sensor, uint8_t axis, uint8_t *data, unsigned int timeout_s);

/*
 * Gets the acceleration for the x-axis out of the given data
 *
 * data	-	Buffer of data to be evaluated
 */
short adxl345_get_x (adxl345_axis_data *data);

/*
 * Gets the acceleration for the y-axis out of the given data
 *
 * data	-	Buffer of data to be evaluated
 */
short adxl345_get_y (adxl345_axis_data *data);

/*
 * Gets the acceleration for the z-axis out of the given data
 *
 * data	-	Buffer of data to be evaluated
 */
short adxl345_get_z (adxl345_axis_data *data);

/*! \brief 
 * Gets the roll out of the given data
 *
 * data	-	Buffer of data to be evaluated
 */
float adxl345_get_roll (adxl345_axis_data *data);

/*
 * Gets the pitch out of the given data
 *
 * data	-	Buffer of data to be evaluated
 */
float adxl345_get_pitch (adxl345_axis_data *data);

// I think it's easier to use x and y instead of roll and pitch
#define adxl345_get_rot_x(data) adxl345_get_roll (data)
#define adxl345_get_rot_y(data) adxl345_get_pitch (data)

/*
 * Sets the offset on all three axis of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be written to
 * x		-	Offset on the x-axis (15.6 mg / LSB)
 * y		-	Offset on the y-axis (15.6 mg / LSB)
 * z		-	Offset on the z-axis (15.6 mg / LSB)
 */
bool adxl345_set_offset (adxl345_sensor *sensor, uint8_t x, uint8_t y, uint8_t z);

/*
 * Sets the offset on the x-axis of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be written to
 * x		-	Offset on the x-axis (15.6 mg / LSB)
 */
#define adxl345_set_offset_x(sensor,x)	adxl345_set_offset (sensor, x, 0, 0)

uint8_t adxl345_get_offset_x (adxl345_sensor *sensor);

/*
 * Sets the offset on the y-axis of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be written to
 * y		-	Offset on the y-axis (15.6 mg / LSB)
 */
#define adxl345_set_offset_y(sensor,y)	adxl345_set_offset (sensor, 0, y, 0)

uint8_t adxl345_get_offset_y (adxl345_sensor *sensor);

/*
 * Sets the offset on the z-axis of the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be written to
 * z		-	Offset on the z-axis (15.6 mg / LSB)
 */
#define adxl345_set_offset_z(sensor,z)	adxl345_set_offset (sensor, 0, 0, z)

uint8_t adxl345_get_offset_z (adxl345_sensor *sensor);

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - General settings - */

uint8_t adxl345_get_device_id (adxl345_sensor *sensor);
bool adxl345_set_bandwidth (adxl345_sensor *sensor, bool low_power, uint8_t bandwidth);
bool adxl345_power_settings (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_data_settings (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_fifo_settings (adxl345_sensor *sensor, uint8_t flags, uint8_t samples);

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Single Tap / Double Tap - */

/*
 * Sets the threshold for the tap interrupt of the given adxl345 sensor
 *
 * sensor		-	adxl345 instance to be written to
 * threshold	-	Threshold
 */
bool adxl345_tap_set_threshold (adxl345_sensor *sensor, uint8_t threshold);

uint8_t adxl345_tap_get_threshold (adxl345_sensor *sensor);

/*
 * Sets the maximum duration an event must be above the threshold (adxl345_tap_set_threshold)
 * to qualify as a tap event
 *
 * sensor	-	adxl345 instance to be written to
 * duration	-	Maximum duration of a potential tap event (625 us / LSB)
 */
bool adxl345_tap_set_duration (adxl345_sensor *sensor, uint8_t duration);

uint8_t adxl345_tap_get_duration (adxl345_sensor *sensor);

/*
 * Sets the waiting time from the detection of a tap event to the start of the time window
 * during which a possible second tap can be detected
 *
 * sensor	-	adxl345 instance to be written to
 * latent	-	Waiting time to the start of time window for detection of second tap (1.25 ms / LSB)
 */
bool adxl345_tap_set_latent (adxl345_sensor *sensor, uint8_t latent);

uint8_t adxl345_tap_get_latent (adxl345_sensor *sensor);

/*
 * Sets the amount of time after the latency during which a second tap can be detected
 *
 * sensor	-	adxl345 instance to be written to
 * window	-	Amount of time during which a second tap can be detected (1.25 ms / LSB)
 * 				Set to zero to deactivate second tap detection
 */
bool adxl345_tap_set_window (adxl345_sensor *sensor, uint8_t window);

uint8_t adxl345_tap_get_window (adxl345_sensor *sensor);

bool adxl345_tap_settings (adxl345_sensor *sensor, uint8_t flags);

uint8_t adxl345_tap_get_settings (adxl345_sensor *sensor);
#define adxl345_tap_is_suppressed(settings)	((settings & ADXL345_TAP_SUPRESS) != 0)
#define adxl345_tap_is_x_enabled(settings)	((settings & ADXL345_TAP_ENABLE_X) != 0)
#define adxl345_tap_is_y_enabled(settings)	((settings & ADXL345_TAP_ENABLE_Y) != 0)
#define adxl345_tap_is_z_enabled(settings)	((settings & ADXL345_TAP_ENABLE_Z) != 0)

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Activity / Inactivity - */

/*
 * Sets the threshold for the activity interrupt of the given adxl345 sensor
 *
 * sensor		-	adxl345 instance to be written to
 * threshold	-	Threshold (62.5 mg / LSB)
 */
bool adxl345_activity_set_threshold (adxl345_sensor *sensor, uint8_t threshold);

uint8_t adxl345_activity_get_threshold (adxl345_sensor *sensor);

/*
 * Sets the threshold for the inactivity interrupt of the given adxl345 sensor
 *
 * sensor		-	adxl345 instance to be written to
 * threshold	-	Threshold (62.5 mg / LSB)
 */
bool adxl345_inactivity_set_threshold (adxl345_sensor *sensor, uint8_t threshold);

uint8_t adxl345_inactivity_get_threshold (adxl345_sensor *sensor);

/*
 * Sets the amount of time that acceleration must be below the threshold (adxl345_inactivity_set_threshold)
 *
 * sensor		-	adxl345 instance to be written to
 * threshold	-	Amount of time acceleration must be below the threshold (1 s / LSB)
 */
bool adxl345_inactivity_set_time (adxl345_sensor *sensor, uint8_t time);

uint8_t adxl345_inactivity_get_time (adxl345_sensor *sensor);

/*
 * Sets the settings of the activity / inactivity interrupts
 *
 * sensor	-	adxl345 instance to be written to
 * flags	-	Options
 */
bool adxl345_act_inact_settings (adxl345_sensor *sensor, uint8_t flags);

uint8_t adxl345_act_inact_get_settings (adxl345_sensor *sensor);
#define adxl345_activity_is_threshold_relative(settings)	((settings & ADXL345_ACT_RELATIVE) != 0)
#define adxl345_activity_is_x_enabled(settings)				((settings & ADXL345_ACT_ENABLE_X) != 0)
#define adxl345_activity_is_y_enabled(settings)				((settings & ADXL345_ACT_ENABLE_Y) != 0)
#define adxl345_activity_is_z_enabled(settings)				((settings & ADXL345_ACT_ENABLE_Z) != 0)
#define adxl345_inactivity_is_threshold_relative(settings)	((settings & ADXL345_INACT_RELATIVE) != 0)
#define adxl345_inactivity_is_x_enabled(settings)			((settings & ADXL345_INACT_ENABLE_X) != 0)
#define adxl345_inactivity_is_y_enabled(settings)			((settings & ADXL345_INACT_ENABLE_Y) != 0)
#define adxl345_inactivity_is_z_enabled(settings)			((settings & ADXL345_INACT_ENABLE_Z) != 0)

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Free fall - */

bool adxl345_freefall_set_threshold (adxl345_sensor *sensor, uint8_t threshold);

uint8_t adxl345_freefall_get_threshold (adxl345_sensor *sensor);

bool adxl345_freefall_set_time (adxl345_sensor *sensor, uint8_t time);

uint8_t adxl345_freefall_get_time (adxl345_sensor *sensor);

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Interrupts - */

bool adxl345_enable_interrupts (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_map_interrupts (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_reset_interrupts (adxl345_sensor *sensor);
uint8_t adxl345_interrupt_get_involvement (adxl345_sensor *sensor);
#define adxl345_activity_is_x_involved(involvement) ((involvement & _ADXL345_ACT_X_INV) != 0)
#define adxl345_activity_is_y_involved(involvement) ((involvement & _ADXL345_ACT_Y_INV) != 0)
#define adxl345_activity_is_z_involved(involvement)	((involvement & _ADXL345_ACT_Z_INV) != 0)
#define adxl345_tap_is_x_involved(involvement)		((involvement & _ADXL345_TAP_X_INV) != 0)
#define adxl345_tap_is_y_involved(involvement)		((involvement & _ADXL345_TAP_Y_INV) != 0)
#define adxl345_tap_is_z_involved(involvement)		((involvement & _ADXL345_TAP_Z_INV) != 0)
// Function for getting BW_RATE
// Function for getting POWER_CTL
uint8_t adxl345_get_interrupt_status (adxl345_sensor *sensor);
#define adxl345_int_is_enabled(status,event)	((status & event) != 0)
uint8_t adxl345_get_interrupt_map (adxl345_sensor *sensor);
#define adxl345_int_is_mapped_to_2(map,event)	((map & event) != 0)
uint8_t adxl345_get_interrupt_source (adxl345_sensor *sensor);
#define adxl345_has_triggered(source,event)		((source & event) != 0)
// Function for getting DATA_FORMAT
// Function for getting FIFO_CTL
// Function for getting FIFO_STATUS

/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
/* - Advanced darkness - */
// It's easier to use the functions above, but feel free

#define ADXL345_REG_DEVID			0x00 // Read
// 0x01 - 0x1c are reserved
#define ADXL345_REG_THRESH_TAP		0x1d // Read / Write
#define ADXL345_REG_OFSX			0x1e // Read / Write
#define ADXL345_REG_OFSY			0x1f // Read / Write
#define ADXL345_REG_OFSZ			0x20 // Read / Write
#define ADXL345_REG_DUR				0x21 // Read / Write
#define ADXL345_REG_LATENT			0x22 // Read / Write
#define ADXL345_REG_WINDOW			0x23 // Read / Write
#define ADXL345_REG_THRESH_ACT		0x24 // Read / Write
#define ADXL345_REG_THRESH_INACT	0x25 // Read / Write
#define ADXL345_REG_TIME_INACT		0x26 // Read / Write
#define ADXL345_REG_ACT_INACT_CTL	0x27 // Read / Write
#define ADXL345_REG_THRESH_FF		0x28 // Read / Write
#define ADXL345_REG_TIME_FF			0x29 // Read / Write
#define ADXL345_REG_TAP_AXES		0x2a // Read / Write
#define ADXL345_REG_ACT_TAP_STATUS	0x2b // Read
#define ADXL345_REG_BW_RATE			0x2c // Read / Write
#define ADXL345_REG_POWER_CTL		0x2d // Read / Write
#define ADXL345_REG_INT_ENABLE		0x2e // Read / Write
#define ADXL345_REG_INT_MAP			0x2f // Read / Write
#define ADXL345_REG_INT_SOURCE		0x30 // Read
#define ADXL345_REG_DATA_FORMAT		0x31 // Read / Write
#define ADXL345_REG_DATAX0			0x32 // Read
#define ADXL345_REG_DATAX1			0x33 // Read
#define ADXL345_REG_DATAY0			0x34 // Read
#define ADXL345_REG_DATAY1			0x35 // Read
#define ADXL345_REG_DATAZ0			0x36 // Read
#define ADXL345_REG_DATAZ1			0x37 // Read
#define ADXL345_REG_FIFO_CTL		0x38 // Read / Write
#define ADXL345_REG_FIFO_STATUS		0x39 // Read

#define ADXL345_REG_DATA_BEGIN_AXIS	ADXL345_REG_DATAX0

#define ADXL345_REG_DATA_BEGIN_X	ADXL345_REG_DATAX0
#define ADXL345_REG_DATA_BEGIN_Y	ADXL345_REG_DATAY0
#define ADXL345_REG_DATA_BEGIN_Z	ADXL345_REG_DATAZ0

#define ADXL345_REG_DATA_SIZE_AXIS_SINGLE	2

#define ADXL345_REG_DATA_SIZE_AXIS_ALL		6

/*
 * Send a single byte to the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be written to
 * reg_addr	-	Address of the register to write in (not the address of the device)
 * command	-	Single byte to write
 * no_stop	-	Sending stop signal (false = sending, true = not sending)
 */
int adxl345_write (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop);

/*
 * Send a single byte to the given adxl345 sensor with a timeout (in seconds)
 *
 * sensor		-	adxl345 instance to be written to
 * reg_addr		-	Address of the register to write in (not the address of the device)
 * command		-	Single byte to write
 * no_stop		-	Sending stop signal (false = sending, true = not sending)
 * timeout_s	-	Amount of time (in seconds) the function tries to write
 */
int adxl345_write_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop, unsigned int timeout_s);

/*
 * Reads bytes from the given adxl345 sensor
 *
 * sensor	-	adxl345 instance to be read from
 * reg_addr	-	Address of the register to read from (not the address of the device)
 * buffer	-	Buffer the bytes will be written into
 * length	-	Amount of bytes to read
 * no_stop	-	Sending stop signal (false = sending, true = not sending)
 */
int adxl345_read (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop);

/*
 * Reads bytes from the given adxl345 sensor with a timeout (in seconds)
 *
 * sensor		-	adxl345 instance to be read from
 * reg_addr		-	Address of the register to read from (not the address of the device)
 * buffer		-	Buffer the bytes will be written into
 * no_stop		-	Sending stop signal (false = sending, true = not sending)
 * timeout_s	-	Amount of time (in seconds) the function tries to read
 */
int adxl345_read_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop, unsigned int timeout_s);

#endif