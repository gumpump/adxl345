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

#define ADXL345_I2C_ADDRESS 0x53

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

/*
	The measured values starting here

	- 0x32 - DATAX0
		Lower byte of x value
	- 0x33 - DATAX1
		Upper byte of x value
	- 0x34 - DATAY0
		Lower byte of y value
	- 0x35 - DATAY1
		Upper byte of y value
	- 0x36 - DATAZ0
		Lower byte of z value
	- 0x37 - DATAZ1
		Upper byte of z value
*/
#define ADXL345_REG_VAL				0x32

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

#define ADXL345_TAP_SUPRESS			1<<3
#define ADXL345_TAP_ENABLE_X		1<<2
#define ADXL345_TAP_ENABLE_Y		1<<1
#define ADXL345_TAP_ENABLE_Z		1

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

#define ADXL345_POWER_LINK			1<<5
#define ADXL345_POWER_AUTO_SLEEP	1<<4
#define ADXL345_POWER_MEASURE		1<<3
#define ADXL345_POWER_SLEEP			1<<2
#define ADXL345_POWER_S_1HZ			3
#define ADXL345_POWER_S_2HZ			1<<1
#define ADXL345_POWER_S_4HZ			1
#define ADXL345_POWER_S_8HZ			0

#define ADXL345_INT_DATA_READY		1<<7
#define ADXL345_INT_SINGLE_TAP		1<<6
#define ADXL345_INT_DOUBLE_TAP		1<<5
#define ADXL345_INT_ACTIVITY		1<<4
#define ADXL345_INT_INACTIVITY		1<<3
#define ADXL345_INT_FREE_FALL		1<<2
#define	ADXL345_INT_WATERMARK		1<<1
#define ADXL345_INT_OVERRUN			1

#define ADXL345_DATA_SELF_TEST		1<<7
#define ADXL345_DATA_SPI_3W			1<<6
#define ADXL345_DATA_INT_INVERT		1<<5
#define ADXL345_DATA_FULL_RES		1<<3
#define ADXL345_DATA_MSB			1<<2
#define ADXL345_DATA_RANGE_16G		3
#define ADXL345_DATA_RANGE_8G		1<<1
#define ADXL345_DATA_RANGE_4G		1
#define ADXL345_DATA_RANGE_2G		0

#define ADXL345_FIFO_MODE_TRIGGER	192
#define ADXL345_FIFO_MODE_STREAM	1<<7
#define ADXL345_FIFO_MODE_FIFO		1<<6
#define ADXL345_FIFO_MODE_BYPASS	0
#define ADXL345_FIFO_TRIGGER_INT1	0
#define ADXL345_FIFO_TRIGGER_INT2	1<<5

/*! \brief Represents a single adxl345 sensor on the I²C bus
 *  \ingroup accel
*/
typedef struct
{
	i2c_inst_t *i2c_port;
	uint8_t i2c_addr;
} adxl345_sensor;

/*! \brief Represents a single set of values read from a sensor
 *  \ingroup accel
*/
typedef union
{
	uint8_t data_8[8];
	uint16_t data_16[4];
} adxl345_data;

/*! \brief Initializes the given adxl345 sensor
 *  \ingroup accel
 *
 * Initialize \p sensor at \p i2c_addr on \p i2c_port
 *
 * \param sensor adxl345 instance to be initialized
 * \param i2c_port I²C instance specifier, either \ref i2c0 or \ref i2c1
 * \param i2c_addr I²C address of the sensor
 * \return true if succeeded, false if failed
*/
bool adxl345_init (adxl345_sensor *sensor, i2c_inst_t *i2c_port, uint8_t i2c_addr);

/*! \brief Send a single byte to the given adxl345 sensor
 *  \ingroup accel
 *
 * Writes a \p command into \p reg_addr of \p sensor
 *
 * \param sensor adxl345 instance to be written to
 * \param reg_addr Adress of the register to write in (not the adress of the device)
 * \param command Single byte to write
 * \param no_stop Sending stop signal (false = sending, true = not sending)
 * \return Bytes written
*/
int adxl345_write (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop);

/*! \brief Send a single byte to the given adxl345 sensor with atimeout (in seconds)
 *  \ingroup accel
 *
 * Writes a \p command into \p reg_addr of \p sensor with a \p timeout_s (in seconds)
 *
 * \param sensor adxl345 instance to be written to
 * \param reg_addr Adress of the register to write in (not the adress of the device)
 * \param command Single byte to write
 * \param no_stop Sending stop signal (false = sending, true = not sending)
 * \param timeout_s Amount of time (in seconds) the function tries to write
 * \return Bytes written
*/
int adxl345_write_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop, unsigned int timeout_s);

/*! \brief Reads the acceleration and FIFO status of the given adxl345 sensor
 *  \ingroup accel
 *
 * Reads \p data out of \p sensor
 *
 * \param sensor adxl345 instance to be read from
 * \param data Buffer the data will be saved in
 * \return Bytes read
*/
int adxl345_read_8 (adxl345_sensor *sensor, adxl345_data *data);

/*! \brief Reads the acceleration and FIFO status of the given adxl345 sensor with a timeout (in seconds)
 *  \ingroup accel
 *
 * Reads \p data out of \p sensor with a \p timeout_s (in seconds)
 *
 * \param sensor adxl345 instance to be read from
 * \param data Buffer the data will be saved in
 * \param timeout_s Amount of time (in seconds) the function tries to read
 * \return Bytes read
*/
int adxl345_read_8_timeout (adxl345_sensor *sensor, adxl345_data *data, unsigned int timeout_s);

/*! \brief Reads only the acceleration of the given adxl345 sensor
 *  \ingroup accel
 *
 * Reads \p data out of \p sensor
 *
 * \param sensor adxl345 instance to be read from
 * \param data Buffer the data will be saved in
 * \return Bytes read
*/
int adxl345_read_6 (adxl345_sensor *sensor, adxl345_data *data);

/*! \brief Reads only the acceleration of the given adxl345 sensor
 *  \ingroup accel
 *
 * Reads \p data out of \p sensor with a \p timeout_s (in seconds)
 *
 * \param sensor adxl345 instance to be read from
 * \param data Buffer the data will be saved in
 * \param timeout_s Amount of time (in seconds) the function tries to read
 * \return Bytes read
*/
int adxl345_read_6_timeout (adxl345_sensor *sensor, adxl345_data *data, unsigned int timeout_s);

/*! \brief Gets the acceleration for the x-axis out of the given data
 *  \ingroup accel
 *
 * Gets the acceleration for the x-axis out ouf \p data
 *
 * \param data Buffer of data to be evaluated
 * \return Value for the x-axis
*/
int adxl345_get_x (adxl345_data *data);

/*! \brief Gets the acceleration for the y-axis out of the given data
 *  \ingroup accel
 *
 * Gets the acceleration for the y-axis out ouf \p data
 *
 * \param data Buffer of data to be evaluated
 * \return Value for the y-axis
*/
int adxl345_get_y (adxl345_data *data);

/*! \brief Gets the acceleration for the z-axis out of the given data
 *  \ingroup accel
 *
 * Gets the acceleration for the z-axis out ouf \p data
 *
 * \param data Buffer of data to be evaluated
 * \return Value for the z-axis
*/
int adxl345_get_z (adxl345_data *data);

/*! \brief Gets the roll out of the given data
 *  \ingroup accel
 *
 * Gets the roll out ouf \p data
 *
 * \param data Buffer of data to be evaluated
 * \return Roll
*/
float adxl345_get_roll (adxl345_data *data);

/*! \brief Gets the pitch out of the given data
 *  \ingroup accel
 *
 * Gets the pitch out ouf \p data
 *
 * \param data Buffer of data to be evaluated
 * \return Pitch
*/
float adxl345_get_pitch (adxl345_data *data);

// I think it's easier to use x and y instead of roll and pitch
#define adxl345_get_rot_x(data) adxl345_get_roll(data)
#define adxl345_get_rot_y(data) adxl345_get_pitch(data)

// New stuff
bool adxl345_tap_set_threshold (adxl345_sensor *sensor, uint8_t threshold);
bool adxl345_set_offset (adxl345_sensor *sensor, uint8_t x, uint8_t y, uint8_t z);
#define adxl345_set_offset_x(s,x)	adxl345_set_offset(s,x,0,0)
#define adxl345_set_offset_y(s,y)	adxl345_set_offset(s,0,y,0)
#define adxl345_set_offset_z(s,z)	adxl345_set_offset(s,0,0,z)
bool adxl345_tap_set_duration (adxl345_sensor *sensor, uint8_t duration);
bool adxl345_tap_set_latent (adxl345_sensor *sensor, uint8_t latent);
bool adxl345_tap_set_window (adxl345_sensor *sensor, uint8_t window);
bool adxl345_activity_set_threshold (adxl345_sensor *sensor, uint8_t threshold);
bool adxl345_inactivity_set_threshold (adxl345_sensor *sensor, uint8_t threshold);
bool adxl345_inactivity_set_time (adxl345_sensor *sensor, uint8_t time);
bool adxl345_act_inact_settings (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_freefall_set_threshold (adxl345_sensor *sensor, uint8_t threshold);
bool adxl345_freefall_set_time (adxl345_sensor *sensor, uint8_t time);
bool adxl345_tap_settings (adxl345_sensor *sensor, uint8_t flags);
// Function for ACT_TAP_STATUS
bool adxl345_set_bandwidth (adxl345_sensor *sensor, bool low_power, uint8_t bandwidth);
bool adxl345_power_settings (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_enable_interrupts (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_map_interrupts (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_reset_interrupts (adxl345_sensor *sensor);
bool adxl345_data_settings (adxl345_sensor *sensor, uint8_t flags);
bool adxl345_fifo_settings (adxl345_sensor *sensor, uint8_t flags, uint8_t samples);

#endif
