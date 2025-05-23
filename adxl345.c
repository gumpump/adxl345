/*
 * Copyright (c) Wolfgang Neue
 *
 * SPDX-License-Identifier: MIT
 */

#include "adxl345.h"

#include <math.h>

#define ADXL345_I2C_ADDRESS 0x53

#define ADXL345_CMD_SIZE(size) (size+1)

bool adxl345_init (adxl345_sensor *sensor, i2c_inst_t *i2c_port, uint8_t i2c_addr)
{
	if (!sensor)
	{
		return false;
	}

	sensor->i2c_port = i2c_port;

	if (i2c_addr != 0)
	{
		sensor->i2c_addr = i2c_addr;
	}

	else
	{
		sensor->i2c_addr = ADXL345_I2C_ADDRESS;
	}

	return true;
}

int adxl345_axis_get_data (adxl345_sensor *sensor, adxl345_axis_data *data)
{
	uint8_t command = ADXL345_REG_DATA_BEGIN_AXIS;
	i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, &command, sizeof (command), false);
	return i2c_read_blocking (sensor->i2c_port, sensor->i2c_addr, data->raw, ADXL345_REG_DATA_SIZE_AXIS_ALL, false);
}

int adxl345_axis_get_data_timeout (adxl345_sensor *sensor, adxl345_axis_data *data, unsigned int timeout_s)
{
	uint8_t command = ADXL345_REG_DATA_BEGIN_AXIS;
	unsigned int actual_timeout = (timeout_s * 1000 * 1000) / 2;
	i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &command, sizeof (command), false, actual_timeout);
	return i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, data->raw, ADXL345_REG_DATA_SIZE_AXIS_ALL, false, actual_timeout);
}

int adxl345_axis_get_data_raw (adxl345_sensor *sensor, uint8_t axis, uint8_t *data)
{
	uint8_t command = ADXL345_REG_DATA_BEGIN_AXIS + (axis * 2);
	i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, &command, sizeof (command), false);
	return i2c_read_blocking (sensor->i2c_port, sensor->i2c_addr, data, ADXL345_REG_DATA_SIZE_AXIS_SINGLE, false);
}

int adxl345_axis_get_data_raw_timeout (adxl345_sensor *sensor, uint8_t axis, uint8_t *data, unsigned int timeout_s)
{
	uint8_t command = ADXL345_REG_DATA_BEGIN_AXIS + (axis * 2);
	unsigned int actual_timeout = (timeout_s * 1000 * 1000) / 2;
	i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &command, sizeof (command), false, actual_timeout);
	return i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, data, ADXL345_REG_DATA_SIZE_AXIS_SINGLE, false, actual_timeout);
}

short adxl345_get_x (adxl345_axis_data *data)
{
	return (short)data->axis[ADXL345_INDEX_X];
}

short adxl345_get_y (adxl345_axis_data *data)
{
	return (short)data->axis[ADXL345_INDEX_Y];
}

short adxl345_get_z (adxl345_axis_data *data)
{
	return (short)data->axis[ADXL345_INDEX_Z];
}

float adxl345_get_roll (adxl345_axis_data *data)
{
	float y = (float) adxl345_get_y (data);
	float z = (float) adxl345_get_z (data);

	return atan2 (y, z) * 57.3;
}

float adxl345_get_pitch (adxl345_axis_data *data)
{
	float x = (float) adxl345_get_x (data);
	float y = (float) adxl345_get_y (data);
	float z = (float) adxl345_get_z (data);

	return atan2 (-x, sqrt (y * y + z * z)) * 57.3;
}

bool adxl345_tap_set_threshold (adxl345_sensor *sensor, uint8_t threshold)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_THRESH_TAP, threshold, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_set_offset (adxl345_sensor *sensor, uint8_t x, uint8_t y, uint8_t z)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_OFSX, x, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	if (adxl345_write_timeout (sensor, ADXL345_REG_OFSY, y, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	if (adxl345_write_timeout (sensor, ADXL345_REG_OFSZ, z, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_tap_set_duration (adxl345_sensor *sensor, uint8_t duration)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_DUR, duration, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_tap_set_latent (adxl345_sensor *sensor, uint8_t latent)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_LATENT, latent, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_tap_set_window (adxl345_sensor *sensor, uint8_t window)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_WINDOW, window, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_activity_set_threshold (adxl345_sensor *sensor, uint8_t threshold)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_THRESH_ACT, threshold, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_inactivity_set_threshold (adxl345_sensor *sensor, uint8_t threshold)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_THRESH_INACT, threshold, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_inactivity_set_time (adxl345_sensor *sensor, uint8_t time)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_TIME_INACT, time, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_act_inact_settings (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_ACT_INACT_CTL, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_freefall_set_threshold (adxl345_sensor *sensor, uint8_t threshold)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_THRESH_FF, threshold, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_freefall_set_time (adxl345_sensor *sensor, uint8_t time)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_TIME_FF, time, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_tap_settings (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_TAP_AXES, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_set_bandwidth (adxl345_sensor *sensor, bool low_power, uint8_t bandwidth)
{
	uint8_t command = 0;

	if (low_power == true)
	{
		command += 16;
	}

	command += bandwidth;

	if (adxl345_write_timeout (sensor, ADXL345_REG_BW_RATE, command, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_power_settings (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_POWER_CTL, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_enable_interrupts (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_INT_ENABLE, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_map_interrupts (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_INT_MAP, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_reset_interrupts (adxl345_sensor *sensor)
{
	uint8_t command = ADXL345_REG_INT_SOURCE;
	unsigned int actual_timeout = (1 * 1000 * 1000) / 2;

	if (i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &command, 1, false, actual_timeout) < 1)
	{
		return false;
	}

	uint8_t result;

	if (i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, &result, 1, false, actual_timeout) < 1)
	{
		return false;
	}

	return true;
}

bool adxl345_data_settings (adxl345_sensor *sensor, uint8_t flags)
{
	if (adxl345_write_timeout (sensor, ADXL345_REG_DATA_FORMAT, flags, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

bool adxl345_fifo_settings (adxl345_sensor *sensor, uint8_t flags, uint8_t samples)
{
	if (samples > 31)
	{
		samples = 31;
	}

	if (adxl345_write_timeout (sensor, ADXL345_REG_FIFO_CTL, flags | samples, false, 1) < ADXL345_CMD_SIZE (1))
	{
		return false;
	}

	return true;
}

uint8_t adxl345_get_device_id (adxl345_sensor *sensor)
{
	uint8_t id = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_DEVID, &id, 1, false, 1) < sizeof (id))
	{
		return 0;
	}

	return id;
}

uint8_t adxl345_tap_get_threshold (adxl345_sensor *sensor)
{
	uint8_t threshold = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_THRESH_TAP, &threshold, 1, false, 1) < sizeof (threshold))
	{
		return 0;
	}

	return threshold;
}

uint8_t adxl345_get_offset_x (adxl345_sensor *sensor)
{
	uint8_t offset_x = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_OFSX, &offset_x, 1, false, 1) < sizeof (offset_x))
	{
		return 0;
	}

	return offset_x;
}

uint8_t adxl345_get_offset_y (adxl345_sensor *sensor)
{
	uint8_t offset_y = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_OFSY, &offset_y, 1, false, 1) < sizeof (offset_y))
	{
		return 0;
	}

	return offset_y;
}

uint8_t adxl345_get_offset_z (adxl345_sensor *sensor)
{
	uint8_t offset_z = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_OFSZ, &offset_z, 1, false, 1) < sizeof (offset_z))
	{
		return 0;
	}

	return offset_z;
}

uint8_t adxl345_tap_get_duration (adxl345_sensor *sensor)
{
	uint8_t duration = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_DUR, &duration, 1, false, 1) < sizeof (duration))
	{
		return 0;
	}

	return duration;
}

uint8_t adxl345_tap_get_latent (adxl345_sensor *sensor)
{
	uint8_t latent = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_LATENT, &latent, 1, false, 1) < sizeof (latent))
	{
		return 0;
	}

	return latent;
}

uint8_t adxl345_tap_get_window (adxl345_sensor *sensor)
{
	uint8_t window = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_WINDOW, &window, 1, false, 1) < sizeof (window))
	{
		return 0;
	}

	return window;
}

uint8_t adxl345_activity_get_threshold (adxl345_sensor *sensor)
{
	uint8_t threshold = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_THRESH_ACT, &threshold, 1, false, 1) < sizeof (threshold))
	{
		return 0;
	}

	return threshold;
}

uint8_t adxl345_inactivity_get_threshold (adxl345_sensor *sensor)
{
	uint8_t threshold = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_THRESH_INACT, &threshold, 1, false, 1) < sizeof (threshold))
	{
		return 0;
	}

	return threshold;
}

uint8_t adxl345_inactivity_get_time (adxl345_sensor *sensor)
{
	uint8_t time = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_TIME_INACT, &time, 1, false, 1) < sizeof (time))
	{
		return 0;
	}

	return time;
}

uint8_t adxl345_act_inact_get_settings (adxl345_sensor *sensor)
{
	uint8_t settings = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_ACT_INACT_CTL, &settings, 1, false, 1) < sizeof (settings))
	{
		return 0;
	}

	return settings;
}

uint8_t adxl345_freefall_get_threshold (adxl345_sensor *sensor)
{
	uint8_t threshold = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_THRESH_FF, &threshold, 1, false, 1) < sizeof (threshold))
	{
		return 0;
	}

	return threshold;
}

uint8_t adxl345_freefall_get_time (adxl345_sensor *sensor)
{
	uint8_t time = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_TIME_FF, &time, 1, false, 1) < sizeof (time))
	{
		return 0;
	}

	return time;
}

uint8_t adxl345_tap_get_settings (adxl345_sensor *sensor)
{
	uint8_t settings = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_TAP_AXES, &settings, 1, false, 1) < sizeof (settings))
	{
		return 0;
	}

	return settings;
}

uint8_t adxl345_interrupt_get_involvement (adxl345_sensor *sensor)
{
	uint8_t involvement = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_ACT_TAP_STATUS, &involvement, 1, false, 1) < sizeof (involvement))
	{
		return 0;
	}

	return involvement;
}

// Function for getting BW_RATE
// Function for getting POWER_CTL

uint8_t adxl345_get_interrupt_status (adxl345_sensor *sensor)
{
	uint8_t status = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_INT_ENABLE, &status, 1, false, 1) < sizeof (status))
	{
		return 0;
	}

	return status;
}

uint8_t adxl345_get_interrupt_map (adxl345_sensor *sensor)
{
	uint8_t map = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_INT_MAP, &map, 1, false, 1) < sizeof (map))
	{
		return 0;
	}

	return map;
}

uint8_t adxl345_get_interrupt_source (adxl345_sensor *sensor)
{
	uint8_t source = 0;
	if (adxl345_read_timeout (sensor, ADXL345_REG_INT_SOURCE, &source, 1, false, 1) < sizeof (source))
	{
		return 0;
	}

	return source;
}

// Function for getting DATA_FORMAT
// Function for getting FIFO_CTL
// Function for getting FIFO_STATUS

/* ------------------------------------------------------------------------------------- */

// These functions can be used, but it's easier to use the functions above.

int adxl345_write (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop)
{
	uint8_t commands[2];
	commands[0] = reg_addr;
	commands[1] = command;

	return i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, commands, sizeof (commands), no_stop);
}

int adxl345_write_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop, unsigned int timeout_s)
{
	uint8_t commands[2];
	commands[0] = reg_addr;
	commands[1] = command;

	return i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, commands, sizeof (commands), no_stop, timeout_s * 1000 * 1000);
}

int adxl345_read (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop)
{
	if (i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, &reg_addr, sizeof (reg_addr), no_stop) < sizeof (reg_addr))
	{
		return 0;
	}

	return i2c_read_blocking (sensor->i2c_port, sensor->i2c_addr, buffer, length, no_stop);
}

int adxl345_read_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t *buffer, size_t length, bool no_stop, unsigned int timeout_s)
{
	unsigned int actual_timeout = (timeout_s * 1000 * 1000) / 2;
	if (i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &reg_addr, sizeof (reg_addr), no_stop, actual_timeout) < sizeof (reg_addr))
	{
		return 0;
	}

	return i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, buffer, length, no_stop, actual_timeout);
}