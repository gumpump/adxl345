/*
 * Copyright (c) Wolfgang Neue
 *
 * SPDX-License-Identifier: MIT
 */

#include "adxl345.h"

#include <math.h>

#define ADXL345_X 0
#define ADXL345_Y 1
#define ADXL345_Z 2

#define ADXL345_V_MAX 32767
#define ADXL345_V_OFFSET 65536

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

int adxl345_write (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop)
{
	uint8_t commands[2];
	commands[0] = reg_addr;
	commands[1] = command;

	return i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, commands, 2, no_stop);
}

int adxl345_write_timeout (adxl345_sensor *sensor, uint8_t reg_addr, uint8_t command, bool no_stop, unsigned int timeout_s)
{
	uint8_t commands[2];
	commands[0] = reg_addr;
	commands[1] = command;

	return i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, commands, 2, no_stop, timeout_s * 1000 * 1000);
}

int adxl345_read_8 (adxl345_sensor *sensor, adxl345_data *data)
{
	uint8_t command;
	command = ADXL345_REG_VAL;
	i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, &command, 1, false);
	return i2c_read_blocking (sensor->i2c_port, sensor->i2c_addr, data->data_8, 8, false);
}

int adxl345_read_8_timeout (adxl345_sensor *sensor, adxl345_data *data, unsigned int timeout_s)
{
	uint8_t command;
	command = ADXL345_REG_VAL;
	unsigned int actual_timeout = (timeout_s * 1000 * 1000) / 2;
	i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &command, 1, false, actual_timeout);
	return i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, data->data_8, 8, false, actual_timeout);
}

int adxl345_read_6 (adxl345_sensor *sensor, adxl345_data *data)
{
	uint8_t command;
	command = ADXL345_REG_VAL;
	i2c_write_blocking (sensor->i2c_port, sensor->i2c_addr, &command, 1, false);
	return i2c_read_blocking (sensor->i2c_port, sensor->i2c_addr, data->data_8, 6, false);
}

int adxl345_read_6_timeout (adxl345_sensor *sensor, adxl345_data *data, unsigned int timeout_s)
{
	uint8_t command;
	command = ADXL345_REG_VAL;
	unsigned int actual_timeout = (timeout_s * 1000 * 1000) / 2;
	i2c_write_timeout_us (sensor->i2c_port, sensor->i2c_addr, &command, 1, false, actual_timeout);
	return i2c_read_timeout_us (sensor->i2c_port, sensor->i2c_addr, data->data_8, 6, false, actual_timeout);
}

int adxl345_get_x (adxl345_data *data)
{
	int x = (int)data->data_16[ADXL345_X];

	return (x > ADXL345_V_MAX) ? x - ADXL345_V_OFFSET : x;
}

int adxl345_get_y (adxl345_data *data)
{
	int y = (int)data->data_16[ADXL345_Y];

	return (y > ADXL345_V_MAX) ? y - ADXL345_V_OFFSET : y;
}

int adxl345_get_z (adxl345_data *data)
{
	int z = (int)data->data_16[ADXL345_Z];

	return (z > ADXL345_V_MAX) ? z - ADXL345_V_OFFSET : z;
}

float adxl345_get_roll (adxl345_data *data)
{
	float y = (float) adxl345_get_y (data);
	float z = (float) adxl345_get_z (data);

	return atan2 (y, z) * 57.3;
}

float adxl345_get_pitch (adxl345_data *data)
{
	float x = (float) adxl345_get_x (data);
	float y = (float) adxl345_get_y (data);
	float z = (float) adxl345_get_z (data);

	return atan2 (-x, sqrt (y * y + z * z)) * 57.3;
}
