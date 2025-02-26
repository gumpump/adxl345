#include "adxl345.h"

#define I2C_PORT	i2c0		// I²C-port 0
#define I2C_FREQ	100 * 1000	// 100 kHz

// GPIO number, not pin number
#define I2C_SDA_PIN	12
#define I2C_SCL_PIN	13

adxl345_sensor sensor;
adxl345_axis_data data;

int main ()
{
	// You need this to send the printf outputs to the computer
	stdio_init_all ();

	// Initializing I²C
	gpio_set_function (I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function (I2C_SCL_PIN, GPIO_FUNC_I2C);

	i2c_init (I2C_PORT, I2C_FREQ);

	// Initializing the sensor
	if (adxl345_init (&sensor, I2C_PORT, 0) == false)
	{
		printf ("Couldn't initialize sensor");
	}

	// Starting measurement
	if (adxl345_power_settings (&sensor, ADXL345_POWER_MEASURE) == false)
	{
		printf ("Couldn't start measuring");
	}

	while (1)
	{
		// Get data from the sensor
		adxl345_axis_get_data_timeout (&sensor, &data, 1);

		// Access data for z-axis and print it
		printf ("Value on z-axis: %i", data.axis[ADXL345_INDEX_Z]);

		// Wait 100 ms.
		// If you want more data per second, change the value or remove it.
		sleep_ms (100);
	}

	return 0;
}