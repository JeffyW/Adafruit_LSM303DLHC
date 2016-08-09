/***************************************************************************
This is a library for the LSM303 Accelerometer and magnentometer/compass

Designed specifically to work with the Adafruit LSM303DLHC Breakout

These displays use I2C to communicate, 2 pins are required to interface.

Adafruit invests time and resources providing this open source code,
please support Adafruit andopen-source hardware by purchasing products
from Adafruit!

Written by Kevin Townsend for Adafruit Industries.
BSD license, all text above must be included in any redistribution
***************************************************************************/
#include "Arduino.h"
#include "Adafruit_LSM303_Accel_Unified.h"

static float _lsm303Accel_MG_LSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb

#define write(reg, value) _wire->write(LSM303_ADDRESS_ACCEL, reg, (uint8_t)value)
#define read8(reg, value) _wire->read(LSM303_ADDRESS_ACCEL, reg, 1, value)

/***************************************************************************
PRIVATE FUNCTIONS
***************************************************************************/

/**************************************************************************/
/*!
@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::read()
{
	// Read the accelerometer
	const uint8_t bytesToRead = 6;
	if (_wire->read(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, bytesToRead))
	{
		// Error.
		return false;
	}

	uint8_t xlo = _wire->receive();
	uint8_t xhi = _wire->receive();
	uint8_t ylo = _wire->receive();
	uint8_t yhi = _wire->receive();
	uint8_t zlo = _wire->receive();
	uint8_t zhi = _wire->receive();

	// Shift values to create properly formed integer (low byte first)
	_accelData.x = (int16_t)(xlo | xhi << 8) >> 4;
	_accelData.y = (int16_t)(ylo | yhi << 8) >> 4;
	_accelData.z = (int16_t)(zlo | zhi << 8) >> 4;

	return true;
}

/***************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************/
/*!
@brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::begin()
{
	// Enable I2C
	_wire->begin();

	// Enable the accelerometer (100Hz)
	write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
	// if we are connected or not
	uint8_t reg1_a;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, &reg1_a);
	if (reg1_a != 0x57)
	{
		return false;
	}

	// Disable the Data Ready interrupt so that it doesn't go high before it's attached
	enableInt1DataReady(false);

	// Default to normal mode
	enableLowPower(false);

	return true;
}

/**************************************************************************/
/*!
@brief  Set the Accelerometer range
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::setAccelRange(lsm303AccelRange range)
{
	switch (range)
	{
	case LSM303_ACCEL_RANGE_2:
		_lsm303Accel_MG_LSB = 0.001F;
		break;
	case LSM303_ACCEL_RANGE_4:
		_lsm303Accel_MG_LSB = 0.002F;
		break;
	case LSM303_ACCEL_RANGE_8:
		_lsm303Accel_MG_LSB = 0.004F;
		break;
	case LSM303_ACCEL_RANGE_16:
		_lsm303Accel_MG_LSB = 0.012F;
		break;
	}

	uint8_t existing;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG4_A, &existing);
	write(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (existing |= range << 4));
}

/**************************************************************************/
/*!
@brief  Sets Data Ready on Int1
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::enableInt1DataReady(bool enabled) {
	byte existing;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG3_A, &existing);

	if (enabled) {
		write(LSM303_REGISTER_ACCEL_CTRL_REG3_A, (existing |= 1 << 4));
	}
	else {
		write(LSM303_REGISTER_ACCEL_CTRL_REG3_A, (existing &= ~(1 << 4)));
	}
}

/**************************************************************************/
/*!
@brief  Sets the Accelerometer Output Data Rate
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::setAccelRate(lsm303AccelRate odr)
{
	byte existing;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, &existing);

	// unset the ODR then set it
	write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, (existing &= ~(0x0f << 4)));
	write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, (existing |= odr << 4));
}

/**************************************************************************/
/*!
@brief  Enables Low Power mode
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::enableLowPower(bool enabled)
{
	byte existing1;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, &existing1);
	byte existing4;
	read8(LSM303_REGISTER_ACCEL_CTRL_REG4_A, &existing4);

	if (enabled) {
		// Set CTRL_REG1_A/LPen and unset CTRL_REG4_A/HR
		write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, (existing1 |= 1 << 3));
		write(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (existing4 &= ~(1 << 3)));
	}
	else {
		// Unset CTRL_REG1_A/LPen and set CTRL_REG4_A/HR
		write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, (existing1 &= ~(1 << 3)));
		write(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (existing4 |= 1 << 3));
	}
}

/**************************************************************************/
/*!
@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::getEvent(sensors_vec_t* acceleration) {
	/* Clear the event */
	memset(acceleration, 0, sizeof(sensors_vec_t));

	/* Read new data */
	if (!read())
	{
		return false;
	}

	acceleration->x = _accelData.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	acceleration->y = _accelData.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	acceleration->z = _accelData.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	return true;
}
