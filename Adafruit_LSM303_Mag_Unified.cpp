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
#include "Adafruit_LSM303_Mag_Unified.h"

static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z = 980.0F;   // Varies with gain

#define write(reg, value) _wire->write(LSM303_ADDRESS_MAG, reg, (uint8_t)value)
#define read8(reg, value) _wire->read(LSM303_ADDRESS_MAG, reg, 1, value)

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
	@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::read()
{
	// Read the magnetometer
	const uint8_t bytesToRead = 6;
	if (_wire->read(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M, bytesToRead))
	{
		// Error.
		return false;
	}

	// Note high before low (different than accel)
	uint8_t xhi = _wire->receive();
	uint8_t xlo = _wire->receive();
	uint8_t zhi = _wire->receive();
	uint8_t zlo = _wire->receive();
	uint8_t yhi = _wire->receive();
	uint8_t ylo = _wire->receive();

	// Shift values to create properly formed integer (low byte first)
	_magData.x = (int16_t)(xlo | xhi << 8);
	_magData.y = (int16_t)(ylo | yhi << 8);
	_magData.z = (int16_t)(zlo | zhi << 8);

	// ToDo: Calculate orientation
	// _magData.orientation = 0.0;

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
bool Adafruit_LSM303_Mag_Unified::begin()
{
	// Enable I2C
	_wire->begin();

	// Enable the magnetometer
	write(LSM303_REGISTER_MAG_MR_REG_M, 0x00);

	// Reset the default ODR (15 Hz)
	setMagRate(LSM303_MAGRATE_15);

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
	// the default value (0b00010000/0x10)
	uint8_t reg1_a;
	read8(LSM303_REGISTER_MAG_CRA_REG_M, &reg1_a);
	if (reg1_a != 0x10)
	{
		return false;
	}

	// Set the gain to a known level
	setMagGain(LSM303_MAGGAIN_1_3);

	return true;
}

/**************************************************************************/
/*!
	@brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::enableAutoRange(bool enabled)
{
	_autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
	@brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::setMagGain(lsm303MagGain gain)
{
	write(LSM303_REGISTER_MAG_CRB_REG_M, gain);

	_magGain = gain;

	switch (gain)
	{
	case LSM303_MAGGAIN_1_3:
		_lsm303Mag_Gauss_LSB_XY = 1100;
		_lsm303Mag_Gauss_LSB_Z = 980;
		break;
	case LSM303_MAGGAIN_1_9:
		_lsm303Mag_Gauss_LSB_XY = 855;
		_lsm303Mag_Gauss_LSB_Z = 760;
		break;
	case LSM303_MAGGAIN_2_5:
		_lsm303Mag_Gauss_LSB_XY = 670;
		_lsm303Mag_Gauss_LSB_Z = 600;
		break;
	case LSM303_MAGGAIN_4_0:
		_lsm303Mag_Gauss_LSB_XY = 450;
		_lsm303Mag_Gauss_LSB_Z = 400;
		break;
	case LSM303_MAGGAIN_4_7:
		_lsm303Mag_Gauss_LSB_XY = 400;
		_lsm303Mag_Gauss_LSB_Z = 355;
		break;
	case LSM303_MAGGAIN_5_6:
		_lsm303Mag_Gauss_LSB_XY = 330;
		_lsm303Mag_Gauss_LSB_Z = 295;
		break;
	case LSM303_MAGGAIN_8_1:
		_lsm303Mag_Gauss_LSB_XY = 230;
		_lsm303Mag_Gauss_LSB_Z = 205;
		break;
	}
}

/**************************************************************************/
/*!
	@brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::setMagRate(lsm303MagRate rate)
{
	uint8_t reg_m = ((uint8_t)rate & 0x07) << 2;
	write(LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}


/**************************************************************************/
/*!
	@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::getEvent(sensors_vec_t *magnetic) {
	bool readingValid = false;

	/* Clear the event */
	memset(magnetic, 0, sizeof(sensors_vec_t));

	while (!readingValid)
	{
		uint8_t reg_mg;
		read8(LSM303_REGISTER_MAG_SR_REG_Mg, &reg_mg);
		if (!(reg_mg & 0x1)) {
			return false;
		}

		/* Read new data */
		if (!read())
		{
			return false;
		}

		/* Make sure the sensor isn't saturating if auto-ranging is enabled */
		readingValid = true;
		if (_autoRangeEnabled)
		{
			/* Check if the sensor is saturating or not */
			if ((_magData.x >= 2040) | (_magData.x <= -2040) |
				(_magData.y >= 2040) | (_magData.y <= -2040) |
				(_magData.z >= 2040) | (_magData.z <= -2040))
			{
				/* Saturating .... increase the range if we can */
				switch (_magGain)
				{
					case LSM303_MAGGAIN_5_6:
						setMagGain(LSM303_MAGGAIN_8_1);
						readingValid = false;
						break;
					case LSM303_MAGGAIN_4_7:
						setMagGain(LSM303_MAGGAIN_5_6);
						readingValid = false;
						break;
					case LSM303_MAGGAIN_4_0:
						setMagGain(LSM303_MAGGAIN_4_7);
						readingValid = false;
						break;
					case LSM303_MAGGAIN_2_5:
						setMagGain(LSM303_MAGGAIN_4_0);
						readingValid = false;
						break;
					case LSM303_MAGGAIN_1_9:
						setMagGain(LSM303_MAGGAIN_2_5);
						readingValid = false;
						break;
					case LSM303_MAGGAIN_1_3:
						setMagGain(LSM303_MAGGAIN_1_9);
						readingValid = false;
						break;
					default:
						break;
				}
			}
		}
	}
	magnetic->x = _magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	magnetic->y = _magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	magnetic->z = _magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

	return true;
}
