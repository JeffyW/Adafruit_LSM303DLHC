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
#include <Wire.h>
#include <limits.h>
#include "Adafruit_LSM303_U.h"

static float _lsm303Accel_MG_LSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z = 980.0F;   // Varies with gain

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/
 /***************************************************************************
  PRIVATE FUNCTIONS
  ***************************************************************************/

  /**************************************************************************/
  /*!
	  @brief  Abstract away platform differences in Arduino wire library
  */
  /**************************************************************************/
void Adafruit_LSM303_Accel_Unified::write8(uint8_t address, uint8_t reg, uint8_t value)
{
	_wire->beginTransmission(address);
	_wire->write(reg);
	_wire->write(value);
	_wire->endTransmission();
}

/**************************************************************************/
/*!
	@brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t Adafruit_LSM303_Accel_Unified::read8(uint8_t address, uint8_t reg)
{
	_wire->requestFrom(address, 1, reg, 1, true);
	return _wire->read();
}

/**************************************************************************/
/*!
	@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::read()
{
	// Read the accelerometer
	const uint8_t bytesToRead = 6;
	if (_wire->requestFrom(LSM303_ADDRESS_ACCEL, bytesToRead, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, 1, true) != bytesToRead)
	{
		// Error.
		return false;
	}

	uint8_t xlo = _wire->read();
	uint8_t xhi = _wire->read();
	uint8_t ylo = _wire->read();
	uint8_t yhi = _wire->read();
	uint8_t zlo = _wire->read();
	uint8_t zhi = _wire->read();

	// Shift values to create properly formed integer (low byte first)
	_accelData.x = (int16_t)(xlo | (xhi << 8)) >> 4;
	_accelData.y = (int16_t)(ylo | (yhi << 8)) >> 4;
	_accelData.z = (int16_t)(zlo | (zhi << 8)) >> 4;

	return true;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

 /**************************************************************************/
 /*!
	 @brief  Instantiates a new Adafruit_LSM303 class with the specified I2C wire
 */
 /**************************************************************************/
Adafruit_LSM303_Accel_Unified::Adafruit_LSM303_Accel_Unified(TwoWire* wire, int32_t sensorID) {
	_wire = wire;
	_sensorID = sensorID;
}

/**************************************************************************/
/*!
@brief  Instantiates a new Adafruit_LSM303 class using the default I2C wire
*/
/**************************************************************************/
Adafruit_LSM303_Accel_Unified::Adafruit_LSM303_Accel_Unified(int32_t sensorID) : Adafruit_LSM303_Accel_Unified(&Wire, sensorID) {
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
	write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
	// if we are connected or not
	uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
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

	byte existing = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A);

	write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, existing |= range << 4);
}

/**************************************************************************/
/*!
	@brief  Sets Data Ready on Int1
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::enableInt1DataReady(bool enabled) {
	byte existing = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG3_A);

	if (enabled) {
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG3_A, existing |= 1 << 4);
	}
	else {
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG3_A, existing &= ~(1 << 4));
	}
}

/**************************************************************************/
/*!
	@brief  Sets the Accelerometer Output Data Rate
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::setAccelRate(lsm303AccelRate odr)
{
	byte existing = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);

	// unset the ODR then set it
	write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, existing &= ~(0x0f << 4));
	write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, existing |= odr << 4);
}

/**************************************************************************/
/*!
	@brief  Enables Low Power mode
*/
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::enableLowPower(bool enabled)
{
	byte existing1 = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	byte existing4 = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A);

	if (enabled) {
		// Set CTRL_REG1_A/LPen and unset CTRL_REG4_A/HR
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, existing1 |= 1 << 3);
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, existing4 &= ~(1 << 3));
	}
	else {
		// Unset CTRL_REG1_A/LPen and set CTRL_REG4_A/HR
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, existing1 &= ~(1 << 3));
		write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, existing4 |= 1 << 3);
	}
}

/**************************************************************************/
/*!
	@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::getEvent(sensors_event_t *event) {
	/* Clear the event */
	memset(event, 0, sizeof(sensors_event_t));

	/* Read new data */
	if (!read())
	{
		return false;
	}

	event->version = sizeof(sensors_event_t);
	event->sensor_id = _sensorID;
	event->type = SENSOR_TYPE_ACCELEROMETER;
	event->timestamp = millis();
	event->acceleration.x = _accelData.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	event->acceleration.y = _accelData.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	event->acceleration.z = _accelData.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

	return true;
}

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
 /***************************************************************************
  PRIVATE FUNCTIONS
  ***************************************************************************/

  /**************************************************************************/
  /*!
	  @brief  Abstract away platform differences in Arduino wire library
  */
  /**************************************************************************/
void Adafruit_LSM303_Mag_Unified::write8(uint8_t address, uint8_t reg, uint8_t value)
{
	_wire->beginTransmission(address);
	_wire->write(reg);
	_wire->write(value);
	_wire->endTransmission();
}

/**************************************************************************/
/*!
	@brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t Adafruit_LSM303_Mag_Unified::read8(uint8_t address, uint8_t reg)
{
	_wire->requestFrom(address, 1, reg, 1, true);
	return _wire->read();
}

/**************************************************************************/
/*!
	@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::read()
{
	// Read the magnetometer
	const uint8_t bytesToRead = 6;
	if (_wire->requestFrom(LSM303_ADDRESS_MAG, bytesToRead, LSM303_REGISTER_MAG_OUT_X_H_M, 1, true) != bytesToRead)
	{
		// Error.
		return false;
	}

	// Note high before low (different than accel)
	uint8_t xhi = _wire->read();
	uint8_t xlo = _wire->read();
	uint8_t zhi = _wire->read();
	uint8_t zlo = _wire->read();
	uint8_t yhi = _wire->read();
	uint8_t ylo = _wire->read();

	// Shift values to create properly formed integer (low byte first)
	_magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
	_magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
	_magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));

	// ToDo: Calculate orientation
	// _magData.orientation = 0.0;

	return true;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

 /**************************************************************************/
 /*!
	 @brief  Instantiates a new Adafruit_LSM303 class
 */
 /**************************************************************************/
Adafruit_LSM303_Mag_Unified::Adafruit_LSM303_Mag_Unified(TwoWire* wire, int32_t sensorID) {
	_wire = wire;
	_sensorID = sensorID;
	_autoRangeEnabled = false;
}

/**************************************************************************/
/*!
@brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
Adafruit_LSM303_Mag_Unified::Adafruit_LSM303_Mag_Unified(int32_t sensorID) : Adafruit_LSM303_Mag_Unified(&Wire, sensorID) {
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
	write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

	// Reset the default ODR (15 Hz)
	setMagRate(LSM303_MAGRATE_15);

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
	// the default value (0b00010000/0x10)
	uint8_t reg1_a = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
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
	write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);

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
	byte reg_m = ((byte)rate & 0x07) << 2;
	write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
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

		uint8_t reg_mg = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_SR_REG_Mg);
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
