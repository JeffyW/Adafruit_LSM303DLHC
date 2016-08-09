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
#ifndef __LSM303_H_Accel__
#define __LSM303_H_Accel__

#include <Adafruit_Sensor.h>
#include <I2C-Master-Library/I2C.h>

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
/*=========================================================================*/

/*=========================================================================
REGISTERS
-----------------------------------------------------------------------*/
typedef enum
{                                                     // DEFAULT    TYPE
	LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20,   // 00000111   rw
	LSM303_REGISTER_ACCEL_CTRL_REG2_A = 0x21,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG3_A = 0x22,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG5_A = 0x24,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG6_A = 0x25,   // 00000000   rw
	LSM303_REGISTER_ACCEL_REFERENCE_A = 0x26,   // 00000000   r
	LSM303_REGISTER_ACCEL_STATUS_REG_A = 0x27,   // 00000000   r
	LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28,
	LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29,
	LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2A,
	LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2B,
	LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2C,
	LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2D,
	LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E,
	LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F,
	LSM303_REGISTER_ACCEL_INT1_CFG_A = 0x30,
	LSM303_REGISTER_ACCEL_INT1_SOURCE_A = 0x31,
	LSM303_REGISTER_ACCEL_INT1_THS_A = 0x32,
	LSM303_REGISTER_ACCEL_INT1_DURATION_A = 0x33,
	LSM303_REGISTER_ACCEL_INT2_CFG_A = 0x34,
	LSM303_REGISTER_ACCEL_INT2_SOURCE_A = 0x35,
	LSM303_REGISTER_ACCEL_INT2_THS_A = 0x36,
	LSM303_REGISTER_ACCEL_INT2_DURATION_A = 0x37,
	LSM303_REGISTER_ACCEL_CLICK_CFG_A = 0x38,
	LSM303_REGISTER_ACCEL_CLICK_SRC_A = 0x39,
	LSM303_REGISTER_ACCEL_CLICK_THS_A = 0x3A,
	LSM303_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B,
	LSM303_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C,
	LSM303_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D
} lsm303AccelRegisters_t;

/*=========================================================================*/

/*=========================================================================
ACCELEROMETER RANGE SETTINGS
-----------------------------------------------------------------------*/
typedef enum
{
	LSM303_ACCEL_RANGE_2 = 0x00, // +/- 2G
	LSM303_ACCEL_RANGE_4 = 0x01, // +/- 4G
	LSM303_ACCEL_RANGE_8 = 0x02, // +/- 8G
	LSM303_ACCEL_RANGE_16 = 0x03  // +/- 16G
} lsm303AccelRange;
/*=========================================================================*/

/*=========================================================================
ACCELEROMETER OUTPUT DATA RATE SETTINGS
-----------------------------------------------------------------------*/
typedef enum
{
	LSM303_ACCEL_ODR_OFF = 0x00, // Power-down mode
	LSM303_ACCEL_ODR_1 = 0x01, // Normal / low-power mode (1 Hz)
	LSM303_ACCEL_ODR_10 = 0x02, // Normal / low-power mode (10 Hz)
	LSM303_ACCEL_ODR_25 = 0x03, // Normal / low-power mode (25 Hz)
	LSM303_ACCEL_ODR_50 = 0x04, // Normal / low-power mode (50 Hz)
	LSM303_ACCEL_ODR_100 = 0x05, // Normal / low-power mode (100 Hz)
	LSM303_ACCEL_ODR_200 = 0x06, // Normal / low-power mode (200 Hz)
	LSM303_ACCEL_ODR_400 = 0x07, // Normal / low-power mode (400 Hz)
	LSM303_ACCEL_ODR_1344 = 0x09, // Normal (1.344 kHz) / low-power mode (5.376 KHz)
	LSM303_ACCEL_ODR_1620 = 0x08, // Low-power mode (1.620 KHz)
	LSM303_ACCEL_ODR_5376 = 0x09, // Normal (1.344 kHz) / low-power mode (5.376 KHz)
} lsm303AccelRate;
/*=========================================================================*/

/*=========================================================================
INTERNAL ACCELERATION DATA TYPE
-----------------------------------------------------------------------*/
typedef struct lsm303AccelData_s
{
	float x;
	float y;
	float z;
} lsm303AccelData;
/*=========================================================================*/

/* Unified sensor driver for the accelerometer */
class Adafruit_LSM303_Accel_Unified
{
public:
	Adafruit_LSM303_Accel_Unified(I2C* wire, int32_t sensorID = -1) :
		_wire(wire),
		_sensorID(sensorID) {}
	Adafruit_LSM303_Accel_Unified(int32_t sensorID = -1) :
		Adafruit_LSM303_Accel_Unified(&I2c, sensorID) {}

	bool begin(void);
	void setAccelRange(lsm303AccelRange);
	void enableInt1DataReady(bool);
	void setAccelRate(lsm303AccelRate);
	void enableLowPower(bool);
	bool getEvent(sensors_vec_t* acceleration);
	void enableAutoRange(bool enabled) {};

private:
	I2C*        _wire;
	lsm303AccelData _accelData;   // Last read accelerometer data will be available here
	int32_t         _sensorID;

	bool read(void);
};

#endif
