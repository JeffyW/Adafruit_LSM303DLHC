/*
 * LSM303 Interrupt example.
 *
 * Breakout: https://www.adafruit.com/products/1120
 * 10DOF: https://www.adafruit.com/product/1604
 *
 * Connect DRDY (or LRDY on the 10DOF) to interrupt 0
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


#define DEBUG

// Set the output rate.  This is independent of the sensor sampling.
#define SERIAL_OUTPUT_HZ 10


Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

unsigned int loop_count;
unsigned int accel_count;
unsigned int mag_count;

unsigned long serial_output_timer;

float ax, ay, az, mx, my, mz;

volatile boolean magDataReady = false;


void setup() {
  Serial.begin(115200);

  attachInterrupt(0, magDataReadyISR, RISING);      // DRDY, Mag Data Ready

  if (!accel.begin()) {
    Serial.println(F("Ooops, no LSM303 detected (accel)... Check your wiring!"));
    while(1);
  }
  accel.setOutputDataRate(LSM303_ACCEL_ODR_200);  // Set a 200mhz data rate

  if (!mag.begin()) {
    Serial.println("Ooops, no LSM303 detected (mag)... Check your wiring!");
    while(1);
  }
  mag.setOutputDataRate(LSM303_MAG_ODR_30);      // Set a 30mhz data rate
  // mag interrupt is on by default
}


void magDataReadyISR () {
  magDataReady = true;
}


void loop() {
#ifdef DEBUG
  if (millis() > 1000) {
    Serial.println();
    Serial.print(F("Loop count: ")); Serial.println(loop_count);
    Serial.print(F("LSM303 (accel) reads: ")); Serial.println(accel_count);
    Serial.print(F("LSM303 (mag) reads: ")); Serial.println(mag_count);

    while(1) {}
  }
#endif

  loop_count++;

  if (accel.dataReady()) {
    accel_count++;

    sensors_event_t event;

    accel.getEvent(&event);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;
  }

  if (magDataReady) {
    mag_count++;
    magDataReady = false;

    sensors_event_t event;

    mag.getEvent(&event);
    mx = event.magnetic.x;
    my = event.magnetic.y;
    mz = event.magnetic.z;
  }

  if (SERIAL_OUTPUT_HZ > 0 && millis() > serial_output_timer + (1000 / SERIAL_OUTPUT_HZ)) {
    serial_output_timer = millis();

    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.println(mz);
  }

  // Guard against millis() rollover
  if (serial_output_timer > millis()) {
    serial_output_timer = 0;
  }
}
