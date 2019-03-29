#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "MPU6050.h"
#include "main.h"
#include "binary_serial.hpp"

ADXL345 accel;
MPU6050 accelgyro;

typedef struct {bool adxl; bool mpu;} testStruct;

typedef struct {
  uint32_t timestamp;
  int16_t adxlAx; int16_t adxlAy; int16_t adxlAz;
  int16_t mpuAx; int16_t mpuAy; int16_t mpuAz;
  int16_t mpuGx; int16_t mpuGy; int16_t mpuGz;
  int16_t mpuTemp;
} rawStruct;

testStruct test;
rawStruct data;

uint8_t nbMeasureCallibration;
bool okCallibration;
uint32_t waitTime;

void setup() {
  // Initialization
  Wire.begin();
  Serial.begin(38400);
  accel.initialize();
  accelgyro.initialize();

  // Test connection of the sensors
  test.adxl = accel.testConnection();
  test.mpu = accelgyro.testConnection();

  // Send the status of the sensors
  writeData(&test, sizeof(test));

  // Calibration
  // calibrate();

  //Read the time between each measures
  readData(&waitTime, sizeof(waitTime));
}

void loop() {
  sendMeasures();

  //Wait
  delay(waitTime);
}

void sendMeasures() {
  data.timestamp = millis();
  if (test.adxl)
    accel.getAcceleration(&data.adxlAx, &data.adxlAy, &data.adxlAz);
  if (test.mpu) {
    accelgyro.getMotion6(&data.mpuAx, &data.mpuAy, &data.mpuAz,
      &data.mpuGx, &data.mpuGy, &data.mpuGz);
    data.mpuTemp = accelgyro.getTemperature();
  }

  //Send the raw data
  writeData(&data, sizeof(data));
}

void calibrate() {
  readData(&nbMeasureCallibration, sizeof(nbMeasureCallibration));
  readData(&okCallibration, sizeof(okCallibration));
  if (okCallibration)
    for (int i = 0 ; i < nbMeasureCallibration ; i++)
      sendMeasures();
}
