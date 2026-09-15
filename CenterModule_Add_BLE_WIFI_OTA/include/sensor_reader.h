#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>


struct BNO085Reading {

  float accelX;
  float accelY;
  float accelZ;

  float gyroX;
  float gyroY;
  float gyroZ;

  float magX;
  float magY;
  float magZ;

  float quatW;
  float quatX;
  float quatY;
  float quatZ;

};


bool initBNO085Sensor();

bool readBNO085Sensor(
    BNO085Reading &reading
);

bool isBNO085SensorReady();
bool isBNO085ReadingReady();


#endif
