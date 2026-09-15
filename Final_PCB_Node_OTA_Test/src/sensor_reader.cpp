#include "sensor_reader.h"

#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include "project_config.h"
#include "hardware.h"


namespace {

Adafruit_BNO08x bno08x(IMU_RST);

sh2_SensorValue_t sensorValue;


bool hasAcceleration = false;
bool hasGyroscope = false;
bool hasQuaternion = false;


void clearReportFlags() {

  hasAcceleration = false;
  hasGyroscope = false;
  hasQuaternion = false;

}


bool setBNO085Reports() {

  Serial.println("Setting BNO085 SPI reports at 20Hz");


  bool allReportsEnabled = true;


  if(!bno08x.enableReport(
      SH2_LINEAR_ACCELERATION,
      BNO085_REPORT_INTERVAL_US))
  {
    Serial.println("Linear acceleration report failed");
    allReportsEnabled=false;
  }


  if(!bno08x.enableReport(
      SH2_GYROSCOPE_CALIBRATED,
      BNO085_REPORT_INTERVAL_US))
  {
    Serial.println("Gyro report failed");
    allReportsEnabled=false;
  }


  if(!bno08x.enableReport(
      SH2_ROTATION_VECTOR,
      BNO085_REPORT_INTERVAL_US))
  {
    Serial.println("Quaternion report failed");
    allReportsEnabled=false;
  }


  return allReportsEnabled;
}



}


bool initBNO085Sensor()
{

  Serial.println("Starting BNO085 SPI");


  SPI.begin(
      IMU_CLK,
      IMU_MISO,
      IMU_MOSI,
      IMU_CS
  );


  if(!bno08x.begin_SPI(
        IMU_CS,
        IMU_INT,
        &SPI))
  {

    Serial.println(
      "Failed to find BNO085 using SPI"
    );

    return false;

  }


  Serial.println(
      "BNO085 SPI detected successfully"
  );


  clearReportFlags();


  if(!setBNO085Reports())
  {
    Serial.println(
      "Failed enabling reports"
    );

    return false;
  }


  delay(250);


  return true;

}




bool updateBNO085Sensor(IMU_Node_Frame &frame)
{


  if(bno08x.wasReset())
  {

    Serial.println(
      "BNO085 reset detected"
    );

    clearReportFlags();

    setBNO085Reports();

  }



  if(!bno08x.getSensorEvent(&sensorValue))
  {
    return false;
  }



  switch(sensorValue.sensorId)
  {


    case SH2_LINEAR_ACCELERATION:

      frame.accelX =
      sensorValue.un.linearAcceleration.x;

      frame.accelY =
      sensorValue.un.linearAcceleration.y;

      frame.accelZ =
      sensorValue.un.linearAcceleration.z;

      hasAcceleration=true;

      break;



    case SH2_GYROSCOPE_CALIBRATED:

      frame.gyroX =
      sensorValue.un.gyroscope.x;

      frame.gyroY =
      sensorValue.un.gyroscope.y;

      frame.gyroZ =
      sensorValue.un.gyroscope.z;

      hasGyroscope=true;

      break;



    case SH2_ROTATION_VECTOR:

      frame.quatW =
      sensorValue.un.rotationVector.real;

      frame.quatX =
      sensorValue.un.rotationVector.i;

      frame.quatY =
      sensorValue.un.rotationVector.j;

      frame.quatZ =
      sensorValue.un.rotationVector.k;


      hasQuaternion=true;

      break;


  }


  return true;

}



bool isBNO085FrameReady()
{

 return hasAcceleration &&
        hasGyroscope &&
        hasQuaternion;

}



void sensorFailureLoop(uint8_t failLedPin)
{

 while(true)
 {

  digitalWrite(
      failLedPin,
      HIGH
  );

  delay(100);


  digitalWrite(
      failLedPin,
      LOW
  );

  delay(100);

 }

}
