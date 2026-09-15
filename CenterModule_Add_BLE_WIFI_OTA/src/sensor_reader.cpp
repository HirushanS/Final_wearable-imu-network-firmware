#include "sensor_reader.h"

#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include "project_config.h"

namespace {

Adafruit_BNO08x bno08x(
    ProjectConfig::CENTER_IMU_RST
);

sh2_SensorValue_t sensorValue;

bool sensorReady = false;
bool hasAcceleration = false;
bool hasGyroscope = false;
bool hasMagnetometer = false;
bool hasQuaternion = false;

void clearReportFlags()
{
    hasAcceleration = false;
    hasGyroscope = false;
    hasMagnetometer = false;
    hasQuaternion = false;
}

bool enableBNO085Reports()
{
    Serial.println("Configuring BNO085 SPI reports...");

    bool status = true;


    if (!bno08x.enableReport(
            SH2_ACCELEROMETER,
            50000))
    {
        Serial.println(
            "Failed enabling accelerometer"
        );
        status = false;
    }



    if (!bno08x.enableReport(
            SH2_GYROSCOPE_CALIBRATED,
            50000))
    {
        Serial.println(
            "Failed enabling gyroscope"
        );
        status = false;
    }



    if (!bno08x.enableReport(
            SH2_MAGNETIC_FIELD_CALIBRATED,
            50000))
    {
        Serial.println(
            "Failed enabling magnetometer"
        );
        status = false;
    }



    if (!bno08x.enableReport(
            SH2_ROTATION_VECTOR,
            50000))
    {
        Serial.println(
            "Failed enabling rotation vector"
        );
        status = false;
    }


    return status;
}


}



bool initBNO085Sensor()
{

    Serial.println(
        "Initializing Center BNO085 using SPI..."
    );


    SPI.begin(
        ProjectConfig::CENTER_IMU_CLK,
        ProjectConfig::CENTER_IMU_MISO,
        ProjectConfig::CENTER_IMU_MOSI,
        ProjectConfig::CENTER_IMU_CS
    );



    if (!bno08x.begin_SPI(
            ProjectConfig::CENTER_IMU_CS,
            ProjectConfig::CENTER_IMU_INT,
            &SPI))
    {

        Serial.println(
            "BNO085 SPI initialization failed"
        );

        sensorReady = false;
        clearReportFlags();

        return false;
    }



    Serial.println(
        "BNO085 SPI detected"
    );



    delay(200);


    clearReportFlags();


    if(!enableBNO085Reports())
    {

        Serial.println(
            "BNO085 report configuration failed"
        );

        sensorReady = false;
        clearReportFlags();

        return false;
    }



    sensorReady = true;


    Serial.println(
        "BNO085 SPI sensor ready"
    );


    return true;
}





bool readBNO085Sensor(
    BNO085Reading &reading
)
{

    if(!sensorReady)
    {
        return false;
    }



    if(bno08x.wasReset())
    {

        Serial.println(
            "BNO085 reset detected. Re-enabling reports..."
        );

        clearReportFlags();
        enableBNO085Reports();

    }




    if(!bno08x.getSensorEvent(
            &sensorValue))
    {
        return false;
    }




    switch(sensorValue.sensorId)
    {


        case SH2_ACCELEROMETER:

            reading.accelX =
                sensorValue.un.accelerometer.x;

            reading.accelY =
                sensorValue.un.accelerometer.y;

            reading.accelZ =
                sensorValue.un.accelerometer.z;

            hasAcceleration = true;

            break;



        case SH2_GYROSCOPE_CALIBRATED:

            reading.gyroX =
                sensorValue.un.gyroscope.x;

            reading.gyroY =
                sensorValue.un.gyroscope.y;

            reading.gyroZ =
                sensorValue.un.gyroscope.z;

            hasGyroscope = true;

            break;



        case SH2_MAGNETIC_FIELD_CALIBRATED:

            reading.magX =
                sensorValue.un.magneticField.x;

            reading.magY =
                sensorValue.un.magneticField.y;

            reading.magZ =
                sensorValue.un.magneticField.z;

            hasMagnetometer = true;

            break;



        case SH2_ROTATION_VECTOR:

            reading.quatW =
                sensorValue.un.rotationVector.real;

            reading.quatX =
                sensorValue.un.rotationVector.i;

            reading.quatY =
                sensorValue.un.rotationVector.j;

            reading.quatZ =
                sensorValue.un.rotationVector.k;

            hasQuaternion = true;

            break;



        default:

            break;

    }



    return true;

}





bool isBNO085SensorReady()
{
    return sensorReady;
}


bool isBNO085ReadingReady()
{
    return
        hasAcceleration &&
        hasGyroscope &&
        hasQuaternion;
}
