#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

// Pin definitions
#define DebugSerial Serial1
#define TeensySerial Serial2

#define LED 29

#define IMU_SCL PB10
#define IMU_SDA PB11