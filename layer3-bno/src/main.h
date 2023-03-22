#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <SPI.h>

// Pin definitions
#define TeensySerial Serial2

#define LED PA8

#define IMU_SCL PB10
#define IMU_SDA PB11