#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"
#include "Wire.h"
#include "SPI.h"
#include "EEPROM.h"

#define SYNC_BYTE 0b11010110

// Pin Definitions
#define FL_PWM 3
#define FL_DIR 4
#define FR_PWM 5
#define FR_DIR 6
#define BR_PWM 9
#define BR_DIR 10
#define BL_PWM 11
#define BL_DIR 12
#define BALL_DETECT 18
#define DRIBBLER_PWM 19
#define BUILTIN_LED 13
// Serial Definitions
#define DEBUG Serial
#define IR_SERIAL Serial2
#define CAMERA_SERIAL Serial3
#define LAYER1_SERIAL Serial4
// Global Variables
int16_t bearingToAngle(float bearing);
int16_t robotAngle = 0;
int ballAngle, ballStrength;
bool ballCaptured, ballFound, kick = false;
float offsetAngle;

// Movement Config

#define COS45 0.70710678118654752440084436210485F
#define SIN45 0.70710678118654752440084436210485F

// Should ideally be <=1.0F to preserve resolution
#define ANGULAR_VELOCITY_MULTIPLIER 0.25F
#define DRIVE_STALL_SPEED (int16_t)50
#define DRIVE_MAX_SPEED (int16_t)100

#define BALL_MOVEMENT_A 1e-4F
#define BALL_MOVEMENT_B 10.0F

#endif