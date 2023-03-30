#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"

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
#define COMPASS_SERIAL Serial1
#define IR_SERIAL Serial2
#define CAMERA_SERIAL Serial3
#define LAYER1_SERIAL Serial4

// Global Variables
bool ballCaptured, ballFound, kick = false;

// Movement Config

#define COS45 0.70710678118654752440084436210485F
#define SIN45 0.70710678118654752440084436210485F

// Should ideally be <=1.0F to preserve resolution
#define ANGULAR_VELOCITY_MULTIPLIER 0.25F
#define DRIVE_STALL_SPEED (int16_t)50
#define DRIVE_MAX_SPEED (int16_t)200

#define BALL_MOVEMENT_A 1e-4F
#define BALL_MOVEMENT_B 10.0F

// #define STOP_WITHIN_LINE_BALL_RANGE  40
// #define STOP_WITHIN_LINE_STRENGTH    0.2
// #define BEGIN_ENTERING_LINE_STRENGTH 0.5
// #define ENTER_LINE_SPEED_MULTIPLIER  150.0
// #define ENTER_LINE_MAX_SPEED         200

// PID Constants
#define KP_ROBOT_ANGLE 3e-3F
#define KI_ROBOT_ANGLE 1e-9F
#define KD_ROBOT_ANGLE 1e2F
#define MIN_DT_ROBOT_ANGLE 5000

#endif