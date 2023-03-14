#include "Arduino.h"

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
#define ULTRA_SERIAL Serial5

// Global Variables
bool ballCaptured, ballFound, kick = false;
