#include "main.h"

void setup()
{
  // Serial Defintions
  COMPASS_SERIAL.begin(115200);
  IR_SERIAL.begin(115200);
  CAMERA_SERIAL.begin(115200);
  LAYER1_SERIAL.begin(115200);
  ULTRA_SERIAL.begin(115200);

  // Built in LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  // Motor Pins
  pinMode(FL_PWM, OUTPUT);
  pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(FR_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  pinMode(BR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BL_DIR, OUTPUT);

  // Ball Detection
  pinMode(BALL_DETECT, INPUT);

  // Dribbler PWM
  pinMode(DRIBBLER_PWM, OUTPUT);
}

void readLightData() {}
void processLightData() {}
void readCompassData() {}
void readUltraData() {}
void readCameraData() {}
void readIRData() {}
void findBall() {}
void chaseBall() {}
void chaseGoal() {}

void loop()
{
  findBall();
  // if ball not found within certain duration, go to center of the field
  if (ballFound)
  {
    // Move to ball
    chaseBall();
    if (ballCaptured)
    {
      chaseGoal();
    }
  }
}
