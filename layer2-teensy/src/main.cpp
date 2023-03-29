#include "main.h"

void setup()
{
  // Serial Defintions
  DEBUG.begin(115200);
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

// CANNOT GO BELOW 40

void loop()
{
  // DEBUG.println(IR_SERIAL.available());
  digitalWrite(FL_DIR, LOW);
  analogWrite(FL_PWM, 40);
  digitalWrite(FR_DIR, HIGH);
  analogWrite(FR_PWM, 40);
  digitalWrite(BR_DIR, HIGH);
  analogWrite(BR_PWM, 40);
  digitalWrite(BL_DIR, LOW);
  analogWrite(BL_PWM, 40);
}
