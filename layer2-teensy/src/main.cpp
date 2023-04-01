#include "main.h"

void setup()
{
  // Serial Defintions
  DEBUG.begin(115200);
  IR_SERIAL.begin(115200);
  CAMERA_SERIAL.begin(115200);
  LAYER1_SERIAL.begin(115200);

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

void loop()
{
  if (LAYER1_SERIAL.available() > 0)
  {
    DEBUG.print(char(LAYER1_SERIAL.read()));
  }
  // Loop through the IR_SERIAL buffer to find the sync byte
  // while (IR_SERIAL.available() >= 9U)
  // {
  //   if (IR_SERIAL.read() == SYNC_BYTE)
  //   {
  //     // First we subtract the buffer by -1 to remove the SYNC_BYTE before reading the rest of the buffer
  //     byte buf[8U];
  //     IR_SERIAL.readBytes(buf, 8U);

  //     // Copy the last 8 buffer bytes into the ballAngle and ballStrength variables
  //     memcpy(&ballAngle, buf, 4U);
  //     memcpy(&ballStrength, buf + 4U, 4U);

  //     // // Print the buffer to serial with printf
  //     // for (int i = 0; i < 8; ++i)
  //     //   DEBUG.printf("%02x", buf[i]);
  //     // DEBUG.print("\n ");
  //   }
  // }
  // DEBUG.print("Ball Angle: ");
  // DEBUG.println(ballAngle);
}