#include "main.h"

struct Movement
{
  int16_t angle = 0;           // -179(.)99º to +180(.)00º
  int16_t speed = 0;           // ±50 to ±100
  int16_t angularVelocity = 0; // ~±140 to ~±1024
} movement;

struct ballData
{
  int angle = 0;
  int strength = 0;
} balldata;

void drive()
{
  // Convert polar to cartesian
  const auto x = sinf((int)movement.angle * DEG_TO_RAD);
  const auto y = cosf((int)movement.angle * DEG_TO_RAD);

  // Compute the speeds of the individual motors
  const auto transformSpeed = [](float speed, float angularComponent)
  {
    return (int16_t)roundf(speed * movement.speed + angularComponent);
  };
  // Find angular component
  const auto angular = ANGULAR_VELOCITY_MULTIPLIER * movement.angularVelocity;
  // Compute speeds
  const int16_t FLSpeed = transformSpeed(x * COS45 + y * SIN45, angular);
  const int16_t FRSpeed = transformSpeed(x * -COS45 + y * SIN45, -angular);
  const int16_t BLSpeed = transformSpeed(x * -COS45 + y * SIN45, +angular);
  const int16_t BRSpeed = transformSpeed(x * COS45 + y * SIN45, -angular);

  // Set the motor directions and speeds
  digitalWriteFast(FL_DIR, FLSpeed > 0 ? LOW : HIGH);
  digitalWriteFast(FR_DIR, FRSpeed > 0 ? HIGH : LOW);
  digitalWriteFast(BL_DIR, BLSpeed > 0 ? LOW : HIGH);
  digitalWriteFast(BR_DIR, BRSpeed > 0 ? HIGH : LOW);
  analogWrite(FL_PWM,
              constrain(abs(FLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
  analogWrite(FR_PWM,
              constrain(abs(FRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
  analogWrite(BL_PWM,
              constrain(abs(BLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
  analogWrite(BR_PWM,
              constrain(abs(BRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
}

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
}

void loop()
{
  movement.speed = 50;

  // if (IR_SERIAL.available() > 0)
  // {
  //   DEBUG.print(char(IR_SERIAL.read()));
  // }
  // Loop through the IR_SERIAL buffer to find the sync byte
  while (IR_SERIAL.available() >= 9U)
  {
    if (IR_SERIAL.read() == SYNC_BYTE)
    {
      // Subtract the buffer by -1 to remove the SYNC_BYTE before reading the rest of the buffer
      byte buf[8U];
      IR_SERIAL.readBytes(buf, 8U);

      // Copy the last 8 buffer bytes into the ballAngle and ballStrength variables
      memcpy(&balldata.angle, buf, 4U);
      memcpy(&balldata.strength, buf + 4U, 4U);

      // // Print the buffer to serial with printf
      // for (int i = 0; i < 8; ++i)
      //   DEBUG.printf("%02x", buf[i]);
      // DEBUG.print("\n ");
    }
  }
  movement.angle = balldata.angle;
  drive();
  DEBUG.print("Ball Angle: ");
  DEBUG.print(movement.angle);
  DEBUG.print(" | ");
  DEBUG.print("Ball Strength: ");
  DEBUG.println(balldata.strength);
}