#include "main.h"

struct Movement
{
  int16_t angle = 0;           // -179(.)99º to +180(.)00º
  int16_t speed = 0;           // ±50 to ±1024
  int16_t angularVelocity = 0; // ±200 to ±4096
} movement;

void drive()
{
  // Convert Polar to Cartesian
  const auto x = sinf((float)movement.angle / 100 * DEG_TO_RAD);
  const auto y = cosf((float)movement.angle / 100 * DEG_TO_RAD);
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

  // Constrain motor speed
  auto constrainSpeed = [](int16_t speed)
  {
    if (speed == 0)
      return speed;
    return (int16_t)constrain(abs(speed), DRIVE_STALL_SPEED,
                              DRIVE_MAX_SPEED);
  };
  // Set motor directions and speeds
  digitalWriteFast(FL_DIR, FLSpeed > 0 ? LOW : HIGH);
  digitalWriteFast(FR_DIR, FRSpeed > 0 ? HIGH : LOW);
  digitalWriteFast(BR_DIR, BRSpeed > 0 ? HIGH : LOW);
  digitalWriteFast(BL_DIR, BLSpeed > 0 ? LOW : HIGH);
  analogWrite(FL_PWM, constrainSpeed(FLSpeed));
  analogWrite(FR_PWM, constrainSpeed(FRSpeed));
  analogWrite(BR_PWM, constrainSpeed(BRSpeed));
  analogWrite(BL_PWM, constrainSpeed(BLSpeed));
}

void setup()
{
  // Serial Defintions
  DEBUG.begin(115200);
  COMPASS_SERIAL.begin(115200);
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

// CANNOT GO BELOW 50

void loop()
{
  // // DO NOT REMOVE THIS IS FOR YOUR SANITY CHECK
  // if (IR_SERIAL.available() > 0)
  // {
  //   DEBUG.print(char(IR_SERIAL.read()));
  // }

  // // Loop through the IR_SERIAL buffer to find the sync byte
  // while (IR_SERIAL.available() >= IR_SERIAL_BUFFER_SIZE)
  // {
  //   if (IR_SERIAL.read() == SYNC_BYTE)
  //   {
  //     // First we subtract the buffer by -1 to remove the SYNC_BYTE before reading the rest of the buffer
  //     byte buf[IR_SERIAL_BUFFER_SIZE - 1];
  //     IR_SERIAL.readBytes(buf, IR_SERIAL_BUFFER_SIZE - 1);

  //     // Copy the last 8 buffer bytes into the ballAngle and ballStrength variables
  //     memcpy(&ballAngle, buf, sizeof(int));
  //     memcpy(&ballStrength, buf + sizeof(int), sizeof(int));

  //     // // Print the buffer to serial with printf
  //     for (int i = 1; i < 9; ++i)
  //       Serial2.printf("%02x", buf[i]);
  //     Serial.print("\n ");
  //   }
  // }
}