#include "main.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int8_t temp = bno.getTemp();

#define BNO055_SAMPLERATE_DELAY_MS (100)

struct Movement
{
  int16_t angle = 0; // -179(.)99º to +180(.)00º
  int16_t speed = 40;
  int16_t angularVelocity = 0; // ~±140 to ~±1024
} movement;

struct ballData
{
  int angle = 0;
  int strength = 0;
} balldata;

// IMU Stuff

void printAllIMUData()
{
  sensors_event_t eul, gyr, lac, mag, acc, gra;
  bno.getEvent(&eul, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&acc, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&lac, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&gra, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Get calibration states
  uint8_t systemCalib, gyroCalib, accCalib, magCalib;
  bno.getCalibration(&systemCalib, &gyroCalib, &accCalib, &magCalib);

  // Print everything to serial
  const auto printVector = [](const char *name, const sensors_vec_t &vector)
  {
    DEBUG.printf(
        "%s: x = %4d.%02d y = %4d.%02d z = %4d.%02d\n", name,
        (int16_t)vector.x, abs((int32_t)(vector.x * 100) % 100),
        (int16_t)vector.y, abs((int32_t)(vector.y * 100) % 100),
        (int16_t)vector.z, abs((int32_t)(vector.z * 100) % 100));
  };
  printVector("Euler Angle (º)            ", eul.orientation);
  // printVector("Angular Velocity (rad s⁻¹) ", gyr.gyro);
  // printVector("Acceleration (m s⁻²)       ", acc.acceleration);
  // printVector("Linear Acceleration (m s⁻²)", lac.acceleration);
  // printVector("Gravity (m s⁻²)            ", gra.acceleration);
  // printVector("Magnetic Field (μT)        ", mag.magnetic);

  // DEBUG.printf(
  //     "Calibration: System = %d Gyroscope = %d Accelerometer = %d "
  //     "Magnetometer = %d\n\n",
  //     systemCalib, gyroCalib, accCalib, magCalib);
}

void calculateRobotAngle()
{
  sensors_event_t eulerAngles;
  bno.getEvent(&eulerAngles, Adafruit_BNO055::VECTOR_EULER);
  int robotBearing = round(eulerAngles.orientation.x);
  int epoopoo = robotBearing <= 180 ? robotBearing : robotBearing - 360;
  int error = -epoopoo;
  int correctionKP = error * IMUKP;
  int correctionKD = (error - lastError) * IMUKD;
  correction = correctionKP + correctionKD;
  lastError = error;
}

void drive()
{
  // Convert polar to cartesian
  const auto x = sinf(movement.angle * M_PI / 180);
  const auto y = cosf(movement.angle * M_PI / 180);

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

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  DEBUG.print("Accelerometer: ");
  DEBUG.print(calibData.accel_offset_x);
  DEBUG.print(" ");
  DEBUG.print(calibData.accel_offset_y);
  DEBUG.print(" ");
  DEBUG.print(calibData.accel_offset_z);
  DEBUG.print(" ");

  DEBUG.print("\nGyro: ");
  DEBUG.print(calibData.gyro_offset_x);
  DEBUG.print(" ");
  DEBUG.print(calibData.gyro_offset_y);
  DEBUG.print(" ");
  DEBUG.print(calibData.gyro_offset_z);
  DEBUG.print(" ");

  DEBUG.print("\nMag: ");
  DEBUG.print(calibData.mag_offset_x);
  DEBUG.print(" ");
  DEBUG.print(calibData.mag_offset_y);
  DEBUG.print(" ");
  DEBUG.print(calibData.mag_offset_z);
  DEBUG.print(" ");

  DEBUG.print("\nAccel Radius: ");
  DEBUG.print(calibData.accel_radius);

  DEBUG.print("\nMag Radius: ");
  DEBUG.print(calibData.mag_radius);
}

void getIRData()
{
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
}

void getLightData()
{
  while (LAYER1_SERIAL.available() > 0)
  {
    DEBUG.print(char(LAYER1_SERIAL.read()));
  }
}

void processDrive()
{
  if (0 <= balldata.angle && balldata.angle <= 180)
  {
    offsetAngle = min(balldata.angle * BALL_MOVEMENT_A, 90);
  }
  else
  {
    offsetAngle = max((balldata.angle - 360) * BALL_MOVEMENT_A, -90);
  }
  movement.angle = balldata.angle + (offsetAngle * BALL_MOVEMENT_B);
}

void setup()
{
  // Serial Defintions
  DEBUG.begin(115200);
  IR_SERIAL.begin(115200);
  CAMERA_SERIAL.begin(115200);
  LAYER1_SERIAL.begin(115200);

  Wire.begin();
  bno.begin();
  delay(1000);
  bno.setMode(OPERATION_MODE_IMUPLUS);

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  // Get BNO calibration data from EEPROM
  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    DEBUG.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    DEBUG.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    DEBUG.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    DEBUG.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  bno.setExtCrystalUse(true);

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
  // read all sensors
  // getIRData();
  // getLightData();
  // check inbounds
  // if light.In false
  // moveout
  // check if ball exist ? move to ball : center to field via camera
  // if ball.stregnth != 400
  // movement.angle = ball.angle
  // if ball caught analogread
  // keep straight
  calculateRobotAngle();
  if (correction < 0)
  {
    analogWrite(FL_PWM, 40 - correction);
    analogWrite(FR_PWM, 40 - correction);
    analogWrite(BL_PWM, 40 - correction);
    analogWrite(BR_PWM, 40 - correction);
    digitalWrite(FL_DIR, HIGH);
    digitalWrite(FR_DIR, HIGH);
    digitalWrite(BL_DIR, HIGH);
    digitalWrite(BR_DIR, HIGH);
  }
  else if (correction > 0)
  {
    analogWrite(FL_PWM, 40 + correction);
    analogWrite(FR_PWM, 40 + correction);
    analogWrite(BL_PWM, 40 + correction);
    analogWrite(BR_PWM, 40 + correction);
    digitalWrite(FL_DIR, LOW);
    digitalWrite(FR_DIR, LOW);
    digitalWrite(BL_DIR, LOW);
    digitalWrite(BR_DIR, LOW);
  }
  // chaseGoal(); // while ball still caputred chase goal else search ball

  // processDrive();

  // drive();
  // DEBUG.print("Ball Angle: ");
  // DEBUG.print(movement.angle);
  // DEBUG.print(" | ");
  // DEBUG.print("Ball Strength: ");
  // DEBUG.println(balldata.strength);
  // getLightData();
  //  printAllIMUData();
  // readRobotAngle();
  // delay(BNO055_SAMPLERATE_DELAY_MS);
  // DEBUG.println(correction);
}