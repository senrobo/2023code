#include "main.h"

void setup()
{
  // put your setup code here, to run once:
  TeensySerial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(ultraSensorOneTrigger, OUTPUT);
  pinMode(ultraSensorOneEcho, INPUT);
  digitalWrite(LED, HIGH);
}

// Workaround because delayMicroseconds is not working
void delayMicros(uint32_t us)
{
  uint32_t start = micros();
  while (micros() - start < us)
  {
    __asm__ __volatile__("nop");
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Write a program to get the data from the ultrasonic sensors in cm

  // Get ultraSensorOne data
  digitalWrite(ultraSensorOneTrigger, LOW);
  delayMicros(2);
  digitalWrite(ultraSensorOneTrigger, HIGH);
  delayMicros(10);
  digitalWrite(ultraSensorOneTrigger, LOW);
  long durationUS1 = pulseIn(ultraSensorOneEcho, HIGH);
  long distanceUS1 = (durationUS1 * 0.034) / 2;

  // Get ultraSensorTwo data
  digitalWrite(ultraSensorTwoTrigger, LOW);
  delayMicros(2);
  digitalWrite(ultraSensorTwoTrigger, HIGH);
  delayMicros(10);
  digitalWrite(ultraSensorTwoTrigger, LOW);
  long durationUS2 = pulseIn(ultraSensorTwoEcho, HIGH);
  long distanceUS2 = (durationUS2 * 0.034) / 2;

  // Get ultraSensorThree data
  digitalWrite(ultraSensorThreeTrigger, LOW);
  delayMicros(2);
  digitalWrite(ultraSensorThreeTrigger, HIGH);
  delayMicros(10);
  digitalWrite(ultraSensorThreeTrigger, LOW);
  long durationUS3 = pulseIn(ultraSensorThreeEcho, HIGH);
  long distanceUS3 = (durationUS3 * 0.034) / 2;

  //  Get ultraSensorFour data
  digitalWrite(ultraSensorFourTrigger, LOW);
  delayMicros(2);
  digitalWrite(ultraSensorFourTrigger, HIGH);
  delayMicros(10);
  digitalWrite(ultraSensorFourTrigger, LOW);
  long durationUS4 = pulseIn(ultraSensorFourEcho, HIGH);
  long distanceUS4 = (durationUS4 * 0.034) / 2;

  TeensySerial.println(distanceUS1);
  TeensySerial.println(distanceUS2);
  TeensySerial.println(distanceUS3);
  TeensySerial.println(distanceUS4);
  delay(100);
}
