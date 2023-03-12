// TODO : Add like some timeout sensor like if pinng not reviced for x duration den have 2 arrays using unions or sth like 1 is true or false if sensor is pinged
#include "main.h"
void setup()
{
  // put your setup code here, to run once:
  DebugSerial.begin(15200);
  TeenySerial.begin(15200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(ultraSensorOneTrigger, OUTPUT);
  pinMode(ultraSensorOneEcho, INPUT);

  pinMode(ultraSensorTwoTrigger, OUTPUT);
  pinMode(ultraSensorTwoEcho, INPUT);

  pinMode(ultraSensorThreeTrigger, OUTPUT);
  pinMode(ultraSensorThreeEcho, INPUT);

  pinMode(ultraSensorFourTrigger, OUTPUT);
  pinMode(ultraSensorFourEcho, INPUT);
}
void loop()
{
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 4; i++)
  {
    delay(50);
    sensorDistance[i] = sonar[i].ping_cm();
    DebugSerial.print(sensorDistance[i]);
    DebugSerial.print(" ");
  }
  // Read data from teensy
  // Send data to teensy via serial
}