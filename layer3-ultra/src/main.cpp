#include "main.h"
void setup()
{
  // put your setup code here, to run once:
  TeensySerial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}

void loop()
{
  // put your main code here, to run repeatedly:
  sensorDistance[0] = sonar[0].ping_cm();
  TeensySerial.write(sensorDistance[0]);
}