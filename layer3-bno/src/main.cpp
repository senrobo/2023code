#include "main.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  TeensySerial.write(3);
}