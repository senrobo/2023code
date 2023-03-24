#include "main.h"

TwoWire myWire(PB11, PB10);

Adafruit_BNO055 myImu = Adafruit_BNO055(55);

void setup()
{
  // put your setup code here, to run once:

  myWire.begin();
  TeensySerial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  // while (!myImu.begin())
  // {
  //   TeensySerial.println("IMU not found");
  // };
  TeensySerial.println("\n I2C Scanner");
}

void loop()
{
  // put your main code here, to run repeatedly:
  byte error, address;
  int nDevices;

  TeensySerial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    myWire.beginTransmission(address);
    error = myWire.endTransmission();

    if (error == 0)
    {
      TeensySerial.print("I2C device found at address 0x");
      if (address < 16)
        TeensySerial.print("0");
      TeensySerial.print(address, HEX);
      TeensySerial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      TeensySerial.print("Unknown error at address 0x");
      if (address < 16)
        TeensySerial.print("0");
      TeensySerial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    TeensySerial.println("No I2C devices found\n");
  else
    TeensySerial.println("done\n");

  delay(5000); // wait 5 seconds for next scan
}