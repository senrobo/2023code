// TODO: EEPROM, AUTO CALIBRATE SENSORS, Vector Move Out Calcualtion,GET DATA FROM TEENSY, PRINT VALUES, PRINT THRESHOLDS

#include "main.h"

void setup()
{
  // put your setup code here, to run once:
  TeensySerial.begin(115200); // Teensy

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);

  pinMode(mux1, INPUT);
  pinMode(mux2, INPUT);

  // if want to use digitalWriteFast us pinMode(pinNametoDigitalPin(PINNAME), OUTPUT/INPUT))
  pinMode(m1s0, OUTPUT);
  pinMode(m1s1, OUTPUT);
  pinMode(m1s2, OUTPUT);
  pinMode(m1s3, OUTPUT);
  pinMode(m2s0, OUTPUT);
  pinMode(m2s1, OUTPUT);
  pinMode(m2s2, OUTPUT);
  pinMode(m2s3, OUTPUT);
  pinMode(sol, OUTPUT);

  digitalWrite(m1s0, LOW);
  digitalWrite(m1s1, LOW);
  digitalWrite(m1s2, LOW);
  digitalWrite(m1s3, LOW);
  digitalWrite(m2s0, LOW);
  digitalWrite(m2s1, LOW);
  digitalWrite(m2s2, LOW);
  digitalWrite(m2s3, LOW);

  // Deactivate Solenoid
  digitalWrite(sol, LOW);
}

// Light Stuff
void updateSensors()
{
  if (micros() - readTimer >= MUX_DELAY)
  {
    int idx = lightCnt;
    lightVals[idx] = analogRead(mux1);

    if (lightVals[idx] > fixedThreshFirstBot[idx])
    {
      lineDetected[outSensors] = idx;
      outSensors++;
    }

    idx = lightCnt + 16;

    lightVals[idx] = analogRead(mux2);
    if (lightVals[idx] > fixedThreshFirstBot[idx])
    {
      lineDetected[outSensors] = idx;
      outSensors++;
    }

    lightCnt++;
    lightCnt %= 16;

    // Ping via using nested lists [lightCnt] to access which mux "configuration" and [0-3] to access which pin to set [HIGH/LOW
    digitalWrite(m1s0, muxChannel[lightCnt][0]);
    digitalWrite(m1s1, muxChannel[lightCnt][1]);
    digitalWrite(m1s2, muxChannel[lightCnt][2]);
    digitalWrite(m1s3, muxChannel[lightCnt][3]);
    digitalWrite(m2s0, muxChannel[lightCnt][0]);
    digitalWrite(m2s1, muxChannel[lightCnt][1]);
    digitalWrite(m2s2, muxChannel[lightCnt][2]);
    digitalWrite(m2s3, muxChannel[lightCnt][3]);
    readTimer = micros();
  }
}

void processLightData()
{
  if (outSensors != 0)
  {
    for (int i = 0; i < outSensors; i++)
    {
      for (int j = 1; j < outSensors; j++)
      {
        int tmpDiff = angleDiff(lineDetected[i] * ldrAngle, lineDetected[j] * ldrAngle);
        if (tmpDiff > largestDiff)
        {
          clusterStart = lineDetected[i] * ldrAngle;
          clusterEnd = lineDetected[j] * ldrAngle;
          largestDiff = tmpDiff;
        }
      }
    }

    float chordLength = angleDiff(clusterStart, clusterEnd) / 180;
    float lineAngle = angleBetween(clusterStart, clusterEnd) <= 180 ? midAngleBetween(clusterStart, clusterEnd) : midAngleBetween(clusterEnd, clusterStart);
  }
  else
  {
    bool onLine = false;
  }
  outSensors = 0;
}

// Teensy Stuff
void readDataFromTeensy()
{
  // Get the SYNC Byte thingy
  // Check if have to actiavte Solenoid
}

void sendDataToTeensy()
{
  // Send if its onLine
  if (onLine)
  {
    // Send Chord Angle
    // Send Chord Length
  }
}

// Print Stuff
void printValues()
{
  Serial1.print("Light Values: ");
  for (int i = 0; i < 30; i++)
  {
    Serial1.print(lightVals[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}

void printThresholds()
{
  Serial1.print("Thresholds: ");
  for (int i = 0; i < 30; i++)
  {
    Serial1.print(fixedThreshFirstBot[i]);
    Serial1.print(" ");
  }
  Serial1.println();
}
void loop()
{
  // put your main code here, to run repeatedly:
  updateSensors();
  // processLightData();
  // printValues();
  printThresholds();
  readDataFromTeensy();
  sendDataToTeensy();
}