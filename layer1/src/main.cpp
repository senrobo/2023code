// TODO: EEPROM, AUTO CALIBRATE SENSORS, Vector Move Out Calcualtion,GET DATA FROM TEENSY, PRINT VALUES, PRINT THRESHOLDS

#include "main.h"

// Light Stuff

void updateSensors()
{
  if (micros() - readTimer >= 100)
  {
    int idx = lightCnt;
    lightVals[idx] = analogRead(mux1);

    lightVals[idx] > fixedThreshFirstBot[idx] ? (lineDetected[outSensors] = idx, outSensors++) : 0;

    lightCnt++;
    lightCnt %= 16;

    idx = lightCnt + 18;
    lightVals[idx] = analogRead(mux2);

    lightVals[idx] > fixedThreshFirstBot[idx] ? (lineDetected[outSensors] = idx, outSensors++) : 0;

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
  float vecX = 0;
  float vecY = 0;
  clusterStart = 0;
  clusterEnd = 0;
  float largestDiff = 0;
  bool previouslyOnLine = onLine;
  float closestAngle = 0;
  onLine = outSensors > 0;
  if (onLine)
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

    // update data
    onLine = onLine;
    lineAngle = lineAngle;
    chordLength = chordLength;

    // save inital line angle
    if (onLine && !previouslyOnLine)
    {
      initialLineAngle = lineAngle;
    }
    lineTrackAngle = closestAngle;
  }
}

void calibrate()
{
  for (int i = 0; i < 30; i++)
  {
    maxVals[i] = 0;
    minVals[i] = 1200;
  }
  while (true)
  {
    updateSensors();
    for (int i = 0; i < 30; i++)
    {
      if (lightVals[i] > maxVals[i])
      {
        maxVals[i] = lightVals[i];
      }
      if (lightVals[i] < minVals[i])
      {
        minVals[i] = lightVals[i];
      }

      lightThresh[i] = (maxVals[i] + minVals[i]) / 2;
    }
    for (int i = 0; i < 30; i++)
    {
      TeensySerial.print(i);
      TeensySerial.print(" | ");
      TeensySerial.print(lightThresh[i]);
      TeensySerial.print(" | ");
    }
    TeensySerial.println();
  }
}

void setup()
{
  // put your setup code here, to run once:
  TeensySerial.begin(115200); // Teensy

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(mux1, INPUT);
  pinMode(PB7, INPUT);

  // if want to use digitalWriteFast us pinMode(pinNametoDigitalPin(PINNAME), OUTPUT/INPUT))
  pinMode(m1s0, OUTPUT);
  pinMode(m1s1, OUTPUT);
  pinMode(m1s2, OUTPUT);
  pinMode(m1s3, OUTPUT);

  pinMode(m2s0, OUTPUT);
  pinMode(m2s1, OUTPUT);
  pinMode(m2s2, OUTPUT);
  pinMode(m2s3, OUTPUT);

  digitalWrite(m1s0, LOW);
  digitalWrite(m1s1, LOW);
  digitalWrite(m1s2, LOW);
  digitalWrite(m1s3, LOW);
  digitalWrite(m2s0, LOW);
  digitalWrite(m2s1, LOW);
  digitalWrite(m2s2, LOW);
  digitalWrite(m2s3, LOW);
}

void loop()
{
  updateSensors();
  processLightData();
  if (onLine)
  {
    double lastTimeOut = millis();
    if (lastLineAngle >= 0 && abs(lastLineAngle - lineAngle) >= 90)
    {
      lineAngle = lastLineAngle;
      chordLength = 2 - chordLength;
    }
  }
  if (onLine)
  {
    int moveAngle = fmod(lineAngle + 180, 360);
    lastLineAngle = lineAngle;
    lastChordLength = chordLength;
  }
// put your main code here, to run repeatedly:
// updateSensors();

// for (int i = 0; i < 16; i++)
// {
//   TeensySerial.print(i);
//   TeensySerial.print(" | ");
//   TeensySerial.print(lightVals[i]);
//   TeensySerial.print(" | ");
// }
// TeensySerial.println();
// Print TH Values
// for (int i = 0; i < 30; i++)
// {
//   TeensySerial.print(i);
//   TeensySerial.print(" | ");
//   TeensySerial.print(minVals[i]);
//   TeensySerial.print(" | ");
//   TeensySerial.print(maxVals[i]);
//   TeensySerial.print(" | ");
//   TeensySerial.print(lightThresh[i]);
//   TeensySerial.print(" | ");
// }
// TeensySerial.println();
// calibrate();
}