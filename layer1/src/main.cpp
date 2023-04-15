// TODO: EEPROM, AUTO CALIBRATE SENSORS, Vector Move Out Calcualtion,GET DATA FROM TEENSY, PRINT VALUES, PRINT THRESHOLDS

#include "main.h"

// Light Stuff

void updateSensors()
{
  if (micros() - readTimer >= 100)
  {
    int idx = lightCnt;
    lightVals[idx] = analogRead(mux1);

    idx = lightCnt + 16;
    lightVals[idx] = analogRead(mux2);

    lightCnt++;
    lightCnt %= 16;

    digitalWrite(m1s0, muxChannel[lightCnt][0]);
    digitalWrite(m1s1, muxChannel[lightCnt][1]);
    digitalWrite(m1s2, muxChannel[lightCnt][2]);
    digitalWrite(m1s3, muxChannel[lightCnt][3]);
    digitalWrite(m2s0, muxChannelTwo[lightCnt][0]);
    digitalWrite(m2s1, muxChannelTwo[lightCnt][1]);
    digitalWrite(m2s2, muxChannelTwo[lightCnt][2]);
    digitalWrite(m2s3, muxChannelTwo[lightCnt][3]);

    readTimer = micros();
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
      // TeensySerial.print(i);
      // TeensySerial.print(" | ");
      TeensySerial.print(lightThresh[i]);
      TeensySerial.print(" , ");
      // TeensySerial.print(maxVals[i]);
      // TeensySerial.print(" , ");
      // TeensySerial.print(minVals[i]);
      // TeensySerial.print(" | ");
    }
    TeensySerial.println();
  }
}

void printAllValues()
{
  for (int i = 0; i < 30; i++)
  {
    TeensySerial.print(i);
    TeensySerial.print(" | ");
    TeensySerial.print(lightVals[i]);
    TeensySerial.print(" , ");
  }
  TeensySerial.println();
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
  for (int i = 0; i < 30; i++)
  {
    if (lightVals[i] >= fixedThreshFirstBot[i])
    {
      outSensors++; // Sensors that decteded the lines
      lineDetected[outSensors] = i;
    }
  }
  for (int i = 0; i < outSensors; i++)
  {
    TeensySerial.print(lineDetected[i]);
    TeensySerial.print(" , ");
  }

  onLine = outSensors > 0;

  if (onLine)
  {
    for (int i = 1; i < outSensors; i++)
    {
      for (int j = 2; j < outSensors; j++)
      {
        u_int16_t tmpDiff = angleDiff(lineDetected[i] * (12), lineDetected[j] * (12));
        if (tmpDiff > largestDiff)
        {
          clusterStart = lineDetected[i] * 12;
          clusterEnd = lineDetected[j] * 12;
          largestDiff = tmpDiff;
        }
      }
    }
    if (lineAngle = angleBetween(clusterStart, clusterEnd) <= 180)
    {
      lineAngle = midAngleBetween(clusterStart, clusterEnd);
    }
    else
    {
      lineAngle = midAngleBetween(clusterEnd, clusterStart);
    }
  }

  moveAngle = lineAngle + 180;
  (moveAngle >= 360) ? moveAngle -= 360 : moveAngle;

  // // Create a buffer to send the data over serial and the size of the buffer is the total combined size of the angle stregnth and sync byte in BYTES
  byte buf[7U];

  // Set the first byte of the buffer to the sync byte
  buf[0] = SYNC_BYTE;

  // Copy the angle and strength into the buffer
  memcpy(buf + 1U, &onLine, sizeof(onLine));
  memcpy(buf + 1U + sizeof(onLine), &moveAngle, sizeof(moveAngle));

  // Print the buffer to serial with printf
  Serial2.write(buf, sizeof(buf));

  // TeensySerial.print("Cluster Start: ");
  // TeensySerial.print(clusterStart);
  // TeensySerial.print(" | ");
  // TeensySerial.print("Cluster End: ");
  // TeensySerial.print(clusterEnd);
  // TeensySerial.print(" | ");
  // TeensySerial.print("Move Angle: ");
  // TeensySerial.print(moveAngle);
  // TeensySerial.print(" | ");
  // TeensySerial.print("Largest Diff: ");
  // TeensySerial.print(largestDiff);
  // TeensySerial.println();
  outSensors = 0;
  largestDiff = 0;
}