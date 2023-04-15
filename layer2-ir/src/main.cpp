#include "main.h"

double radiansToDegrees(double radians)
{
  return radians * 180 / PI;
}

double degreesToRadians(double degrees)
{
  return degrees * PI / 180;
}

int mod(int x, int m)
{
  return (x % m + m) % m;
}

void setup()
{
  Serial2.begin(115200);
  pinMode(PB15, OUTPUT);
  digitalWrite(PB15, HIGH);

  pinMode(PA5, INPUT);
  pinMode(PA4, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA0, INPUT);
  pinMode(PC15, INPUT);
  pinMode(PC14, INPUT);
  pinMode(PB9, INPUT);
  pinMode(PB8, INPUT);
  pinMode(PB7, INPUT);
  pinMode(PB6, INPUT);
  pinMode(PB5, INPUT);
  pinMode(PB4, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PA15, INPUT);
  pinMode(PB14, INPUT);
  pinMode(PB13, INPUT);
  pinMode(PB12, INPUT);
  pinMode(PB11, INPUT);
  pinMode(PB10, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PB0, INPUT);
  pinMode(PA7, INPUT);
  pinMode(PA6, INPUT);

  double temp_angle;

  for (int i = 0; i < 24; i++)
  {
    temp_angle = degreesToRadians(i * 15);
    scaledCos[i] = cos(temp_angle);
    scaledSin[i] = sin(temp_angle);
  }

  unsigned long lastUp = micros();
}

void irUpdateOnce()
{
  tempVal[0] += digitalRead(PA5) ^ 1;
  tempVal[1] += digitalRead(PA4) ^ 1;
  tempVal[2] += digitalRead(PA1) ^ 1;
  tempVal[3] += digitalRead(PA0) ^ 1;
  tempVal[4] += digitalRead(PC15) ^ 1;
  tempVal[5] += digitalRead(PC14) ^ 1;
  tempVal[6] += digitalRead(PB9) ^ 1;
  tempVal[7] += digitalRead(PB8) ^ 1;
  tempVal[8] += digitalRead(PB7) ^ 1;
  tempVal[9] += digitalRead(PB6) ^ 1;
  tempVal[10] += digitalRead(PB5) ^ 1;
  tempVal[11] += digitalRead(PB4) ^ 1;
  tempVal[12] += digitalRead(PB3) ^ 1;
  tempVal[13] += digitalRead(PA15) ^ 1;
  tempVal[14] += digitalRead(PB14) ^ 1;
  tempVal[15] += digitalRead(PB13) ^ 1;
  tempVal[16] += digitalRead(PB12) ^ 1;
  tempVal[17] += digitalRead(PB11) ^ 1;
  tempVal[18] += digitalRead(PB10) ^ 1;
  tempVal[19] += digitalRead(PB2) ^ 1;
  tempVal[20] += digitalRead(PB1) ^ 1;
  tempVal[21] += digitalRead(PB0) ^ 1;
  tempVal[22] += digitalRead(PA7) ^ 1;
  tempVal[23] += digitalRead(PA6) ^ 1;

  tsopCounter++;
}

void finishRead()
{
  for (int i = 0; i < 24; i++)
  {
    values[i] = 100 * (double)tempVal[i] / (double)tsopCounter;
    tempVal[i] = 0;
    sortedValues[i] = 0;
    indexes[i] = 0;
  }
  tsopCounter = 0;
}

void sortValues()
{
  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 24; j++)
    {
      if (values[i] > sortedValues[j])
      {
        if (j <= i)
        {
          ARRAYSHIFTDOWN(sortedValues, j, i);
          ARRAYSHIFTDOWN(indexes, j, i);
        }
        sortedValues[j] = values[i];
        indexes[j] = i;
        break;
      }
    }
  }
}

void calculateAngleStrength(int n)
{
  int x = 0;
  int y = 0;

  for (int i = 0; i < n; i++)
  {
    x += scaledCos[indexes[i]] * sortedValues[i];
    y += scaledSin[indexes[i]] * sortedValues[i];
  }

  if (x == 0 && y == 0)
  {
    angle = 400;
  }
  else
  {
    angle = mod(radiansToDegrees(atan2(y, x)), 360);
  }
  strength = sortedValues[0];
}

void loop()
{
  irUpdateOnce();
  if (micros() - lastUp > 3333)
  {
    finishRead();
    sortValues();
    calculateAngleStrength(6);
  }
  lastUp = micros();
  // Serial2.print("Angle: ");
  // Serial2.print(angle);
  // Serial2.print(" Strength: ");
  // Serial2.println(strength);

  // Create a buffer to send the data over serial and the size of the buffer is the total combined size of the angle stregnth and sync byte in BYTES
  byte buf[9U];

  // Set the first byte of the buffer to the sync byte
  buf[0] = SYNC_BYTE;

  // Copy the angle and strength into the buffer
  memcpy(buf + 1U, &angle, sizeof(angle));
  memcpy(buf + 1U + sizeof(angle), &strength, sizeof(strength));

  // Print the buffer to serial with printf
  Serial2.write(buf, sizeof(buf));
}