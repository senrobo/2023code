#include "Ardiuno.h"

int tsopCounter = 0

    int irVal[24] = [0];
int tempVal[24] = [0];
int irTolerance = 5;
int irBottomLine = 10;
int IR_LOOP = 10;
int irAngle = 15;
bool ballNotFound = true;
void setup()
{
  Serial1.begin(115200); // Debug
  Serial2.begin(115200); // Teensy
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

  lastUp = micros();
}

float mod(float x, float y)
{
  x = fmod(x, y);
  return x < 0 ? x + y : x;
}

irVal[0] = digitalRead(PA5);
irVal[1] = digitalRead(PA4);
irVal[2] = digitalRead(PA1);
irVal[3] = digitalRead(PA0);
irVal[4] = digitalRead(PC15);
irVal[5] = digitalRead(PC14);
irVal[6] = digitalRead(PB9);
irVal[7] = digitalRead(PB8);
irVal[8] = digitalRead(PB7);
irVal[9] = digitalRead(PB6);
irVal[10] = digitalRead(PB5);
irVal[11] = digitalRead(PB4);
irVal[12] = digitalRead(PB3);
irVal[13] = digitalRead(PA15);
irVal[14] = digitalRead(PB14);
irVal[15] = digitalRead(PB13);
irVal[16] = digitalRead(PB12);
irVal[17] = digitalRead(PB11);
irVal[18] = digitalRead(PB10);
irVal[19] = digitalRead(PB2);
irVal[20] = digitalRead(PB1);
irVal[21] = digitalRead(PB0);
irVal[22] = digitalRead(PA7);
irVal[23] = digitalRead(PA6);

void getIRVals()
{
  for (int i = 0; i < IR_LOOP; i++)
  {
    for (int j = 0; j < 24; j++)
    {
      tempVal[j] += digitalRead(j);
    }
  }
}

void processIR()
{
  // Bascially Loop through the IR List value and find the highest value and once the highest value is check for the next few values to see if they are about the same value based on a creatin + - range of 10% of the highest value
  for (int i = 0; i < 24; i++ && ballNotFound)
  {
    if (tempVal[i] > irBottomLine)
    {
      // check the next 2 sensors in the array to see if they fall within the +- range of the highest value of irTolerance and to prevent false positive or weird sensors

      if (tempVal[i + 1] > tempVal[i] - irTolerance && tempVal[i + 1] < tempVal[i] + irTolerance)
      {
        if (tempVal[i + 2] > tempVal[i] - irTolerance && tempVal[i + 2] < tempVal[i] + irTolerance])
        {
          ballNotFound = false;
          // calculate the angle of the ball
          float angle = tempVal[i] * irAngle;
        }
        else
        {

          tsopCounter++
        }
      }
    }
    else
    {
      tsopCounter++;
    }
  }

  void loop()
  {
    // read IR
    updateIROnce();
  }