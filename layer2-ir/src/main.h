#include <Arduino.h>

#define ARRAYSHIFTDOWN(a, lower, upper)              \
    {                                                \
        if (upper == (sizeof(a) / sizeof(a[0])) - 1) \
        {                                            \
            for (int q = upper - 1; q >= lower; q--) \
            {                                        \
                *(a + q + 1) = *(a + q);             \
            }                                        \
        }                                            \
        else                                         \
        {                                            \
            for (int q = upper; q >= lower; q--)     \
            {                                        \
                *(a + q + 1) = *(a + q);             \
            }                                        \
        }                                            \
    }

bool ball = false;
int angle = 0;
int strength = 0;
int tsopCounter = 0;
int values[24] = {0};
int indexes[24] = {0};
int tempVal[24] = {0};
int sortedValues[24] = {0};
double scaledSin[24] = {0};
double scaledCos[24] = {0};

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

int mod(int x, int m);

// Pin Definitions
#define LED PB15;

#define IROne PA5;
#define IRTwo PA4;
#define IRThree PA1;
#define IRFour PA0;
#define IRFive PC15;
#define IRSix PC14;
#define IRSeven PB9;
#define IREight PB8;
#define IRNine PB7;
#define IRTen PB6;
#define IREleven PB5;
#define IRTwelve PB4;
#define IRThirteen PB3;
#define IRFourteen PA15;
#define IRFifteen PB14;
#define IRSixteen PB13;
#define IRSeventeen PB12;
#define IREighteen PB11;
#define IRNineteen PB10;
#define IRTwenty PB2;
#define IRTwentyOne PB1;
#define IRTwentyTwo PB0;
#define IRTwentyThree PA7;
#define IRTwentyFour PA6;


