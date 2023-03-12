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
