#include "main.h"

int mod(int x, int m)
{
    int r = x % m;
    return r < 0 ? r + m : r;
}

double angleBetween(double angleCounterClockwise, double angleClockwise)
{
    return mod(angleClockwise - angleCounterClockwise, 360);
}

double smallestAngleBetween(double angle1, double angle2)
{
    double ang = angleBetween(angle1, angle2);
    return fmin(ang, 360 - ang);
}

int sign(int value)
{
    return value >= 0 ? 1 : -1;
}

int sign(double value)
{
    return value >= 0 ? 1 : -1;
}

void clearEEPROM()
{
    // initialize the LED pin as an output.
    pinMode(BUILTIN_LED, OUTPUT);

    for (int i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.write(i, 0);
    }

    // turn the LED on when we're done
    digitalWrite(BUILTIN_LED, HIGH);
    DEBUG.println("EEPROM cleared");
}
