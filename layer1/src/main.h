#ifndef LIGHT_H
#define LIGHT_H

#include "Arduino.h"
#include "EEPROM.h"

#define TeensySerial Serial2

// STM32 Pinouts
#define mux1 PB0
#define mux2 PA4

#define m1s0 PB1
#define m1s1 PB2
#define m1s2 PA6
#define m1s3 PA5
#define m2s0 PB8
#define m2s1 PB9
#define m2s2 PB6
#define m2s3 PB5
#define led PC13

// Initilalize Variables
long lastOutTime, lastInTime;
float vecX = 0;
float vecY = 0;
float closestAngle = 0;
float lineAngle = 0;
float lineTrackAngle;
float lastLineAngle = 0;
float initialLineAngle = 0;
float chordLength = 0;
float lastChordLength = 0;
bool onLine = false;
bool previouslyOnLine = onLine;
bool prevLine;
int lineDetected[30];
int outSensors = 0;
int lightVals[30];
int maxVals[30];
int minVals[30];
long readTimer = 0;
int lightCnt = 0;
int largestDiff = 0;
int clusterStart = 0;
int clusterEnd = 0;
int lightThresh[30];
int sciCenterThresh[30] = {
    94,
    71,
    278,
    291,
    350,
    406,
    350,
    221,
    285,
    331,
    443,
    448,
    443,
    439,
    442,
    445,
    475,
    445,
    450,
    446,
    439,
    444,
    445,
    439,
    438,
    428,
    438,
    435,
    434,
    392,
};
int fixedThreshFirstBot[30] = {
    455,
    437,
    349,
    312,
    347,
    386,
    363,
    348,
    391,
    360,
    436,
    435,
    429,
    408,
    431,
    437,
    489,
    433,
    444,
    443,
    409,
    434,
    443,
    432,
    433,
    441,
    332,
    422,
    439,
    437,
};

int muxChannel[16][4] = {
    // Flipped it around cos i was dumb
    {1, 1, 1, 1},
    {0, 1, 1, 1},
    {1, 0, 1, 1},
    {0, 0, 1, 1},
    {1, 1, 0, 1},
    {0, 1, 0, 1}, // 10
    {1, 0, 0, 1}, // 9
    {0, 0, 0, 1}, // 8
    {1, 1, 1, 0}, // 7
    {0, 1, 1, 0}, // 6
    {1, 0, 1, 0}, // 5
    {0, 0, 1, 0}, // 4
    {1, 1, 0, 0}, // 3
    {0, 1, 0, 0}, // 2
    {1, 0, 0, 0}, // 1
    {0, 0, 0, 0}  // 0
};

int muxChannelTwo[14][4] = {
    // Flipped it around cos i was dumb
    {1, 0, 1, 1}, // 13
    {0, 0, 1, 1}, // 12
    {1, 1, 0, 1}, // 11
    {0, 1, 0, 1}, // 10
    {1, 0, 0, 1}, // 9
    {0, 0, 0, 1}, // 8
    {1, 1, 1, 0}, // 7
    {0, 1, 1, 0}, // 6
    {1, 0, 1, 0}, // 5
    {0, 0, 1, 0}, // 4
    {1, 1, 0, 0}, // 3
    {0, 1, 0, 0}, // 2
    {1, 0, 0, 0}, // 1
    {0, 0, 0, 0}  // 0
};

float mod(float x, float y)
{
    x = fmod(x, y);
    return x < 0 ? x + y : x;
}
float angleBetween(float x, float y) { return mod((y - x), 360); }

float midAngleBetween(float x, float y)
{
    return mod(x + angleBetween(x, y) / 2.0, 360);
}

float angleDiff(float x, float y)
{
    float diff = angleBetween(x, y);
    return fmin(diff, 360 - diff);
}
#endif
