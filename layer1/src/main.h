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
float lineAngle = 0;
float lineTrackAngle;
float lastLineAngle = 0;
float initialLineAngle = 0;
float chordLength = 0;
float lastChordLength = 0;
bool onLine = false;
int lineDetected[30];
int outSensors = 0;
int lightVals[30];
int maxVals[30];
int minVals[30];
long readTimer = 0;
int lightCnt = 0;
int ldrAngle = 12;
int largestDiff = 0;
int clusterStart = 0;
int clusterEnd = 0;
int lightThresh[30];
int sciCenterThresh[30] = {
    455,
    437,
    424,
    434,
    430,
    383,
    432,
    444,
    447,
    448,
    445,
    441,
    441,
    445,
    432,
    395,
    452,
    446,
    450,
    446,
    440,
    446,
    446,
    442,
    445,
    446,
    444,
    438,
    446,
    443,
};
int fixedThreshFirstBot[30] = {
    455,
    437,
    340,
    429,
    393,
    358,
    430,
    441,
    444,
    447,
    443,
    429,
    398,
    440,
    440,
    434,
    432,
    376,
    446,
    440,
    390,
    427,
    445,
    434,
    439,
    439,
    438,
    407,
    441,
    435,
};

int muxChannel[14][4] = {
    // Flipped it around cos i was dumb
    // Ignore front 2 sensors cos values
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

float mod(float x, float y);

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
