#ifndef LIGHT_H
#define LIGHT_H

#include "Arduino.h"
#include "EEPROM.h"

#define TeensySerial Serial2

#define SYNC_BYTE 0b11010110

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
int moveAngle = 0;
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
    453,
    445,
    433,
    411,
    431,
    439,
    439,
    434,
    444,
    448,
    448,
    450,
    445,
    441,
    444,
    446,
    465,
    448,
    452,
    448,
    445,
    448,
    448,
    443,
    445,
    446,
    444,
    441,
    447,
    445,
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

int moveBack = 0;
int moveForward = 0;
int moveLeft = 0;
int moveRight = 0;

int largest = 0;

int moveBackValues[8] = {1, 2, 3, 4, 29, 28, 27, 26};
int moveForwardValues[8] = {14, 13, 12, 11, 16, 17, 18, 19};
int moveLeftValues[8] = {5, 6, 7, 8, 9, 10, 11, 12};
int moveRightValues[8] = {26, 25, 24, 23, 22, 21, 20, 19};

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
