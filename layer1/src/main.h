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
int fixedThreshFirstBot[30] = {224, 333, 372, 224, 388, 224, 398, 406, 224, 400, 391, 402, 336, 164, 224, 224, 224, 224, 224, 398, 391, 402, 336, 164, 224, 271, 224, 333, 224, 224};

int muxChannel[16][4] = { // Flipped it around cos i was dumb
    {1,1,1,1},
    {1,1,1,0},
    {1,1,0,1},
    {1,1,0,0},
    {1,0,1,1},
    {1,0,1,0},
    {1,0,0,1},
    {1,0,0,0},
    {0,1,1,1},
    {0,1,1,0},
    {0,1,0,1},
    {0,1,0,0},
    {0,0,1,1},
    {0,0,1,0},
    {0,0,0,1},
    {0,0,0,0}

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
