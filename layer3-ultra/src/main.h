// Include Libraries
#include <Arduino.h>
#include <NewPing.h>

// Pin Definitions
#define LED PA15


#define TeensySerial Serial2

#define ultraSensorOneTrigger PB1
#define ultraSensorOneEcho PB0
#define ultraSensorTwoTrigger PA5
#define ultraSensorTwoEcho PA4
#define ultraSensorThreeTrigger PA6
#define ultraSensorThreeEcho PA7
#define ultraSensorFourTrigger PB5
#define ultraSensorFourEcho PB6

// Global Variables
bool newData = false;
int maxDist = 200;
int sensorDistance[4];

NewPing sonar[4] = {
    NewPing(ultraSensorOneTrigger, ultraSensorOneEcho, maxDist),
    NewPing(ultraSensorTwoTrigger, ultraSensorTwoEcho, maxDist),
    NewPing(ultraSensorThreeTrigger, ultraSensorThreeEcho, maxDist),
    NewPing(ultraSensorFourTrigger, ultraSensorFourEcho, maxDist)};
