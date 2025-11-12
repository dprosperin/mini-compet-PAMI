#ifndef ULTRASON_HEADER
#define ULTRASON_HEADER

#include <Arduino.h>

const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.39370

extern QueueHandle_t qh;

void setPinsTrigEcho();
float getDistanceUltrason();
void ultrasonTask(void *arg);

#endif