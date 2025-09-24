#include "ultrason.hpp"
#include <Arduino.h>

void setPinsTrigEcho()
{
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void clearTrig()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
}

float getDistanceUltrason()
{
    float duration = 0, distanceCm = 0;

    clearTrig();

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
  
    // Calculate the distance
    distanceCm = duration * SOUND_SPEED/2;

    return distanceCm;
}