#include <Arduino.h>
#include "ultrason.hpp"

void setup() {
  Serial.begin(115200); // Starts the serial communication
  setPinsTrigEcho();
}

void loop() {
  Serial.print("Distance (cm): ");
  Serial.println(getDistanceUltrason());
  delay(1000);
}