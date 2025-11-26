#include <Arduino.h>
#include "Encoder.hpp"

extern void updateTrajectory();

void setup() {
    Serial.begin(115200);
    Encoder::init();

    // initialise les points de départ
    startX = x;
    startY = y;
}

void loop() {
    updateTrajectory();
    delay(10);
}
