#include <Arduino.h>
#include "ultrason.hpp"

void setup() {
  Serial.begin(9600); // Starts the serial communication
  setPinsTrigEcho();

  qh = xQueueCreate(8,sizeof(float));

  xTaskCreatePinnedToCore(
    ultrasonTask,   // Display task
    "ultrasonTask", // Task name
    2048,       // Stack size
    NULL,       // No parameters
    1,          // Priority
    NULL,       // No handle returned
    0);         // CPU 0
}

void loop() {
  float distance = 0;
  if (xQueueReceive(qh, &distance, portMAX_DELAY))
  {
     Serial.print("Distance (cm): ");
     Serial.println(distance);
  }
}