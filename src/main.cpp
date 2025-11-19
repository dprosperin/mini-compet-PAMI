#include <Arduino.h>
#include "ultrason.hpp"

const int SERVO_PIN  = 4;
const int SERVO_FREQ = 50;     // 50 Hz
const int SERVO_RES  = 16;     // 16 bits
const int SERVO_CH   = 0;

// --- Pour servo positionnel ---
int SERVO_MIN_US = 1000;       // impulsion à ~0°
int SERVO_MAX_US = 2000;       // impulsion à ~180°
float ANGLE_MIN  = 0.0f;
float ANGLE_MAX  = 180.0f;

// ====== UTILS ======
uint32_t usToDuty(int us){
  const uint32_t maxDuty = (1u << SERVO_RES) - 1u;
  return (uint32_t)(( (float)us / 20000.0f ) * maxDuty); // période 20 ms
}

void writeServoUs(int us){
  us = constrain(us, 1000, 2000);
  ledcWrite(SERVO_CH, usToDuty(us));
}

// ====== MODE POSITION (servo à palonnier 0–180°) ======
int angleToUs(float angle){
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
  float k = (angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
  return (int)round(SERVO_MIN_US + k * (SERVO_MAX_US - SERVO_MIN_US));
}

void writeServoAngle(float angle_deg){
  writeServoUs(angleToUs(angle_deg));
}

// Déplacement fluide vers une position en un temps donné (ms)
void moveToAngle(float target_deg, uint16_t duration_ms = 800){
  target_deg = constrain(target_deg, ANGLE_MIN, ANGLE_MAX);
  // on lit "virtuellement" la position courante via le dernier PWM (pas de retour réel)
  // on suppose la position courante proche de ce qu'on a demandé en dernier
  static float current_deg = 90.0f;
  const uint16_t steps = 40;             // + de steps = plus fluide
  const float step_deg = (target_deg - current_deg) / steps;
  const uint16_t step_ms = duration_ms / steps;

  for(uint16_t i=0;i<steps;i++){
    current_deg += step_deg;
    writeServoAngle(current_deg);
    delay(step_ms);
  }
  writeServoAngle(target_deg);
  current_deg = target_deg;
}

// **Nouvelles** fonctions "en position"
void range_pos(){               // analogue de range() mais en ANGLE
  // exemple : autour de 90°, ±20°
  moveToAngle(110, 500);
  delay(300);
  moveToAngle(90,  400);
  delay(300);
  moveToAngle(70,  500);
  delay(300);
  moveToAngle(90,  400);
  delay(300);
}

void whole_range_pos(){         // analogue de whole_range() mais 0–180°
  moveToAngle(0,    800);  delay(400);
  moveToAngle(90,   800);  delay(400);
  moveToAngle(180,  800);  delay(400);
  moveToAngle(90,   800);  delay(400);
}

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
  
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
  writeServoAngle(90.0f);
}

void loop() {
  float distance = 0;
  if (xQueueReceive(qh, &distance, portMAX_DELAY))
  {
     Serial.print("Distance (cm): ");
     Serial.println(distance);
  }
}
