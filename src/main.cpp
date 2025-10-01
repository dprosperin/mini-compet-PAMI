#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Paramètres servo (ajuste si ton servo sature ou claque)
const uint8_t SERVO_CH = 11;         // ton canal
const float   SERVO_MIN_US = 500;    // µs à 0°
const float   SERVO_MAX_US = 2500;   // µs à 180°
const float   SERVO_FREQ   = 50.0;   // Hz (servos = 50–60 Hz)

uint16_t usToTicks(float us) {
  // PCA9685 = 12 bits (4096 pas) sur une période = 1/freq
  // ticks = us / (1e6 / (4096 * freq))
  const float tick_us = 1000000.0f / (4096.0f * SERVO_FREQ);
  return (uint16_t)roundf(us / tick_us);
}

void writeServoAngle(uint8_t ch, float angle_deg) {
  angle_deg = constrain(angle_deg, 0.0f, 180.0f);
  float pulse_us = SERVO_MIN_US +
                   (SERVO_MAX_US - SERVO_MIN_US) * (angle_deg / 180.0f);
  uint16_t off = usToTicks(pulse_us);
  pwm.setPWM(ch, 0, off);
}

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Place le servo à 90° au démarrage
  writeServoAngle(SERVO_CH, 65);
}

void loop() {
  // Exemple: balayage 0° -> 180° -> 0°
  for (int a = 65; a <= 135; a += 5) {
    writeServoAngle(SERVO_CH, a);
    delay(20);
  }
  for (int a = 135; a >= 65; a -= 5) {
    writeServoAngle(SERVO_CH, a);
    delay(20);
  }
}
