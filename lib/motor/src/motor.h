/**
 * @file motor.h
 * @brief Pilotage PWM LEDC (ESP32) d’un moteur DC (1 fil PWM).
 *
 * API minimale :
 *   - begin() pour configurer LEDC
 *   - setPercent(0..100) pour commander
 *   - setMinPercent(%) pour vaincre les frottements au démarrage
 */

#pragma once
#include <Arduino.h>

class Motor
{
public:
  Motor(uint8_t pin, uint8_t channel = 0, uint32_t pwmHz = 20000, uint8_t resBits = 8) noexcept;

  void begin() noexcept;
  void setDuty(uint32_t duty) noexcept;      // brut [0..maxDuty]
  void setPercent(float percent) noexcept;   // 0..100 (%)
  void setMinPercent(float minPct) noexcept; // 0..30 (%)

  inline void stop() noexcept { setDuty(0); }
  inline void brake() noexcept { setDuty(0); }

  inline uint32_t maxDuty() const noexcept { return _maxDuty; }
  inline uint32_t duty() const noexcept { return _duty; }

private:
  uint8_t _pin;
  uint8_t _ch;
  uint32_t _hz;
  uint8_t _res;

  uint32_t _maxDuty = 0;
  uint32_t _duty = 0;

  float _minPct = 8.0f; // min pour faire tourne le moteur
};