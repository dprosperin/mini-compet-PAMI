/**
 * @file pid.h
 * @brief PID minimal lisible, avec anti-windup par dé-intégration en saturation.
 *
 * compute() calcule dt via millis(), applique saturation, met à jour *_out.
 */

#pragma once
#include <Arduino.h>

class PID
{
public:
  PID(float kp, float ki, float kd, float outMin = 0.0f, float outMax = 100.0f) noexcept;

  void setIO(const float *setpoint, const float *input, float *output) noexcept;
  void setTunings(float kp, float ki, float kd) noexcept;
  void setOutputLimits(float min, float max) noexcept;
  void reset() noexcept;

  float compute() noexcept; // renvoie la sortie (et écrit *_out)

private:
  const float *_sp = nullptr;
  const float *_in = nullptr;
  float *_out = nullptr;

  float _kp, _ki, _kd;
  float _omin, _omax;

  float _i = 0.0f;     // intégrale
  float _ePrev = 0.0f; // erreur précédente
  uint32_t _tPrev = 0; // timestamp (ms)
};