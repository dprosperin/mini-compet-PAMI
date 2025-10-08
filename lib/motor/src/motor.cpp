#include "motor.h"

Motor::Motor(uint8_t pin, uint8_t channel, uint32_t pwmHz, uint8_t resBits) noexcept
    : _pin(pin), _ch(channel), _hz(pwmHz), _res(resBits) {}

void Motor::begin() noexcept
{
  if (_ch > 15)
    _ch = 0;
  if (_res < 1)
    _res = 1;
  if (_res > 16)
    _res = 16;

  _maxDuty = (1u << _res) - 1u;

  ledcSetup(_ch, _hz, _res);
  ledcAttachPin(_pin, _ch);
  setDuty(0);
}

void Motor::setDuty(uint32_t duty) noexcept
{
  if (duty > _maxDuty)
    duty = _maxDuty;
  _duty = duty;
  ledcWrite(_ch, _duty);
}

void Motor::setPercent(float percent) noexcept
{
  if (percent <= 0.0f)
  {
    setDuty(0);
    return;
  }
  if (percent >= 100.0f)
  {
    setDuty(_maxDuty);
    return;
  }

  // Plancher simple : force un minimum pour “lancer” le moteur
  const float eff = _minPct + (100.0f - _minPct) * (percent * 0.01f);
  const uint32_t d = (uint32_t)(eff * _maxDuty * 0.01f + 0.5f);
  setDuty(d);
}

void Motor::setMinPercent(float minPct) noexcept
{
  if (minPct < 0.0f)
    minPct = 0.0f;
  if (minPct > 30.0f)
    minPct = 30.0f;
  _minPct = minPct;
}