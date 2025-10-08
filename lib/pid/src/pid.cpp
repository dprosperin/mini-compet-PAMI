#include "pid.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax) noexcept
    : _kp(kp), _ki(ki), _kd(kd), _omin(outMin), _omax(outMax) {}

void PID::setIO(const float *setpoint, const float *input, float *output) noexcept
{
  _sp = setpoint;
  _in = input;
  _out = output;
}

void PID::setTunings(float kp, float ki, float kd) noexcept
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::setOutputLimits(float min, float max) noexcept
{
  if (min >= max)
    return;
  _omin = min;
  _omax = max;
  if (_out)
  {
    if (*_out < _omin)
      *_out = _omin;
    if (*_out > _omax)
      *_out = _omax;
  }
}

void PID::reset() noexcept
{
  _i = 0.0f;
  _ePrev = 0.0f;
  _tPrev = millis();
  if (_out)
    *_out = _omin;
}

float PID::compute() noexcept
{
  if (!_sp || !_in || !_out)
    return 0.0f;

  const uint32_t now = millis();
  float dt = (now - _tPrev) * 0.001f;
  if (dt <= 0.0f)
  {
    _tPrev = now;
    return *_out;
  }
  _tPrev = now;

  const float e = (*_sp) - (*_in);

  // PID “nu”
  _i += e * dt;
  const float d = (e - _ePrev) / dt;

  const float u = _kp * e + _ki * _i + _kd * d;

  // Saturation + anti-windup simple
  float out = u;
  if (out > _omax)
    out = _omax;
  if (out < _omin)
    out = _omin;

  const bool satHigh = (u > _omax);
  const bool satLow = (u < _omin);
  if ((satHigh && e > 0.0f) || (satLow && e < 0.0f))
    _i -= e * dt; // désintégration si ça pousse dans la saturation

  _ePrev = e;
  *_out = out;
  return out;
}