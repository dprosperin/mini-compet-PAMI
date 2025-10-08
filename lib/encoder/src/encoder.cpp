#include "encoder.h"

/* Diff de quadrature compacte : (old<<2)|new -> +1 / -1 / 0 */
int8_t Encoder::quadDiff(uint8_t packed) noexcept
{
  switch (packed)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    return +1;
  case 0b0010:
  case 0b1011:
  case 0b1101:
  case 0b0100:
    return -1;
  default:
    return 0; // rebond/saut
  }
}

Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint16_t ticksPerRev) noexcept
    : _pinA(pinA), _pinB(pinB), _tpr(ticksPerRev)
{
  if (_tpr == 0)
    _tpr = 1;
  _invTPR = 1.0f / (float)_tpr;
}

void Encoder::begin() noexcept
{
  // Entrées pures : surtout pas OUTPUT sur 34/35 ; pas de pullup interne.
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);
  _lastAB = readAB();
  _ticks = 0;
  _prevTicks = 0;
  _prevMs = millis();
  _tps = 0.0f;
}

void IRAM_ATTR Encoder::handleISR() noexcept
{
  const uint8_t ab = readAB();
  const int8_t d = quadDiff((uint8_t)((_lastAB << 2) | ab));
  _lastAB = ab;
  if (d)
    _ticks += d;
}

void Encoder::update() noexcept
{
  const uint32_t now = millis();
  const uint32_t dtMs = now - _prevMs;
  if (dtMs < _updatePeriodMs)
    return;

  long snap;
  noInterrupts();
  snap = _ticks;
  interrupts();

  const long dTicks = snap - _prevTicks;
  const float dt = (float)dtMs * 0.001f;

  _tps = (dTicks * _invTPR) / dt; // tours/s roue

  _prevTicks = snap;
  _prevMs = now;
}

void Encoder::reset() noexcept
{
  noInterrupts();
  _ticks = 0;
  _lastAB = readAB();
  interrupts();
  _prevTicks = 0;
  _tps = 0.0f;
  _prevMs = millis();
}

void Encoder::setTicksPerRev(uint16_t tpr) noexcept
{
  if (!tpr)
    return;
  _tpr = tpr;
  _invTPR = 1.0f / (float)_tpr;
}

void Encoder::setUpdatePeriodMs(uint32_t ms) noexcept
{
  _updatePeriodMs = ms == 0 ? 1u : ms;
}