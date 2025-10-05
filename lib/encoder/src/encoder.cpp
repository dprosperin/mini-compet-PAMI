/**
 * @file encoder.cpp
 * @brief Implémentation de l’encodeur quadrature (ESP32).
 */

#include "encoder.h"

/**
 * @brief Table compacte quadrature: (old<<2)|new -> +1 / -1 / 0.
 * @param packed 4 bits (état précédent << 2) | état courant.
 * @return +1 (sens horaire), -1 (anti-horaire), 0 (rebond / saut).
 */
static inline int8_t quad_diff(uint8_t packed) noexcept
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
    return 0; // rebond / saut
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
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  _lastState = readStateFast();
  _ticks = 0;
  _prevTicks = 0;
  _prevMs = millis();
  _tps = 0.0f;
  _lastEdgeUs = (uint32_t)esp_timer_get_time();
  _edgePeriodUs = 0;
}

void IRAM_ATTR Encoder::handleInterrupt() noexcept
{
  const uint8_t newState = readStateFast();
  const int8_t d = quad_diff((uint8_t)((_lastState << 2) | newState));

  if (d != 0)
  {
    _ticks += d;

    const uint32_t nowUs = (uint32_t)esp_timer_get_time();
    const uint32_t p = nowUs - _lastEdgeUs;
    _lastEdgeUs = nowUs;

    if (_edgePeriodUs == 0)
    {
      _edgePeriodUs = p;
    }
    else
    {
      // EMA entier: edge = (old * w + p) >> shift
      _edgePeriodUs = ((uint32_t)_emaOldW * _edgePeriodUs + p) >> _emaShift;
    }
  }
  _lastState = newState;
}

void Encoder::update() noexcept
{
  const uint32_t nowMs = millis();
  const uint32_t dtMs = nowMs - _prevMs;
  if (dtMs < _updatePeriodMs)
    return;

  // (1) Counting (fenêtre robuste)
  long ticksSnap;
  noInterrupts();
  ticksSnap = _ticks;
  interrupts();
  const long dTicks = ticksSnap - _prevTicks;
  const float dt = (float)dtMs * 0.001f;
  const float tps_count = ((float)dTicks * _invTPR) / dt;

  // (2) Period (réactif bas régime)
  uint32_t pSnap;
  noInterrupts();
  pSnap = _edgePeriodUs;
  interrupts();
  float tps_period = 0.0f;
  if (pSnap > 0)
  {
    const float tick_per_s = 1e6f / (float)pSnap;
    tps_period = tick_per_s * _invTPR;
  }

  // (3) Fusion adaptative
  const float a = fabsf(tps_period);
  const float w = (a > 0.0f) ? (a / (a + _blendA)) : 0.0f; // 0..1
  _tps = (1.0f - w) * tps_count + w * tps_period;

  _prevTicks = ticksSnap;
  _prevMs = nowMs;
}

void Encoder::reset() noexcept
{
  noInterrupts();
  _ticks = 0;
  _edgePeriodUs = 0;
  _lastEdgeUs = (uint32_t)esp_timer_get_time();
  interrupts();

  _prevTicks = 0;
  _tps = 0.0f;
  _prevMs = millis();
}

void Encoder::setTicksPerRev(uint16_t tpr) noexcept
{
  if (tpr == 0)
    return;
  _tpr = tpr;
  _invTPR = 1.0f / (float)tpr;
}

void Encoder::setUpdatePeriodMs(uint32_t ms) noexcept
{
  _updatePeriodMs = (ms == 0) ? 1u : ms;
}

void Encoder::setBlendThreshold(float a) noexcept
{
  _blendA = (a < 0.0f) ? 0.0f : a;
}

void Encoder::setEmaParams(uint8_t old_w, uint8_t shift) noexcept
{
  _emaOldW = old_w;
  _emaShift = (shift > 7) ? 7 : shift;
}