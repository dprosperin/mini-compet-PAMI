#pragma once
#include <Arduino.h>

/**
 * Encodeur quadrature (ESP32) — comptage + estimation TPS (tours/s roue).
 * A/B : entrées uniquement (ex: GPIO34/35). Pas de pullups internes ici.
 * Fenêtre d'estimation : Δticks / Δt / TPR.
 */
class Encoder
{
public:
  Encoder(uint8_t pinA, uint8_t pinB, uint16_t ticksPerRev = 48) noexcept;

  void begin() noexcept;               // à appeler dans setup()
  void IRAM_ATTR handleISR() noexcept; // ISR unique pour A et B (CHANGE)
  void update() noexcept;              // calcule _tps si période écoulée
  void reset() noexcept;

  // lecture
  inline long ticks() const noexcept { return _ticks; } // signé
  inline float tps() const noexcept { return _tps; }    // tours/s (roue)
  inline uint16_t tpr() const noexcept { return _tpr; } // ticks / tour (roue)

  // réglages
  void setTicksPerRev(uint16_t tpr) noexcept;
  void setUpdatePeriodMs(uint32_t ms) noexcept;

private:
  const uint8_t _pinA, _pinB;

  volatile long _ticks = 0;
  volatile uint8_t _lastAB = 0; // (A<<1)|B dernier

  uint16_t _tpr = 48; // ticks par tour de roue (à calibrer)
  float _invTPR = 1.0f / 48.0f;

  uint32_t _prevMs = 0;
  long _prevTicks = 0;
  uint32_t _updatePeriodMs = 10;

  float _tps = 0.0f;

  inline uint8_t readAB() const noexcept
  {
    // Pins input-only (34/35) OK | pas de pull-ups internes
    const uint8_t a = digitalRead(_pinA);
    const uint8_t b = digitalRead(_pinB);
    return (uint8_t)((a << 1) | b);
  }
  static inline int8_t quadDiff(uint8_t packed) noexcept;
};