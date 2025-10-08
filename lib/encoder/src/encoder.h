/**
 * @file encoder.h
 * @brief Encodeur quadrature simple (ESP32/Arduino) basé sur le comptage de ticks.
 *
 * Principe :
 *  - Deux interruptions CHANGE (A et B) appellent handleISR().
 *  - À chaque front, on relit A et B, on compare à l’état précédent, on ajoute +1/-1/0.
 *  - La vitesse (TPS) est estimée dans update() sur une fenêtre temporelle (Δticks / Δt / TPR).
 *
 * Lisible d’abord. Suffisant pour la plupart des N20 + réducteur.
 */

#pragma once
#include <Arduino.h>

class Encoder
{
public:
  Encoder(uint8_t pinA, uint8_t pinB, uint16_t ticksPerRev = 48) noexcept;

  /// À appeler dans setup() avant attachInterrupt().
  void begin() noexcept;

  /// ISR à attacher sur A ET sur B (CHANGE).
  void IRAM_ATTR handleISR() noexcept;

  /// Calcule la vitesse moyenne sur la fenêtre écoulée.
  /// Appeler à cadence régulière (ex: toutes les 10 ms).
  void update() noexcept;

  /// Réinitialise compteurs et mesure.
  void reset() noexcept;

  /// Accesseurs
  inline long ticks() const noexcept { return _ticks; } // compteur brut (signé)
  inline float tps() const noexcept { return _tps; }    // tours/s estimés
  inline uint16_t tpr() const noexcept { return _tpr; } // ticks par tour

  /// Réglages
  void setTicksPerRev(uint16_t tpr) noexcept;
  void setUpdatePeriodMs(uint32_t ms) noexcept; // par défaut 10 ms

private:
  // GPIO
  const uint8_t _pinA;
  const uint8_t _pinB;

  // Quadrature
  volatile long _ticks = 0;     // compteur de ticks
  volatile uint8_t _lastAB = 0; // (A<<1)|B dernier

  // Cinématique
  uint16_t _tpr = 48; // ticks par tour
  float _invTPR = 1.0f / 48.0f;

  // Fenêtre counting
  uint32_t _prevMs = 0;
  long _prevTicks = 0;
  uint32_t _updatePeriodMs = 10; // période d’update recommandée (ms)

  // Sortie
  float _tps = 0.0f;

  // Lecture instantanée
  inline uint8_t readAB() const noexcept
  {
    const uint8_t a = digitalRead(_pinA);
    const uint8_t b = digitalRead(_pinB);
    return (uint8_t)((a << 1) | b);
  }

  // Diff quadrature compacte : (old<<2)|new -> +1 / -1 / 0
  static inline int8_t quadDiff(uint8_t packed) noexcept;
};