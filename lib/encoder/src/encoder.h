#pragma once
#include <Arduino.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/**
 * Encodeur quadrature A/B pour moteur N20.
 *
 * Convention :
 *   - on déclenche l'ISR sur la voie A (CHANGE)
 *   - on lit la voie B pour connaître le sens
 *   - le câblage physique doit être fait pour que
 *     "marche avant" => ticks qui augmentent (rps > 0).
 *
 * Fournit :
 *   - un compteur de ticks global
 *   - une vitesse instantanée en tours/s (rps)
 */
class Encoder
{
public:
  // ticksPerRev = nombre de ticks par tour mécanique (à mesurer / vérifier)
  Encoder(uint8_t pinA, uint8_t pinB,
          uint16_t ticksPerRev);

  // À appeler dans setup()
  void begin();

  // À appeler depuis l'ISR attachInterrupt (sur la voie A)
  void IRAM_ATTR handleISR();

  // À appeler régulièrement dans loop()
  // (période définie par setUpdatePeriodMs())
  void update();

  // Accès au compteur brut de ticks
  long getTicks() const { return _ticks; }

  void resetTicks();

  // Vitesse en tours/s (instantanée, non filtrée)
  float getRps() const { return _rps; }

  // Période cible de mise à jour de la vitesse (en ms)
  void setUpdatePeriodMs(uint32_t periodMs);

private:
  uint8_t _pinA;
  uint8_t _pinB;
  uint16_t _tpr;        // ticks par tour
  volatile long _ticks; // compteur global de ticks (modifié en ISR)

  uint32_t _prevUpdateMs; // dernier appel à update()
  long _prevTicks;
  uint32_t _periodMs; // période cible (ms)
  float _rps;         // vitesse en tours/s

  inline void readAB(uint8_t &a, uint8_t &b) const
  {
    a = (uint8_t)digitalRead(_pinA);
    b = (uint8_t)digitalRead(_pinB);
  }
};
