#include "encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB,
                 uint16_t ticksPerRev)
    : _pinA(pinA),
      _pinB(pinB),
      _tpr(ticksPerRev ? ticksPerRev : 1),
      _ticks(0),
      _prevUpdateMs(0),
      _prevTicks(0),
      _periodMs(20), // par défaut : 20 ms (50 Hz)
      _rps(0.0f)
{
}

void Encoder::begin()
{
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  // Lecture initiale, juste pour "réveiller" les entrées
  uint8_t a, b;
  readAB(a, b);

  noInterrupts();
  _ticks = 0;
  interrupts();

  _prevUpdateMs = millis();
  _prevTicks = 0;
  _rps = 0.0f;
}

void Encoder::setUpdatePeriodMs(uint32_t periodMs)
{
  if (periodMs == 0)
    periodMs = 1;
  _periodMs = periodMs;
}

void IRAM_ATTR Encoder::handleISR()
{
  // Interruption sur A, on lit A et B pour déterminer le sens
  uint8_t a, b;
  readAB(a, b);

  // Quadrature simple : si A == B -> +1, sinon -1
  // Le sens "avant" doit être garanti par le câblage physique.
  int8_t delta = (a == b) ? +1 : -1;

  _ticks += delta;
}

void Encoder::update()
{
  uint32_t now = millis();
  uint32_t dtMs = now - _prevUpdateMs;

  // On ne recalcule la vitesse que si la période cible est atteinte
  if (dtMs < _periodMs)
    return;

  long ticks;
  noInterrupts();
  ticks = _ticks;
  interrupts();

  long dTicks = ticks - _prevTicks;
  _prevTicks = ticks;
  _prevUpdateMs = now;

  if (dtMs == 0 || _tpr == 0)
  {
    _rps = 0.0f;
    return;
  }

  // Conversion en tours/s : dTicks / TPR / (dt en seconde)
  float dt = dtMs * 0.001f;
  float instRps = (float)dTicks / (float)_tpr / dt;

  // Pas de filtrage : on garde la valeur instantanée
  _rps = instRps;
}

void Encoder::resetTicks()
{
  noInterrupts();
  _ticks = 0;
  interrupts();

  _prevTicks = 0;
}
