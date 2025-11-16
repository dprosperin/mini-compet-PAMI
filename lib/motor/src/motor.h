#pragma once
#include <Arduino.h>

// Déclaration anticipée (pour le pointeur)
class Encoder;

/**
 * Gestion d'un moteur N20 via un DRV8833 (2 entrées : IN1 / IN2) avec PWM LEDC.
 *
 * Convention :
 *   - PWM > 0  => marche avant
 *   - PWM < 0  => marche arrière
 *   - PWM = 0  => roue libre
 *
 * Le câblage moteur DOIT être fait pour que "marche avant"
 * corresponde au sens physique souhaité pour ton robot.
 *
 * L'encodeur associé doit être câblé pour que ce même sens
 * donne une vitesse rps > 0.
 */
class Motor
{
public:
  // in1Pin / in2Pin = pins DRV8833
  // ch1 / ch2 = canaux LEDC utilisés pour ces pins
  Motor(uint8_t in1Pin, uint8_t in2Pin,
        uint8_t ch1, uint8_t ch2);

  // À appeler dans setup()
  void begin();

  // Commande brute en PWM signé (-255..+255)
  void setPwm(int pwm);

  // Commande en pourcentage [-100..+100]
  void setPercent(float pct);

  // Arrêt roue libre (LOW/LOW)
  void stop();

  // Freinage actif (HIGH/HIGH)
  void brake();

  // Association d'un encodeur (optionnel)
  void attachEncoder(Encoder *enc) { _enc = enc; }

  // Vitesse mesurée en tours/s (retourne 0.0f si pas d'encodeur)
  float getRps() const;

private:
  uint8_t _in1Pin;
  uint8_t _in2Pin;
  uint8_t _ch1;
  uint8_t _ch2;
  Encoder *_enc; // non possédé

  void applyPwmInternal(int pwm);
};
