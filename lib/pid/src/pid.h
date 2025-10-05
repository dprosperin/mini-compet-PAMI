/**
 * @file pid.h
 * @brief PID minimal avec anti-windup simple (conditional integration).
 *
 * @details
 * - @c compute() calcule @c dt via @c millis() (pas de cadence interne dédiée).
 * - Anti-windup : si saturation et erreur pousse dans le même sens -> dé-intègre.
 * - Pas de filtre D ni de feed-forward (volontairement compact).
 */

#pragma once
#ifndef PID_H
#define PID_H

#include <Arduino.h>

/**
 * @class PID
 * @brief Contrôleur PID minimaliste (float).
 *
 * @code
 * PID pid(0.9f, 6.0f, 0.0f, 0.0f, 100.0f);
 * pid.setIO(&sp, &meas, &out);
 * pid.setOutputLimits(0.0f, 100.0f);
 * pid.reset();
 * // boucle:
 * float u = pid.compute(); // écrit aussi *out
 * @endcode
 */
class PID
{
public:
  /**
   * @brief Constructeur.
   * @param kp Kp.
   * @param ki Ki.
   * @param kd Kd.
   * @param outMin Limite min de sortie.
   * @param outMax Limite max de sortie.
   */
  PID(float kp, float ki, float kd, float outMin = 0.0f, float outMax = 100.0f) noexcept;

  /**
   * @brief Lie les pointeurs I/O (consigne, mesure, sortie).
   * @param setpoint Pointeur consigne.
   * @param input Pointeur mesure.
   * @param output Pointeur sortie (écrite par @c compute()).
   */
  void setIO(const float *setpoint, const float *input, float *output) noexcept;

  /**
   * @brief Met à jour les gains.
   */
  void setTunings(float kp, float ki, float kd) noexcept;

  /**
   * @brief Fixe les bornes de sortie et sature immédiatement la sortie si besoin.
   */
  void setOutputLimits(float min, float max) noexcept;

  /**
   * @brief Remise à zéro : intégrateur, erreur, timestamp, sortie=min.
   */
  void reset() noexcept;

  /**
   * @brief Calcule la sortie PID, applique saturation et anti-windup.
   * @return Valeur de sortie écrite (@c *_out).
   */
  float compute() noexcept;

private:
  const float *_sp = nullptr; ///< Consigne.
  const float *_in = nullptr; ///< Mesure.
  float *_out = nullptr;      ///< Sortie.

  float _kp, _ki, _kd; ///< Gains.
  float _omin, _omax;  ///< Bornes de sortie.

  float _i = 0.0f;     ///< Terme intégral.
  float _ePrev = 0.0f; ///< Erreur précédente.
  uint32_t _tPrev = 0; ///< Timestamp (ms).
};

#endif