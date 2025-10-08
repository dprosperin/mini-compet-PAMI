/**
 * @file motor.h
 * @brief Pilotage PWM par LEDC (ESP32) d’un moteur DC (1 fil PWM).
 *
 * @details
 * - Abstraction simple autour de LEDC : fréquence, résolution, canal.
 * - Sortie en @c percent et en @c duty brut.
 * - Plancher @c setMinPercent() pour vaincre les frottements au démarrage.
 */

#pragma once

#include <Arduino.h>

#ifndef ARDUINO_ARCH_ESP32
#error "Motor: LEDC dispo sur ESP32."
#endif

/**
 * @class Motor
 * @brief Wrapper LEDC pour PWM moteur DC (sans H-bridge).
 *
 * @code
 * Motor m(5, 0, 20000, 8);
 * m.begin();
 * m.setMinPercent(8.0f);
 * m.setPercent(35.0f);
 * @endcode
 */
class Motor
{
public:
  /**
   * @brief Constructeur.
   * @param pin Broche PWM.
   * @param channel Canal LEDC [0..15].
   * @param pwmHz Fréquence PWM (Hz).
   * @param resBits Résolution (bits, 1..16).
   */
  Motor(uint8_t pin, uint8_t channel = 0, uint32_t pwmHz = 20000, uint8_t resBits = 8) noexcept;

  /**
   * @brief Initialise LEDC et à 0% duty.
   */
  void begin() noexcept;

  /**
   * @brief Ecrit un duty brut [0..maxDuty()].
   * @param duty Valeur brute.
   */
  void setDuty(uint32_t duty) noexcept;

  /**
   * @brief Ecrit en pourcentage [0..100].
   * @details Applique un plancher @c _minPct pour décoller le moteur.
   * @param percent Pourcentage demandé.
   */
  void setPercent(float percent) noexcept;

  /**
   * @brief Fixe le plancher anti-frottements [0..30%].
   */
  void setMinPercent(float minPct) noexcept;

  /// Stop (équivaut à brake sans H-bridge).
  inline void stop() noexcept { setDuty(0); }

  /// Brake (sans H-bridge : idem stop).
  inline void brake() noexcept { setDuty(0); }

  /// @return Duty max (en fonction de la résolution).
  inline uint32_t maxDuty() const noexcept { return _maxDuty; }

  /// @return Duty courant.
  inline uint32_t duty() const noexcept { return _duty; }

private:
  uint8_t _pin; ///< GPIO PWM.
  uint8_t _ch;  ///< Canal LEDC.
  uint32_t _hz; ///< Fréquence PWM.
  uint8_t _res; ///< Résolution LEDC (bits).

  uint32_t _maxDuty = 0; ///< Duty max = (1<<_res)-1.
  uint32_t _duty = 0;    ///< Duty courant.

  float _minPct = 8.0f; ///< Plancher pour décoller (en %).
};