/**
 * @file encoder.h
 * @brief Encodeur incrémental quadrature ultra-rapide (ESP32) avec fusion comptage/période.
 *
 * @details
 * - Lecture GPIO IRAM-safe (registre @c GPIO.in / @c GPIO.in1.val) pour ISR.
 * - Comptage robuste sur fenêtre (méthode "counting").
 * - Mesure de période d'impulsions (EMA entier) => réactif à bas régime.
 * - Fusion adaptative counting/period via un poids @c w=f(|TPS_period|).
 * - Fréquence d'update conseillée : ~100 Hz (ou plus).
 *
 * @note Optimisé pour ESP32 (Arduino-ESP32). Déclenche les ISR sur A et B en @c CHANGE.
 * @warning Code spécifique ESP32 (registre @c soc/gpio_struct.h). Non portable.
 */

#pragma once
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#ifndef ARDUINO_ARCH_ESP32
#error "Encoder: code optimisé ESP32."
#endif

#include "soc/gpio_struct.h" // GPIO.in / GPIO.in1.val
#include <esp_timer.h>       // esp_timer_get_time()

/**
 * @brief Lecture d’un GPIO **ultra rapide** et IRAM-safe (compatible ISR).
 * @param pin Numéro de broche ESP32.
 * @return 0 ou 1.
 */
#define GPIO_READ_FAST(pin) \
  ((uint8_t)(((pin) < 32) ? ((GPIO.in >> (pin)) & 0x1) : ((GPIO.in1.val >> ((pin) - 32)) & 0x1)))

/**
 * @class Encoder
 * @brief Driver d’encodeur quadrature avec fusion counting/période.
 *
 * @code
 * Encoder enc(18, 19, 48);
 * enc.begin();
 * attachInterrupt(digitalPinToInterrupt(18), onEnc, CHANGE);
 * attachInterrupt(digitalPinToInterrupt(19), onEnc, CHANGE);
 * // Dans loop():
 * enc.update();
 * float tps = enc.getTPS();
 * @endcode
 */
class Encoder
{
public:
  /**
   * @brief Constructeur.
   * @param pinA GPIO canal A.
   * @param pinB GPIO canal B.
   * @param ticksPerRev @c TPR (ticks par tour).
   */
  Encoder(uint8_t pinA, uint8_t pinB, uint16_t ticksPerRev = 48) noexcept;

  /**
   * @brief Init des GPIO et des états internes.
   */
  void begin() noexcept;

  /**
   * @brief ISR à attacher sur A **et** B en @c CHANGE.
   * @note Toujours marquer @c IRAM_ATTR.
   */
  void IRAM_ATTR handleInterrupt() noexcept;

  /**
   * @brief Mise à jour de la mesure TPS (~100 Hz recommandé).
   * @details Calcule TPS via comptage + période et fait la fusion adaptative.
   */
  void update() noexcept;

  /** @name Lecture */
  ///@{
  /**
   * @brief Compteur brut de ticks (signé).
   */
  inline long getTicks() const noexcept { return _ticks; }

  /**
   * @brief Vitesse estimée (tours par seconde).
   */
  inline float getTPS() const noexcept { return _tps; }
  ///@}

  /** @name Réglages (à chaud possible) */
  ///@{
  /**
   * @brief Change @c TPR (ticks par tour).
   * @param tpr Nouveau TPR (>0).
   */
  void setTicksPerRev(uint16_t tpr) noexcept;

  /**
   * @brief Période d’update (ms) pour la voie "counting".
   * @param ms Période (>=1 ms).
   */
  void setUpdatePeriodMs(uint32_t ms) noexcept;

  /**
   * @brief Seuil de fusion @c _blendA (en TPS).
   * @param a Paramètre d’échelle (>0). Plus grand => favorit "counting".
   */
  void setBlendThreshold(float a) noexcept;

  /**
   * @brief Paramètres EMA entier sur la période d’impulsion.
   * @param old_w Poids sur l’ancien (ex:3 => (old*3+new)>>2).
   * @param shift Décalage binaire (0..7). @c shift=2 => /4.
   */
  void setEmaParams(uint8_t old_w, uint8_t shift) noexcept;
  ///@}

  /**
   * @brief Remise à zéro des compteurs et filtres.
   */
  void reset() noexcept;

private:
  const uint8_t _pinA, _pinB;

  volatile long _ticks = 0;        ///< Compteur de ticks (signé).
  volatile uint8_t _lastState = 0; ///< Dernier état (A<<1)|B.

  uint16_t _tpr = 48;           ///< Ticks par tour.
  float _invTPR = 1.0f / 48.0f; ///< 1/TPR.

  // Counting
  uint32_t _prevMs = 0;          ///< Timestamp dernier update (ms).
  long _prevTicks = 0;           ///< Ticks à l’update précédente.
  uint32_t _updatePeriodMs = 10; ///< Période d’update counting (ms).

  // Period (réactif bas régime)
  volatile uint32_t _lastEdgeUs = 0;   ///< Dernier front (µs).
  volatile uint32_t _edgePeriodUs = 0; ///< Période moyenne (µs).
  uint8_t _emaOldW = 3;                ///< Poids EMA old.
  uint8_t _emaShift = 2;               ///< Décalage EMA.

  // Fusion
  float _tps = 0.0f;    ///< Estimation TPS fusionnée.
  float _blendA = 0.2f; ///< Seuil adaptatif.

  /**
   * @brief Lecture instantanée (A,B) en 2 bits.
   * @return @c (A<<1)|B
   */
  inline uint8_t readStateFast() const noexcept
  {
    const uint8_t a = GPIO_READ_FAST(_pinA);
    const uint8_t b = GPIO_READ_FAST(_pinB);
    return (uint8_t)((a << 1) | b);
  }
};

#endif