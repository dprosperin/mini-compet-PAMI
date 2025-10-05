#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "pid.h"

/* ========= Matériel ========= */
constexpr uint8_t PIN_PWM = 5;    ///< GPIO PWM moteur.
constexpr uint8_t PIN_ENC_A = 18; ///< GPIO encodeur A.
constexpr uint8_t PIN_ENC_B = 19; ///< GPIO encodeur B.
constexpr uint16_t TPR = 48;      ///< Ticks par tour de l’encodeur.

/* ========= PWM (LEDC) ========= */
constexpr uint8_t LEDC_CH = 0;       ///< Canal LEDC.
constexpr uint32_t LEDC_FREQ = 2000; ///< Fréquence PWM (Hz).
constexpr uint8_t LEDC_RES = 8;      ///< Résolution (bits).

/* ========= Consigne carrée 30 <-> 50 TPS ========= */
constexpr float SP_LOW_TPS = 30.0f;
constexpr float SP_HIGH_TPS = 65.0f;
constexpr uint32_t STEP_HIGH_MS = 5000; ///< Temps haut (ms).
constexpr uint32_t STEP_PER_MS = 10000; ///< Période totale (ms).

/* ========= Objets ========= */
Motor motor(PIN_PWM, LEDC_CH, LEDC_FREQ, LEDC_RES);
Encoder enc(PIN_ENC_A, PIN_ENC_B, TPR);

static float g_sp_tps = SP_LOW_TPS; ///< Consigne TPS.
static float g_meas_tps = 0.0f;     ///< Mesure TPS.
static float g_out_pct = 0.0f;      ///< Sortie PID (%).

/**
 * @brief PID: gains à adapter selon ton système.
 * @note @c outMin=0 / @c outMax=100 => %.
 */
PID pid(
    0.9f, // Kp
    6.0f, // Ki
    0.0f, // Kd
    0.0f,
    100.0f);

/** @brief ISR encodeur à connecter sur A et B. */
void IRAM_ATTR onEncISR() { enc.handleInterrupt(); }

/**
 * @brief Consigne carrée (50 TPS pendant 5 s puis 30 TPS pendant 5 s).
 * @param nowMs @c millis().
 * @return Consigne TPS.
 */
static inline float squareSetpoint(uint32_t nowMs)
{
  const uint32_t ph = nowMs % STEP_PER_MS;
  return (ph < STEP_HIGH_MS) ? SP_HIGH_TPS : SP_LOW_TPS;
}

/**
 * @brief Setup matériel, ISR et PID.
 */
void setup()
{
  Serial.begin(115200);
  delay(50);

  motor.begin();
  motor.setMinPercent(8.0f);

  enc.begin();
  enc.setUpdatePeriodMs(10);
  enc.setBlendThreshold(0.2f);
  enc.setEmaParams(3, 2);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncISR, CHANGE);

  pid.setIO(&g_sp_tps, &g_meas_tps, &g_out_pct);
  pid.setOutputLimits(0.0f, 100.0f);
  pid.reset();

  Serial.println("OK: contrôle vitesse prêt (créneau 30↔50 TPS, 5s/5s, en boucle).");
}

/**
 * @brief Boucle : met à jour consigne, mesure, PID, PWM, et log périodique.
 */
void loop()
{
  static uint32_t lastLog = 0;
  const uint32_t now = millis();

  // Consigne
  g_sp_tps = squareSetpoint(now);

  // Mesure
  enc.update();
  g_meas_tps = enc.getTPS();

  // PID -> PWM
  const float out = pid.compute();
  (void)out; // g_out_pct est écrit par compute()
  motor.setPercent(g_out_pct);

  // Log
  if (now - lastLog >= 50)
  {
    Serial.print("TPS=");
    Serial.print(g_meas_tps, 2);
    Serial.print("  SP=");
    Serial.print(g_sp_tps, 1);
    Serial.print("  Out(%)=");
    Serial.println(g_out_pct, 1);
    lastLog = now;
  }
}