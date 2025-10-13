#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include <QuickPID.h>

/* =======================================================================
   BLOC 1 — CÂBLAGE
   - Ultrasons occupent GPIO5 et GPIO18  -> évités ici.
   - D34/D35 = entrées uniquement        -> parfaits pour l’encodeur.
   ======================================================================= */
constexpr uint8_t PIN_PWM = 25;   // PWM moteur (LEDC)
constexpr uint8_t PIN_ENC_A = 34; // Encodeur A (input-only)
constexpr uint8_t PIN_ENC_B = 35; // Encodeur B (input-only)

/* =======================================================================
   BLOC 2 — PARAM MATÉRIEL
   ======================================================================= */
constexpr uint8_t LEDC_CH = 0;
constexpr uint32_t LEDC_HZ = 20000; // 20 kHz -> silencieux
constexpr uint8_t LEDC_RES = 8;

/* TPR = ticks par tour DE ROUE (pas du moteur).
 * Mesure rapide : 1 tour de roue EXACT -> lire Δticks -> reporter ici.
 */
constexpr uint16_t WHEEL_TPR = 48; // À CALIBRER selon ton réducteur // 4224 44×96 (ou 48×88)

/* =======================================================================
   BLOC 3 — SCÉNARIO DE TEST (créneau 30↔65 TPS)
   ======================================================================= */
constexpr float SP_LOW_TPS = 30.0f;
constexpr float SP_HIGH_TPS = 65.0f;
constexpr uint32_t STEP_HIGH_MS = 5000; // 5 s à 65 TPS
constexpr uint32_t STEP_PER_MS = 10000; // période 10 s (5s/5s)

/* =======================================================================
   BLOC 4 — OBJETS
   ======================================================================= */
Motor motor(PIN_PWM, LEDC_CH, LEDC_HZ, LEDC_RES);
Encoder enc(PIN_ENC_A, PIN_ENC_B, WHEEL_TPR);

/* =======================================================================
   BLOC 5 — VARIABLES I/O PID (QuickPID)
   ======================================================================= */
static float g_setpoint_tps = SP_LOW_TPS; // consigne (tours/s roue)
static float g_meas_tps = 0.0f;           // mesure
static float g_out_pct = 0.0f;            // sortie PID -> % PWM

// Gains init (point de départ sûr ; à tuner)
static float Kp = 0.9f, Ki = 6.0f, Kd = 0.0f;

// QuickPID 3.1.9 : constructeur 10-args
QuickPID pid(&g_meas_tps, &g_out_pct, &g_setpoint_tps,
             Kp, Ki, Kd,
             QuickPID::pMode::pOnError,
             QuickPID::dMode::dOnError,
             QuickPID::iAwMode::iAwClamp,
             QuickPID::Action::direct);

/* =======================================================================
   BLOC 6 — UTILITAIRES
   ======================================================================= */
void IRAM_ATTR onEncISR() { enc.handleISR(); }

static inline float squareSetpoint(uint32_t nowMs)
{
  const uint32_t ph = nowMs % STEP_PER_MS;
  return (ph < STEP_HIGH_MS) ? SP_HIGH_TPS : SP_LOW_TPS;
}

/* =======================================================================
   BLOC 7 — SETUP
   ======================================================================= */
void setup()
{
  Serial.begin(115200);
  delay(50);

  // PWM moteur
  motor.begin();
  motor.setMinPercent(8.0f); // plancher pour vaincre les frottements

  // Encodeur
  enc.begin();
  enc.setUpdatePeriodMs(10); // estimation vitesse à 100 Hz

  // Interrupts sur A et B (fronts montants/descendants)
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncISR, CHANGE);

  // QuickPID
  pid.SetOutputLimits(0.0f, 100.0f); // sortie en %
  pid.SetMode(QuickPID::Control::automatic);
  pid.SetSampleTimeUs(10000); // 10 ms

  Serial.println("OK: contrôle vitesse (créneau 30↔65 TPS, 5s/5s).");
  Serial.println("Note: WHEEL_TPR = ticks/tour de ROUE (calibre-le !).");
}

/* =======================================================================
   BLOC 8 — LOOP : Encodeur -> TPS -> PID -> PWM -> Log
   ======================================================================= */
void loop()
{
  static uint32_t lastLog = 0;
  const uint32_t now = millis();

  // 1) Consigne (carré)
  g_setpoint_tps = squareSetpoint(now);

  // 2) Mesure vitesse (Δticks / Δt / TPR)
  enc.update();
  g_meas_tps = enc.tps();

  // 3) PID -> sortie (%)
  pid.Compute();

  // 4) Appliquer au moteur
  motor.setPercent(g_out_pct);

  // 5) Log ~20 Hz
  if (now - lastLog >= 50)
  {
    Serial.print("TPS=");
    Serial.print(g_meas_tps, 2);
    Serial.print("  SP=");
    Serial.print(g_setpoint_tps, 1);
    Serial.print("  Out(%)=");
    Serial.println(g_out_pct, 1);
    lastLog = now;
  }
}