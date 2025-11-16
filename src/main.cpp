#include <Arduino.h>
#include <PIDController.h> // Lib PID utilisée dans le tuto CircuitDigest
#include "encoder.h"
#include "motor.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/* ===================== PINS DRV8833 / ENCODEURS ===================== */

// DRV8833 - Moteur A
const uint8_t MOT_A1_PIN = 25;
const uint8_t MOT_A2_PIN = 26;

// DRV8833 - Moteur B
const uint8_t MOT_B1_PIN = 27;
const uint8_t MOT_B2_PIN = 14;

// Canaux PWM (LEDC) pour chaque entrée moteur
const uint8_t MOT_A1_CH = 0;
const uint8_t MOT_A2_CH = 1;
const uint8_t MOT_B1_CH = 2;
const uint8_t MOT_B2_CH = 3;

// DRV8833 - SLEEP / FAULT
const uint8_t PIN_SLEEP = 32; // HIGH = driver ON
const uint8_t PIN_FAULT = 33; // LOW  = défaut

// Encodeurs (voies A/B)
const uint8_t ENC_A_PIN_A = 34;
const uint8_t ENC_A_PIN_B = 35;
const uint8_t ENC_B_PIN_A = 36;
const uint8_t ENC_B_PIN_B = 39;

/* ===================== PARAMS ENCODEURS / PID ===================== */

// Ticks par tour mécanique (à vérifier sur ton montage)
const uint16_t WHEEL_TPR = 48;

// Période de contrôle (vitesse + PID) en ms
const uint32_t CONTROL_PERIOD_MS = 0; // 50 Hz

// Période de log série / Teleplot
const uint32_t LOG_PERIOD_MS = 0; // 25 Hz

/* ===================== CONSIGNES ET GAINS PID ===================== */

// Consignes de vitesse en tours/s (avant logique = positif)
float setpointA_rps = 0.0f;
float setpointB_rps = 0.0f;

// Gains PID (style CircuitDigest, à ajuster)
float Kp = 260.0f;
float Ki = 2.7f;
float Kd = 2000.0f;

/* ===================== OBJETS GLOBAUX ===================== */

// Encodeurs
Encoder encoderA(ENC_A_PIN_A, ENC_A_PIN_B, WHEEL_TPR);
Encoder encoderB(ENC_B_PIN_A, ENC_B_PIN_B, WHEEL_TPR);

// Moteurs
// Moteurs
Motor motorA(MOT_A1_PIN, MOT_A2_PIN, MOT_A1_CH, MOT_A2_CH);
Motor motorB(MOT_B1_PIN, MOT_B2_PIN, MOT_B1_CH, MOT_B2_CH);

// PID de chaque motor (lib PIDController du tuto)
PIDController pidA;
PIDController pidB;

// Mesures et commandes
float speedA_rps = 0.0f;
float speedB_rps = 0.0f;
int pwmA = 0;
int pwmB = 0;

/* ===================== OUTILS DRV8833 ===================== */

static void drvWake()
{
   digitalWrite(PIN_SLEEP, HIGH);
   delay(1); // temps de réveil du driver
}

static void drvSleep()
{
   digitalWrite(PIN_SLEEP, LOW);
}

/* ===================== ROUTINES D'INTERRUPTION ENCODEURS ===================== */

void IRAM_ATTR encoderA_isr()
{
   encoderA.handleISR();
}

void IRAM_ATTR encoderB_isr()
{
   encoderB.handleISR();
}

/* ===================== FONCTIONS HAUT NIVEAU DE PILOTAGE ===================== */

/**
 * Fixe directement les consignes de vitesse des 2 moteurs.
 *   spA / spB en tours/s
 *   > 0 = marche avant
 *   < 0 = marche arrière
 */
void setMotorsRps(float spA, float spB)
{
   setpointA_rps = spA;
   setpointB_rps = spB;
}

/** Marche avant avec les deux roues à la même vitesse (rps > 0). */
void driveForward(float rps)
{
   setMotorsRps(rps, rps);
}

/** Marche arrière avec les deux roues. */
void driveBackward(float rps)
{
   setMotorsRps(-rps, -rps);
}

/** Rotation sur place vers la gauche (A arrière, B avant). */
void turnLeft(float rps)
{
   setMotorsRps(-rps, rps);
}

/** Rotation sur place vers la droite (A avant, B arrière). */
void turnRight(float rps)
{
   setMotorsRps(rps, -rps);
}

/** Arrêt (consigne nulle, les moteurs passent en roue libre via la boucle). */
void driveStop()
{
   setMotorsRps(0.0f, 0.0f);
}

/* ===================== SETUP ===================== */

void setup()
{
   Serial.begin(115200);
   delay(100);

   // DRV8833 : pins de gestion globale
   pinMode(PIN_SLEEP, OUTPUT);
   pinMode(PIN_FAULT, INPUT_PULLUP);
   drvWake();

   // Moteurs
   motorA.begin();
   motorB.begin();

   // Encodeurs
   encoderA.setUpdatePeriodMs(CONTROL_PERIOD_MS);
   encoderB.setUpdatePeriodMs(CONTROL_PERIOD_MS);
   encoderA.begin();
   encoderB.begin();

   // Interruptions sur la voie A des encodeurs
   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN_A), encoderA_isr, CHANGE);
   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN_A), encoderB_isr, CHANGE);

   // Lien moteur <-> encodeur
   motorA.attachEncoder(&encoderA);
   motorB.attachEncoder(&encoderB);

   // Configuration des PID (comme dans le tuto CircuitDigest)
   pidA.begin();
   pidA.tune(Kp, Ki, Kd); // Kp, Ki, Kd globaux
   pidA.limit(-255, 255); // sortie directement exploitable en PWM

   pidB.begin();
   pidB.tune(Kp, Ki, Kd);
   pidB.limit(-255, 255);

   // Exemple : démarre en marche avant 50 rps sur A et B
   driveForward(50.0f);
}

/* ===================== LOOP ===================== */

void loop()
{
   static uint32_t lastControl = 0;
   static uint32_t lastLog = 0;

   uint32_t now = millis();

   // 1) Boucle de régulation (vitesse + PID)
   if (now - lastControl >= CONTROL_PERIOD_MS)
   {
      // Mise à jour des vitesses mesurées
      encoderA.update();
      encoderB.update();

      speedA_rps = motorA.getRps();
      speedB_rps = motorB.getRps();

      // Consignes
      pidA.setpoint(setpointA_rps);
      pidB.setpoint(setpointB_rps);

      // Calcul des PWM signés
      pwmA = pidA.compute(speedA_rps);
      pwmB = pidB.compute(speedB_rps);

      // Application aux moteurs
      if (setpointA_rps == 0.0f)
      {
         motorA.stop();
         pwmA = 0;
      }
      else
      {
         motorA.setPwm(pwmA);
      }

      if (setpointB_rps == 0.0f)
      {
         motorB.stop();
         pwmB = 0;
      }
      else
      {
         motorB.setPwm(pwmB);
      }

      lastControl = now;
   }

   // 2) Logs série / Teleplot
   if (now - lastLog >= LOG_PERIOD_MS)
   {
      // Log lisible pour debug rapide
      Serial.print(F("A: SP="));
      Serial.print(setpointA_rps, 2);
      Serial.print(F(" rps, Meas="));
      Serial.print(speedA_rps, 2);
      Serial.print(F(", PWM="));
      Serial.print(pwmA);

      Serial.print(F(" | B: SP="));
      Serial.print(setpointB_rps, 2);
      Serial.print(F(" rps, Meas="));
      Serial.print(speedB_rps, 2);
      Serial.print(F(", PWM="));
      Serial.println(pwmB);

      // Trames Teleplot (si tu veux visualiser plus proprement)
      Serial.print(F(">A_SP:"));
      Serial.println(setpointA_rps);
      Serial.print(F(">A_MEAS:"));
      Serial.println(speedA_rps);
      Serial.print(F(">A_PWM:"));
      Serial.println(pwmA);

      Serial.print(F(">B_SP:"));
      Serial.println(setpointB_rps);
      Serial.print(F(">B_MEAS:"));
      Serial.println(speedB_rps);
      Serial.print(F(">B_PWM:"));
      Serial.println(pwmB);

      lastLog = now;
   }

   // loop() volontairement très simple.
}
