#include "motor.h"
#include "encoder.h"

// Paramètres PWM communs à tous les moteurs
static const uint32_t MOTOR_PWM_FREQ = 20000; // 20 kHz, silencieux pour le N20
static const uint8_t MOTOR_PWM_RES = 8;       // 8 bits -> 0..255
static const uint16_t MOTOR_PWM_MAX = (1 << MOTOR_PWM_RES) - 1;

Motor::Motor(uint8_t in1Pin, uint8_t in2Pin,
             uint8_t ch1, uint8_t ch2)
    : _in1Pin(in1Pin),
      _in2Pin(in2Pin),
      _ch1(ch1),
      _ch2(ch2),
      _enc(NULL)
{
}

void Motor::begin()
{
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);

  // Configuration des canaux LEDC
  // Chaque canal a sa fréquence et résolution
  ledcSetup(_ch1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(_ch2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

  // Attache des pins aux canaux
  ledcAttachPin(_in1Pin, _ch1);
  ledcAttachPin(_in2Pin, _ch2);

  // Démarrage à l'arrêt, roue libre
  ledcWrite(_ch1, 0);
  ledcWrite(_ch2, 0);
}

void Motor::applyPwmInternal(int pwm)
{
  // Saturation simple
  if (pwm > 255)
    pwm = 255;
  if (pwm < -255)
    pwm = -255;

  if (pwm == 0)
  {
    // Ralentissement en roue libre : LOW/LOW
    ledcWrite(_ch1, 0);
    ledcWrite(_ch2, 0);
    return;
  }

  // Pattern identique au tuto DRV8833 :
  //   - sens 1 : PWM sur IN1, IN2 à 0
  //   - sens 2 : PWM sur IN2, IN1 à 0
  if (pwm < 0)
  {
    // Marche arrière : PWM sur IN1
    uint16_t duty = (uint16_t)(-pwm); // pwm<0, on prend la valeur absolue
    ledcWrite(_ch1, duty);
    ledcWrite(_ch2, 0);
  }
  else
  {
    // Marche avant : PWM sur IN2
    uint16_t duty = (uint16_t)pwm;
    ledcWrite(_ch1, 0);
    ledcWrite(_ch2, duty);
  }
}

void Motor::setPwm(int pwm)
{
  applyPwmInternal(pwm);
}

void Motor::setPercent(float pct)
{
  if (pct > 100.0f)
    pct = 100.0f;
  if (pct < -100.0f)
    pct = -100.0f;

  int pwm = (int)(pct * 255.0f / 100.0f);
  applyPwmInternal(pwm);
}

void Motor::stop()
{
  // Arrêt en roue libre (LOW/LOW)
  ledcWrite(_ch1, 0);
  ledcWrite(_ch2, 0);
}

void Motor::brake()
{
  // Freinage actif : HIGH/HIGH
  ledcWrite(_ch1, MOTOR_PWM_MAX);
  ledcWrite(_ch2, MOTOR_PWM_MAX);
}

float Motor::getRps() const
{
  if (_enc == NULL)
    return 0.0f;

  // Le signe de la vitesse dépend uniquement du câblage encodeur.
  // On suppose qu'il est cohérent avec la convention PWM :
  // marche avant => rps > 0
  return _enc->getRps();
}
