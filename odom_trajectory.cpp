#include <Arduino.h>
#include "Encoder.hpp"

// --- PARAMS À MODIFIER SELON TON ROBOT ---
float R = 0.033;          // rayon roue (m)
long ticksPerRev = 360;   // ticks par tour encodeur
float wheelBase = 0.18;   // entraxe roues (m)


float x = 0, y = 0, theta = 0;

enum State { 
    STEP1_FORWARD, // étape 1 : avance de
    STEP2_TURN,  // étape 2 : 
    STEP3_FORWARD, // étape 3 : 
    DONE // fin
};

State state = STEP1_FORWARD;

float startX, startY, startTheta;

// fonctions

void updateOdometry() {
    long left = Encoder::getLeftVal();
    long right = Encoder::getRightVal();

    static long prevL = left;
    static long prevR = right;

    long dL = left - prevL;
    long dR = right - prevR;

    prevL = left;
    prevR = right;

    float distL = dL * (2 * PI * R) / ticksPerRev;
    float distR = dR * (2 * PI * R) / ticksPerRev;

    float d = (distL + distR) / 2.0;
    float dtheta = (distR - distL) / wheelBase;

    x += d * cos(theta);
    y += d * sin(theta);
    theta += dtheta;
}

// à faire jsp les frères 
void setMotors(float leftSpeed, float rightSpeed) {
}

void stopMotors() {
    setMotors(0, 0);
}

void updateTrajectory() {
    updateOdometry();

    switch(state) {

    case STEP1_FORWARD: { // étape 1 : avance de 50cm
        float dist = sqrt(pow(x - startX, 2) + pow(y - startY, 2));
        setMotors(0.2, 0.2);

        if(dist >= 0.50) {
            stopMotors();
            startTheta = theta;
            state = STEP2_TURN;
        }
        break;
    }

    case STEP2_TURN: { // étape 2 : tourne de 90°
        float dtheta = fabs(theta - startTheta);
        setMotors(0.15, -0.15);

        if(dtheta >= PI/2) { // 90°
            stopMotors();
            startX = x;
            startY = y;
            state = STEP3_FORWARD;
        }
        break;
    }

    case STEP3_FORWARD: { // étape 3 : avance de 25cm
        float dist = sqrt(pow(x - startX, 2) + pow(y - startY, 2));
        setMotors(0.2, 0.2);

        if(dist >= 0.25) {
            stopMotors();
            state = DONE;
        }
        break;
    }

    case DONE:
        stopMotors();
        break;
    }
}
