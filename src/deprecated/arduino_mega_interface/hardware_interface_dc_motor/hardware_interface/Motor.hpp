#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include "status.hpp"

const float TICKS_PER_REV = 341.2;          // Impulsions par tour
const float RAD_PER_TICK = 2 * PI / TICKS_PER_REV; // Radians par impulsion
const float TIMER_FREQUENCY_SEC = 1;    // Période du timer en secondes
const float WHEEL_RADIUS = 0.06;           // Rayon de la roue (mètres)

class Motor {
private:
    int dirPin;         // Broche pour la direction
    int enablePin;      // Broche pour activer/désactiver
    int encoderAPin;    // Broche A de l'encodeur
    int encoderBPin;    // Broche B de l'encodeur

    volatile long encoderTicks;            // Nombre de ticks de l'encodeur
    volatile float position;               // Position en radians
    volatile float speed;                  // Vitesse mesurée en rad/s
    volatile float desiredSpeed;           // Consigne de vitesse en rad/s
    volatile float cmd;                    // Commande en PWM

public:
    char id[3];                          // Identifiant du moteur
    MotorStatus status;                    // Statut du moteur

    Motor(int dirPin, int enablePin, int encoderAPin, int encoderBPin,const char* id, uint8_t pinCurrent, uint8_t pinVoltage);

    void init();
    void setDirection(bool forward);
    void setDesiredSpeed(float desiredspeed);
    float getDesiredSpeed() const;

    void setCmd(float _cmd);
    float computeSpeed();
    float computePosition();
    long getEncoderTicks() const;
    void resetTicks();
    void handleEncoderA();

    // Fonctions accesseuses
    float getSpeed() const;      // Obtenir la vitesse mesurée
    float getPosition() const;   // Obtenir la position mesurée
};

#endif // MOTOR_HPP
