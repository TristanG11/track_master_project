#include "Motor.hpp"

Motor::Motor(int dirPin, int enablePin, int encoderAPin, int encoderBPin, const char* id, uint8_t pinCurrent, uint8_t pinVoltage)
    : dirPin(dirPin), enablePin(enablePin), encoderAPin(encoderAPin), encoderBPin(encoderBPin),
      encoderTicks(0), position(0.0), speed(0.0), cmd(0.0), status(id, pinCurrent, pinVoltage) {
    strncpy(this->id, id, sizeof(this->id));
    this->id[sizeof(this->id) - 1] = '\0';
}

void Motor::init() {
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(encoderAPin, INPUT);
    pinMode(encoderBPin, INPUT);
    digitalWrite(enablePin, LOW); // Désactiver le moteur par défaut
    setDirection(true);           // Direction avant par défaut
}

void Motor::setDirection(bool forward) {
    digitalWrite(dirPin, forward ? HIGH : LOW);
}


void Motor::setDesiredSpeed(float desiredspeed) {
    desiredSpeed = desiredspeed;
}

float Motor::getDesiredSpeed() const {
    return desiredSpeed;
}

float Motor::computeSpeed() {
    speed = encoderTicks * RAD_PER_TICK / TIMER_FREQUENCY_SEC;
    return speed;
}

void Motor::setCmd(float _cmd) {
    if (_cmd < 0) {
        setDirection(false); // Direction arrière
        cmd = -_cmd;         // Prendre la valeur absolue de _cmd
    } else {
        setDirection(true);  // Direction avant
        cmd = _cmd;
    }
    analogWrite(enablePin, min((int)cmd, 255));
}

float Motor::computePosition() {
    position += speed * WHEEL_RADIUS * TIMER_FREQUENCY_SEC;
    return position;
}

long Motor::getEncoderTicks() const {
    noInterrupts();
    long ticks = encoderTicks;
    interrupts();
    return ticks;
}

void Motor::resetTicks() {
    encoderTicks = 0;
}

void Motor::handleEncoderA() {
    if (digitalRead(encoderAPin) == digitalRead(encoderBPin)) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

// Accesseurs
float Motor::getSpeed() const {
    return speed; // Retourne la vitesse mesurée
}

float Motor::getPosition() const {
    return position; // Retourne la position calculée
}
