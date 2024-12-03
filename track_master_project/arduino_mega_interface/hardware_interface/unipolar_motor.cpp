#include "unipolar_motor.hpp"

// Constructor
UnipolarMotor::UnipolarMotor(uint8_t IN1, uint8_t IN3, uint8_t IN2, uint8_t IN4, String id)
    : Motor(id,AccelStepper::FULL4WIRE,IN1,IN3,IN2,IN4), IN1_(IN1), IN2_(IN2), IN3_(IN3), IN4_(IN4) {}

// Setup the motor
void UnipolarMotor::setup() {
    pinMode(IN1_, OUTPUT);
    pinMode(IN2_, OUTPUT);
    pinMode(IN3_, OUTPUT);
    pinMode(IN4_, OUTPUT);
    AccelStepper::setMaxSpeed(1000);             // Set max speed
    
}
