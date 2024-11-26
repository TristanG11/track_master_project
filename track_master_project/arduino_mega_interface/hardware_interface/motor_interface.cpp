#include "motor_interface.hpp"

// Constructor
Motor::Motor(String id, uint8_t interfaceType)
    : AccelStepper(interfaceType), speed_(0.0f), id_(id) {}

// Set motor speed
void Motor::setSpeed(float speed) {
    speed_ = speed;
    AccelStepper::setSpeed(speed_);
}

// Stop the motor
void Motor::stop() {
    AccelStepper::stop();
}

// Get current speed
float Motor::getSpeed() const {
    return speed_;
}

// Get current position
float Motor::getPosition() const {
    return currentPosition();
}

// Get motor ID
String Motor::getId() const {
    return id_;
}

// Set motor ID
void Motor::setId(String id) {
    id_ = id;
}

// Run the motor
void Motor::run() {
    AccelStepper::runSpeed();
}
