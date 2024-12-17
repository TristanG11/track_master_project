#include "driver_motor.hpp"

// Constructor
DriverMotor::DriverMotor(uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK, String id,uint8_t pin_current,uint8_t pin_voltage)
    : Motor(id, AccelStepper::DRIVER, pinEnable, pinCW,pinCLK,pin_current,pin_voltage), pinEnable_(pinEnable), pinCW_(pinCW), pinCLK_(pinCLK) {}

// Setup the motor
void DriverMotor::setup() {
    pinMode(pinEnable_, OUTPUT);
    pinMode(pinCW_, OUTPUT);
    pinMode(pinCLK_, OUTPUT);
    digitalWrite(pinEnable_, LOW);  // Disable the motor initially
    setMaxSpeed(1000);             // Set max speed
    setAcceleration(500);          // Set acceleration
}
