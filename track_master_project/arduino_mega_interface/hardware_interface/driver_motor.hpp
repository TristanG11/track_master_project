#ifndef DRIVER_MOTOR_HPP
#define DRIVER_MOTOR_HPP

#include "motor_interface.hpp"

class DriverMotor : public Motor {
public:
    // Constructor
    DriverMotor(uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK, String id);

    // Setup the motor
    void setup() override;

private:
    uint8_t pinEnable_;
    uint8_t pinCW_;
    uint8_t pinCLK_;
};

#endif // DRIVER_MOTOR_HPP
