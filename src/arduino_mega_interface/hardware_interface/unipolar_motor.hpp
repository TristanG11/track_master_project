#ifndef UNIPOLAR_MOTOR_HPP
#define UNIPOLAR_MOTOR_HPP

#include "motor_interface.hpp"

class UnipolarMotor : public Motor {
public:
    // Constructor
    UnipolarMotor(uint8_t IN1, uint8_t IN3, uint8_t IN2, uint8_t IN4, String id);

    // Setup the motor
    void setup() override;

private:
    uint8_t IN1_;
    uint8_t IN2_;
    uint8_t IN3_;
    uint8_t IN4_;
};

#endif // UNIPOLAR_MOTOR_HPP
