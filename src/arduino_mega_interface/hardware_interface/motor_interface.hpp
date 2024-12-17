#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include <Arduino.h>
#include <AccelStepper.h>
#include <stdint.h>
#include "status.hpp"

class Motor : public AccelStepper {
public:
    // Constructor
    Motor(String id, uint8_t interface_type, uint8_t IN1,uint8_t IN2, uint8_t IN3, uint8_t IN4,uint8_t pin_current,uint8_t pin_voltage);
    Motor(String id, uint8_t interface_type,uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK,uint8_t pin_current,uint8_t pin_voltage);
    // Setup the motor (to be specialized by derived classes)
    virtual void setup() = 0;

    String getId() const;
    void setId(String id);
    void updateStatus();
    String getStatus();


protected:
    String id_;         // Motor identifier
    MotorStatus status;

};

#endif // MOTOR_INTERFACE_HPP
