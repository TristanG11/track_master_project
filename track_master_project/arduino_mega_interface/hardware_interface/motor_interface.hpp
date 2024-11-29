#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include <Arduino.h>
#include <AccelStepper.h>
#include <stdint.h>

class Motor : public AccelStepper {
public:
    // Constructor
    Motor(String id);
    Motor(String id, uint8_t interface_type, uint8_t IN1,uint8_t IN2, uint8_t IN3, uint8_t IN4);
    Motor(String id, uint8_t interface_type,uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK);
    // Setup the motor (to be specialized by derived classes)
    virtual void setup() = 0;

    String getId() const;
    void setId(String id);


protected:
    String id_;         // Motor identifier
};

#endif // MOTOR_INTERFACE_HPP
