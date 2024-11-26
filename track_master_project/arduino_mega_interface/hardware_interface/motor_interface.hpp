#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include <Arduino.h>
#include <AccelStepper.h>
#include <stdint.h>

class Motor : public AccelStepper {
public:
    // Constructor
    Motor(String id, uint8_t interfaceType);

    // Setup the motor (to be specialized by derived classes)
    virtual void setup() = 0;

    // Set motor speed
    void setSpeed(float speed);

    // Stop the motor
    void stop();

    // Get current speed
    float getSpeed() const;

    // Get current position
    float getPosition() const;

    String getId() const;

    void setId(String id);

    void run();

protected:
    float speed_;       // Current speed
    String id_;         // Motor identifier
};

#endif // MOTOR_INTERFACE_HPP
