#include "motor_interface.hpp"

// Constructor

Motor::Motor(String id, uint8_t interface_type, uint8_t IN1,uint8_t IN3, uint8_t IN2, uint8_t IN4):
AccelStepper(interface_type,IN1,IN3,IN2,IN4),id_(id){}

Motor::Motor(String id, uint8_t interface_type,uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK):
AccelStepper(interface_type,pinEnable,pinCW,pinCLK),id_(id){}

Motor::Motor(String id){}

// Get motor ID
String Motor::getId() const {
    return id_;
}

// Set motor ID
void Motor::setId(String id) {
    id_ = id;
}