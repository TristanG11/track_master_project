#include "motor_interface.hpp"

// Constructor

Motor::Motor(String id, uint8_t interface_type, uint8_t IN1,uint8_t IN3, uint8_t IN2, uint8_t IN4,uint8_t pin_current,uint8_t pin_voltage):
AccelStepper(interface_type,IN1,IN3,IN2,IN4),id_(id),status(id,pin_current, pin_voltage){
  
}

Motor::Motor(String id, uint8_t interface_type,uint8_t pinEnable, uint8_t pinCW, uint8_t pinCLK,uint8_t pin_current,uint8_t pin_voltage):
AccelStepper(interface_type,pinEnable,pinCW,pinCLK),id_(id),status(id,pin_current, pin_voltage){
  
}

// Get motor ID
String Motor::getId() const {
    return id_;
}

// Set motor ID
void Motor::setId(String id) {
    id_ = id;
}

void Motor::updateStatus(){
  status.updateAll(speed());
}

String Motor::getStatus(){
  return status.serialize();
}