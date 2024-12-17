#include "hardware_interface.hpp"

HardwareInterface::HardwareInterface(): batt_status(BATTERY.voltage,BATTERY.current) {}

void HardwareInterface::addMotor(Motor* motor) {
    motors_[motor->getId()] = motor; // Add motor to the map
}


void HardwareInterface::setup() {
    for (auto& [key, motor] : motors_) {
        motor->setup(); // Call setup on each motor
    }
}

void HardwareInterface::handleCommand(const String& cmd) {
    // Example input: "front_left_wheel_joint,0.5;front_right_wheel_joint,-0.5;rear_left_wheel_joint,0.3;rear_right_wheel_joint,-0.3;"

    int start = 0;
    int end = cmd.indexOf(';'); // Find the first ';'

    while (end != -1) {
        // Extract each segment (e.g., "front_left_wheel_joint,0.5")
        String segment = cmd.substring(start, end);

        // Find the separator between motor ID and speed
        int separator = segment.indexOf(',');
        if (separator != -1) {
            // Extract motor ID and speed
            String motorId = segment.substring(0, separator); // e.g., "front_left_wheel_joint"
            String speedValueStr = segment.substring(separator + 1); // e.g., "0.5"
            float speedValue = speedValueStr.toFloat(); // Convert speed to float

            // Find the motor by ID and set the speed
            auto it = motors_.find(motorId.c_str());
            if (it != motors_.end()) {
                Motor* motor = it->second;
                motor->setSpeed(speedValue);
            } else {
                Serial.print(" Error : Motor ID not found: ");
                Serial.println(motorId);
            }
        } else {
            Serial.print("Error : Malformed segment: ");
            Serial.println(segment);
        }

        // Update start and find the next ';'
        start = end + 1;
        end = cmd.indexOf(';', start);
    }
}


void HardwareInterface::run(){
  for(auto& motor:motors_){
    motor.second->AccelStepper::runSpeed();
  }
}

void HardwareInterface::feedback() {
  
    String feedback;
    for (auto& [key, motor] : motors_) {
        feedback += key;                    // Append motor ID
        feedback += ',';                    // Separator
        feedback += String(motor->currentPosition(), 2);      // Add position
        feedback += ',';                    // Separator
        feedback += String(motor->speed(), 2);  // Add speed
        feedback += ';';                    // End of entry
        motor->updateStatus();
    }

    Serial.println(feedback);

}


void HardwareInterface::sendStatus(){

  // Motors and battery status
  String statutes;
  for (auto& [key, motor] : motors_) {
    if (motor) {  // Vérifie si le pointeur est valide
            statutes += motor->getStatus();
        }
  }
  batt_status.updateAll();
  statutes += batt_status.serialize();
  Serial.println(statutes);
}



