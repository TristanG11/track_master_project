#include "hardware_interface.hpp"

#define DPP 0.0009425
#define PPS 1061.03

HardwareInterface::HardwareInterface(): batt_status(BATTERY.voltage,BATTERY.current) {}

void HardwareInterface::addMotor(Motor* motor) {
    motors_[motor->getId()] = motor; // Add motor to the map
    vec_motors_.push_back(motor);
}


void HardwareInterface::setup() {
    for (auto& [key, motor] : motors_) {
        motor->setup(); // Call setup on each motor
    }
}
/*
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
            float speedValue = speedValueStr.toFloat() * PPS; // Convert speed to float

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
}*/

void HardwareInterface::handleCommand(const String& cmd) {
    // Example input: "0.5;0.6;-0.3;-0.4;"
    int start = 0;
    int end = cmd.indexOf(';'); // Trouve la première occurrence de ';'

    // Vérifier que la taille du vecteur correspond au nombre de commandes
    if (vec_motors_.size() == 0) {
        Serial.println("Error: No motors initialized.");
        return;
    }

    size_t motor_index = 0;

    while (end != -1 && motor_index < vec_motors_.size()) {
        // Extraire chaque valeur float (cmd)
        String cmd_value_str = cmd.substring(start, end);
        float cmd_value = cmd_value_str.toFloat(); // Convertir en float

        // Assigner directement la vitesse au moteur dans le vector
        vec_motors_[motor_index]->setSpeed(cmd_value * PPS); // Multiplier par PPS si nécessaire

        // Debug : Afficher la commande assignée
        Serial.print("Motor ");
        Serial.print(motor_index);
        Serial.print(" set to speed: ");
        Serial.println(cmd_value);

        // Mise à jour de l'index du moteur
        motor_index++;

        // Passer à la prochaine valeur
        start = end + 1;
        end = cmd.indexOf(';', start);
    }

    // Gestion des erreurs si moins ou plus de commandes que de moteurs
    if (motor_index < vec_motors_.size()) {
        Serial.println("Warning: Not all motors received a command.");
    } else if (end != -1) {
        Serial.println("Warning: Too many commands provided.");
    }
}


void HardwareInterface::run() {
    for (const auto& motor : vec_motors_) {
        //motor->AccelStepper::setSpeed(1500);
        motor->AccelStepper::runSpeed();
    }
}




void HardwareInterface::feedback() {
  
    String feedback;
    for (const auto& motor : vec_motors_) {
        feedback += motor->getId();                    // Append motor ID
        feedback += ',';                    // Separator
        feedback += motor->currentPosition() * DPP;      // Add position
        feedback += ',';                    // Separator
        feedback += motor->speed() * 1/PPS ;  // Add speed
        feedback += ';';                    // End of entry
        
    }

    Serial.println(feedback);

}



/*void HardwareInterface::feedback() {
    constexpr size_t BUFFER_SIZE = 256;  // Ajuster la taille selon vos besoins
    char feedback[BUFFER_SIZE];
    size_t index = 0;

    for (int i = 0; i < 4; ++i) {
        auto motor = vec_motors_[i];  // Récupérer le pointeur du moteur
        if (motor) {  // Vérifie si le pointeur est valide
            index += snprintf(&feedback[index], BUFFER_SIZE - index, "%s,%ld,%.2f;",
                              motor->getId().c_str(), motor->currentPosition(), motor->speed());

            // Prévenir un dépassement du buffer
            if (index >= BUFFER_SIZE - 1) {
                break;
            }
        }
    }

    // Le buffer est prêt, vous pouvez ici activer le Serial si nécessaire
    // Serial.println(feedback);
}*/


void HardwareInterface::sendStatus(){

  // Motors and battery status
  String statutes;
  
  for (const auto& motor : vec_motors_) {
    if (motor) {  // Vérifie si le pointeur est valide
    motor->updateStatus();
            statutes += motor->getStatus();
        }
  }
  batt_status.updateAll();
  statutes += batt_status.serialize();
  Serial.println(statutes);
}



