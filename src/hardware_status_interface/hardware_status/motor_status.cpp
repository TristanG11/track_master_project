#include "motor_status.hpp"

MotorStatus::MotorStatus(const char* name, uint8_t pin_current, uint8_t pin_voltage)
    : pin_current_(pin_current), pin_voltage_(pin_voltage) {
    strncpy(motor_name_, name, sizeof(motor_name_) - 1);
    motor_name_[sizeof(motor_name_) - 1] = '\0'; // Assurez-vous que le tableau est terminé par un '\0'
    pinMode(pin_voltage_, INPUT);
    pinMode(pin_current_, INPUT);
}

const char* MotorStatus::serialize() const {
    char currentStr[10], voltageStr[10];
    static char buffer[128]; 
    // Convertir les floats en chaînes avec dtostrf
    dtostrf(current_, 5, 2, currentStr);
    dtostrf(voltage_, 5, 2, voltageStr);

    // Remplacer les espaces initiaux par des zéros
    if (currentStr[0] == ' ') {
        currentStr[0] = '0';
    }
    if (voltageStr[0] == ' ') {
        voltageStr[0] = '0';
    }

    // Construire la chaîne finale
    snprintf(buffer, sizeof(buffer), "st_%s,%s,%s;", motor_name_, currentStr, voltageStr);

    return buffer; // Retourne un pointeur vers le tampon de l'instance
}



void MotorStatus::updateVoltage() {
    int analog_value = analogRead(pin_voltage_); // Lire la tension brute
    voltage_ = analog_value * voltage_resolution * voltage_factor;
}

void MotorStatus::updateCurrent() {
    int analog_value = analogRead(pin_current_); // Lire la tension brute du capteur
    float voltage = analog_value * voltage_resolution; // Convertir en volts
    current_ = (voltage - offset_voltage) / sensitivity; // Calculer le courant
}

void MotorStatus::updateAll() {
    updateVoltage(); // Mettre à jour la tension
    updateCurrent(); // Mettre à jour le courant
}
