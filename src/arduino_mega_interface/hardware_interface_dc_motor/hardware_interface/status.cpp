#include "status.hpp"

// Battery status

BatteryStatus::BatteryStatus(uint8_t pin_current,uint8_t pin_voltage): pin_current_(pin_current),pin_voltage_(pin_voltage) {
  pinMode(pin_voltage_, INPUT);
  pinMode(pin_current_, INPUT);
}

const char* BatteryStatus::serialize() const {
    static char buffer[50]; // Taille suffisante pour contenir les données sérialisées
    char voltageStr[10];
    char currentStr[10];
    char chargeLevelStr[10];

    // Convertir les floats en chaînes avec dtostrf
    dtostrf(voltage_, 5, 2, voltageStr);
    dtostrf(current_, 5, 2, currentStr);
    dtostrf(charge_level_, 5, 2, chargeLevelStr);

    // Remplacer les espaces initiaux par des zéros
    if (voltageStr[0] == ' ') {
        voltageStr[0] = '0';
    }
    if (currentStr[0] == ' ') {
        currentStr[0] = '0';
    }
    if (chargeLevelStr[0] == ' ') {
        chargeLevelStr[0] = '0';
    }

    // Construire la chaîne finale dans le buffer
    snprintf(buffer, sizeof(buffer), "st_batt,%s,%s,%s,%d;",
             voltageStr, currentStr, chargeLevelStr, charging_ ? 1 : 0);

    return buffer;
}

void BatteryStatus::updateVoltage() {
    // Lire la tension via le pin de tension
    int analog_value = analogRead(pin_voltage_);
    voltage_ = analog_value * voltage_resolution * voltage_factor;
}


void BatteryStatus::updateCurrent() {
    // Lire le courant via le pin de courant
    int analog_value = analogRead(pin_current_);
    float voltage = (analog_value * 5.0) / 1023.0;  // Conversion de la valeur brute en volts
    float offset_voltage = 2.5;  // Offset du capteur (ACS712 typiquement)
    float sensitivity = 0.185;   // Sensibilité en V/A (dépend du capteur utilisé)
    current_ = (voltage - offset_voltage) / sensitivity;
}

void BatteryStatus::updateChargeLevel() {
    // Mettre à jour le niveau de charge estimé (exemple basé sur une batterie 12V)
    float max_voltage = 12.6;  // Tension maximale de la batterie
    float min_voltage = 11.0;  // Tension minimale avant décharge complète
    charge_level_ = ((voltage_ - min_voltage) / (max_voltage - min_voltage)) * 100.0;
    if (charge_level_ < 0) charge_level_ = 0.0;  // Pas de charge négative
    if (charge_level_ > 100) charge_level_ = 100.0;  // Limite à 100%
}

void BatteryStatus::updateChargingStatus() {
    // Mettre à jour le statut de charge (exemple basé sur une condition simple)
    charging_ = current_ > 0.1;  // Si le courant est supérieur à un seuil, la batterie est en charge
}

void BatteryStatus::updateAll() {
    updateVoltage();
    updateCurrent();
    updateChargeLevel();
    updateChargingStatus();
}
// Motor status

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
