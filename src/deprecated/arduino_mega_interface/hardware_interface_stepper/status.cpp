#include "status.hpp"

// Battery status

BatteryStatus::BatteryStatus(uint8_t pin_current,uint8_t pin_voltage): pin_current_(pin_current),pin_voltage_(pin_voltage) {
  pinMode(pin_voltage_, INPUT);
  pinMode(pin_current_, INPUT);
}

String BatteryStatus::serialize() const {
    // Sérialiser les données de la batterie dans une chaîne lisible
    return "status_battery," + String(voltage_, 2) + "," +
           String(current_, 2) + "," +
           String(charge_level_, 2) + "," +
           String(charging_ ? 1 : 0) + ";";
}

void BatteryStatus::updateVoltage() {
    // Lire la tension via le pin de tension
    int analog_value = analogRead(pin_voltage_);
    float voltage_resolution = 5.0 / 1023.0;  // Résolution ADC en volts
    float voltage_factor = 5.0;  // Facteur de division du module de détection
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

MotorStatus::MotorStatus(String name, uint8_t pin_current, uint8_t pin_voltage)
    : motor_name_(name), pin_current_(pin_current), pin_voltage_(pin_voltage) {
    pinMode(pin_voltage_, INPUT);
    pinMode(pin_current_, INPUT);
}

String MotorStatus::serialize() const {
    String name = motor_name_;
    
    if (motor_name_ == "front_left_wheel_joint") {
        name = "fl";
    } else if (motor_name_ == "front_right_wheel_joint") {
        name = "fr";
    } else if (motor_name_ == "rear_left_wheel_joint") {
        name = "rl";
    } else if (motor_name_ == "rear_right_wheel_joint") {
        name = "rr";
    }

    return "status_" + name + "," + String(desired_speed_, 2) + "," +
           String(speed_, 2) + "," + String(current_, 2) + "," + String(voltage_, 2) + ";";
}

void MotorStatus::updateVoltage() {
    int analog_value = analogRead(pin_voltage_); // Lire la tension brute
    voltage_ = analog_value * voltage_resolution * voltage_factor;
}
void MotorStatus::updateSpeed(float speed){
  speed_ = speed;
  desired_speed_ = speed;
}
void MotorStatus::updateCurrent() {
    int analog_value = analogRead(pin_current_); // Lire la tension brute du capteur
    float voltage = analog_value * voltage_resolution; // Convertir en volts
    current_ = (voltage - offset_voltage) / sensitivity; // Calculer le courant
}

void MotorStatus::updateAll(float speed) {
    updateVoltage(); // Mettre à jour la tension
    updateCurrent(); // Mettre à jour le courant
    updateSpeed(speed);
}
