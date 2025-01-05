#include <Arduino.h>
#include <stdint.h>

class BatteryStatus {
  private : 
    float voltage_ = 0.0;
    float current_ = 0.0;
    float charge_level_ = 0.0;
    bool charging_ = false;
    uint8_t pin_current_;
    uint8_t pin_voltage_;

  public : 
    BatteryStatus(uint8_t pin_current,uint8_t pin_voltage);
    const char* serialize() const ;
    void updateCurrent();
    void updateVoltage();
    void updateChargeLevel();
    void updateChargingStatus();
    void updateAll();
};

// Floats for resistor values in divider (in ohms)
const float R1 = 30000.0;
const float R2 = 7500.0; 
const float voltage_factor = (R1+R2)/R2;
const float voltage_resolution = 5.0 / 1023.0; // Résolution ADC (en volts)
const float offset_voltage = 2.5;              // Offset du capteur de courant (typique)
const float sensitivity = 0.185;               // Sensibilité du capteur (en V/A)

class MotorStatus {
private:
    char buffer[25];
    char motor_name_[3];
    float desired_speed_ = 0.0;
    float speed_ = 0.0;
    float current_ = 0.0;
    float voltage_ = 0.0;
    uint8_t pin_current_;
    uint8_t pin_voltage_;

public:
    // Constructeur
    MotorStatus(const char* name, uint8_t pin_current, uint8_t pin_voltage);

    // Sérialisation
    const char* serialize() const;

    // Méthodes de mise à jour
    void updateVoltage();
    void updateCurrent();
    void updateAll(); // Met à jour la tension et le courant

    // Accesseurs (si nécessaires)
    float getCurrent() const { return current_; }
    float getVoltage() const { return voltage_; }
};