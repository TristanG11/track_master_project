#include <Arduino.h>
#include <stdint.h>
#include "common.hpp"

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