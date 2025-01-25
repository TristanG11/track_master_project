#include <Arduino.h>
#include <stdint.h>
#include "common.hpp"

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

