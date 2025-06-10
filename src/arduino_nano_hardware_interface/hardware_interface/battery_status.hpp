#ifndef BATTERY_STATUS_HPP
#define BATTERY_STATUS_HPP

#include <Arduino.h>
#include <stdint.h>

class BatteryStatus {
  public:
    float voltage_ = 0.0; // Measured battery voltage (in volts)
    uint8_t pin_voltage_; // Analog pin number used to read the battery voltage

  
    uint8_t batt_id_; // Unique battery identifier 
    BatteryStatus(uint8_t pin_voltage, uint8_t batt_id);
    void BatteryStatus::serialize(char* buffer, size_t len) const ; // Serializes the battery status into the provided buffer
    void updateVoltage();
};

#endif // BATTERY_STATUS_HPP
