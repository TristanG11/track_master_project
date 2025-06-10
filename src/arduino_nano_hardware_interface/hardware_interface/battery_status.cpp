#include "battery_status.hpp"

const float R1 = 30000.0;
const float R2 = 7500.0;
const float voltage_factor = (R1 + R2) / R2;
const float voltage_resolution = 5.0 / 1023.0;

BatteryStatus::BatteryStatus(uint8_t pin_voltage, uint8_t batt_id)
  : pin_voltage_(pin_voltage), batt_id_(batt_id) {
  pinMode(pin_voltage_, INPUT);
}

void BatteryStatus::updateVoltage() {
  int analog_value = analogRead(pin_voltage_);
  voltage_ = analog_value * voltage_resolution * voltage_factor;
}

void BatteryStatus::serialize(char* buffer, size_t len) const {

}

