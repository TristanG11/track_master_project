#include <Arduino.h>
#include "battery_status.hpp"

BatteryStatus PowerBatt(A0, 0x01);
BatteryStatus LogicalBatt(A1, 0x02);

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT); // 5V ref
}

void loop() {
  char buffer[32];
  PowerBatt.updateVoltage(); LogicalBatt.updateVoltage();
  char v1[10], v2[10];
  dtostrf(PowerBatt.voltage_, 4, 2, v1); 
  dtostrf(LogicalBatt.voltage_, 4, 2, v2);
  snprintf(buffer, 32, "<ID:%d,V:%s;ID:%d,V:%s>", PowerBatt.batt_id_, v1, LogicalBatt.batt_id_, v2);
  Serial.println(buffer);
  delay(1000);
}