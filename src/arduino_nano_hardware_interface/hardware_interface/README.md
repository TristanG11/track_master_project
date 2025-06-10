# Battery Voltage Monitoring with Arduino

This project allows you to **monitor the voltage of two batteries** using analog inputs on an Arduino board. It reads the voltages, formats them into a structured message, and sends the result via **Serial communication** for external monitoring or logging.

---

## Features

* Reads analog voltage from **two separate batteries**.
* Computes and converts the voltage to a readable format.
* Sends the battery status as a **formatted Serial message** every second.
* Uses a simple and extensible `BatteryStatus` class.

---

## Hardware Setup

| Battery       | Analog Pin | Battery ID |
| ------------- | ---------- | ---------- |
| Power Battery | A0         | 0x01       |
| Logic Battery | A1         | 0x02       |

---

## How It Works

1. The `BatteryStatus` class handles:

   * Reading the voltage from an analog pin.
   * Storing the battery ID and voltage.
   * Optionally serializing the data (extendable).
2. In the main loop:

   * Each battery is updated with a new voltage reading.
   * The values are formatted using `dtostrf`.
   * A string like `<ID:1,V:12.34;ID:2,V:11.89>` is printed to the Serial monitor every second.

---

## Serial Output Format

```
<ID:1,V:12.34;ID:2,V:11.89>
```

* `ID:x`: Battery identifier
* `V:xx.xx`: Battery voltage in volts

---

## Dependencies

* Arduino board (Uno, Nano)
* Arduino IDE
* Proper voltage divider circuits

---

## File Structure

* `battery_status.hpp`: Class definition for battery monitoring.
* `hardware_interface.ino`: Main Arduino sketch (setup and loop functions).

---
