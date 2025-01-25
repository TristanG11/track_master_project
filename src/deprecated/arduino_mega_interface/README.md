# Stepper Motor Serial Control

This project provides an Arduino-based solution to control stepper motors through a serial interface. It allows receiving commands via serial communication, parsing those commands, and directing them to the appropriate motor.

## Features

- Control individual stepper motors through serial commands.
- Supports basic operations: setting speed, changing direction, and stopping.
- Modular design with:
  - A `DriverMotor` and `UnipolarMotor` class for individual stepper motor control.
  - An `HardwareInterface` class for managing multiple motors.
- Extendable to multiple motors.

## Hardware Requirements

- **Microcontroller**: Arduino Mega 2560
- **Motors**: Stepper motors compatible with the AccelStepper library.
- **Driver**: Motor drivers such as A4988 or DRV8825 or Unipolar motors 


## Software Requirements

- Arduino IDE 1.8+ or PlatformIO
- Libraries:
  - [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

## Commands Protocol

The serial commands follow a simple structure:

<MOTOR_ID> <VALUE>

## Usage

1. **Set up the hardware**:
   - Connect stepper motors to the appropriate pins as per your motor driver.
   - Ensure correct power supply to the motor drivers.

2. **Upload the code**:
   - Open `hardware_interface.ino` in Arduino IDE.
   - Compile and upload the code to the Arduino Mega.

3. **Send commands via Serial**:
   - Use the Arduino Serial Monitor or any serial communication tool.
   - Enter commands as per the protocol.

## Future Improvements

- Add support for different types of motors.
- Implement feedback from sensors for closed-loop control.
- Develop a GUI for sending commands more intuitively.

## Author

Developed by Tristan Guerick Agoumbi Ogandaga.
