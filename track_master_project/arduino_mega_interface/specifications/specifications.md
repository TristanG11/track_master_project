# Stepper Motor Serial Control - Specifications

This document defines the technical specifications for the stepper motor control system via serial communication.

## Project Objective

To control multiple stepper motors via an Arduino Mega using serial commands. The system parses incoming commands, assigns them to the appropriate motor, and executes the desired actions.

## Functional Requirements

### **Core Features**
1. **Serial Command Parsing**:
   - Accept commands in the format `<MOTOR_ID> <VALUE>`.
   - Parse the command to identify the target motor, the desired action, and the parameter value.

2. **Motor Control**:
   - Adjust the motor's speed.
   - Move to a specified position.
   - Stop the motor immediately.

3. **Multi-Motor Support**:
   - Manage up to 4 motors (can be extended as needed).
   - Ensure non-blocking operations for all motors.

---

## Technical Specifications

### **Hardware Requirements**
1. **Microcontroller**: Arduino Mega 2560.
2. **Motors**: Stepper motors with drivers like A4988 or DRV8825.
3. **Pin Assignments**:
   - Each motor requires 3 pins (EN, CW, CLK).

### **Software Design**

#### **Motor Class**
A reusable class for controlling individual stepper motors.
- **Constructor Parameters**:
  - `enablePin`: Pin for enabling/disabling the motor.
  - `cwPin`: Pin for setting the motor direction.
  - `clkPin`: Pin for sending clock pulses.
- **Methods**:
  - `setup()`: Initialize the motor pins.
  - `setSpeed(int speed)`: Set the motor speed.
  - `moveTo(int position)`: Move the motor to a specific position.
  - `stop()`: Stop the motor.

#### **MotorsManager Class**
Manages multiple `Motor` instances and parses commands.
- **Attributes**:
  - `Motor motors[]`: Array of `Motor` objects.
- **Methods**:
  - `addMotor(Motor motor)`: Add a motor to the system.
  - `processCommand(String command)`: Parse and execute a serial command.

#### **Serial Communication**
- Baud rate: `9600`.
- Protocol: Simple text commands.

---

## Command Details

| Command Format            | Example       | Description                          |
|---------------------------|---------------|--------------------------------------|
| `<MOTOR_ID> SPEED <VALUE>`| `1 SPEED 500` | Set motor 1 speed to 500 steps/sec.  |
| `<MOTOR_ID> POS <VALUE>`  | `2 POS 1000`  | Move motor 2 to position 1000.       |
| `<MOTOR_ID> STOP`         | `3 STOP`      | Stop motor 3 immediately.            |

---

## Implementation Details

### **StepMotorControl.ino**
- Initializes the serial interface and sets up the motors.
- Continuously listens for serial input.
- Delegates command execution to `MotorsManager`.

### **Motor.hpp & Motor.cpp**
- Provides an abstraction for individual motor control.
- Wraps `AccelStepper` functionalities.

### **MotorsManager.hpp & MotorsManager.cpp**
- Handles parsing and distribution of commands to the appropriate motor.


## Test Cases

### Unit Tests
1. **Motor Class**:
   - Verify that `setSpeed()` adjusts speed correctly.

2. **MotorsManager Class**:
   - Ensure that commands are correctly parsed and assigned to motors.
   - Test error handling for invalid commands.

### Integration Tests
1. Send serial commands for multiple motors and verify execution.
2. Simulate invalid commands and ensure proper handling.

---

## Performance Constraints

- Maximum number of motors: 6.
- Maximum serial input rate: 9600 baud.
- Motor speed range: -1000 to 1000 steps/sec.

---

## Future Extensions

1. Add support for new motor types or drivers.
2. Incorporate feedback systems (e.g., encoders).
3. Develop wireless control via Bluetooth or Wi-Fi.
