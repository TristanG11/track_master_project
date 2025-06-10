# ESP32 Hardware Interface – Motor Controller

This project provides a **real-time motor control system** for an ESP32-based robotic platform written in **Rust** using `esp-idf`. It enables precise speed control for **DC motors with quadrature encoders**, using a **PID controller**, and provides an interface for **serial communication** with a PC or ROS 2 system.

---

## Key Features

**Closed-loop control** of 4 DC motors with PID (Proportional–Integral–Derivative) regulation.
* High-speed **UART communication** (230400 baud) for real-time command and feedback exchange.
**Quadrature encoder integration** using PCNT hardware peripheral.
**Real-time processing** at 20 Hz using multithreading.
* Feedback message generation for monitoring in real-time.
* PID tuning via serial commands.
* Modular architecture: `Motor`, `MotorController`, `Encoder`, `PID`, `State`.

---

## Architecture Overview

```
┌────────────┐       ┌──────────────┐       ┌────────────┐
│  Host PC   │ <---> │  UART Comm   │ <---> │  ESP32     │
└────────────┘       └──────────────┘       └────────────┘
                                            │  4x DC motors
                                            │  4x Encoders
                                            │  PID loop (20Hz)
```

---

## Serial Command Interface

* Set motor speeds:

  ```
  <CMD=fl:0.9;rl:1.6;>
  ```

* Update PID gains:

  ```
  <PID=fl:500.0,0.0,0.0;fr:400.0,10.0,0.0>
  ```

* Receive feedback (every 50ms):

  ```
  <FB=fl,2.34;fr,2.31;rl,2.40;rr,2.39;ts=50;>
  ```

---

## Hardware Requirements

* ESP32 (tested with ESP32-S3)
* 4x DC motors with encoders
* Motor drivers 
* UART connection to PC 

---

## Software Dependencies

Listed in `Cargo.toml`, key crates include:

* [`esp-idf-svc`](https://crates.io/crates/esp-idf-svc)
* [`esp-idf-hal`](https://crates.io/crates/esp-idf-hal)
* [`esp-idf-sys`](https://crates.io/crates/esp-idf-sys)
* `log`, `parking_lot`, `embuild`

---

## Project Structure

```bash
src/
├── main.rs             # Main application
├── motor.rs            # Motor struct and control logic
├── motor_controller.rs # Multi-motor management, command parsing
├── motor_pid.rs        # PID controller implementation
├── motor_pin.rs        # GPIO abstraction for PWM & DIR
├── motor_state.rs      # Speed/state representation
├── encoder.rs          # Quadrature encoder driver (PCNT)
```

---

## Build & Flash Instructions

1. Setup your ESP32 Rust toolchain:

   ```bash
   rustup show # make sure esp is installed
   ```
2. Build with:

   ```bash
   cargo build --release
   ```
3. Flash the board:

   ```bash
   cargo espflash /dev/ttyesp32
   ```

---

## Integration with ROS 2

* Easily integrate with ROS 2 using a serial bridge node in Python or Rust.
* Use the `FB=` messages for odometry or diagnostics.
* Send `CMD=` and `PID=` commands for real-time control.

---