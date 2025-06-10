# esp32\_serial\_interface\_rs

This Rust ROS 2 node manages serial communication between a host computer and an ESP32-based motor controller.

## Features

* Subscribes to `/cmd_vel_desired` (type: `WheelCommands`) and sends motor speed commands via UART.
* Subscribes to `/pid_gains` (type: `FourMotorsPid`) to update PID parameters on the ESP32.
* Publishes motor feedback to `/cmd_vel_feedback` (type: `FourMotorsFeedback`).
* Publishes diagnostic messages to `/diagnostics` (type: `DiagnosticArray`).
* Handles connection, disconnection, and errors with automatic reconnection attempts.

## Requirements

* ROS 2 and `rclrs`
* A connected ESP32 that understands the serial message format
* Custom message definitions provided in `msg_utils`

## Usage

Make sure the `esp32_serial_interface_node` binary is built and run with the correct ROS 2 parameters:

```bash
ros2 run esp32_serial_interface_rs esp32_serial_interface_node
```

You can configure parameters such as:

* `baud_rate` (default: 230400)
* `topic_cmd_vel_feedback`
* `topic_diagnostics`

These can be passed via a launch file or parameter YAML.

## Notes

* Serial device is expected at `/dev/esp32` by default.
* Handles feedback messages starting with `FB=` and PID/command updates using `<...>` formatting.
* Includes basic timestamp-based position estimation.
