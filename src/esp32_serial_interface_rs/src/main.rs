use msg_utils::msg::{ BatteryStatus, FourMotorsFeedback, FourMotorsStatus, WheelCommands};
use rclrs::{Publisher, Subscription};
use std::sync::{Arc, Mutex};

enum MessageType {
    Error(()),
    Feedback(()),
    Status(()),
    Problematic(()),
}

fn main() {
    // Initialize the serial port
    let serial_port = serialport::new("/dev/ttyUSB0", 115200)
        .timeout(std::time::Duration::from_secs(2))
        .open()
        .expect("Unable to open the serial port");

    let serial_port = Arc::new(Mutex::new(serial_port));

    // Initialize the ROS 2 context
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "esp32_serial_interface_node").unwrap();

    // Create a subscriber for the topic /cmd_vel_to_send
    let _subscription = node
        .create_subscription::<WheelCommands, _>(
            "/cmd_vel_to_send",
            rclrs::QOS_PROFILE_DEFAULT,
            {
                let serial_port = serial_port.clone();
                let buffer_size: u32 = 256; // Buffer size on Arduino
                let min_free_space: u32 = 64; // Minimum required space before writing
                let max_buffer_size: u32 = 64; // Maximum allowed send size
                move |msg: WheelCommands| {
                    let command = format!(
                        "<CMD=fl:{:.2};fr:{:.2};rl:{:.2};rr:{:.2}>",
                        msg.front_left_wheel_speed,
                        msg.front_right_wheel_speed,
                        msg.rear_left_wheel_speed,
                        msg.rear_right_wheel_speed
                    );

                    if let Ok(mut port) = serial_port.lock() {
                        // Check available space in the buffer before writing
                        match port.bytes_to_write() {
                            Ok(bytes_pending) => {
                                let space_available: i32 = buffer_size as i32 - bytes_pending as i32;
                                if bytes_pending > max_buffer_size {
                                    println!("Too many data in the send buffer. Ignoring the command.");
                                    return; // Ignore this command
                                }
                                if space_available >= min_free_space as i32 {
                                    // Write to the serial port
                                    match port.write(command.as_bytes()) {
                                        Ok(_) => { /* Command sent successfully */ }
                                        Err(e) => eprintln!("Error while sending command to the serial port: {}", e),
                                    }
                                } else {
                                    println!(
                                        "Insufficient space in the buffer. Available: {} bytes, Required: {} bytes",
                                        space_available, min_free_space
                                    );
                                }
                            }
                            Err(e) => eprintln!("Error while checking available buffer space: {}", e),
                        }
                    } else {
                        eprintln!("Error accessing the serial port");
                    }
                }
            },
        )
        .expect("Error creating the subscriber");

    // Create publishers
    let feedback_publisher = node
        .create_publisher::<FourMotorsFeedback>("/cmd_vel_feedback", rclrs::QoSProfile::default())
        .unwrap();
    let motor_status_publisher = node
        .create_publisher::<FourMotorsStatus>("/motor_status", rclrs::QoSProfile::default())
        .unwrap();
    let battery_status_publisher = node
        .create_publisher::<BatteryStatus>("/battery_status", rclrs::QoSProfile::default())
        .unwrap();

    let _t: std::thread::JoinHandle<()>;
    {
        let mut feedback_msg = FourMotorsFeedback::default();
        let (mut motors_status, mut battery_status) =
            (FourMotorsStatus::default(), BatteryStatus::default());
        let feedback_publisher = feedback_publisher.clone();
        let motor_status_publisher = motor_status_publisher.clone();
        let battery_status_publisher = battery_status_publisher.clone();
        let node = node.clone();

        _t = std::thread::spawn(move || {
            let mut buffer = Vec::new();
            loop {
                if let Ok(mut port) = serial_port.lock() {
                    let mut temp_buffer = [0; 256];
                    if let Ok(size) = port.read(&mut temp_buffer) {
                        buffer.extend_from_slice(&temp_buffer[..size]);

                        // Look for complete messages delimited by `<` and `>`
                        while let Some(start) = buffer.iter().position(|&b| b == b'<') {
                            if let Some(end) = buffer.iter().skip(start).position(|&b| b == b'>') {
                                // Extract the complete message between `<` and `>`
                                let end = start + end;
                                let message = buffer.drain(start..=end).collect::<Vec<_>>();
                                let message =
                                    String::from_utf8_lossy(&message[1..message.len() - 1])
                                        .to_string();
                                let (message_type, original_line) =
                                    determine_message_type(&message);

                                match message_type {
                                    MessageType::Error(()) => {
                                        eprintln!("Error detected: {}", original_line);
                                    }
                                    MessageType::Feedback(()) => {
                                        feedback_msg = parse_feedback(original_line); // Use the original line for parsing
                                        let now = node.get_clock().now().to_ros_msg().unwrap();
                                        feedback_msg.header.stamp.nanosec = now.nanosec;
                                        feedback_msg.header.stamp.sec = now.sec;
                                        feedback_publisher.publish(&feedback_msg).unwrap();
                                    }
                                    MessageType::Status(()) => {
                                        (motors_status, battery_status) =
                                            parse_status(original_line); // Use the original line for parsing
                                        let now = node.get_clock().now().to_ros_msg().unwrap();
                                        motors_status.header.stamp.nanosec = now.nanosec;
                                        motors_status.header.stamp.sec = now.sec;
                                        battery_status.header.stamp.sec = now.sec;
                                        battery_status.header.stamp.nanosec = now.nanosec;
                                        motor_status_publisher.publish(&motors_status).unwrap();
                                        battery_status_publisher.publish(&battery_status).unwrap();
                                    }
                                    MessageType::Problematic(()) => {
                                        println!("Received problematic message: {}", original_line);
                                    }
                                }
                            } else {
                                break; // No end of message yet, wait for more data
                            }
                        }
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10)); // 25 Hz
            }
        });
    }

    // Spin to keep the ROS 2 node active
    rclrs::spin(node).unwrap();
}

// Determine the type of message
fn determine_message_type(line: &str) -> (MessageType, &str) {
    if line.contains("Error") {
        (MessageType::Error(()), line)
    } else if line.starts_with("ST") {
        let trimmed_line = &line[3..]; // Remove the first 3 characters
        (MessageType::Status(()), trimmed_line)
    } else if line.starts_with("FB") {
        let trimmed_line = &line[3..]; // Remove the first 3 characters
        (MessageType::Feedback(()), trimmed_line)
    } else {
        (MessageType::Problematic(()), line)
    }
}

// Parse a feedback message
fn parse_feedback(line: &str) -> FourMotorsFeedback {
    let mut feedback_msg = FourMotorsFeedback::default();
    let segments: Vec<&str> = line.split(';').collect();
    for segment in segments {
        let parts: Vec<&str> = segment.split(',').collect();
        if parts.len() == 3 {
            if let (Some(motor_name), Some(position), Some(speed)) = (
                parts.first(),
                parts.get(1).and_then(|pos| pos.parse().ok()),
                parts.get(2).and_then(|spd| spd.parse().ok()),
            ) {
                match *motor_name {
                    "fl" => {
                        feedback_msg.motor_front_left.position = position;
                        feedback_msg.motor_front_left.speed = speed;
                    }
                    "fr" => {
                        feedback_msg.motor_front_right.position = position;
                        feedback_msg.motor_front_right.speed = speed;
                    }
                    "rl" => {
                        feedback_msg.motor_rear_left.position = position;
                        feedback_msg.motor_rear_left.speed = speed;
                    }
                    "rr" => {
                        feedback_msg.motor_rear_right.position = position;
                        feedback_msg.motor_rear_right.speed = speed;
                    }
                    _ => (),
                }
            }
        }
    }
    feedback_msg
}

// Parse a status message
fn parse_status(line: &str) -> (FourMotorsStatus, BatteryStatus) {
    let mut motors_status = FourMotorsStatus::default();
    let mut battery_status = BatteryStatus::default();
    let segments: Vec<&str> = line.split(';').collect();

    for segment in segments {
        if segment.contains("batt") {
            let parts: Vec<&str> = segment.split(',').collect();
            battery_status.voltage = parts.get(1).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            battery_status.current = parts.get(2).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            battery_status.charge_level = parts.get(3).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            battery_status.charging = parts.get(4).map(|v| *v == "1").unwrap_or(false);
        } else {
            let parts: Vec<&str> = segment.split(',').collect();
            let motor_name = parts.get(0).unwrap_or(&"");
            let current = parts.get(1).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let voltage = parts.get(2).and_then(|v| v.parse().ok()).unwrap_or(0.0);

            match *motor_name {
                "fl" => {
                    motors_status.motor_front_left.motor_name = String::from("front_left");
                    motors_status.motor_front_left.current = current;
                    motors_status.motor_front_left.voltage = voltage;
                }
                "fr" => {
                    motors_status.motor_front_right.motor_name = String::from("front_right");
                    motors_status.motor_front_right.current = current;
                    motors_status.motor_front_right.voltage = voltage;
                }
                "rl" => {
                    motors_status.motor_rear_left.motor_name = String::from("rear_left");
                    motors_status.motor_rear_left.current = current;
                    motors_status.motor_rear_left.voltage = voltage;
                }
                "rr" => {
                    motors_status.motor_rear_right.motor_name = String::from("rear_right");
                    motors_status.motor_rear_right.current = current;
                    motors_status.motor_rear_right.voltage = voltage;
                }
                _ => (),
            }
        }
    }
    (motors_status, battery_status)
}
