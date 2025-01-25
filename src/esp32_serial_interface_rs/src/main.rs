use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use msg_utils::msg::{
    BatteryStatus, FourMotorsFeedback, FourMotorsPid, FourMotorsStatus, WheelCommands,
};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
enum MessageType {
    Error(()),
    Feedback(()),
    Status(()),
    Problematic(()),
}

fn main() {
    // Initialize the ROS 2 context
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "esp32_serial_interface_node").unwrap();

    //Diagnostics part :
    let (diag_tx, diag_rx) = mpsc::channel::<DiagnosticStatus>();

    // Initialize the serial port
    let serial_port = {
        let mut serial_port = None;
        let mut count = 0;

        while count < 5 {
            match serialport::new("/dev/ttyUSB0", 115200)
                .timeout(Duration::from_secs(2))
                .open()
            {
                Ok(port) => {
                    serial_port = Some(port);
                    break;
                }
                Err(e) => {
                    eprintln!(
                        "Failed to open serial port: {} (Attempt {}/{})",
                        e,
                        count + 1,
                        5
                    );
                    count += 1;
                    std::thread::sleep(Duration::from_secs(10)); // Wait 10 seconds before retrying
                }
            }
        }

        if serial_port.is_none() {
            // If all attempts failed, send a diagnostic message
            let mut diag_status = DiagnosticStatus::default();
            diag_status.level = DiagnosticStatus::ERROR;
            diag_status.name = "Serial Port Initialization".to_string();
            diag_status.message = "Failed to open the serial port after 5 attempts.".to_string();
            diag_status.hardware_id = "".to_string();
            diag_status.values.push(KeyValue {
                key: "Attempts".to_string(),
                value: "5".to_string(),
            });
            diag_status.values.push(KeyValue {
                key: "Delay (secs)".to_string(),
                value: "10".to_string(),
            });
            loop {
                if let Err(e) = diag_tx.send(diag_status.to_owned()) {
                    eprintln!("{}", e);
                }
                std::thread::sleep(Duration::from_secs(10));
            }
        } else {
            let diag_status = DiagnosticStatus {
                level: DiagnosticStatus::OK,
                name: "Serial port state".to_string(),
                message: "Port is open and functionning normally".to_string(),
                hardware_id: "".to_string(),
                values: vec![
                    KeyValue {
                        key: "Last sending time".to_string(),
                        value: "No timestamp".to_string(),
                    },
                    KeyValue {
                        key: "Last received time".to_string(),
                        value: "No timestamp".to_string(),
                    },
                ],
            };
            if let Err(e) = diag_tx.send(diag_status) {
                eprintln!("{}", e);
            }
        }
        serial_port
    };

    let serial_port = Arc::new(Mutex::new(serial_port));

    // Create a subscriber for the topic /cmd_vel_to_send
    let _cmd_vel_subscription = node
        .create_subscription::<WheelCommands, _>(
            "/cmd_vel_to_send",
            rclrs::QOS_PROFILE_DEFAULT,
            {
                let serial_port = serial_port.clone();
                let buffer_size: u32 = 256; // Buffer size on ESP32
                let min_free_space: u32 = 64; // Minimum required space before writing
                let max_buffer_size: u32 = 64; // Maximum allowed send size
                let diag_tx = diag_tx.clone();
                let mut last_sending_time: Option<std::time::Instant> = None;
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
                        if let Some(port) = &mut *port {
                        match port.bytes_to_write() {
                            Ok(bytes_pending) => {
                                let space_available: i32 = buffer_size as i32 - bytes_pending as i32;
                                if bytes_pending > max_buffer_size {
                                    println!("Too many data in the send buffer. Ignoring the command.");
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::WARN,
                                        "Command Ignored".to_string(),
                                        "Send buffer is full".to_string(),
                                        vec![KeyValue {
                                            key: "Bytes Pending".to_string(),
                                            value: bytes_pending.to_string(),
                                        }],
                                    );
                                    return; // Ignore this command
                                }
                                if space_available >= min_free_space as i32 {
                                    // Write to the serial port
                                   /*  match port.write_all(command.as_bytes()) {
                                        Ok(_) => { /* Command sent successfully */
                                            last_sending_time = Some(Instant::now());
                                            // Send diagnostic for successful command
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::OK,
                                                "Command Sent".to_string(),
                                                "Command sent to serial port successfully"
                                                    .to_string(),
                                                vec![
                                                    KeyValue {
                                                        key: "Last Sending Time".to_string(),
                                                        value: format!(
                                                            "{:?} ms",
                                                            last_sending_time.unwrap().elapsed().as_millis()
                                                        ),
                                                    },
                                                    KeyValue {
                                                        key: "Command".to_string(),
                                                        value: command.clone(),
                                                    },
                                                ],
                                            );

                                        }
                                        Err(e) => {
                                            eprintln!("Error while sending command to the serial port: {}", e);
                                            // Send diagnostic for failed command
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::ERROR,
                                                "Command Send Failed".to_string(),
                                                format!(
                                                    "Failed to send command: {}",
                                                    e.to_string()
                                                ),
                                                vec![KeyValue {
                                                    key: "Error".to_string(),
                                                    value: e.to_string(),
                                                }],
                                            );}
                                    }*/
                                } else {
                                    println!(
                                        "Insufficient space in the buffer. Available: {} bytes, Required: {} bytes",
                                        space_available, min_free_space
                                    );
                                    // Send diagnostic for insufficient buffer space
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::WARN,
                                        "Insufficient Buffer Space".to_string(),
                                        "Not enough space in the serial buffer".to_string(),
                                        vec![
                                            KeyValue {
                                                key: "Available Space".to_string(),
                                                value: space_available.to_string(),
                                            },
                                            KeyValue {
                                                key: "Required Space".to_string(),
                                                value: min_free_space.to_string(),
                                            },
                                        ],
                                    );
                                }
                            }
                            Err(e) => {
                                eprintln!("Error while checking available buffer space: {}", e);
                                send_diagnostic(
                                    &diag_tx,
                                    DiagnosticStatus::ERROR,
                                    "Buffer Check Failed".to_string(),
                                    format!(
                                        "Error checking available buffer space: {}",
                                        e.to_string()
                                    ),
                                    vec![KeyValue {
                                        key: "Error".to_string(),
                                        value: e.to_string(),
                                    }],
                                );
                            }
                        }
                    }
                    } else {
                        eprintln!("Error accessing the serial port");
                        // Send diagnostic for port access error
                        send_diagnostic(
                            &diag_tx,
                            DiagnosticStatus::ERROR,
                            "Serial Port Access Failed".to_string(),
                            "Failed to access the serial port".to_string(),
                            vec![],
                        );
                    }
                }
            },
        )
        .expect("Error creating the subscriber");

    let _pid_subscription = node.create_subscription::<FourMotorsPid, _>(
            "/pid_gains",
            rclrs::QOS_PROFILE_DEFAULT,
            {
                let serial_port = serial_port.clone();
                let buffer_size: u32 = 256; // Buffer size on ESP32
                let min_free_space: u32 = 64; // Minimum required space before writing
                let max_buffer_size: u32 = 64; // Maximum allowed send size
                let diag_tx = diag_tx.clone();
                let mut last_sending_time: Option<std::time::Instant> = None;
                move |msg: FourMotorsPid| {
                    // Formatting the PID command
                    let command = format!(
                        "<PID=fl:{:.2},{:.2},{:.2};fr:{:.2},{:.2},{:.2};rl:{:.2},{:.2},{:.2};rr:{:.2},{:.2},{:.2}>",
                        msg.motor_front_left.kp, msg.motor_front_left.ki, msg.motor_front_left.kd,
                        msg.motor_front_right.kp, msg.motor_front_right.ki, msg.motor_front_right.kd,
                        msg.motor_rear_left.kp, msg.motor_rear_left.ki, msg.motor_rear_left.kd,
                        msg.motor_rear_right.kp, msg.motor_rear_right.ki, msg.motor_rear_right.kd
                    );
                    if let Ok(mut port) = serial_port.lock() {
                        // Checking the available buffer space before writing
                        if let Some(port) = &mut *port {
                            match port.bytes_to_write() {
                                Ok(bytes_pending) => {
                                    let space_available: i32 = buffer_size as i32 - bytes_pending as i32;
                                    if bytes_pending > max_buffer_size {
                                        println!("Too many data in the send buffer. Ignoring the command.");
                                        send_diagnostic(
                                            &diag_tx,
                                            DiagnosticStatus::WARN,
                                            "Command Ignored".to_string(),
                                            "Send buffer is full".to_string(),
                                            vec![KeyValue {
                                                key: "Bytes Pending".to_string(),
                                                value: bytes_pending.to_string(),
                                            }],
                                        );
                                        return; // Ignore this command
                                    }
                                    if space_available >= min_free_space as i32 {
                                        // Writing to the serial port
                                        match port.write_all(command.as_bytes()) {
                                            Ok(_) => {
                                                println!("sent : {}",command);
                                                last_sending_time = Some(Instant::now());
                                                // Send a success diagnostic
                                                send_diagnostic(
                                                    &diag_tx,
                                                    DiagnosticStatus::OK,
                                                    "PID Command Sent".to_string(),
                                                    "PID command sent to serial port successfully"
                                                        .to_string(),
                                                    vec![
                                                        KeyValue {
                                                            key: "Last Sending Time".to_string(),
                                                            value: format!(
                                                                "{:?} ms",
                                                                last_sending_time.unwrap().elapsed().as_millis()
                                                            ),
                                                        },
                                                        KeyValue {
                                                            key: "Command".to_string(),
                                                            value: command.clone(),
                                                        },
                                                    ],
                                                );
                                            }
                                            Err(e) => {
                                                eprintln!(
                                                    "Error while sending PID command to the serial port: {}",
                                                    e
                                                );
                                                // Send an error diagnostic
                                                send_diagnostic(
                                                    &diag_tx,
                                                    DiagnosticStatus::ERROR,
                                                    "PID Command Send Failed".to_string(),
                                                    format!(
                                                        "Failed to send PID command: {}",
                                                        e.to_string()
                                                    ),
                                                    vec![KeyValue {
                                                        key: "Error".to_string(),
                                                        value: e.to_string(),
                                                    }],
                                                );
                                            }
                                        }
                                    } else {
                                        println!(
                                            "Insufficient space in the buffer. Available: {} bytes, Required: {} bytes",
                                            space_available, min_free_space
                                        );
                                        // Send a diagnostic for insufficient buffer space
                                        send_diagnostic(
                                            &diag_tx,
                                            DiagnosticStatus::WARN,
                                            "Insufficient Buffer Space".to_string(),
                                            "Not enough space in the serial buffer".to_string(),
                                            vec![
                                                KeyValue {
                                                    key: "Available Space".to_string(),
                                                    value: space_available.to_string(),
                                                },
                                                KeyValue {
                                                    key: "Required Space".to_string(),
                                                    value: min_free_space.to_string(),
                                                },
                                            ],
                                        );
                                    }
                                }
                                Err(e) => {
                                    eprintln!("Error while checking available buffer space: {}", e);
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::ERROR,
                                        "Buffer Check Failed".to_string(),
                                        format!(
                                            "Error checking available buffer space: {}",
                                            e.to_string()
                                        ),
                                        vec![KeyValue {
                                            key: "Error".to_string(),
                                            value: e.to_string(),
                                        }],
                                    );
                                }
                            }
                        }
                    } else {
                        eprintln!("Error accessing the serial port");
                        // Send a diagnostic for the serial port access error
                        send_diagnostic(
                            &diag_tx,
                            DiagnosticStatus::ERROR,
                            "Serial Port Access Failed".to_string(),
                            "Failed to access the serial port".to_string(),
                            vec![],
                        );
                    }
                }
            },
        );

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
    let diag_publisher = node
        .create_publisher::<DiagnosticArray>("/diagnostics", rclrs::QoSProfile::default())
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
        let mut last_feedback_rcv: Option<std::time::Instant> = None;
        let mut last_status_rcv: Option<std::time::Instant> = None;
        let mut last_error_status_rcv: Option<std::time::Instant> = None;
        let diag_tx = diag_tx.clone();

        _t = std::thread::spawn(move || {
            let mut buffer = Vec::new();
            loop {
                if let Ok(mut port) = serial_port.lock() {
                    let mut temp_buffer = [0; 256];
                    if let Some(port) = &mut *port {
                        if let Ok(size) = port.read(&mut temp_buffer) {
                            buffer.extend_from_slice(&temp_buffer[..size]);

                            // Look for complete messages delimited by `<` and `>`
                            while let Some(start) = buffer.iter().position(|&b| b == b'<') {
                                if let Some(end) =
                                    buffer.iter().skip(start).position(|&b| b == b'>')
                                {
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
                                            last_error_status_rcv = Some(std::time::Instant::now());
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::ERROR,
                                                "Error Message Received".to_string(),
                                                format!("Error detected: {}", original_line),
                                                vec![KeyValue {
                                                    key: "Timestamp".to_string(),
                                                    value: format!(
                                                        "{:?}",
                                                        last_error_status_rcv.unwrap()
                                                    ),
                                                }],
                                            );
                                        }
                                        MessageType::Feedback(()) => {
                                            feedback_msg = parse_feedback(original_line); // Use the original line for parsing
                                            last_feedback_rcv = Some(std::time::Instant::now());
                                            let now = node.get_clock().now().to_ros_msg().unwrap();
                                            feedback_msg.header.stamp.nanosec = now.nanosec;
                                            feedback_msg.header.stamp.sec = now.sec;
                                            if let Err(e) =
                                                feedback_publisher.publish(&feedback_msg)
                                            {
                                                eprintln!("{}", e);
                                            }
                                        }
                                        MessageType::Status(()) => {
                                            (motors_status, battery_status) =
                                                parse_status(original_line, &feedback_msg); // Use the original line for parsing
                                            let now = node.get_clock().now().to_ros_msg().unwrap();
                                            motors_status.header.stamp.nanosec = now.nanosec;
                                            motors_status.header.stamp.sec = now.sec;
                                            battery_status.header.stamp.sec = now.sec;
                                            battery_status.header.stamp.nanosec = now.nanosec;
                                            if let Err(e) =
                                                motor_status_publisher.publish(&motors_status)
                                            {
                                                eprintln!("{}", e);
                                            }
                                            if let Err(e) =
                                                battery_status_publisher.publish(&battery_status)
                                            {
                                                eprintln!("{}", e);
                                            }
                                            last_status_rcv = Some(std::time::Instant::now());
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::OK,
                                                "Motor Status Received".to_string(),
                                                "Motor status successfully received and published."
                                                    .to_string(),
                                                vec![KeyValue {
                                                    key: "Last Motor Status Timestamp".to_string(),
                                                    value: format!(
                                                        "{:?} ms",
                                                        last_status_rcv
                                                            .unwrap()
                                                            .elapsed()
                                                            .as_millis()
                                                    ),
                                                }],
                                            );
                                        }
                                        MessageType::Problematic(()) => {
                                            println!(
                                                "Received problematic message: {}",
                                                original_line
                                            );
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::WARN,
                                                "Problematic Message".to_string(),
                                                format!(
                                                    "Received problematic message: {}",
                                                    original_line
                                                ),
                                                vec![],
                                            );
                                        }
                                    }
                                } else {
                                    break; // No end of message yet, wait for more data
                                }
                            }
                        }
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10)); // 25 Hz
            }
        });
    }

    // Thread to process diagnostic messages
    let _diagnostics_thread = {
        let mut diag_array = DiagnosticArray::default();
        let node = node.clone();
        std::thread::spawn(move || loop {
            while let Ok(diag) = diag_rx.try_recv() {
                diag_array.status.push(diag.clone());
            }
            diag_publisher.publish(diag_array.clone());
            std::thread::sleep(Duration::from_millis(1000));
        })
    };

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
        if parts.len() == 4 {
            if let (Some(motor_name), Some(position), Some(speed), Some(desired_speed)) = (
                parts.first(),
                parts.get(1).and_then(|pos| pos.parse().ok()),
                parts.get(2).and_then(|spd| spd.parse().ok()),
                parts.get(3).and_then(|des| des.parse().ok()),
            ) {
                match *motor_name {
                    "fl" => {
                        feedback_msg.motor_front_left.position = position;
                        feedback_msg.motor_front_left.speed = speed;
                        feedback_msg.motor_front_left.desired_speed = desired_speed;
                    }
                    "fr" => {
                        feedback_msg.motor_front_right.position = position;
                        feedback_msg.motor_front_right.speed = speed;
                        feedback_msg.motor_front_right.desired_speed = desired_speed;
                    }
                    "rl" => {
                        feedback_msg.motor_rear_left.position = position;
                        feedback_msg.motor_rear_left.speed = speed;
                        feedback_msg.motor_rear_left.desired_speed = desired_speed;
                    }
                    "rr" => {
                        feedback_msg.motor_rear_right.position = position;
                        feedback_msg.motor_rear_right.speed = speed;
                        feedback_msg.motor_rear_right.desired_speed = desired_speed;
                    }
                    _ => (),
                }
            }
        }
    }
    feedback_msg
}

// Parse a status message
fn parse_status(line: &str, feedback: &FourMotorsFeedback) -> (FourMotorsStatus, BatteryStatus) {
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
            let motor_name = parts.first().unwrap_or(&"");
            let current = parts.get(1).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let voltage = parts.get(2).and_then(|v| v.parse().ok()).unwrap_or(0.0);

            match *motor_name {
                "fl" => {
                    motors_status.motor_front_left.motor_name = String::from("front_left");
                    motors_status.motor_front_left.current = current;
                    motors_status.motor_front_left.voltage = voltage;
                    motors_status.motor_front_left.speed = feedback.motor_front_left.speed;
                }
                "fr" => {
                    motors_status.motor_front_right.motor_name = String::from("front_right");
                    motors_status.motor_front_right.current = current;
                    motors_status.motor_front_right.voltage = voltage;
                    motors_status.motor_front_right.speed = feedback.motor_front_right.speed;
                }
                "rl" => {
                    motors_status.motor_rear_left.motor_name = String::from("rear_left");
                    motors_status.motor_rear_left.current = current;
                    motors_status.motor_rear_left.voltage = voltage;
                    motors_status.motor_rear_left.speed = feedback.motor_rear_left.speed;
                }
                "rr" => {
                    motors_status.motor_rear_right.motor_name = String::from("rear_right");
                    motors_status.motor_rear_right.current = current;
                    motors_status.motor_rear_right.voltage = voltage;
                    motors_status.motor_rear_right.speed = feedback.motor_rear_right.speed;
                }
                _ => (),
            }
        }
    }
    (motors_status, battery_status)
}

fn send_diagnostic(
    diag_tx: &mpsc::Sender<DiagnosticStatus>,
    level: u8,
    name: String,
    message: String,
    values: Vec<KeyValue>,
) {
    let diag_status = DiagnosticStatus {
        level,
        name,
        message,
        hardware_id: "".to_string(),
        values,
    };
    if let Err(e) = diag_tx.send(diag_status) {
        eprintln!("Failed to send diagnostic: {}", e);
    }
}
