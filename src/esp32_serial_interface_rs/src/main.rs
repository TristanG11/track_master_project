use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use msg_utils::msg::{FourMotorsFeedback, WheelCommands};
use msg_utils::msg::{FourMotorsPid, SerialPorts};
use parking_lot::FairMutex;
use rclrs::MandatoryParameter;
use rust_utils::serial::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};
enum MessageType {
    Error(()),
    Feedback(()),
    //Status(()),
    Problematic(()),
}

fn main() {
    // Initialize the ROS 2 context
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "esp32_serial_interface_node").unwrap();

    // Parameters :
    let baud_rate: MandatoryParameter<i64> = node
        .declare_parameter("baud_rate")
        .default(115200)
        .mandatory()
        .unwrap();

    let topic_cmd_vel_feedback: MandatoryParameter<Arc<str>> = node
        .declare_parameter("topic_cmd_vel_feedback")
        .default(Arc::from("/cmd_vel_feedback"))
        .mandatory()
        .unwrap();

    let topic_diagnostics: MandatoryParameter<Arc<str>> = node
        .declare_parameter("topic_diagnostics")
        .default(Arc::from("/diagnostics"))
        .mandatory()
        .unwrap();

    // Create publishers
    let feedback_publisher = node
        .create_publisher::<FourMotorsFeedback>(
            &topic_cmd_vel_feedback.get(),
            rclrs::QoSProfile::default(),
        )
        .unwrap();

    let diag_publisher = node
        .create_publisher::<DiagnosticArray>(&topic_diagnostics.get(), rclrs::QoSProfile::default())
        .unwrap();

    let disconnected_flag = Arc::new(AtomicBool::new(true));
    //Diagnostics part :
    let (diag_tx, diag_rx) = mpsc::channel::<DiagnosticStatus>();

    let active_pid_update = Arc::new(AtomicBool::new(false));
    let diag_name: &'static str = "esp32_serial_interface_node";

    // Thread to process diagnostic messages
    {
        let node = node.clone();
        let mut diag_array = DiagnosticArray::default();
        std::thread::spawn(move || loop {
            while let Ok(diag) = diag_rx.try_recv() {
                diag_array.status.push(diag.clone());
            }
            let now = node.get_clock().now().to_ros_msg().unwrap();
            diag_array.header.stamp.sec = now.sec;
            diag_array.header.stamp.nanosec = now.nanosec;
            if !diag_array.status.is_empty() {
                diag_publisher.publish(diag_array.clone()).unwrap();
            }
            diag_array.status.clear();
            std::thread::sleep(Duration::from_millis(1000));
        });
    };

    let port_name_from_param = String::new();

    let serial_port: Arc<
        parking_lot::lock_api::Mutex<
            parking_lot::RawFairMutex,
            Option<Box<dyn serialport::SerialPort + 'static>>,
        >,
    > = Arc::new(parking_lot::FairMutex::new(None));
    // let mut last_name = String::from(port_name.clone());
    let port_name: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, String>> =
        Arc::new(FairMutex::new(String::new()));

    let _subscription = node
        .create_subscription::<SerialPorts, _>(
            "/available_serial_ports",
            rclrs::QoSProfile::default(),
            {
                let mut last_name = port_name_from_param.clone();
                let port_name = Arc::clone(&port_name);
                move |msg: SerialPorts| {
                    if last_name != msg.esp32_port {
                        println!("New port {}", msg.esp32_port);
                        if let Some(mut name) = port_name.try_lock_for(Duration::from_millis(100)) {
                            *name = msg.esp32_port.clone();
                            last_name = msg.esp32_port.clone();
                        }
                    }
                }
            },
        )
        .unwrap();

    // Thread for connexion /reconnexion
    spawn_reconnection_thread(
        serial_port.clone(),
        disconnected_flag.clone(),
        port_name.clone(),
        diag_tx.clone(),
        baud_rate.get(),
        diag_name.to_string(),
    );


    // Sender / receiver for serial_port_writing :

    let (cmd_tx, cmd_rx) = mpsc::channel::<String>();
    let (feedback_tx, feedback_rx) = mpsc::channel::<Vec<u8>>();

    // Create a subscriber for the topic /cmd_vel_to_send
    let _cmd_vel_subscription = node
        .create_subscription::<WheelCommands, _>("/cmd_vel_desired", rclrs::QOS_PROFILE_DEFAULT, {
            let cmd_tx = cmd_tx.clone();
            let active_pid_update = active_pid_update.clone();
            move |msg: WheelCommands| {
                let command = format!(
                    "<CMD=fl:{:.2};rl:{:.2}>", //"<CMD=fl:{:.2};fr:{:.2};rl:{:.2};rr:{:.2}>",
                    msg.front_left_wheel_speed, msg.rear_left_wheel_speed
                );
               // println!("{}",command);
                if !active_pid_update.load(Ordering::SeqCst) {
                    if let Err(e) = cmd_tx.send(command) {
                        println!("{}", e);
                    }
                }
            }
        })
        .expect("Error creating the subscriber");

    // thread for pid tuning

    let _pid_subscription = node.create_subscription::<FourMotorsPid, _>(
    "/pid_gains",
    rclrs::QOS_PROFILE_DEFAULT,
    {
        let active_pid_update = active_pid_update.clone();
        let cmd_tx = cmd_tx.clone();
        move |msg: FourMotorsPid| {
            // Formatting the PID command
            active_pid_update.store(true, std::sync::atomic::Ordering::SeqCst);  // 12H
            println!("active pid update set to true");

            let command = format!(
                "<PID=fl:{:.2},{:.2},{:.2};fr:{:.2},{:.2},{:.2};rl:{:.2},{:.2},{:.2};rr:{:.2},{:.2},{:.2}>",
                msg.motor_front_left.kp, msg.motor_front_left.ki, msg.motor_front_left.kd,
                msg.motor_front_right.kp, msg.motor_front_right.ki, msg.motor_front_right.kd,
                msg.motor_rear_left.kp, msg.motor_rear_left.ki, msg.motor_rear_left.kd,
                msg.motor_rear_right.kp, msg.motor_rear_right.ki, msg.motor_rear_right.kd
            );
            /*let command = format!(
                "<PID=fr:{:.2},{:.2},{:.2}>",
                msg.motor_front_right.kp, msg.motor_front_right.ki, msg.motor_front_right.kd,
            );*/

            for i in 0..5
            {
                if let Err(e) = cmd_tx.send(command.clone()){
                    println!("pid updates");
            }
            std::thread::sleep(Duration::from_secs(1));
            }
            active_pid_update.store(false, std::sync::atomic::Ordering::SeqCst);
        }
    },
    );

    // Thread for publishing feedback

    let _feedback_thread = {
        let node = node.clone();
        //let feedback_publisher = feedback_publisher.clone(); // ROS 2 publisher
        let diag_tx = diag_tx.clone(); // diagnostics publisher
       let mut feedback_msg = FourMotorsFeedback::default();
      std::thread::spawn(move || {
            //let mut last_feedback_rcv: Option<std::time::Instant> = None;

            while let Ok(mut buffer_feedback) = feedback_rx.recv() {
                //last_feedback_rcv = Some(std::time::Instant::now());

                // buffer
                /*let message =
                            String::from_utf8_lossy(&buffer_feedback[1..buffer_feedback.len() - 1]).to_string();
                            */
                            
                            
                while let Some(start) = buffer_feedback.iter().position(|&b| b == b'<') {
                    if let Some(end) = buffer_feedback.iter().skip(start).position(|&b| b == b'>') {
                        let end = start + end;
                        let message = buffer_feedback.drain(start..=end).collect::<Vec<_>>();
                        let message =
                            String::from_utf8_lossy(&message[1..message.len() - 1]).to_string();
                            println!("feed {}",message);
                        let (message_type, original_line) = determine_message_type(&message);

                        match message_type {
                            MessageType::Error(()) => {
                                /*last_error_status_rcv = Some(std::time::Instant::now());
                                eprintln!("Error detected: {}", original_line);
                                send_diagnostic(
                                    &diag_tx,
                                    DiagnosticStatus::ERROR,
                                    diag_name,
                                    format!("Error detected: {}", original_line),
                                    vec![KeyValue {
                                        key: "Timestamp".to_string(),
                                        value: format!("{:?}", last_error_status_rcv.unwrap()),
                                    }],
                                );*/
                            }
                            MessageType::Feedback(()) => {
                                let ts = parse_feedback(original_line, &mut feedback_msg); // ts est en millisecondes
                                let dt = ts as f64 / 1000.0; // convertit en secondes

                                feedback_msg.motor_front_left.position += feedback_msg.motor_front_left.speed * 0.06 * dt;
                                feedback_msg.motor_front_right.position += feedback_msg.motor_front_right.speed * 0.06 * dt;
                                feedback_msg.motor_rear_left.position   += feedback_msg.motor_rear_left.speed * 0.06 * dt;
                                feedback_msg.motor_rear_right.position  += feedback_msg.motor_rear_right.speed * 0.06  * dt;
                                let now = node.get_clock().now().to_ros_msg().unwrap();
                                //println!("{:?}",feedback_msg.clone());
                                feedback_msg.header.stamp.nanosec = now.nanosec;
                                feedback_msg.header.stamp.sec = now.sec;
                                if let Err(e) = feedback_publisher.publish(&feedback_msg) {
                                    eprintln!("{}", e);
                                }
                            }
                            MessageType::Problematic(()) => {
                                println!("Received problematic message: {}", original_line);
                                send_diagnostic(
                                    &diag_tx,
                                    DiagnosticStatus::WARN,
                                    diag_name,
                                    format!("Received problematic message: {}", original_line),
                                    vec![],
                                );
                            }
                        }
                    } else {
                        break;
                    }
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        })
    };

    // Thread for managing serial port

    let _serial_thread = {
        let feedback_tx = feedback_tx.clone();
        let serial_port = serial_port.clone();
        std::thread::spawn(move || {
            let mut temp_buffer = [0; 256];
            loop {
                //  Feedback
                let mut serial_error: Option<std::io::Error> = None;
                if let Some(mut port_guard) = serial_port.try_lock_for(Duration::from_millis(1)) {
                    if let Some(port) = &mut *port_guard {
                        // Read in port
                        match port.bytes_to_write() {
                            Ok(n) =>
                            {
                               // Write to port
                        if let Ok(command) = cmd_rx.recv_timeout(Duration::from_millis(1)) {
                            match port.write_all(command.as_bytes()) {
                                Ok(_) => {println!("cmd_sent_success");}
                                Err(e) => {
                                    // Send an error diagnostic
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::ERROR,
                                        diag_name,
                                        format!("Failed to send command: {}", e),
                                        vec![KeyValue {
                                            key: "Error".to_string(),
                                            value: e.to_string(),
                                        }],
                                    );
                                    serial_error = Some(e.into());
                                }
                            }
                        }
                            }
                            Err(e)=>
                            {
                                println!("{}",e);
                            }
                        }
                        

                         match port.read(&mut temp_buffer) {
                                    Ok(size) => {
                                        let mut buffer = Vec::new();
                                        buffer.extend_from_slice(&temp_buffer[..size]);
                                        {
                                            let message =
                            String::from_utf8_lossy(&buffer[1..buffer.len() - 1]).to_string();
                            println!("feed {}",message);
                                            //println!("size = {}",size);
                                            feedback_tx.send(buffer.clone()).unwrap();
                                        }
                                        
                                        //println!("feedbzck recv succ {:?}",buffer.to_ascii_lowercase());
                                    }
                                    Err(e) => {
                                        serial_error = Some(e);
                                    }
                        }

                        

                        // if there is an error
                        if let Some(e) = serial_error {
                            if !disconnected_flag.load(Ordering::SeqCst) {
                                handle_serial_error(e.into(), &diag_tx, diag_name, &disconnected_flag);
                            } else {
                                while !disconnected_flag.load(Ordering::SeqCst) {
                                    std::thread::sleep(Duration::from_secs(2));
                                }
                            }
                        }
                    }
                }
               // std::thread::sleep(Duration::from_millis(10));
            }
        })
    };

    // Spin to keep the ROS 2 node active
    rclrs::spin(node).unwrap();

}


// Determine the type of message
fn determine_message_type(line: &str) -> (MessageType, &str) {
    if line.contains("Error") {
        (MessageType::Error(()), line)
    } else if line.starts_with("FB") {
        let trimmed_line = &line[3..]; // Remove the first 3 characters
        (MessageType::Feedback(()), trimmed_line)
    } else {
        (MessageType::Problematic(()), line)
    }
}

// Parse a feedback message
fn parse_feedback(line: &str, feedback_msg: &mut FourMotorsFeedback)-> u64 {
    // Supprimer le préfixe "FB=" si présent
    let line = line.strip_prefix("FB=").unwrap_or(line);
    let mut elapsed_s = 0;
    let segments: Vec<&str> = line.split(';').filter(|s| !s.is_empty()).collect();

    for segment in segments {
        // Champ ts ?
        if segment.starts_with("ts=") {
            if let Some(ts_str) = segment.strip_prefix("ts=") {
                if let Ok(ts_ms) = ts_str.parse::<u64>() {
                    elapsed_s = ts_ms;
                }
            }
            continue;
        }

        // Données moteur : "fl,0.00"
        let parts: Vec<&str> = segment.split(',').collect();
        if parts.len() == 2 {
            let motor_name = parts[0].trim();
            if let Ok(speed) = parts[1].trim().parse::<f64>() {
                match motor_name {
                    "fl" => feedback_msg.motor_front_left.speed = speed,
                    "fr" => feedback_msg.motor_front_right.speed = speed,
                    "rl" => feedback_msg.motor_rear_left.speed = speed,
                    "rr" => feedback_msg.motor_rear_right.speed = speed,
                    _ => (), // ignorer les moteurs inconnus
                }
            }
        }
    }
    elapsed_s
}

/*
// thread for sending commmands
    {
        let buffer_size: u32 = 512; // Buffer size on ESP32
        let min_free_space: u32 = 128; // Minimum required space before writing
        let max_buffer_size: u32 = 128; // Maximum allowed send size
        let diag_tx = diag_tx.clone();
        let serial_port = serial_port.clone();
        let disconnected_flag = disconnected_flag.clone();
        let mut last_sending_time: Option<std::time::Instant> = None;
        std::thread::spawn(move || loop {
            let mut serial_error: Option<serialport::Error> = None;
            match rx.try_recv() {
                Ok(command) => {
                    if let Some(mut port) = serial_port.try_lock_for(Duration::from_millis(50)) {
                        // Checking the available buffer space before writing
                        if let Some(port) = &mut *port {
                            match port.bytes_to_write() {
                                Ok(bytes_pending) => {
                                    let space_available: i32 =
                                        buffer_size as i32 - bytes_pending as i32;
                                    if bytes_pending > max_buffer_size {
                                        println!("Too many data in the send buffer. Ignoring the command.");
                                        send_diagnostic(
                                            &diag_tx,
                                            DiagnosticStatus::WARN,
                                            diag_name,
                                            "Send buffer is full".to_string(),
                                            vec![KeyValue {
                                                key: "Bytes Pending".to_string(),
                                                value: bytes_pending.to_string(),
                                            }],
                                        );
                                    }
                                    if space_available >= min_free_space as i32 {
                                        // Writing to the serial port
                                        match port.write_all(command.as_bytes()) {
                                            Ok(_) => {
                                                println!("cmd sent suceesful");
                                                last_sending_time = Some(Instant::now());
                                                // Send a success diagnostic
                                               // println!("cmd_sent successfully");
                                                send_diagnostic(
                                                    &diag_tx,
                                                    DiagnosticStatus::OK,
                                                    diag_name,
                                                    "Command Sent Successfully".to_string(),
                                                    vec![
                                                        KeyValue {
                                                            key: "Last Sending Time".to_string(),
                                                            value: format!(
                                                                "{:?} ms",
                                                                last_sending_time
                                                                    .unwrap()
                                                                    .elapsed()
                                                                    .as_millis()
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
                                                // Send an error diagnostic
                                                send_diagnostic(
                                                    &diag_tx,
                                                    DiagnosticStatus::ERROR,
                                                    diag_name,
                                                    format!(
                                                        "Failed to send command: {}",
                                                        e
                                                    ),
                                                    vec![KeyValue {
                                                        key: "Error".to_string(),
                                                        value: e.to_string(),
                                                    }],
                                                );
                                                serial_error = Some(e.into());
                                            }
                                        }
                                    } else {
                                        // Send a diagnostic for insufficient buffer space
                                        send_diagnostic(
                                            &diag_tx,
                                            DiagnosticStatus::WARN,
                                            diag_name,
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
                                    serial_error = Some(e.clone());
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::ERROR,
                                        diag_name,
                                        format!(
                                            "Error checking available buffer space: {}",
                                            e.description.clone()
                                        ),
                                        vec![KeyValue {
                                            key: "Error".to_string(),
                                            value: e.description.clone(),
                                        }],
                                    );
                                }
                            }
                        }
                    } else {
                        eprintln!("Error accessing the serial port from sender thread");
                        // Send a diagnostic for the serial port access error
                        send_diagnostic(
                            &diag_tx,
                            DiagnosticStatus::ERROR,
                            diag_name,
                            "Failed to access the serial port".to_string(),
                            vec![],
                        );
                    }
                }
                Err(e) => {
                    eprintln!("Error accessing rx sender");
                    send_diagnostic(
                        &diag_tx,
                        DiagnosticStatus::ERROR,
                        diag_name,
                        format!("Error detected: {}", e),
                        vec![KeyValue {
                            key: "Timestamp".to_string(),
                            value: format!("{:?}", e),
                        }],
                    );
                }
            }
            if let Some(e) = serial_error {
                println!("6");
                if !disconnected_flag.load(Ordering::SeqCst)
                {
                    println!("4");
                    handle_serial_error(e, &diag_tx, diag_name, &disconnected_flag);
                }else {
                    while !disconnected_flag.load(Ordering::SeqCst) {
                        std::thread::sleep(Duration::from_secs(2));
                        println!("5");
                    }
                }

            }
            std::thread::sleep(Duration::from_millis(20));
        })
    };

    {
        let node = node.clone();
        let mut last_feedback_rcv: Option<std::time::Instant> = None;
        let mut last_error_status_rcv: Option<std::time::Instant> = None;
        let diag_tx = diag_tx.clone();
        let disconnected_flag = disconnected_flag.clone();
        std::thread::spawn(move || {
            loop {
                let mut temp_buffer = [0; 256];
                let mut serial_error: Option<std::io::Error> = None;
                let size_opt = {
                    if let Some(mut port_guard) = serial_port.try_lock_for(Duration::from_millis(20)) {
                        if let Some(port) = &mut *port_guard {
                            match port.read(&mut temp_buffer) {
                                Ok(size) => {
                                    println!("feedbzck recv succ");
                                    Some(size)},
                                Err(e) => {
                                    serial_error = Some(e);
                                    None
                                }
                            }
                        } else {
                            None
                        }
                    } else {
                        eprintln!("Error accessing the serial port from feedback thread");
                        // Send a diagnostic for the serial port access error
                        send_diagnostic(
                            &diag_tx,
                            DiagnosticStatus::ERROR,
                            diag_name,
                            "Failed to access the serial port".to_string(),
                            vec![],
                        );
                        None
                    }
                };
                if let Some(e) = serial_error {
                    println!("1");
                    if !disconnected_flag.load(Ordering::SeqCst)
                    {
                        println!("2");
                        handle_serial_error(e.into(), &diag_tx, diag_name, &disconnected_flag);
                    }else {
                        while !disconnected_flag.load(Ordering::SeqCst) {
                            std::thread::sleep(Duration::from_secs(2));
                            println!("3");
                        }

                    }
                };

                if let Some(size) = size_opt {
                    let mut buffer = Vec::new();
                    buffer.extend_from_slice(&temp_buffer[..size]);

                    while let Some(start) = buffer.iter().position(|&b| b == b'<') {
                        if let Some(end) = buffer.iter().skip(start).position(|&b| b == b'>') {
                            let end = start + end;
                            let message = buffer.drain(start..=end).collect::<Vec<_>>();
                            let message =
                                String::from_utf8_lossy(&message[1..message.len() - 1]).to_string();
                            let (message_type, original_line) = determine_message_type(&message);

                            match message_type {
                                MessageType::Error(()) => {
                                    last_error_status_rcv = Some(std::time::Instant::now());
                                    eprintln!("Error detected: {}", original_line);
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::ERROR,
                                        diag_name,
                                        format!("Error detected: {}", original_line),
                                        vec![KeyValue {
                                            key: "Timestamp".to_string(),
                                            value: format!("{:?}", last_error_status_rcv.unwrap()),
                                        }],
                                    );

                                }
                                MessageType::Feedback(()) => {
                                    let mut feedback_msg = parse_feedback(original_line); // Use the original line for parsing
                                    last_feedback_rcv = Some(std::time::Instant::now());
                                    let now = node.get_clock().now().to_ros_msg().unwrap();
                                    feedback_msg.header.stamp.nanosec = now.nanosec;
                                    feedback_msg.header.stamp.sec = now.sec;
                                    if let Err(e) = feedback_publisher.publish(&feedback_msg) {
                                        eprintln!("{}", e);
                                    }
                                }
                                MessageType::Problematic(()) => {
                                    println!("Received problematic message: {}", original_line);
                                    send_diagnostic(
                                        &diag_tx,
                                        DiagnosticStatus::WARN,
                                        diag_name,
                                        format!("Received problematic message: {}", original_line),
                                        vec![],
                                    );
                                }
                            }
                        } else {
                            break;
                        }
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(25));
            }
        });
    }

*/
