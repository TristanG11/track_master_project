use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use msg_utils::msg::FourMotorsPid;
use msg_utils::msg::{FourMotorsFeedback, WheelCommands};
use parking_lot::FairMutex;
use rclrs::*;
use rust_utils::serial::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;
enum MessageType {
    Error(()),
    Feedback(()),
    //Status(()),
    Problematic(()),
}
 
    
fn main() {
    // Initialize the ROS 2 context
    let context = rclrs::Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("esp32_serial_interface_node").unwrap();

    // Parameters :
    let baud_rate: MandatoryParameter<i64> = node
        .declare_parameter("baud_rate")
        .default(230400)
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
   // let worker = node.create_worker(options)
    // Create publishers
    let binding = topic_cmd_vel_feedback.get();
    let topic_name: &str = &binding.as_ref();
    let feedback_publisher = node
        .create_publisher::<FourMotorsFeedback>(
            topic_name
            .keep_last(10)
            .transient_local()
        )
        .unwrap();

    let binding = topic_diagnostics.get();
    let topic_name: &str = &binding.as_ref();
    let diag_publisher = node
        .create_publisher::<DiagnosticArray>(
            topic_name
            .keep_last(10)
            .transient_local()
        )
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

    let serial_port: Arc<
        parking_lot::lock_api::Mutex<
            parking_lot::RawFairMutex,
            Option<Box<dyn serialport::SerialPort + 'static>>,
        >,
    > = Arc::new(parking_lot::FairMutex::new(None));

    let port_name: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, String>> =
        Arc::new(FairMutex::new(String::from("/dev/esp32")));

    // Thread for connexion /reconnexion
    spawn_reconnection_thread(
        serial_port.clone(),
        disconnected_flag.clone(),
        port_name.clone(),
        diag_tx.clone(),
        baud_rate.get(),
        diag_name.to_string(),
    );

    //let worker = node.create_worker::<usize>(0);
    // Sender / receiver for serial_port_writing :

    let (cmd_tx, cmd_rx) = mpsc::channel::<String>();

    // Create a subscriber for the topic /cmd_vel_to_send
    let _cmd_vel_subscription = node
        .create_subscription::<WheelCommands, _>("/cmd_vel_desired", {
            let cmd_tx = cmd_tx.clone();
            let active_pid_update = active_pid_update.clone();
            move |msg: WheelCommands| {
                let command = format!(
                    "<CMD=fl:{:.2};fr:{:.2}>", //"<CMD=fl:{:.2};fr:{:.2};rl:{:.2};rr:{:.2}>",
                    msg.front_left_wheel_speed, msg.front_right_wheel_speed
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


            for _i in 0..5
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
        
        let mut buffer_feedback = [0; 256];
        let serial_port = serial_port.clone();
        let node = node.clone();
        //let feedback_publisher = feedback_publisher.clone(); // ROS 2 publisher
        let diag_tx = diag_tx.clone(); // diagnostics publisher
        let mut feedback_msg = FourMotorsFeedback::default();
        let disconnected_flag = disconnected_flag.clone();

        std::thread::spawn(move || {
            //let mut last_feedback_rcv: Option<std::time::Instant> = None;
            loop {
                std::thread::sleep(Duration::from_millis(100)); // petite pause avant nouvelle tentative
                
                let mut reader =
                {
                    let port_guard = serial_port.lock();
                    match &*port_guard {
                    Some(p) => match p.try_clone() {
                        Ok(clone) => clone,
                        Err(e) => {
                            eprintln!("Failed to clone serial port: {}", e);
                            continue;
                        }
                    },
                    None => continue,
                }
                }; 
                

                loop {

                    //println!("reader");
                    let mut serial_error: Option<std::io::Error> = None;
                    match reader.read(&mut buffer_feedback) {
                        Ok(size) => {
                            let mut temp_buffer = Vec::new();
                            temp_buffer.extend_from_slice(&buffer_feedback[..size]);
                            while let Some(start) = temp_buffer.iter().position(|&b| b == b'<') {
                                if let Some(end) =
                                    temp_buffer.iter().skip(start).position(|&b| b == b'>')
                                {
                                    let end = start + end;
                                    let message =
                                        temp_buffer.drain(start..=end).collect::<Vec<_>>();
                                    let message =
                                        String::from_utf8_lossy(&message[1..message.len() - 1])
                                            .to_string();
                                    let (message_type, original_line) =
                                        determine_message_type(&message);

                                    match message_type {
                                        MessageType::Error(()) => {
                                            let last_error_status_rcv =
                                                Some(std::time::Instant::now());
                                            eprintln!("Error detected: {}", original_line);
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::ERROR,
                                                diag_name,
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
                                            let ts =
                                                parse_feedback(original_line, &mut feedback_msg); // ts est en millisecondes
                                            let dt = ts as f64 / 1000.0; // convertit en secondes

                                            feedback_msg.motor_front_left.position +=
                                                feedback_msg.motor_front_left.speed * 0.06 * dt;
                                            feedback_msg.motor_front_right.position +=
                                                feedback_msg.motor_front_right.speed * 0.06 * dt;
                                            feedback_msg.motor_rear_left.position +=
                                                feedback_msg.motor_rear_left.speed * 0.06 * dt;
                                            feedback_msg.motor_rear_right.position +=
                                                feedback_msg.motor_rear_right.speed * 0.06 * dt;
                                            let now = node.get_clock().now().to_ros_msg().unwrap();
                                            feedback_msg.header.stamp.nanosec = now.nanosec;
                                            feedback_msg.header.stamp.sec = now.sec;
                                            if let Err(e) =
                                                feedback_publisher.publish(&feedback_msg)
                                            {
                                                eprintln!("{}", e);
                                            }
                                        }
                                        MessageType::Problematic(()) => {
                                            /*println!(
                                                "Received problematic message: {}",
                                                original_line
                                            );
                                            send_diagnostic(
                                                &diag_tx,
                                                DiagnosticStatus::WARN,
                                                diag_name,
                                                format!(
                                                    "Received problematic message: {}",
                                                    original_line
                                                ),
                                                vec![],
                                            );*/
                                        }
                                    }
                                } else {
                                    break;
                                }
                            }
                        }

                        Err(e) => {
                            serial_error = Some(e);
                        }
                    }
                    if let Some(e) = serial_error {
                        if !disconnected_flag.load(Ordering::SeqCst) {
                            {
                                let mut guard = serial_port.lock();
                                *guard = None;
                           } 
                            handle_serial_error(e.into(), &diag_tx, diag_name, &disconnected_flag);
                            break;
                        } else {
                            while !disconnected_flag.load(Ordering::SeqCst) {
                                std::thread::sleep(Duration::from_secs(2));
                            }
                            break;
                        }
                    }
                    //std::thread::sleep(Duration::from_millis(10));
                }
            }
        })
    };

    // Thread for managing serial port

    let _sender_thread = {
        let serial_port = serial_port.clone();

        std::thread::spawn(move || {
            
            let disconnected_flag = disconnected_flag.clone();
            let diag_tx = diag_tx.clone();

            loop {
                std::thread::sleep(Duration::from_millis(100)); // petite pause avant nouvelle tentative
                let mut writer =
                {
                    let port_guard = serial_port.lock();
                    match &*port_guard {
                    Some(p) => match p.try_clone() {
                        Ok(clone) => clone,
                        Err(e) => {
                            eprintln!("Failed to clone serial port: {}", e);
                            continue;
                        }
                    },
                    None => continue,
                }
                }; 

                loop {

                    let mut serial_error: Option<std::io::Error> = None;
                    match writer.bytes_to_write() {
                        Ok(_) => { // try to see if there is enough in buffer
                            // Write to port
                            if let Ok(command) = cmd_rx.recv() {
                                //Duration::from_millis(1)
                                match writer.write_all(command.as_bytes()) {
                                    Ok(_) => { /*println!("cmd_sent_success");*/ }
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
                                        serial_error = Some(e);
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            println!("{}", e);
                            serial_error = Some(e.into());
                        }
                    }

                    // if there is an error
                    if let Some(e) = serial_error {
                        if !disconnected_flag.load(Ordering::SeqCst) {
                           {
                                let mut guard = serial_port.lock();
                                *guard = None;
                           } 
                            handle_serial_error(e.into(), &diag_tx, diag_name, &disconnected_flag);
                            break;
                        } else {
                            while !disconnected_flag.load(Ordering::SeqCst) {
                                std::thread::sleep(Duration::from_secs(2));
                            }
                            break;
                        }
                    }
                }
            }
        })
    };

    // Spin to keep the ROS 2 node active
    executor.spin(SpinOptions::default());
    //rclrs::spin(node).unwrap();
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
fn parse_feedback(line: &str, feedback_msg: &mut FourMotorsFeedback) -> u64 {

    let line = line.strip_prefix("FB=").unwrap_or(line);
    let mut elapsed_s = 0;
    let segments: Vec<&str> = line.split(';').filter(|s| !s.is_empty()).collect();

    for segment in segments {
        if segment.starts_with("ts=") {
            if let Some(ts_str) = segment.strip_prefix("ts=") {
                if let Ok(ts_ms) = ts_str.parse::<u64>() {
                    elapsed_s = ts_ms;
                }
            }
            continue;
        }

        let parts: Vec<&str> = segment.split(',').collect();
        if parts.len() == 2 {
            let motor_name = parts[0].trim();
            if let Ok(speed) = parts[1].trim().parse::<f64>() {
                match motor_name {
                    "fl" => feedback_msg.motor_front_left.speed = speed,
                    "fr" => feedback_msg.motor_front_right.speed = speed,
                    "rl" => feedback_msg.motor_rear_left.speed = speed,
                    "rr" => feedback_msg.motor_rear_right.speed = speed,
                    _ => (), // ignore unknown 
                }
            }
        }
    }
    elapsed_s
}