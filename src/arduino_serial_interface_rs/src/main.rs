use rclrs;
use rclrs::{Node, Publisher, Subscription};
use serialport;
use std::sync::{Arc, Mutex};
use msg_utils::msg::{self, BatteryStatus, FourMotorsFeedback, FourMotorsStatus, WheelCommands};

enum MessageType {
    Error(String),
    Feedback(String),
    Status(String),
    Problematic(String)
}

fn main() {
    // Initialisation du port série
    let serial_port = serialport::new("/dev/ttyUSB0", 500_000)
        .timeout(std::time::Duration::from_secs(2))
        .open()
        .expect("Impossible d'ouvrir le port série");

    let serial_port = Arc::new(Mutex::new(serial_port));

    // Initialisation du contexte ROS 2
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let mut node = rclrs::create_node(&context, "arduino_serial_interface_node").unwrap();
    
    // Création du subscriber pour le topic /cmd_vel_to_send    

    let _subscription= node
    .create_subscription::<WheelCommands,_>(
        "/cmd_vel_desired",
        rclrs::QOS_PROFILE_DEFAULT,
        {
        let serial_port = serial_port.clone();
        let buffer_size: u32 = 256; // Taille du tampon sur Arduino
        let min_free_space: u32 = 64; // Espace minimum requis avant d'écrires
        let max_buffer_size: u32 = 64; // Taille maximale autorisée pour l'envoi
        move |msg: WheelCommands| {

            let command = format!(
                "fl:{:.2};fr:{:.2};rl:{:.2};rr:{:.2}\n",
                msg.front_left_wheel_speed,
                msg.front_right_wheel_speed,
                msg.rear_left_wheel_speed,
                msg.rear_right_wheel_speed
            );

            if let Ok(mut port) = serial_port.lock() {
                // Vérifier l'espace disponible dans le tampon avant d'écrire
                match port.bytes_to_write() {
                    
                    Ok(bytes_pending) => {
                        //println!("{}",bytes_pending);
                        let space_available: i32 = buffer_size as i32 - bytes_pending as i32 ;
                        if bytes_pending as u32 > max_buffer_size {
                            println!(
                                "Trop de données dans le tampon d'envoi. Ignorer la commande."
                            );
                            return; // Ignorer cette commande
                        }
                        if space_available >= min_free_space as i32{
                            // Écrire dans le port série
                            match port.write(command.as_bytes()) {
                                Ok(_) => {println!("Commande envoyée : {}", command)} //
                                Err(e) => eprintln!("Erreur lors de l'envoi de la commande au port série : {}", e),
                            }
                        } else {
                            println!(
                                "Espace insuffisant dans le tampon. Disponible : {} bytes, Requis : {} bytes",
                                space_available, min_free_space
                            );
                        }
                    }
                    Err(e) => eprintln!("Erreur lors de la vérification de l'espace disponible : {}", e),
                }
            } else {
                eprintln!("Erreur lors de l'accès au port série");
            }
        }
    }
    )
    .expect("Erreur lors de la création du subscriber");


    // Création des publishers
    let feedback_publisher = node.create_publisher::<FourMotorsFeedback>("/cmd_vel_feedback", rclrs::QoSProfile::default()).unwrap();
    let motor_status_publisher = node.create_publisher::<FourMotorsStatus>("/motor_status", rclrs::QoSProfile::default()).unwrap();
    let battery_status_publisher = node.create_publisher::<BatteryStatus>("/battery_status", rclrs::QoSProfile::default()).unwrap();
    
    let t: std::thread::JoinHandle<()>; 
    {   let mut feedback_msg = FourMotorsFeedback::default();
        let  (mut motors_status, mut battery_status) = (FourMotorsStatus::default(), BatteryStatus::default());
        let feedback_publisher = feedback_publisher.clone();
        let motor_status_publisher = motor_status_publisher.clone();
        let battery_status_publisher = battery_status_publisher.clone();
        let node = node.clone();

        t = std::thread::spawn(move||{
            let mut buffer = Vec::new();
            loop {
                if let Ok(mut port) = serial_port.lock() {
                    let mut temp_buffer = [0; 256];
                    if let Ok(size) = port.read(&mut temp_buffer) {
                        buffer.extend_from_slice(&temp_buffer[..size]);

                        // Rechercher les messages complets délimités par `<` et `>`
                    while let Some(start) = buffer.iter().position(|&b| b == b'<') {
                        if let Some(end) = buffer.iter().skip(start).position(|&b| b == b'>') {
                            // Extraire le message complet entre `<` et `>`
                            let end = start + end;
                            let message = buffer.drain(start..=end).collect::<Vec<_>>();
                            let message = String::from_utf8_lossy(&message[1..message.len() - 1]).to_string();
                            match determine_message_type(&message) {

                                MessageType::Error(msg) => {
                                    eprintln!("Erreur détectée : {}", msg);
                                }
                                MessageType::Feedback(msg) => {
                                    feedback_msg = parse_feedback(&msg);
                                    let now = node.get_clock().now().to_ros_msg().unwrap();
                                    feedback_msg.header.stamp.nanosec = now.nanosec;
                                    feedback_msg.header.stamp.sec = now.sec;
                                    feedback_publisher.publish(&feedback_msg).unwrap();
                                }
                                MessageType::Status(msg) => {
                                    (motors_status, battery_status) = parse_status(&msg);
                                    let now = node.get_clock().now().to_ros_msg().unwrap(); 
                                    motors_status.header.stamp.nanosec  = now.nanosec;
                                    motors_status.header.stamp.sec = now.sec;
                                    battery_status.header.stamp.sec = now.sec;
                                    battery_status.header.stamp.nanosec  = now.nanosec;
                                    motor_status_publisher.publish(&motors_status).unwrap();
                                    battery_status_publisher.publish(&battery_status).unwrap();
                                }
                                MessageType::Problematic(msg) => {
                                    println!("Got problematic msg : {}",msg);
                                }
                            }
                    }
                    else {
                        break; // Pas encore de fin de message, attendre plus de données
                    }
                }
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10)); // 25 Hz
            }
            
        })

    }
    // Spin pour maintenir le node ROS 2 actif
    //t.join().unwrap();
    rclrs::spin(node).unwrap();
}


// Déterminer le type de message
fn determine_message_type(line: &str) -> MessageType {
    if line.contains("Error") {
        MessageType::Error(line.to_string())
    } else if line.starts_with("st_") {
        MessageType::Status(line.to_string())
    } else if line.starts_with("fr") {
        // Par défaut, considérer comme Feedback
        MessageType::Feedback(line.to_string())
    }else {
        MessageType::Problematic(line.to_string())
    }
}
fn parse_feedback(line: &str) -> FourMotorsFeedback {
    let mut feedback_msg = FourMotorsFeedback::default();
    let segments: Vec<&str> = line.split(';').collect();
    for segment in segments {
        if let Some((motor_name, speed)) = segment.split_once(':') {
            match motor_name {
                "fl" => feedback_msg.motor_front_left.speed = speed.parse().unwrap_or(0.0),
                "fr" => feedback_msg.motor_front_right.speed = speed.parse().unwrap_or(0.0),
                "rl" => feedback_msg.motor_rear_left.speed = speed.parse().unwrap_or(0.0),
                "rr" => feedback_msg.motor_rear_right.speed = speed.parse().unwrap_or(0.0),
                _ => (),
            }
        }
    }

    feedback_msg
}

// Fonction pour parser un message de type Status
fn parse_status(line: &str) -> (FourMotorsStatus, BatteryStatus) {
    let mut motors_status = FourMotorsStatus::default();
    let mut battery_status = BatteryStatus::default();
    let segments: Vec<&str> = line.split(';').collect();
    
    for segment in segments {
        if segment.starts_with("st_") {
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
                    "st_fl" => {
                        motors_status.motor_front_left.motor_name = String::from("front_left");
                        motors_status.motor_front_left.current = current;
                        motors_status.motor_front_left.voltage = voltage;
                    }
                    "st_fr" => {
                        motors_status.motor_front_right.motor_name = String::from("front_right");
                        motors_status.motor_front_right.current = current;
                        motors_status.motor_front_right.voltage = voltage;
                    }
                    "st_rl" => {
                        motors_status.motor_rear_left.motor_name =  String::from("rear_left");
                        motors_status.motor_rear_left.current = current;
                        motors_status.motor_rear_left.voltage = voltage;
                    }
                    "st_rr" => {
                        motors_status.motor_rear_right.motor_name = String::from("rear_right");
                        motors_status.motor_rear_right.current = current;
                        motors_status.motor_rear_right.voltage = voltage;
                    }
                    _ => (),
                }
            }
        }
    }
    (motors_status, battery_status)
}