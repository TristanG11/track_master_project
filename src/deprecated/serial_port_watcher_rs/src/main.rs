use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use msg_utils::msg::SerialPorts;
use std::sync::Arc;
use std::time::{Duration, Instant};
use rclrs::MandatoryParameter;
use serde::Deserialize;
use std::collections::HashMap;
#[derive(Debug, Deserialize, PartialEq, Eq, Clone)]
struct DeviceConfig {
    alias: String,
    serial_number: String,
    vid: u16,
    pid: u16,
}

     
fn main() {
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "serial_port_watcher_rs_node").unwrap();

    let default_devices_yaml = r#"
    - alias: "esp32"
      serial_number: "84746aa23c28ee11af9acd0af59e3369"
      vid: 0x10c4
      pid: 0xea60
    - alias: "gnss"
      serial_number: "D30JIZXO"
      vid: 0x0403
      pid: 0x6015
    - alias: "lidar"
      serial_number: "SNNNNNNNN"
      vid: 0x1234
      pid: 0x5678
    "#;

    // Lecture des paramètres simples
    let topic_available_ports: MandatoryParameter<Arc<str>> =
        node.declare_parameter("topic_available_ports")
            .default(Arc::from("/available_serial_ports"))
            .mandatory()
            .unwrap();

    let scan_interval_secs: MandatoryParameter<i64> =
        node.declare_parameter("scan_interval_secs")
            .default(5)
            .mandatory()
            .unwrap();

    // Lecture du paramètre complexe devices sous forme de string YAML
    let devices_yaml: MandatoryParameter<Arc<str>> =
    node.declare_parameter("devices")
        .default(Arc::from(default_devices_yaml))
        .mandatory()
        .unwrap();

    let diag_pub = node
        .create_publisher::<DiagnosticArray>("/diagnostics", rclrs::QoSProfile::default())
        .unwrap();


    // Parsing YAML dans un vecteur de DeviceConfig
    let devices: Vec<DeviceConfig> = serde_yaml::from_str(&devices_yaml.get())
        .expect("Invalid format for 'devices' parameter");

    let scan_interval = scan_interval_secs.get();

    println!("Topic: {}", topic_available_ports.get());
    println!("Scan interval: {} secs", scan_interval_secs.get());
    println!("Devices: {:#?}", devices);

    let publisher = node
    .create_publisher::<SerialPorts>(&topic_available_ports.get(), rclrs::QoSProfile::default())
    .unwrap();

    let mut base_port_map = detect_ports(&devices);
    println!("{:?}",base_port_map);
    
    let mut diag_status = DiagnosticStatus::default();
    diag_status.name = "serial_port_watcher_rs_node".to_string();
    diag_status.hardware_id = "serial_ports".to_string();
    diag_status.level = DiagnosticStatus::OK;
    let mut last_seen_map = HashMap::<String, Option<Instant>>::new();
    for dev in &devices{
        last_seen_map.insert(dev.alias.clone(), None);
    }
    
    std::thread::spawn( 
        {
            let node = node.clone();
            move ||
            loop {
                let new_map =  detect_ports(&devices);
                
                let mut msg = SerialPorts::default();
                for dev in &devices {
                    let alias = &dev.alias;
                    let new_port = new_map.get(alias).cloned().unwrap_or_else(|| "".to_string());
        
                    let old_port = base_port_map.get(alias).cloned().unwrap_or_else(|| "".to_string());
        
                    if new_port != old_port {
                        println!("Changement détecté pour {}: '{}' -> '{}'", alias, old_port, new_port);
                        base_port_map.insert(alias.clone(), new_port.clone());
                    }
                    match alias.as_str() {
                        "esp32" => msg.esp32_port = new_port.clone(),
                        "gnss" => msg.gnss_port = new_port.clone(),
                        "lidar" => msg.lidar_port = new_port.clone(),
                        _ => {},
                    }
                    if !new_port.is_empty() {
                        last_seen_map.insert(alias.clone(), Some(Instant::now()));
                    }
    
                    diag_status.values.push(KeyValue {
                        key: format!("{}_port", alias),
                        value: new_port.clone(),
                    });
                    
                    let last_seen_str = last_seen_map
                        .get(alias)
                        .and_then(|opt| opt.map(|instant| format!("{:.1}s", instant.elapsed().as_secs_f64())))
                        .unwrap_or_else(|| "never".to_string());
    
                    diag_status.values.push(KeyValue {
                            key: format!("{}_last_seen", alias),
                            value: last_seen_str,
                        });
                }
                let _ = publisher.publish(msg);
                let mut diag_msg = DiagnosticArray::default();
                diag_msg.status.push(diag_status.clone());
                let now = node.get_clock().now().to_ros_msg().unwrap();
                diag_msg.header.stamp.sec = now.sec;
                diag_msg.header.stamp.nanosec = now.nanosec;
                let _ = diag_pub.publish(diag_msg);
                diag_status.values.clear();
                std::thread::sleep(Duration::from_secs(scan_interval as u64));
            }
        }
    );
    let _ = rclrs::spin(node);
}

fn detect_ports(devices: &Vec<DeviceConfig>) -> HashMap<String, String> {
    let mut result = HashMap::new();
    let ports = serialport::available_ports().unwrap_or_default();

    for dev in devices {
        for port in &ports {
            if matches!(port.port_type, serialport::SerialPortType::Unknown) {
                continue;
            }
            if let serialport::SerialPortType::UsbPort(usb) = &port.port_type {
                if usb.vid == dev.vid && usb.pid == dev.pid
                    && usb.serial_number.as_deref() == Some(&dev.serial_number)
                {
                    result.insert(dev.alias.clone(), port.port_name.clone());
                }
            }
        }
    }
    result
}