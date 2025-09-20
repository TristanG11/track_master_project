use msg_utils::msg::PowerStatus;
use rclrs::*;
use std_msgs;
use std::{fs::read, io::{BufRead, BufReader}, sync::{atomic::{AtomicBool,Ordering}, Arc}};
use parking_lot::FairMutex;
use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use rust_utils::serial::*;
use std::time::Duration;
fn main() {
    // Initialize the ROS 2 context
    let context = rclrs::Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arduino_nano_serial_interface_node").unwrap();


    let baud_rate: MandatoryParameter<i64> = node
        .declare_parameter("baud_rate")
        .default(115200)
        .mandatory()
        .unwrap();

    let power_status_pub = node.create_publisher::<PowerStatus>(
            "/power_status"
            .keep_last(10)
            .transient_local()
        )
        .unwrap();

    // Emergency stop boolean
    let emergency_stop_cmd = std::sync::Arc::new(AtomicBool::new(false));

    // emergecy stop topic 
    let _emergency_stop_sub = node.create_subscription::<std_msgs::msg::Bool, _>
                        ("/emergency_stop_cmd",
                   {
                    let emergency_stop_cmd = emergency_stop_cmd.clone();
                    move|msg: std_msgs::msg::Bool|
                   {
                    println!("new{:?}  ",msg.data);
                    emergency_stop_cmd.store(msg.data,Ordering::Relaxed);
                   }    
    });


    let serial_port: Arc<
        parking_lot::lock_api::Mutex<
            parking_lot::RawFairMutex,
            Option<Box<dyn serialport::SerialPort + 'static>>,
        >,
    > = Arc::new(parking_lot::FairMutex::new(None));

    let port_name: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, String>> =
        Arc::new(FairMutex::new(String::from("/dev/arduino_nano")));

    let diag_name: &'static str = "arduino_nano_serial_interface_node";
    let (diag_tx, diag_rx) = std::sync::mpsc::channel::<DiagnosticStatus>();
     // Thread to process diagnostic messages
     let diag_publisher = node
        .create_publisher::<DiagnosticArray>(
            "/diagnostics"
            .keep_last(10)
            .transient_local()
        )
        .unwrap();
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
    let disconnected_flag = Arc::new(AtomicBool::new(true));
    // Thread for connexion /reconnexion
    spawn_reconnection_thread(
        serial_port.clone(),
        disconnected_flag.clone(),
        port_name.clone(),
        diag_tx.clone(),
        baud_rate.get(),
        diag_name.to_string(),
    );

    // thread for processing msg
    let _t1 = std::thread::spawn(
        {
            let serial_port = serial_port.clone(); 
            let node = node.clone();
            let disconnected_flag = disconnected_flag.clone();
            let diag_tx = diag_tx.clone();
            move||{
                loop {
                    std::thread::sleep(Duration::from_millis(500));

                    let port = 
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

                    let mut reader = BufReader::new(port);
                    loop {
                        std::thread::sleep(Duration::from_millis(100));
                        let mut line = String::new();
                        let mut serial_error: Option<std::io::Error> = None;

                        match reader.read_line(&mut line)
                       {
                            Ok(n) if n > 0 =>{
                                println!("{}",line);
                                let msg = parse_power_status(line.as_str());
                                if let Some(msg) = msg {
                                    _ = power_status_pub.publish(msg);
                                } 
                            } 

                            Ok(_) => {
                                println!("No data, waiting...");
                            }

                            Err(e) => {
                                eprintln!("Read error: {}", e);
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
                    }
                }
            }  
        }
    );


    //thread for writing msg
    let t2 = std::thread::spawn(
        {
            let emergency_stop_cmd = emergency_stop_cmd.clone(); 
            let node = node.clone();
            let disconnected_flag = disconnected_flag.clone();
            let diag_tx = diag_tx.clone();
            move||{
                loop {
                    std::thread::sleep(Duration::from_millis(500));
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
                        let mut last_state = emergency_stop_cmd.load(Ordering::Relaxed);
                    loop {
                        std::thread::sleep(Duration::from_millis(100));
                        let mut serial_error: Option<std::io::Error> = None;
                        match writer.bytes_to_write(){
                            Ok(_) =>{
                                let command = if !emergency_stop_cmd.load(Ordering::Relaxed){
                                    String::from("1")
                                }else{
                                    String::from("0")
                                };

                            if last_state != emergency_stop_cmd.load(Ordering::Relaxed)
                               {
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
                                };
                                println!("sent new state");
                                last_state = emergency_stop_cmd.load(Ordering::Relaxed);
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
            }
        }
    );   


    executor.spin(SpinOptions::default());
}

/*
Il faut un subscriber pour le bouton d'arret d'urgence à mettre sur l'app


*/



pub fn parse_power_status(line: &str) -> Option<PowerStatus> {
    println!("{}",line);
    let s = line.trim();
    // 1) On vérifie les chevrons
    if !s.starts_with('<') || !s.ends_with('>') {
        return None;
    }
    // 2) On retire < et >
    let inner = &s[1..s.len() - 1];

    // 3) On initialise la struct
    let mut status = PowerStatus::default();

    // 4) On splitte par ';' puis par ',' pour couvrir les deux séparateurs
    for segment in inner.split(&[';', ','][..]).map(str::trim) {
        if let Some(v) = segment.strip_prefix("V1:") {
            status.power_bqtt_voltage = v.parse().unwrap_or(0.0);
        }
        else if let Some(v) = segment.strip_prefix("V2:") {
            status.logical_batt_voltage = v.parse().unwrap_or(0.0);
        }
        else if let Some(v) = segment.strip_prefix("V3:") {
            // V3 est un int 0 ou 1
            println!("{}  ",v);
            status.emergency_button_pressed = match v.trim() {
                "1" => true,
                "0" => false,
                _   => status.emergency_button_pressed,
            };
        }
    }

    Some(status)
}