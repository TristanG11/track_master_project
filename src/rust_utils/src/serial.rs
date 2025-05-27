use serialport;
use diagnostic_msgs::msg::{DiagnosticStatus,KeyValue};
use std::sync::{mpsc,Arc,atomic::AtomicBool};
use parking_lot;
use std::time::Duration;
use std::sync::atomic::Ordering;

pub fn handle_serial_error(
    e: serialport::Error,
    diag_tx: &mpsc::Sender<DiagnosticStatus>,
    diag_name: &str,
    disconnected_flag: &AtomicBool,
) {
    println!("handle_serial_error");
    send_diagnostic(
        diag_tx,
        DiagnosticStatus::ERROR,
        diag_name,
        format!("Error checking available buffer space: {}", e),
        vec![KeyValue {
            key: "Error".to_string(),
            value: e.to_string(),
        }],
    );
    println!("{}",e.clone());
    let mut disconnected = is_serial_port_broken(e);
    disconnected_flag.store(disconnected, Ordering::SeqCst);
    
    while disconnected {
        disconnected = disconnected_flag.load(Ordering::SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(1000)); // pour éviter un spinloop
    }
}

pub fn send_diagnostic(
    diag_tx: &mpsc::Sender<DiagnosticStatus>,
    level: u8,
    name: &str,
    message: String,
    values: Vec<KeyValue>,
) {
    let diag_status = DiagnosticStatus {
        level,
        name: std::string::String::from(name),
        message,
        hardware_id: "".to_string(),
        values,
    };
    if let Err(e) = diag_tx.send(diag_status) {
        eprintln!("Failed to send diagnostic: {}", e);
    }
}

pub fn is_serial_port_broken(err: serialport::Error) -> bool {
    matches!(
        err.kind(),
        serialport::ErrorKind::NoDevice
            | serialport::ErrorKind::Io(std::io::ErrorKind::BrokenPipe)
            | serialport::ErrorKind::Io(std::io::ErrorKind::NotConnected)
            | serialport::ErrorKind::Io(std::io::ErrorKind::Other)
            | serialport::ErrorKind::Io(std::io::ErrorKind::UnexpectedEof)
    ) // Add more errorkind
}

pub fn spawn_reconnection_thread(
    current_port: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, Option<Box<dyn serialport::SerialPort + 'static>>>> ,
    disconnected_flag: Arc<AtomicBool>,
    port_name: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, String>>,
    diag_tx: mpsc::Sender<DiagnosticStatus>,
    baud_rate: i64,
    diag_name: String,
) {
    std::thread::spawn(move || {
        let mut attempt = 0;
        loop {
            std::thread::sleep(Duration::from_secs(2)); // cadence régulière

            if !disconnected_flag.load(Ordering::SeqCst) {
                println!("next");
                continue;
            }

            let current_port_name = {
                if let Some(name) = port_name.try_lock_for(Duration::from_millis(100)) {
                    name.clone()
                } else {
                    continue;
                }
            };

            println!("Trying to reconnect to {}", current_port_name);

            match serialport::new(current_port_name.clone(), baud_rate as u32)
                .timeout(Duration::from_secs(5))
                .open()
            {
                Ok(port) => {
                    if let Some(mut port_guard) =current_port.try_lock_for(Duration::from_millis(100))  {
                        println!("Reconnexion réussie sur {}", current_port_name);
                        disconnected_flag.store(false, Ordering::SeqCst);
                        *port_guard = Some(port);
                    
                    } else {
                        println!("Erreur : impossible de locker le port série pour mise à jour.");
                    }

                    send_diagnostic(
                        &diag_tx,
                        DiagnosticStatus::OK,
                        &diag_name,
                        "Port is open and functioning normally".into(),
                        vec![KeyValue {
                            key: "Attempts".into(),
                            value: attempt.to_string(),
                        }],
                    );
                }

                Err(e) => {
                    attempt += 1;
                    println!("Erreur de reconnexion: {}", e);
                    send_diagnostic(
                        &diag_tx,
                        DiagnosticStatus::ERROR,
                        &diag_name,
                        "Failed to open the serial port.".into(),
                        vec![
                            KeyValue {
                                key: "Error".into(),
                                value: e.to_string(),
                            },
                            KeyValue {
                                key: "Attempts".into(),
                                value: attempt.to_string(),
                            },
                        ],
                    );
                }
            }
        }
    });
}

