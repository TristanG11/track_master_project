use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::task::queue::Queue;
use esp_idf_hal::timer;
mod encoder;
mod motor;
mod motor_controller;
mod motor_pid;
mod motor_pin;
mod motor_state;
use esp_idf_hal::uart;
use esp_idf_hal::uart::config::DataBits;
use motor::Motor;
use motor_controller::MotorController;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const QUEUE_LENGTH: usize = 10;

fn main() {
    
    esp_idf_sys::link_patches(); // Required for ESP-IDF
    let peripherals = Peripherals::take().unwrap();

    // PWM timer configuration
    let timer_config = TimerConfig::new()
        .frequency(1.kHz().into())
        .resolution(esp_idf_hal::ledc::Resolution::Bits8);
    let pwm_timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap();

    // Pins setup for all motors
    // Motor: front right
    let dir_pin_fr = peripherals.pins.gpio1;
    let pwm_pin_fr = LedcDriver::new(
        peripherals.ledc.channel0,
        &pwm_timer,
        peripherals.pins.gpio2,
    )
    .unwrap();
    let encoder_a_pin_fr = peripherals.pins.gpio35;
    let encoder_b_pin_fr = peripherals.pins.gpio36;
    let motor_front_right = Motor::new(
        String::from("fr"),
        dir_pin_fr,
        pwm_pin_fr,
        encoder_a_pin_fr,
        encoder_b_pin_fr,
        peripherals.pcnt0,
    );

    // Motor: front left
    let dir_pin_fl = peripherals.pins.gpio4;
    let pwm_pin_fl = LedcDriver::new(
        peripherals.ledc.channel1,
        &pwm_timer,
        peripherals.pins.gpio5,
    )
    .unwrap();
    let encoder_a_pin_fl = peripherals.pins.gpio6;
    let encoder_b_pin_fl = peripherals.pins.gpio7;
    let motor_front_left = Motor::new(
        String::from("fl"),
        dir_pin_fl,
        pwm_pin_fl,
        encoder_a_pin_fl,
        encoder_b_pin_fl,
        peripherals.pcnt1,
    );
    
    // Motor: rear left
    let dir_pin_rl = peripherals.pins.gpio9;
    let pwm_pin_rl = LedcDriver::new(
        peripherals.ledc.channel2,
        &pwm_timer,
        peripherals.pins.gpio10,
    )
    .unwrap();
    let encoder_a_pin_rl = peripherals.pins.gpio11;
    let encoder_b_pin_rl = peripherals.pins.gpio12;
    let motor_rear_left = Motor::new(
        String::from("rl"),
        dir_pin_rl,
        pwm_pin_rl,
        encoder_a_pin_rl,
        encoder_b_pin_rl,
        peripherals.pcnt2,
    );

    // Motor: rear right
    let dir_pin_rr = peripherals.pins.gpio14;
    let pwm_pin_rr = LedcDriver::new(
        peripherals.ledc.channel3,
        &pwm_timer,
        peripherals.pins.gpio13,
    )
    .unwrap();
    let encoder_a_pin_rr = peripherals.pins.gpio47; // PSRAM DO NOT USE CHANGE IT !!!!
    let encoder_b_pin_rr = peripherals.pins.gpio48;
    let motor_rear_right = Motor::new(
        String::from("rr"),
        dir_pin_rr,
        pwm_pin_rr,
        encoder_a_pin_rr,
        encoder_b_pin_rr,
        peripherals.pcnt3,
    );

    // UART configuration
    let mut config = uart::config::Config::default().baudrate(Hertz(115200));
    config.data_bits = DataBits::DataBits8;
    config.rx_fifo_size = 528 as usize;
    config.event_config.rx_fifo_full = Some(10);

    let uart = uart::UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio43,
        peripherals.pins.gpio44,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    )
    .unwrap();

    let (mut uart_tx, uart_rx) = uart.into_split();

    // Timer setup for PID computation
    let timer_config = timer::config::Config::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer01, &timer_config).unwrap();

    let mut controller = MotorController::new();

    // Add motors to the controller
    if let Err(e) = controller.add_motor(motor_front_left.expect("REASON"))
    {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_front_right.expect("REASON"))
    {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_rear_right.expect("REASON"))
    {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_rear_left.expect("REASON"))
    {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    // Setup periodic timer
    let queue = Arc::new(Queue::new(QUEUE_LENGTH));
    controller.setup_timer(&mut timer, queue.clone());

    let controller = Arc::new(Mutex::new(controller));

        

    // Thread for processing motors
    let motor_processing_thread = {
        let controller = controller.clone();
        let queue = queue.clone();
        std::thread::spawn(move || {
            while let Some(_) = queue.recv_front(10) {
                {
                    if let Ok(mut controller) = controller.lock() {
                        if let Err(e) = controller.process_motors()
                        {
                            uart_tx.write(e.as_bytes()).unwrap();
                        }
                        let message = controller.get_feedback();// Send feedback every 40 ms
                        uart_tx.write(message.as_bytes()).unwrap();
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };



    // Thread for UART communication
    /*let uart_thread = {
        let controller = controller.clone();
        std::thread::spawn(move || {
            let mut buffer = [0u8; 256];
            let mut last_execution_time = std::time::Instant::now();

            loop {
                // Read from UART
                match uart.read(&mut buffer, 2) {
                    Ok(size) => {
                        if size > 0 {
                            
                            if let Ok(recv) = std::str::from_utf8(&buffer[..size]) {
                                let command = recv.trim().to_string();
                                
                                
                                if let Err(e) = controller.lock().unwrap().handle_command(&command) {
                                    // Log errors from handle_command
                                    //eprintln!("<we reeeee {}>",e);
                                }
                            }
                        }
                    }
                    Err(_) => {
                        // Log UART read error
                    }
                }

                // Send feedback every 40 ms
                if last_execution_time.elapsed() >= std::time::Duration::from_millis(40) {
                    let message = controller.lock().unwrap().get_feedback();
                    //let message = "Moteur en marche\n";
                    uart.write(message.as_bytes()).unwrap();
                    //println!("{}", message);
                    last_execution_time = std::time::Instant::now();
                }

                // Delay to reduce loop frequency
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        })
    };*/

    //uart_thread.join().unwrap();
    //motor_processing_thread.join().unwrap();
    // Main loop

    let mut buffer = [0u8; 512];
    loop {
        //println!("main thread");

        match uart_rx.read(&mut buffer, 2) {
            Ok(size) => {
                if size > 0 {
                    
                    if let Ok(recv) = std::str::from_utf8(&buffer[..size]) {
                        let command = recv.trim().to_string();
                        
                        
                        if let Err(e) =  controller.lock().unwrap().handle_command(&command) {

                        }
                    }
                }
            }
            Err(_) => {
                // Log UART read error
            }
        }

        // Delay to reduce loop frequency
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}