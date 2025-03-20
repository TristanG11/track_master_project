/*use esp_idf_hal::gpio::AnyIOPin;
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
fn main() -> Result<(), String> {
    esp_idf_sys::link_patches(); // Required for ESP-IDF

    let peripherals = match Peripherals::take() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("<Error : Failed to acquire peripherals {}>", e);
            return Err(format!("{}", e));
        }
    };

    // PWM timer configuration
    let timer_config = TimerConfig::new()
        .frequency(1.kHz().into())
        .resolution(esp_idf_hal::ledc::Resolution::Bits8);
    let pwm_timer = match LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config) {
        Ok(pwm_timer) => pwm_timer,
        Err(e) => {
            eprintln!("<Error : Failed to create PWM timer : {}>", e);
            return Err(format!("Error : Failed to create PWM timer : {}", e));
        }
    };

    // Pins setup for all motors
    // Motor: front right
    let dir_pin_fr = peripherals.pins.gpio5;
    let pwm_pin_fr = match LedcDriver::new(
        peripherals.ledc.channel0,
        &pwm_timer,
        peripherals.pins.gpio6,
    ) {
        Ok(pwm_pin) => pwm_pin,
        Err(e) => {
            eprintln!(
                "<Error : Failed to initialize PWM driver for front-right motor : {}>",
                e
            );
            return Err(format!(
                "Error : Failed to initialize PWM driver for front-right motor : {}",
                e
            ));
        }
    };
    let encoder_a_pin_fr = peripherals.pins.gpio7;
    let encoder_b_pin_fr = peripherals.pins.gpio8;
    let motor_front_right = match Motor::new(
        String::from("fr"),
        dir_pin_fr,
        pwm_pin_fr,
        encoder_a_pin_fr,
        encoder_b_pin_fr,
        peripherals.pcnt0,
    ) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("<Error : Failed to initialize front-right motor : {}>", e);
            return Err(format!(
                "Error : Failed to initialize front-right motor : {}",
                e
            ));
        }
    };

    // Motor: front left
    let dir_pin_fl = peripherals.pins.gpio15;
    let pwm_pin_fl = match LedcDriver::new(
        peripherals.ledc.channel1,
        &pwm_timer,
        peripherals.pins.gpio16,
    ) {
        Ok(pwm_pin) => pwm_pin,
        Err(e) => {
            eprintln!(
                "<Error : Failed to initialize PWM driver for front-left motor : {}>",
                e
            );
            return Err(format!(
                "Error : Failed to initialize PWM driver for front-left motor : {}",
                e
            ));
        }
    };
    let encoder_a_pin_fl = peripherals.pins.gpio17;
    let encoder_b_pin_fl = peripherals.pins.gpio18;
    let motor_front_left = match Motor::new(
        String::from("fl"),
        dir_pin_fl,
        pwm_pin_fl,
        encoder_a_pin_fl,
        encoder_b_pin_fl,
        peripherals.pcnt1,
    ) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("<Error : Failed to initialize front-left motor : {}>", e);
            return Err(format!(
                "Error : Failed to initialize front-left motor : {}",
                e
            ));
        }
    };

    // Motor: rear left
    let dir_pin_rl = peripherals.pins.gpio9;
    let pwm_pin_rl = match LedcDriver::new(
        peripherals.ledc.channel2,
        &pwm_timer,
        peripherals.pins.gpio10,
    ) {
        Ok(pwm_pin) => pwm_pin,
        Err(e) => {
            eprintln!(
                "<Error : Failed to initialize PWM driver for rear-left motor : {}>",
                e
            );
            return Err(format!(
                "Error : Failed to initialize PWM driver for rear-left motor : {}",
                e
            ));
        }
    };
    let encoder_a_pin_rl = peripherals.pins.gpio11;
    let encoder_b_pin_rl = peripherals.pins.gpio12;
    let motor_rear_left = match Motor::new(
        String::from("rl"),
        dir_pin_rl,
        pwm_pin_rl,
        encoder_a_pin_rl,
        encoder_b_pin_rl,
        peripherals.pcnt2,
    ) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("<Error : Failed to initialize rear-left motor : {}>", e);
            return Err(format!(
                "Error : Failed to initialize rear-left motor : {}",
                e
            ));
        }
    };

    // Motor: rear right
    let dir_pin_rr = peripherals.pins.gpio33;
    let pwm_pin_rr = match LedcDriver::new(
        peripherals.ledc.channel3,
        &pwm_timer,
        peripherals.pins.gpio34,
    ) {
        Ok(pwm_pin) => pwm_pin,
        Err(e) => {
            eprintln!(
                "<Error : Failed to initialize PWM driver for rear-right motor : {}>",
                e
            );
            return Err(format!(
                "Error : Failed to initialize PWM driver for rear-right motor : {}",
                e
            ));
        }
    };
    let encoder_a_pin_rr = peripherals.pins.gpio13;
    let encoder_b_pin_rr = peripherals.pins.gpio14;
    let motor_rear_right = match Motor::new(
        String::from("rr"),
        dir_pin_rr,
        pwm_pin_rr,
        encoder_a_pin_rr,
        encoder_b_pin_rr,
        peripherals.pcnt3,
    ) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("<Error : Failed to initialize rear-right motor : {}>", e);
            return Err(format!(
                "Error : Failed to initialize rear-right motor : {}",
                e
            ));
        }
    };

    // Timer setup for PID computation
    let timer_config = timer::config::Config::new().auto_reload(true);
    let mut timer = match timer::TimerDriver::new(peripherals.timer01, &timer_config) {
        Ok(timer) => timer,
        Err(e) => {
            eprintln!("<Error : Failed to initialize timer : {}>", e);
            return Err(format!("<Error : Failed to initialize timer : {}>", e));
        }
    };

    let mut controller = MotorController::new();

    // Add motors to the controller
    if let Err(e) = controller.add_motor(motor_front_left) {
        eprintln!("<Failed in adding motor front left: {}>", e);
        return Err(format!("<Failed in adding motor front left: {}>", e));
    }

    if let Err(e) = controller.add_motor(motor_front_right) {
        eprintln!("<Failed in adding motor front right: {}>", e);
        return Err(format!("<Failed in adding motor front right: {}>", e));
    }

    if let Err(e) = controller.add_motor(motor_rear_right) {
        eprintln!("<Failed in adding motor rear right: {}>", e);
        return Err(format!("<Failed in adding motor rear right: {}>", e));
    }

    if let Err(e) = controller.add_motor(motor_rear_left) {
        eprintln!("<Failed in adding motor rear left: {}>", e);
        return Err(format!("<Failed in adding motor rear left: {}>", e));
    }

    // Setup periodic timer
    let queue = Arc::new(Queue::new(QUEUE_LENGTH));
    if let Err(e) = controller.setup_timer(&mut timer, queue.clone()) {
        eprintln!("<Error : Failed in timer setup {}>", e);
        return Err(format!("<Error : Failed in timer setup {}>", e));
    }

    let controller = Arc::new(Mutex::new(controller));

    // Thread for processing motors
    let motor_processing_thread = {
        let controller = controller.clone();
        let queue = queue.clone();
        std::thread::spawn(move || /*  -> Result<(), String> */{
            while queue.recv_front(10).is_some() {
                {
                    if let Ok(mut controller) = controller.try_lock() {
                        if let Err(e) = controller.process_motors() {
                            eprintln!("<Error : Failed in processing motors : {}>", e);
                            //return Err(format!("<Error : Failed in processing motors : {}>", e));
                        }
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            //Ok(())
        })
    };

    // UART configuration
    let mut config = uart::config::Config::default().baudrate(Hertz(115200));
    config.data_bits = DataBits::DataBits8;
    config.rx_fifo_size = 528 as usize;
    config.event_config.rx_fifo_full = Some(10);

    let uart = match uart::UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio43,
        peripherals.pins.gpio44,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    ) {
        Ok(uart) => uart,
        Err(e) => {
            eprintln!("<Error : Failed to open uart : {}>", e);
            return Err(format!("<Error : Failed to open uart : {}>", e));
        }
    };

    // Thread for UART communication
    let uart_thread = {
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
                                //println!("<{}>", command);
                                {
                                    match controller.lock() {
                                        Ok(mut controller) => {
                                            if let Err(e) = controller.handle_command(&command) {
                                                eprintln!(
                                                    "<Error : Failed in getting feedback : {}>",
                                                    e
                                                );
                                                /*return Err::<(), String>(format!(
                                                    "Error : Failed to acquire peripherals : {}",
                                                    e
                                                ));*/
                                            }
                                        }
                                        Err(e) => {
                                            eprintln!("<Error : Failed to acquire lock for controller : {}>",e);
                                           // return Err(format!("<Error : Failed to acquire lock for controller : {}>",e));
                                        }
                                    };
                                }
                            }
                        }
                    }
                    Err(e) => {
                        // Log UART read error
                        eprintln!("<Error : Failed when reading from uart : {}", e);
                       // return Err(format!("<Error : Failed when reading from uart : {}", e));
                    }
                }

                // Send feedback every 40 ms
                if last_execution_time.elapsed() >= std::time::Duration::from_millis(40) {
                    match controller.lock() {
                        Ok(controller) => match controller.get_feedback() {
                            Ok(msg) => {
                                last_execution_time = std::time::Instant::now();
                                println!("{}", msg);
                            }
                            Err(e) => {
                                eprintln!(
                                    "<Error : Failed to get feedback from controller : {}>",
                                    e
                                );
                                /*return Err(format!(
                                    "<Error : Failed to get feedback from controller : {}>",
                                    e
                                ));*/
                            }
                        },
                        Err(e) => {
                            eprintln!("<Error : Failed to acquire lock for controller : {}>", e);
                            /*return Err(format!(
                                "<Error : Failed to acquire lock for controller : {}>",
                                e
                            ));*/
                        }
                    }
                    // Delay to reduce loop frequency
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
            }
        })
    };

    //uart_thread.join().unwrap().unwrap();
    //motor_processing_thread.join().unwrap().unwrap();
    loop {
        std::thread::sleep(std::time::Duration::from_secs_f32(0.10));
    }

    Ok(())
}

*/

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
    let dir_pin_fr = peripherals.pins.gpio5;
    let pwm_pin_fr = LedcDriver::new(
        peripherals.ledc.channel0,
        &pwm_timer,
        peripherals.pins.gpio6,
    )
    .unwrap();
    let encoder_a_pin_fr = peripherals.pins.gpio7;
    let encoder_b_pin_fr = peripherals.pins.gpio8;
    let motor_front_right = Motor::new(
        String::from("fr"),
        dir_pin_fr,
        pwm_pin_fr,
        encoder_a_pin_fr,
        encoder_b_pin_fr,
        peripherals.pcnt0,
    );

    // Motor: front left
    let dir_pin_fl = peripherals.pins.gpio15;
    let pwm_pin_fl = LedcDriver::new(
        peripherals.ledc.channel1,
        &pwm_timer,
        peripherals.pins.gpio16,
    )
    .unwrap();
    let encoder_a_pin_fl = peripherals.pins.gpio17;
    let encoder_b_pin_fl = peripherals.pins.gpio18;
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
    let dir_pin_rr = peripherals.pins.gpio33;
    let pwm_pin_rr = LedcDriver::new(
        peripherals.ledc.channel3,
        &pwm_timer,
        peripherals.pins.gpio34,
    )
    .unwrap();
    let encoder_a_pin_rr = peripherals.pins.gpio13;
    let encoder_b_pin_rr = peripherals.pins.gpio14;
    let motor_rear_right = Motor::new(
        String::from("rr"),
        dir_pin_rr,
        pwm_pin_rr,
        encoder_a_pin_rr,
        encoder_b_pin_rr,
        peripherals.pcnt3,
    );

    // Timer setup for PID computation
    let timer_config = timer::config::Config::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer01, &timer_config).unwrap();

    let mut controller = MotorController::new();

    // Add motors to the controller
    controller.add_motor(motor_front_left.expect("REASON"));
    controller.add_motor(motor_front_right.expect("REASON"));
    controller.add_motor(motor_rear_right.expect("REASON"));
    controller.add_motor(motor_rear_left.expect("REASON"));

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
                    if let Ok(mut controller) = controller.try_lock() {
                        controller.process_motors();
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };

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

    // Thread for UART communication
    let uart_thread = {
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
                                //eprintln!("<we reeeee {}>",command);
                                
                                if let Err(e) = controller.lock().unwrap().handle_command(&command) {
                                    // Log errors from handle_command
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
                    println!("{}", message);
                    last_execution_time = std::time::Instant::now();
                }

                // Delay to reduce loop frequency
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        })
    };

    uart_thread.join().unwrap();
    motor_processing_thread.join().unwrap();
    // Main loop
    /*loop {
        std::thread::sleep(std::time::Duration::from_secs_f32(0.10));
    }*/
}