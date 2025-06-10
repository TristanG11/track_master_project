

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
mod encoder;
mod motor;
mod motor_controller;
mod motor_pid;
mod motor_pin;
mod motor_state;
use esp_idf_hal::uart;
use motor::Motor;
use motor_controller::MotorController;
use motor_state::PID_FREQ_SEC;
use std::sync::{Arc, Mutex};


fn main() {
    esp_idf_sys::link_patches(); // Required for ESP-IDF
    let peripherals = Peripherals::take().unwrap();
    let timer_config = TimerConfig::new()
        .frequency(15.kHz().into())
        .resolution(esp_idf_hal::ledc::Resolution::Bits12);
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
    let encoder_a_pin_fr = peripherals.pins.gpio36;
    let encoder_b_pin_fr = peripherals.pins.gpio35;
    let motor_front_right = Motor::new(
        String::from("fr"),
        dir_pin_fr,
        pwm_pin_fr,
        encoder_a_pin_fr,
        encoder_b_pin_fr,
        peripherals.pcnt0,
        500.0,
        0.0,
        0.0,
    );

    // Motor: front left
    let dir_pin_fl = peripherals.pins.gpio4;
    let pwm_pin_fl = LedcDriver::new(
        peripherals.ledc.channel1,
        &pwm_timer,
        peripherals.pins.gpio5,
    )
    .unwrap();
    let encoder_a_pin_fl = peripherals.pins.gpio7;
    let encoder_b_pin_fl = peripherals.pins.gpio6;
    let motor_front_left = Motor::new(
        String::from("fl"),
        dir_pin_fl,
        pwm_pin_fl,
        encoder_a_pin_fl,
        encoder_b_pin_fl,
        peripherals.pcnt1,
        500.0,
        0.0,
        0.0,
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
        500.0,
        0.0,
        0.0,
    );

    // Motor: rear right
    let dir_pin_rr = peripherals.pins.gpio14;
    let pwm_pin_rr = LedcDriver::new(
        peripherals.ledc.channel3,
        &pwm_timer,
        peripherals.pins.gpio13,
    )
    .unwrap();
    let encoder_a_pin_rr = peripherals.pins.gpio48; // PSRAM DO NOT USE CHANGE IT !!!!
    let encoder_b_pin_rr = peripherals.pins.gpio47;
    let motor_rear_right = Motor::new(
        String::from("rr"),
        dir_pin_rr,
        pwm_pin_rr,
        encoder_a_pin_rr,
        encoder_b_pin_rr,
        peripherals.pcnt3,
        500.0,
        0.0,
        0.0,
    );

    let config = uart::config::Config::default()
        .baudrate(Hertz(230400))
        .data_bits(uart::config::DataBits::DataBits8)
        .parity_none()
        .stop_bits(uart::config::StopBits::STOP1)
        .rx_fifo_size(512)
        .tx_fifo_size(512);

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
    //let timer_config = timer::config::Config::new().auto_reload(true);
    //let mut timer = timer::TimerDriver::new(peripherals.timer01, &timer_config).unwrap();

    let mut controller = MotorController::new();

    // Add motors to the controller
    if let Err(e) = controller.add_motor(motor_front_left.expect("REASON")) {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_front_right.expect("REASON")) {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_rear_right.expect("REASON")) {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    if let Err(e) = controller.add_motor(motor_rear_left.expect("REASON")) {
        uart_tx.write(e.as_bytes()).unwrap();
    }

    let controller = Arc::new(Mutex::new(controller));

    // Thread for processing motors
    {
        //let uart_tx = uart_tx.clone();
        // let queue = queue.clone();
        let controller = controller.clone();
        std::thread::Builder::new()
            .name("command_handler".into())
            .stack_size(16384)
            .spawn(move || {
                let mut buffer = [0u8; 128];
                //let mut now = std::time::Instant::now();
                loop {
                    match uart_rx.read(&mut buffer, 40) {
                        Ok(size) if size > 0 => {
                            if let Ok(recv) = std::str::from_utf8(&buffer[..size]) {
                                let command = recv.trim().to_string();
                                if let Ok(mut controller) = controller.lock() {
                                    //println!("<Lock acquired in recv>");
                                    let _ = controller.handle_command(&command);
                                    //println!("{}", command);
                                }
                            }
                        }
                        _ => {}
                    }
                }
            })
            .unwrap();
    };

    {
        let mut last_feedback_time: Option<std::time::Instant> = None;
        let mut message = String::with_capacity(128);
        let controller = controller.clone();
        let period = std::time::Duration::from_millis(50);
        let mut next_tick = std::time::Instant::now();
        std::thread::Builder::new()
            .name("feedback".into())
            .stack_size(16384)
            .spawn(move || {
                loop {
                    message.clear();
                    let now = std::time::Instant::now();

                    // Générer et envoyer le message
                    if let Ok(controller) = controller.lock() {
                        controller.get_feedback(&mut message);
                        if let Some(last) = last_feedback_time {
                            let elapsed = now.duration_since(last).as_millis();
                            if elapsed >= 45 {
                                message.push_str(&format!("ts={};>", elapsed));
                                last_feedback_time = Some(now);
                            }
                        } else {
                            message.push_str(&format!("ts={};", 0.0));
                            last_feedback_time = Some(now);

                        }
                        let _ = uart_tx.write(message.as_bytes());
                    };

                    next_tick += period;
                    let now = std::time::Instant::now();
                    if next_tick > now {
                        std::thread::sleep(next_tick - now);
                    } else {
                        next_tick = now;
                    }
                }
            })
            .unwrap();
    }

    let period = std::time::Duration::from_millis((PID_FREQ_SEC * 1000.0) as u64);
    let mut next_tick = std::time::Instant::now();

    loop {
        if let Ok(mut controller) = controller.lock() {
            let _ = controller.process_motors();
        }
        // Cadencer à 20 Hz
        next_tick += period;
        let now = std::time::Instant::now();
        if next_tick > now {
            std::thread::sleep(next_tick - now);
        } else {
            next_tick = now;
        }
    }
}
