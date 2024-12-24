use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::task::queue::Queue;
use esp_idf_hal::timer;
mod config;
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
/* */
fn main() {
    esp_idf_sys::link_patches(); // Nécessaire pour ESP-IDF
    let peripherals = Peripherals::take().unwrap();

    // Configuration du timer PWM
    let timer_config = TimerConfig::new()
        .frequency(1.kHz().into())
        .resolution(esp_idf_hal::ledc::Resolution::Bits8);
    let pwm_timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap();

    // pins creation for all motors :
    // motor front right
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

    // motor front left
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

    // motor rear left
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

    //motor rear right
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

    // timer for pid computation

    let timer_config = timer::config::Config::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer01, &timer_config).unwrap();

    let mut controller = MotorController::new();

    controller.add_motor(motor_front_left);
    controller.add_motor(motor_front_right);
    controller.add_motor(motor_rear_right);
    controller.add_motor(motor_rear_left);

    // Setup timer
    let queue = Arc::new(Queue::new(QUEUE_LENGTH));

    controller.setup_timer(&mut timer, queue.clone());

    let controller = Arc::new(Mutex::new(controller));

    // thread for processing motor :

    let motor_processing_thread = {
        let controller = controller.clone();
        let queue = queue.clone();
        std::thread::spawn(move || {
            while let Some(_) = queue.recv_front(10) {
                {
                    if let Ok(mut controller) = controller.try_lock() {
                        controller.process_motors();
                        //println!("Processed motors, time since last call: {:?}", duration);
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };

    // uart configuration

    let mut config = uart::config::Config::default().baudrate(Hertz(115200));
    config.data_bits = DataBits::DataBits8;
    config.rx_fifo_size = 528 as usize;
    //config.tx_fifo_size = 4069 as usize;
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

    let uart_thread = {
        let controller = controller.clone();
        std::thread::spawn(move || {
            let mut buffer = [0u8; 256]; // Taille augmentée
            let mut last_execution_time = std::time::Instant::now(); // Initialisez le timer

            loop {
                // Lecture du port série
                match uart.read(&mut buffer, 2) {
                    Ok(size) => {
                        if size > 0 {
                            // Convertir le buffer en String
                            if let Ok(recv) = std::str::from_utf8(&buffer[..size]) {
                                let command = recv.trim().to_string(); // Nettoyer et convertir en String
                                if let Err(e) = controller.lock().unwrap().handle_command(&command)
                                {
                                    //eprintln!("Erreur dans handle_command: {:?}", e);
                                }
                            }
                        }
                    }
                    Err(e) => {
                        //eprintln!("Erreur de lecture UART : {:?}", e);
                    }
                }

                // Écriture périodique toutes les 40 ms
                if last_execution_time.elapsed() >= std::time::Duration::from_millis(40) {
                    let message = controller.lock().unwrap().get_feedback();
                    println!("{}", message);
                    last_execution_time = std::time::Instant::now(); // Réinitialiser le timer
                }

                // Pause pour limiter la fréquence de la boucle
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        })
    };

    // Boucle principale
    loop {
        {
            //let controller = controller.lock().unwrap();
            //println!("{}", controller.get_feedback());
        }

        std::thread::sleep(std::time::Duration::from_secs_f32(0.10));
    }
}
