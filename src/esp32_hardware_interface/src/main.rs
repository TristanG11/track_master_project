use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
//use esp_idf_hal::timer;
mod encoder;
mod motor;
mod motor_controller;
mod motor_pid;
mod motor_pin;
mod motor_state;
use esp_idf_hal::uart;
use motor::Motor;
use motor_controller::MotorController;
use motor_state::UPDATE_FREQUENCY_SEC;

fn main() {
    esp_idf_sys::link_patches(); // Required for ESP-IDF
    let peripherals = Peripherals::take().unwrap();

    // PWM timer configuration  
    // change this to 255????

    /*
    motor rr cmd = 1864.1758 , speed = 4.9162326
motor rl cmd = 1879.9695 , speed = 4.9162326
motor fl cmd = 1792.6039 , speed = 4.9162326
motor fr cmd = 1943.561 , speed = 4.9162326
motor rr cmd = 1864.1758 , speed = 4.9162326
motor rl cmd = 1879.9695 , speed = 4.9162326
motor fl cmd = 1792.6039 , speed = 4.9162326
motor fr cmd = 1979.5634 , speed = 4.8562784
motor rr cmd = 1864.1758 , speed = 4.9162326
motor rl cmd = 1879.9695 , speed = 4.9162326
motor fl cmd = 1792.6039 , speed = 4.9162326
motor fr cmd = 1943.6329 , speed = 4.9761868
ajoute le calcule du temps dereponse
     */
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
        300.0,
        90.0,
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
        300.0,
        90.0,
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
        300.0,
        90.0,
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
        300.0,
        90.0,
        0.0,
    );

    // UART configuration
    /*let mut config = uart::config::Config::default().baudrate(Hertz(115200));
    config.data_bits = DataBits::DataBits8;
    config.rx_fifo_size = 528 as usize;
    config.event_config.rx_fifo_full = Some(10);*/

    let config = uart::config::Config::default()
        .baudrate(Hertz(115200))
        .data_bits(uart::config::DataBits::DataBits8)
        .parity_none()
        .stop_bits(uart::config::StopBits::STOP1);

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

    // Setup periodic timer
    //let queue = Arc::new(Queue::new(QUEUE_LENGTH));
    //controller.setup_timer(&mut timer, queue.clone());

    //let controller = Arc::new(FairMutex::new(controller));

    //let uart_tx = Arc::new(Mutex::new(uart_tx));

    let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<String>();
    let (feedback_tx, feedback_rx) = std::sync::mpsc::channel::<String>();

    // Thread for processing motors
    {
        let mut count = 0;
        let mut updated =false;
        //let uart_tx = uart_tx.clone();
        // let queue = queue.clone();
        std::thread::Builder::new()
            .name("motor_processing".into())
            .stack_size(16384)
            .spawn(move || {
                let period = std::time::Duration::from_millis((UPDATE_FREQUENCY_SEC * 1000.0) as u64);
                let mut next_tick = std::time::Instant::now();
                let mut last_feedback_time: Option<std::time::Instant> = None;
                let mut message = String::with_capacity(128);
                loop {
                    let _ = controller.process_motors();
                    count +=1;
                    if count > 100 
                    {
                        for (_,motor) in controller.motors.iter_mut()
                        {
                            motor.state.set_desired_speed(8.5);
                        }
                    }
                    // Mesurer le temps entre deux feedbacks
                    let now = std::time::Instant::now();
                    
                    // Générer et envoyer le message
                    controller.get_feedback(&mut message);
                    if let Some(last) = last_feedback_time {
                        let elapsed = now.duration_since(last);
                        message.push_str(&format!("ts={};>", elapsed.as_millis()));
                    }else{
                        message.push_str(&format!("ts={};", 0.0));
                    }
                    last_feedback_time = Some(now);
                    let _ = feedback_tx.send(message.clone());

                    // Commandes entrantes UART
                    if let Ok(cmd) = cmd_rx.try_recv() {
                        let _ = controller.handle_command(&cmd);
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
            })
            .unwrap();
    };

    // THREAD TX UART : écrit les feedbacks
    std::thread::Builder::new()
        .name("uart writer".into())
        .stack_size(16384)
        .spawn(move || loop {
            if let Ok(msg) = feedback_rx.recv() {
             //   let _ = uart_tx.write(msg.as_bytes());
            }
            //std::thread::sleep(Duration::from_millis(5));
        })
        .unwrap();

    let mut buffer = [0u8; 512];
    loop {
        match uart_rx.read(&mut buffer, 10) {
            Ok(size) if size > 0 => {
                if let Ok(recv) = std::str::from_utf8(&buffer[..size]) {
                    let command = recv.trim().to_string();
                    let _ = cmd_tx.send(command);
                }
            }
            _ => {}
        }
        //std::thread::sleep(Duration::from_millis(10));
    }
}
