use crate::motor_pid::MotorPID;
use crate::motor_pin::MotorPin;
use crate::motor_state::{Direction, MotorState, TIMER_FREQUENCY_SEC};
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::gpio::IOPin;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::hal::pcnt::Pcnt;

const INTEGRAL_MIN: f32 = -170.0; // Limite inférieure pour l'intégrale
const INTEGRAL_MAX: f32 = 170.0; // Limite supérieure pour l'intégrale
const CONTROL_MIN: f32 = -200.0; // Limite inférieure pour la commande
const CONTROL_MAX: f32 = 200.0; // Limite supérieure pour la commande
                                // Calculer l'erreur

use crate::encoder::Encoder;

pub struct Motor {
    pub name: String,
    pub pins: MotorPin,
    pub state: MotorState,
    pub pid: MotorPID,
    pub encoder: Encoder,
}

impl Motor {
    pub fn new(
        name: String,
        dir_pin: impl IOPin + 'static,
        pwm_pin: LedcDriver<'static>,
        encoder_a_pin: impl Peripheral<P = impl InputPin> + 'static,
        encoder_b_pin: impl Peripheral<P = impl InputPin> + 'static,
        pcnt: impl Peripheral<P = impl Pcnt> + 'static,
    ) -> Self {
        // Initialiser les broches du moteur
        let pins = MotorPin::new(dir_pin, pwm_pin);
        let state = MotorState::new();
        let pid = MotorPID::new(10.0, 5.0, 0.3);
        let encoder = Encoder::new(pcnt, encoder_a_pin, encoder_b_pin).unwrap();

        Motor {
            name,
            pins,
            state,
            pid,
            encoder,
        }
    }

    /// Calcule la commande PID
    pub fn compute_control(&mut self) -> f32 {
        self.state.speed = self.encoder.compute_speed();
        self.state.compute_position();
        let error = self.state.desired_speed - self.state.speed;

        // Calcul de l'intégrale, avec contrainte
        self.pid.integral += error * TIMER_FREQUENCY_SEC;
        self.pid.integral = self.pid.integral.clamp(INTEGRAL_MIN, INTEGRAL_MAX);

        // Calcul de la dérivée
        let derivative = (error - self.pid.prev_error) / TIMER_FREQUENCY_SEC;

        // Calculer la commande PID
        let mut control =
            self.pid.kp * error + self.pid.ki * self.pid.integral + self.pid.kd * derivative;

        // Contraindre la commande
        control = control.clamp(CONTROL_MIN, CONTROL_MAX);
        self.pid.prev_error = error;
        control
    }

    pub fn set_cmd(&mut self) {
        match self.state.cmd {
            n if n < 0.0 => {
                //println!("BACKWARD");
                self.set_dir(Direction::Backward);
                let cmd = (-n) as u32;
                self.pins.pwm_pin.set_duty(cmd).unwrap();
            }
            n if n == 0.0 => {
                //   println!("STOP");
                self.set_dir(Direction::Stop);
                self.pins.pwm_pin.set_duty(0).unwrap();
            }
            n if n > 0.0 => {
                //  println!("FORWARD");
                self.set_dir(Direction::Forward);
                self.pins.pwm_pin.set_duty(n as u32).unwrap();
            }
            _ => {}
        }
    }

    pub fn set_dir(&mut self, direction: Direction) {
        match direction {
            Direction::Backward => {
                self.pins.dir_pin.set_low().unwrap();
            }
            Direction::Forward => {
                self.pins.dir_pin.set_high().unwrap();
            }
            Direction::Stop => {
                self.pins.dir_pin.set_low().unwrap();
            }
        }
    }
}