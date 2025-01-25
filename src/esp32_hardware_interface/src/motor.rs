use crate::motor_pid::MotorPID;
use crate::motor_pin::MotorPin;
use crate::motor_state::{Direction, MotorState, TIMER_FREQUENCY_SEC};
use esp_idf_hal::gpio::IOPin;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::hal::pcnt::Pcnt;
use esp_idf_sys::EspError;

const INTEGRAL_MIN: f32 = -170.0; // Lower limit for the integral
const INTEGRAL_MAX: f32 = 170.0; // Upper limit for the integral
const CONTROL_MIN: f32 = -200.0; // Lower limit for the control signal
const CONTROL_MAX: f32 = 200.0; // Upper limit for the control signal

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
    ) -> Result<Self, EspError> {
        // Initialize motor pins
        let pins = match MotorPin::new(dir_pin, pwm_pin) {
            Ok(pins) => pins,
            Err(e) => return Err(e),
        };

        // Initialize motor state
        let state = MotorState::new();

        // Initialize PID controller with default gains
        let pid = MotorPID::new(10.0, 5.0, 0.3);

        // Initialize encoder
        let encoder = Encoder::new(pcnt, encoder_a_pin, encoder_b_pin).unwrap();

        Ok(Motor {
            name,
            pins,
            state,
            pid,
            encoder,
        })
    }

    /// Computes the PID control signal
    pub fn compute_control(&mut self) -> Result<f32, EspError> {
        // Update current speed from the encoder
        self.state.speed = match self.encoder.compute_speed() {
            Ok(speed) => speed,
            Err(e) => return Err(e),
        };

        // Compute the current position
        self.state.compute_position();

        // Calculate error between desired speed and current speed
        let error = self.state.desired_speed - self.state.speed;

        // Update integral term with constraints
        self.pid.integral += error * TIMER_FREQUENCY_SEC;
        self.pid.integral = self.pid.integral.clamp(INTEGRAL_MIN, INTEGRAL_MAX);

        // Calculate the derivative term
        let derivative = (error - self.pid.prev_error) / TIMER_FREQUENCY_SEC;

        // Compute the PID control signal
        let mut control =
            self.pid.kp * error + self.pid.ki * self.pid.integral + self.pid.kd * derivative;

        // Constrain the control signal
        control = control.clamp(CONTROL_MIN, CONTROL_MAX);

        // Save the current error for the next computation
        self.pid.prev_error = error;

        Ok(control)
    }

    /// Sets the command to the motor based on the control signal
    pub fn set_cmd(&mut self) -> Result<(), EspError> {
        match self.state.cmd {
            n if n < 0.0 => {
                // Set motor to move backward
                self.set_dir(Direction::Backward)?;
                let cmd = (-n) as u32;
                self.pins.pwm_pin.set_duty(cmd)?;
            }
            n if n == 0.0 => {
                // Stop the motor
                self.set_dir(Direction::Stop)?;
                self.pins.pwm_pin.set_duty(0)?;
            }
            n if n > 0.0 => {
                // Set motor to move forward
                self.set_dir(Direction::Forward)?;
                self.pins.pwm_pin.set_duty(n as u32)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// Sets the direction of the motor
    pub fn set_dir(&mut self, direction: Direction) -> Result<(), EspError> {
        match direction {
            Direction::Backward => {
                // Set direction pin to low for backward motion
                self.pins.dir_pin.set_low()?;
            }
            Direction::Forward => {
                // Set direction pin to high for forward motion
                self.pins.dir_pin.set_high()?;
            }
            Direction::Stop => {
                // Set direction pin to low to stop the motor
                self.pins.dir_pin.set_low()?;
            }
        }
        Ok(())
    }
}
