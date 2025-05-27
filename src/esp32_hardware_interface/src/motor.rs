use crate::motor_pid::{MotorPID, CONTROL_MAX, CONTROL_MIN, INTEGRAL_MAX, INTEGRAL_MIN};
use crate::motor_pin::MotorPin;
use crate::motor_state::{Direction, MotorState, UPDATE_FREQUENCY_SEC, CMD_THRESHOLD};
use esp_idf_hal::gpio::IOPin;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::hal::pcnt::Pcnt;
use esp_idf_sys::EspError;

use crate::encoder::Encoder;

const DEADBAND: f32 = 0.05;

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
        kp: f32,
        ki: f32,
        kd: f32,
    ) -> Result<Self, EspError> {
        // Initialize motor pins
        let pins = match MotorPin::new(dir_pin, pwm_pin) {
            Ok(pins) => pins,
            Err(e) => return Err(e),
        };

        // Initialize motor state
        let state = MotorState::new();

        // Initialize PID controller with default gains
        let pid = MotorPID::new(kp, ki, kd);
        //let pid = MotorPID::new(0.0, 0.0, 0.0);
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
    /// Computes the PID control signal
    pub fn compute_control(&mut self) -> Result<f32, EspError> {
        // Update current speed from the encoder
        self.state.speed = self.encoder.compute_speed()?;

        // Compute the current position
        //self.state.compute_position();

        // Calculate error between desired speed and current speed
        let mut error = self.state.desired_speed - self.state.speed;

        // Deadband: ignore small errors to prevent oscillations
        if error.abs() < DEADBAND {
            error = 0.0;
        }

        // Calculate the derivative term
        let derivative = (error - self.pid.prev_error) / UPDATE_FREQUENCY_SEC;

        // Prepare integral update
        self.pid.integral = self.pid.integral + error * UPDATE_FREQUENCY_SEC;
        self.pid.integral = self.pid.integral.clamp(INTEGRAL_MIN, INTEGRAL_MAX);
        
        // Calculate the unclamped control signal
        let control_unclamped =
            self.pid.kp * error + self.pid.ki * self.pid.integral + self.pid.kd * derivative;

        // Constrain the control signal
        let constrained_control = control_unclamped.clamp(CONTROL_MIN, CONTROL_MAX);

        // Save current error for next derivative computation
        self.pid.prev_error = error;

        Ok(constrained_control)
    }

    /// Sets the command to the motor based on the control signal
    pub fn set_cmd(&mut self) -> Result<(), EspError> {
        let cmd = self.state.cmd;

        let (direction, duty) = if cmd.abs() < CMD_THRESHOLD {
            (Direction::Stop, 0)
        } else if cmd > 0.0 {
            (Direction::Forward, cmd as u32)
        } else {
            (Direction::Backward, (-cmd) as u32)
        };
        self.set_dir(direction)?;
        self.pins.pwm_pin.set_duty(duty)?;

        Ok(())
    }

    /// Sets the direction of the motor
    pub fn set_dir(&mut self, direction: Direction) -> Result<(), EspError> {
        match direction {
            Direction::Forward => self.pins.dir_pin.set_high()?,
            Direction::Backward | Direction::Stop => self.pins.dir_pin.set_low()?,
        }
    Ok(())
}
}
