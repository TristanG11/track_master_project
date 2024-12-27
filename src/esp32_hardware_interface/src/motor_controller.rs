use crate::motor::Motor;
use crate::motor_state::TIMER_FREQUENCY_SEC;
use esp_idf_hal::task::queue::Queue;
use esp_idf_hal::timer::TimerDriver;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub struct MotorController {
    pub motors: Arc<Mutex<HashMap<String, Motor>>>, // Shared collection of motors
}

impl MotorController {
    /// Creates a new MotorController instance
    pub fn new() -> Self {
        MotorController {
            motors: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Adds a motor to the controller
    pub fn add_motor(&mut self, motor: Motor) {
        self.motors
            .lock()
            .unwrap()
            .insert(motor.name.clone(), motor);
    }

    /// Changes the PID gains for a specific motor
    pub fn change_pid_gain(&mut self, name: &String, kp: f32, ki: f32, kd: f32) {
        let mut motors = self.motors.lock().unwrap();
        if let Some(motor) = motors.get_mut(name) {
            motor.pid.kd = kd;
            motor.pid.kp = kp;
            motor.pid.ki = ki;
        }
    }

    /// Returns feedback from all motors as a formatted string
    pub fn get_feedback(&self) -> String {
        let motors = self.motors.lock().unwrap();
        let mut feedback = String::from("<");

        for (name, motor) in motors.iter() {
            feedback.push_str(&format!(
                "{},{},{};",
                name, motor.state.position, motor.state.speed,
            ));
        }

        feedback.push('>');
        feedback
    }

    /// Sets up a timer for periodic tasks
    pub fn setup_timer(&mut self, timer: &mut TimerDriver, tx: Arc<Queue<bool>>) {
        let freq_as_us = TIMER_FREQUENCY_SEC * 1_000_000_f32; // Convert frequency to microseconds
        timer.set_alarm(freq_as_us as u64).unwrap();          // Set the alarm period
        timer.enable_interrupt().unwrap();                   // Enable timer interrupt
        timer.enable_alarm(true).unwrap();                   // Enable the alarm
        unsafe {
            timer
                .subscribe(move || {
                    tx.send_front(true, 5).unwrap(); // Send a signal to the queue
                })
                .unwrap();
        }
        timer.enable_interrupt().unwrap();
        timer.enable(true).unwrap();
    }

    /// Handles incoming commands to update motor states
    pub fn handle_command(&mut self, cmd: &String) -> Result<(), String> {
        if !cmd.starts_with('<') || !cmd.ends_with('>') {
            return Err(String::from("Error: missing < or > in stream"));
        }

        let cmd_body = &cmd[1..cmd.len() - 1]; // Remove '<' and '>' from the command

        let mut motors = self.motors.lock().unwrap();
        for segment in cmd_body.split(';') {
            if let Some((name, value)) = segment.split_once(':') {
                if let Ok(desired_speed) = value.trim().parse::<f32>() {
                    if let Some(motor) = motors.get_mut(name.trim()) {
                        motor.state.set_desired_speed(desired_speed); // Update desired speed
                    } else {
                        return Err(format!("<Error: motor '{}' not found>", name));
                    }
                } else {
                    return Err(format!("<Error: invalid speed value '{}'>", value));
                }
            } else {
                return Err(format!("Error: invalid segment '{}'", segment));
            }
        }

        Ok(())
    }

    /// Processes all motors by updating their states and applying commands
    pub fn process_motors(&mut self) {
        for (_, motor) in self.motors.lock().unwrap().iter_mut() {
            motor.state.cmd = motor.compute_control(); // Compute the control command
            motor.set_cmd();                           // Apply the command to the motor
        }
    }
}
