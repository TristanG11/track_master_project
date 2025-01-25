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

    // Adds a motor to the controller
    pub fn add_motor(&mut self, motor: Motor) -> Result<(), String> {
        if let Ok(mut motors) = self.motors.lock() {
            motors.insert(motor.name.clone(), motor);
            Ok(())
        } else {
            Err("Failed to acquire lock while adding motor".to_string())
        }
    }

    // Changes the PID gains for a specific motor
    pub fn change_pid_gain(
        &mut self,
        name: &String,
        kp: f32,
        ki: f32,
        kd: f32,
    ) -> Result<(), String> {
        if let Ok(mut motors) = self.motors.try_lock() {
            if let Some(motor) = motors.get_mut(name) {
                motor.pid.kp = kp;
                motor.pid.ki = ki;
                motor.pid.kd = kd;
                Ok(())
            } else {
                Err(format!("Motor '{}' not found", name))
            }
        } else {
            Err("Failed to acquire lock while changing PID gains".to_string())
        }
    }

    // Returns feedback from all motors as a formatted string
    pub fn get_feedback(&self) -> Result<String, String> {
        if let Ok(motors) = self.motors.lock() {
            let mut feedback = String::from("<FB=");
            for (name, motor) in motors.iter() {
                feedback.push_str(&format!(
                    "{},{},{},{};",
                    name, motor.state.position, motor.state.speed, motor.state.desired_speed
                ));
            }
            feedback.push('>');
            Ok(feedback)
        } else {
            Err("Failed to acquire lock while getting feedback".to_string())
        }
    }

    // Sets up a timer for periodic tasks
    pub fn setup_timer(
        &mut self,
        timer: &mut TimerDriver,
        tx: Arc<Queue<bool>>,
    ) -> Result<(), String> {
        let freq_as_us = TIMER_FREQUENCY_SEC * 1_000_000_f32; // Convert frequency to microseconds

        // Set the alarm period
        if let Err(e) = timer.set_alarm(freq_as_us as u64) {
            return Err(format!("Failed to set alarm: {}", e));
        }

        // Enable timer interrupt
        if let Err(e) = timer.enable_interrupt() {
            return Err(format!("Failed to enable interrupt: {}", e));
        }

        // Enable the alarm
        if let Err(e) = timer.enable_alarm(true) {
            return Err(format!("Failed to enable alarm: {}", e));
        }

        // Set up the interrupt subscription
        unsafe {
            if let Err(e) = timer.subscribe(move || {
                if let Err(err) = tx.send_front(true, 5) {
                    eprintln!("Failed to send signal to queue: {}", err);
                }
            }) {
                return Err(format!("Failed to subscribe to timer: {}", e));
            }
        }

        // Enable timer interrupt again
        if let Err(e) = timer.enable_interrupt() {
            return Err(format!("Failed to enable interrupt: {}", e));
        }

        // Start the timer
        if let Err(e) = timer.enable(true) {
            return Err(format!("Failed to start the timer: {}", e));
        }

        Ok(())
    }

    // Handles incoming commands to update motor states
    pub fn handle_command(&mut self, cmd: &String) -> Result<(), String> {
        if !cmd.starts_with('<') || !cmd.ends_with('>') {
            return Err("Error: missing < or > in stream".to_string());
        }
        if cmd.contains("CMD") {
            let cmd_body = &cmd[5..cmd.len() - 1]; // Remove '<' and '>' from the command

            if let Ok(mut motors) = self.motors.lock() {
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
                        return Err(format!("<Error: invalid segment '{}'>", segment));
                    }
                }
            } else {
                return Err("Failed to acquire lock while handling command".to_string());
            }
        } else if cmd.contains("PID") {
            let cmd_body = &cmd[5..cmd.len() - 1]; // Remove '<PID=' and '>' from the command
            println!("<{}>", cmd_body);
            for segment in cmd_body.split(';') {
                if let Some((name, values)) = segment.split_once(':') {
                    let gains: Vec<&str> = values.split(',').collect();
                    if gains.len() == 3 {
                        if let (Ok(kp), Ok(ki), Ok(kd)) = (
                            gains[0].trim().parse::<f32>(),
                            gains[1].trim().parse::<f32>(),
                            gains[2].trim().parse::<f32>(),
                        ) {
                            // Appelle la fonction change_pid_gain
                            if let Err(e) =
                                self.change_pid_gain(&name.trim().to_string(), kp, ki, kd)
                            {
                                return Err(format!("<Error: {}>", e));
                            }
                        } else {
                            return Err(format!(
                                "<Error: invalid PID values '{}' for motor '{}'>",
                                values, name
                            ));
                        }
                    } else {
                        return Err(format!("<Error: invalid PID format for motor '{}'>", name));
                    }
                } else {
                    return Err(format!("<Error: invalid segment '{}'>", segment));
                }
            }
        }
        Ok(())
    }

    // Processes all motors by updating their states and applying commands
    pub fn process_motors(&mut self) -> Result<(), String> {
        if let Ok(mut motors) = self.motors.lock() {
            for (_, motor) in motors.iter_mut() {
                match motor.compute_control() {
                    Ok(control) => {
                        motor.state.cmd = control; // Set the computed control command
                    }
                    Err(e) => {
                        return Err(format!("Failed to compute control for motor: {}", e));
                    }
                }

                if let Err(e) = motor.set_cmd() {
                    return Err(format!("Failed to set command for motor: {}", e));
                }
            }
            Ok(())
        } else {
            Err("Failed to acquire lock on motors".to_string())
        }
    }
}
