use crate::motor::Motor;
use crate::motor_state::TIMER_FREQUENCY_SEC;
use esp_idf_hal::task::queue::Queue;
use esp_idf_hal::timer::TimerDriver;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
pub struct MotorController {
    pub motors: Arc<Mutex<HashMap<String, Motor>>>,
}

impl MotorController {
    pub fn new() -> Self {
        MotorController {
            motors: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn add_motor(&mut self, motor: Motor) {
        self.motors
            .lock()
            .unwrap()
            .insert(motor.name.clone(), motor);
    }

    pub fn change_pid_gain(&mut self, name: &String, kp: f32, ki: f32, kd: f32) {
        let mut motors = self.motors.lock().unwrap();
        if let Some(motor) = motors.get_mut(name) {
            motor.pid.kd = kd;
            motor.pid.kp = kp;
            motor.pid.ki = ki;
        }
    }

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

    pub fn setup_timer(&mut self, timer: &mut TimerDriver, tx: Arc<Queue<bool>>) {
        let freq_as_us = TIMER_FREQUENCY_SEC * 1_000_000_f32;
        timer.set_alarm(freq_as_us as u64).unwrap();
        timer.enable_interrupt().unwrap();
        timer.enable_alarm(true).unwrap();
        unsafe {
            timer
                .subscribe(move || {
                    tx.send_front(true, 5).unwrap();
                })
                .unwrap();
        }
        timer.enable_interrupt().unwrap();
        timer.enable(true).unwrap();
    }

    pub fn handle_command(&mut self, cmd: &String) -> Result<(), String> {
        if !cmd.starts_with('<') || !cmd.ends_with('>') {
            return Err(String::from("Error: missing < or > in stream"));
        }

        let cmd_body = &cmd[1..cmd.len() - 1];

        let mut motors = self.motors.lock().unwrap();
        for segment in cmd_body.split(';') {
            if let Some((name, value)) = segment.split_once(':') {
                if let Ok(desired_speed) = value.trim().parse::<f32>() {
                    if let Some(motor) = motors.get_mut(name.trim()) {
                        motor.state.set_desired_speed(desired_speed);
                        /*println!(
                            "Moteur {} mis à jour avec vitesse cible {:.2}",
                            name, desired_speed
                        );*/
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

    pub fn process_motors(&mut self) {
        for (_, motor) in self.motors.lock().unwrap().iter_mut() {
            motor.state.cmd = motor.compute_control();
            motor.set_cmd();
            //println!("process motors");
        }
    }
}
