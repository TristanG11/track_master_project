use crate::motor::Motor;
use std::collections::HashMap;

pub struct MotorController {
    pub motors: HashMap<String, Motor>, // Shared collection of motors
}

impl MotorController {
    /// Creates a new MotorController instance
    pub fn new() -> Self {
        MotorController {
            motors: HashMap::new(),
        }
    }

    /// Adds a motor to the controller
    pub fn add_motor(&mut self, motor: Motor) -> Result<(), String> {
            self.motors.insert(motor.name.clone(), motor);
            Ok(())
    }

    /// Returns feedback from all motors as a formatted string
    /// Remplit `feedback_out` avec les vitesses formatées de tous les moteurs
    pub fn get_feedback(&self, feedback_out: &mut String) {
        feedback_out.clear(); // vider l'ancien contenu
            feedback_out.push_str("<FB=");
            for (name, motor) in self.motors.iter() {
                feedback_out.push_str(&format!(
                    "{},{:.2};",
                    name,
                    motor.state.speed
                ));
            }
            // Pas de push final ici, le caller peut ajouter `>` ou d'autres champs comme `ts=...`

    }


    /// Sets up a timer for periodic tasks
    /*pub fn setup_timer(&mut self, timer: &mut TimerDriver, tx: Arc<Queue<bool>>) {
        let freq_as_us = TIMER_FREQUENCY_SEC * 1_000_000_f32; // Convert frequency to microseconds
        println!("tick {}", timer.tick_hz());
        timer.set_alarm(freq_as_us as u64).unwrap(); // Set the alarm period
        timer.enable_interrupt().unwrap(); // Enable timer interrupt
        timer.enable_alarm(true).unwrap(); // Enable the alarm

        unsafe {
            timer
                .subscribe(move || {
                    tx.send_front(true, 5).unwrap(); // Send a signal to the queue
                })
                .unwrap();
        }
        timer.enable_interrupt().unwrap();
        timer.enable(true).unwrap();
    }*/

    /// Handles incoming commands to update motor states
    pub fn handle_command(&mut self, cmd: &String) -> Result<(), String> {
        if !cmd.starts_with('<') || !cmd.ends_with('>') {
            return Err(String::from("<Error: missing delimiter in stream >"));
        }
        if cmd.contains("CMD") {
            //eprintln!("<we reeeee CMD>");
            let cmd_body = &cmd[5..cmd.len() - 1]; // Remove '<' and '>' from the command
                for segment in cmd_body.split(';') {
                    if let Some((name, value)) = segment.split_once(':') {
                        if let Ok(desired_speed) = value.trim().parse::<f32>() {
                            let name = name.trim();
                            // Liste des moteurs à affecter
                            let target_motors: Vec<&str> = match name {
                                "fl" | "fr" => vec!["fl", "fr"],
                                "rl" | "rr" => vec!["rl", "rr"],
                                _ => vec![name],
                            };

                            for motor_name in target_motors {
                                if let Some(motor) = self.motors.get_mut(motor_name) {
                                    motor.state.set_desired_speed(desired_speed);
                                } else {
                                    return Err(format!(
                                        "<Error: motor '{}' not found>",
                                        motor_name
                                    ));
                                }
                            }
                        } else {
                            return Err(format!("<Error: invalid speed value '{}'>", value));
                        }
                    } else {
                        return Err(format!("<Error: invalid segment '{}'>", segment));
                    }
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
    // Changes the PID gains for a specific motor
    pub fn change_pid_gain(
        &mut self,
        name: &String,
        kp: f32,
        ki: f32,
        kd: f32,
    ) -> Result<(), String> {
            if let Some(motor) = self.motors.get_mut(name) {
                motor.pid.kp = kp;
                motor.pid.ki = ki;
                motor.pid.kd = kd;
                Ok(())
            } else {
                Err(format!("<Motor '{}' not found>", name))
            }
    }

    /// Processes all motors by updating their states and applying commands
    pub fn process_motors(&mut self) -> Result<(), String> {
            for (name, motor) in self.motors.iter_mut() {
                if let Ok(cmd) = motor.compute_control() {
                    motor.state.cmd = cmd;
                    println!("motor {} cmd = {} , speed = {}",name,cmd,motor.state.speed);
                    
                    if let Err(e) = motor.set_cmd()
                    // Apply the command to the motor
                    {
                        return Err(format!("<Error: Cannot set cmd'{}'>", e));
                    }
                }
            }
        Ok(())
    }
}
