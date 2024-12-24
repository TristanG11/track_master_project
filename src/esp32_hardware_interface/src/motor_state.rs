
use std::
    f32::consts::PI;

pub const TIMER_FREQUENCY_SEC: f32 = 0.050;
pub const RAD_PER_TICK: f32 = 2.0 * PI / 2096.0;
pub const WHELL_RADIUS: f32 = 0.06;

pub enum Direction {
    Forward,
    Backward,
    Stop,
}
pub struct MotorState {
    pub position: f32,                 // Position en radians
    pub speed: f32,                    // Vitesse mesurée en rad/s
    pub desired_speed: f32,            // Consigne de vitesse en rad/s
    pub cmd: f32,                      // Commande en PWM
}

impl MotorState {
    /// Crée une nouvelle instance de MotorState avec des valeurs par défaut
    pub fn new() -> Self {
        MotorState {
            position: 0.0,
            speed: 0.0,
            desired_speed: 0.0,
            cmd: 0.0,
        }
    }

    pub fn compute_position(&mut self) -> f32 {
        self.position += self.speed * TIMER_FREQUENCY_SEC * WHELL_RADIUS;
        self.position
    }

    pub fn set_desired_speed(&mut self, desired_speed: f32) {
        self.desired_speed = desired_speed;
    }
}
