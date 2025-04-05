use std::f32::consts::PI;

// Timer frequency in seconds (50 ms)
pub const TIMER_FREQUENCY_SEC: f32 = 0.050;

// Radians per encoder tick (assuming 2096 ticks per revolution)
pub const RAD_PER_TICK: f32 = 2.0 * PI / 2096.0;

// Radius of the wheel in meters
pub const WHEEL_RADIUS: f32 = 0.06;

const POSITION_UPDATE_FACTOR: f32 = WHEEL_RADIUS * TIMER_FREQUENCY_SEC;
// Enum representing motor directions
#[derive(PartialEq)]
pub enum Direction {
    Forward,
    Backward,
    Stop,
}

// Structure to represent the state of the motor
pub struct MotorState {
    pub position: f32,      // Current position in radians
    pub speed: f32,         // Measured speed in rad/s
    pub desired_speed: f32, // Desired speed in rad/s
    pub cmd: f32,           // PWM command
}

impl MotorState {
    /// Creates a new instance of MotorState with default values
    pub fn new() -> Self {
        MotorState {
            position: 0.0,
            speed: 0.0,
            desired_speed: 0.0,
            cmd: 0.0,
        }
    }

    /// Computes and updates the position of the motor based on speed
    pub fn compute_position(&mut self) -> f32 {
        self.position += self.speed * POSITION_UPDATE_FACTOR;
        self.position
    }

    /// Sets the desired speed of the motor
    pub fn set_desired_speed(&mut self, desired_speed: f32) {
        self.desired_speed = desired_speed;
    }
}
