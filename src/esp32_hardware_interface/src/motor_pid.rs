pub struct MotorPID {
    pub kp: f32,         // Proportional gain
    pub ki: f32,         // Integral gain
    pub kd: f32,         // Derivative gain
    pub integral: f32,   // Integral sum
    pub prev_error: f32, // Previous error
}

impl MotorPID {
    /// Creates a new PID controller with initialized gains
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        MotorPID {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
        }
    }
}
