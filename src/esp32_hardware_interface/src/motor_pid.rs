pub struct MotorPID {
    pub kp: f32,         // Gain proportionnel
    pub ki: f32,         // Gain intégral
    pub kd: f32,         // Gain dérivé
    pub integral: f32,   // Somme intégrale
    pub prev_error: f32, // Erreur précédente
}

impl MotorPID {
    /// Crée un nouveau PID avec des gains initialisés
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
