use esp_idf_hal::gpio::{AnyOutputPin, IOPin, Output, PinDriver};
use esp_idf_hal::ledc::LedcDriver;
pub struct MotorPin {
    pub dir_pin: PinDriver<'static, AnyOutputPin, Output>, // Broche de direction
    pub pwm_pin: LedcDriver<'static>,                      // Broche PWM
}

impl MotorPin {
    /// Initialisation des broches du moteur
    pub fn new(dir_pin: impl IOPin + 'static, pwm_pin: LedcDriver<'static>) -> Self {
        // Conversion explicite des broches vers leurs types génériques
        let dir_pin = PinDriver::output(dir_pin.downgrade_output()).unwrap();
        MotorPin { dir_pin, pwm_pin }
    }
}
