use esp_idf_hal::gpio::{AnyOutputPin, IOPin, Output, PinDriver};
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_sys::EspError;

pub struct MotorPin {
    pub dir_pin: PinDriver<'static, AnyOutputPin, Output>, // Direction pin
    pub pwm_pin: LedcDriver<'static>,                      // PWM pin
}

impl MotorPin {
    /// Initializes the motor pins
    pub fn new(
        dir_pin: impl IOPin + 'static,
        pwm_pin: LedcDriver<'static>,
    ) -> Result<Self, EspError> {
        // Try to create the pin driver, propagating any errors
        let dir_pin = PinDriver::output(dir_pin.downgrade_output())?;

        // Return the MotorPin struct on success
        Ok(MotorPin { dir_pin, pwm_pin })
    }
}
