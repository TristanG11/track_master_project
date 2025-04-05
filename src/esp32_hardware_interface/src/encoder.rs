use crate::motor_state::RAD_PER_TICK;
use crate::motor_state::TIMER_FREQUENCY_SEC;
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::pcnt::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::EspError;

use std::sync::atomic::AtomicI32;
use std::sync::atomic::Ordering;
use std::sync::Arc;
const LOW_LIMIT: i16 = -100;
const HIGH_LIMIT: i16 = 100;
use std::cmp::min;

pub struct Encoder {
    unit: PcntDriver<'static>,
    approx_value: Arc<AtomicI32>,
    total_ticks: Arc<AtomicI32>, // Total number of ticks
    last_total_ticks: Arc<AtomicI32>,
}

impl Encoder {
    pub fn new(
        pcnt: impl Peripheral<P = impl Pcnt> + 'static,
        pin_a: impl Peripheral<P = impl InputPin> + 'static,
        pin_b: impl Peripheral<P = impl InputPin> + 'static,
    ) -> Result<Self, EspError> {
        let mut unit = match PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            None::<AnyInputPin>,
            None::<AnyInputPin>,
        ) {
            Ok(unit) => unit,
            Err(e) => return Err(e),
        };

        // Configure the first channel of the PCNT unit
        if let Err(e) = unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        ) {
            return Err(e);
        }

        // Configure the second channel of the PCNT unit
        if let Err(e) = unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        ) {
            return Err(e);
        }

        // Set the filter value and enable the filter
        unit.set_filter_value(min(10 * 80, 1023)) ?;
        unit.filter_enable()?;

        let total_ticks = Arc::new(AtomicI32::new(0));
        let last_total_ticks = Arc::new(AtomicI32::new(0));
        let approx_value = Arc::new(AtomicI32::new(0));

        // Unsafe interrupt code to handle overflow and underflow of the encoder
        // Tracks overflow in `approx_value: Arc<AtomicI32>`
        // This is useful for odometry in a wheeled robot
        unsafe {
            let approx_value = Arc::clone(&approx_value);
            if let Err(e) = unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::Relaxed);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::Relaxed);
                }
            }) {
                return Err(e);
            }
        }

        // Enable interrupts for high and low limit events
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?; 

        // Initialize the PCNT unit: pause, clear counter, and resume
        unit.counter_pause()?;
        unit.counter_clear()?;
        unit.counter_resume()?;

        Ok(Self {
            unit,
            approx_value,
            total_ticks,
            last_total_ticks,
        })
    }

    // Get the current value of the encoder and update total ticks
    pub fn get_value(&self) -> Result<(), EspError> {
        match self.unit.get_counter_value() {
            Ok(counter_value) => {
                let value = self.approx_value.load(Ordering::SeqCst) + counter_value as i32;
                self.total_ticks.store(value, Ordering::SeqCst);
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    // Compute the speed of the motor in radians per second
    pub fn compute_speed(&mut self) -> Result<f32, EspError> {
        match self.get_value() {
            Ok(_) => {
                let current_ticks = self.total_ticks.load(Ordering::SeqCst);
                let delta_ticks = (current_ticks
                    - self.last_total_ticks.load(Ordering::SeqCst))
                    >> 2;
                
                let speed = delta_ticks as f32 * RAD_PER_TICK / TIMER_FREQUENCY_SEC;
                self.last_total_ticks.store(current_ticks, Ordering::SeqCst);
                Ok(speed)
            }
            Err(e) => Err(e),
        }
    }
}
