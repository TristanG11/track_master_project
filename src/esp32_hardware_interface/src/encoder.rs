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
        let mut unit = PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;
        
        // Configure the first channel of the PCNT unit
        unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse, // Count in reverse direction
                hctrl_mode: PcntControlMode::Keep,   // Keep the current count direction
                pos_mode: PcntCountMode::Decrement,  // Decrement on positive edge
                neg_mode: PcntCountMode::Increment,  // Increment on negative edge
                counter_h_lim: HIGH_LIMIT,           // Set the high limit
                counter_l_lim: LOW_LIMIT,            // Set the low limit
            },
        )?;
        
        // Configure the second channel of the PCNT unit
        unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse, // Count in reverse direction
                hctrl_mode: PcntControlMode::Keep,   // Keep the current count direction
                pos_mode: PcntCountMode::Increment,  // Increment on positive edge
                neg_mode: PcntCountMode::Decrement,  // Decrement on negative edge
                counter_h_lim: HIGH_LIMIT,           // Set the high limit
                counter_l_lim: LOW_LIMIT,            // Set the low limit
            },
        )?;

        // Set the filter value and enable the filter
        unit.set_filter_value(min(10 * 80, 1023))?;
        unit.filter_enable()?;

        let total_ticks = Arc::new(AtomicI32::new(0));
        let last_total_ticks = Arc::new(AtomicI32::new(0));
        let approx_value = Arc::new(AtomicI32::new(0));

        // Unsafe interrupt code to handle overflow and underflow of the encoder
        // Tracks overflow in `approx_value: Arc<AtomicI32>`
        // This is useful for odometry in a wheeled robot
        unsafe {
            let approx_value = approx_value.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst); // Handle high limit overflow
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst); // Handle low limit overflow
                }
            })?;
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
    pub fn get_value(&self) {
        let value = self.approx_value.load(Ordering::Relaxed)
            + self.unit.get_counter_value().unwrap() as i32;
        self.total_ticks.store(value, Ordering::Relaxed);
    }

    // Compute the speed of the motor in radians per second
    pub fn compute_speed(&mut self) -> f32 {
        self.get_value(); // Update the total ticks

        // Calculate the change in ticks since the last measurement
        let delta_ticks = (self.total_ticks.load(Ordering::Relaxed)
            - self.last_total_ticks.load(Ordering::Relaxed))
            / 4; // Divide by 4 for quadrature decoding

        // Compute the speed using the delta ticks and constants
        let speed = delta_ticks as f32 * RAD_PER_TICK / TIMER_FREQUENCY_SEC;

        // Update the last total ticks for the next computation
        self.last_total_ticks
            .store(self.total_ticks.load(Ordering::Relaxed), Ordering::Relaxed);
        
        speed
    }
}
