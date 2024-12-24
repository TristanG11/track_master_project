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
    total_ticks: Arc<AtomicI32>, // Nombre total de ticks
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
        unit.channel_config(
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
        )?;
        unit.channel_config(
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
        )?;

        unit.set_filter_value(min(10 * 80, 1023))?;
        unit.filter_enable()?;

        let total_ticks = Arc::new(AtomicI32::new(0));
        let last_total_ticks = Arc::new(AtomicI32::new(0));
        let approx_value = Arc::new(AtomicI32::new(0));
        // unsafe interrupt code to catch the upper and lower limits from the encoder
        // and track the overflow in `value: Arc<AtomicI32>` - I plan to use this for
        // a wheeled robot's odomerty
        unsafe {
            let approx_value = approx_value.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                }
            })?;
        }
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?;
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

    pub fn get_value(&self) {
        let value = self.approx_value.load(Ordering::Relaxed)
            + self.unit.get_counter_value().unwrap() as i32;
        self.total_ticks.store(value, Ordering::Relaxed);
    }

    pub fn compute_speed(&mut self) -> f32 {
        self.get_value();

        let delta_ticks = (self.total_ticks.load(Ordering::Relaxed)
            - self.last_total_ticks.load(Ordering::Relaxed))
            / 4;
        let speed = delta_ticks as f32 * RAD_PER_TICK / TIMER_FREQUENCY_SEC;
        self.last_total_ticks
            .store(self.total_ticks.load(Ordering::Relaxed), Ordering::Relaxed);
        //println!("total ticks {} speed {}",self.total_ticks.load(Ordering::Relaxed) ,speed);
        speed
    }
}
