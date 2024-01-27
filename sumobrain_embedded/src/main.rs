#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
//#![deny(unsafe_code)]
//#![allow(clippy::empty_loop)]

use cortex_m;
//use cortex_m::interrupt::{Mutex, CriticalSection};
//use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::gpio::{Output, PushPull, PA8};
use stm32f4xx_hal::prelude::*;
use rtic::app;
use rtic_monotonics::systick::*;

use arrayvec::{ArrayVec, ArrayString};
use nalgebra::{Vector2, Point2, Rotation2};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::fmt::Write; // For ArrayString
use core::borrow::BorrowMut;
use core::sync::atomic::{Ordering, AtomicU32, AtomicBool};
use static_cell::StaticCell;
use futures::future::join;
use futures::join;
use log;
use log::{Record, Metadata, Log, info, warn};
use once_cell::race::OnceRef;
//use defmt::{panic, *};
//use {defmt_rtt as _, panic_probe as _};

use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

struct Robot {
    wheel_speed_left: f32,
    wheel_speed_right: f32,
}

impl Robot {
    fn new() -> Self {
        Robot {
            wheel_speed_left: 0.0,
            wheel_speed_right: 0.0,
        }
    }
}

impl RobotInterface for Robot {
    // Capabilities and dimensions
    fn get_track_width(&self) -> f32 {
        return 9.5;
    }

    // Motor control
    fn set_motor_speed(&mut self, left_speed_cm_s: f32, right_speed_cm_s: f32) {
        self.wheel_speed_left = left_speed_cm_s;
        self.wheel_speed_right = right_speed_cm_s;
    }

    // Weapon control
    // -100 to +100
    fn set_weapon_throttle(&mut self, throttle_percentage: f32) {
        // TODO
    }

    // R/C Receiver Inputs
    // Returns values from all R/C receiver channels
    fn get_rc_input_values(&self, _values: &mut[&f32]) {
        // TODO
    }

    // Sensor readings
    // Current in Amperes
    fn get_weapon_current(&self) -> f32 {
        // TODO
        return 0.0;
    }
    // Returns a list of (angle, distance (cm), something_seen) tuples for each sensor
    fn get_proximity_sensors(&self) -> ArrayVec<(f32, f32, bool), 6> {
        // TODO
        let mut proximity_sensor_readings: ArrayVec<(f32, f32, bool), 6> = ArrayVec::new();
        proximity_sensor_readings.push((  0.0, 30.0, true));
        proximity_sensor_readings.push((-45.0, 30.0, true));
        proximity_sensor_readings.push(( 45.0, 30.0, true));
        proximity_sensor_readings.push((-90.0, 30.0, true));
        proximity_sensor_readings.push(( 90.0, 30.0, true));
        proximity_sensor_readings.push((180.0, 30.0, true));
        return proximity_sensor_readings;
    }
    // X, Y, Z axis values
    fn get_gyroscope_reading(&self) -> (f32, f32, f32) {
        // TODO (at least z)
        return (0.0, 0.0, 0.0);
    }
    // X, Y, Z axis values
    fn get_accelerometer_reading(&self) -> (f32, f32, f32) {
        // TODO
        return (0.0, 0.0, 0.0);
    }
    // Voltages of individual cells
    fn get_battery_cell_voltages(&self, _values: &mut[&f32]) {
        // TODO
    }

    // LED control
    fn set_led_status(&mut self, _status: bool) {
        // TODO
    }

    // Diagnostic data
    fn report_map(&mut self, map: &Map, robot_p: Point2<f32>, robot_r: f32,
            attack_p: Option<Point2<f32>>,
            scan_p: Option<Point2<f32>>,
            wall_avoid_p: Option<Point2<f32>>,
            wall_lines: &[HoughLine]) {
        // TODO: Pass somewhere: USB, wireless link or SD card?
    }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [UART4, UART5, USART6])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        millis_counter: u64,
        wanted_led_state: bool,
    }

    #[local]
    struct Local {
        led_pin: PA8<Output<PushPull>>,
        state: bool, // TODO: Use or remove
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // System clock

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(16.MHz()) // Use external crystal (HSE)
            .hclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .sysclk(168.MHz()) // Set system clock (SYSCLK)
            .freeze(); // Apply the configuration

        // SysTick

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_token);

        // I/O

        let gpioa = cx.device.GPIOA.split();

        let mut led_pin = gpioa.pa8.into_push_pull_output();
        led_pin.set_high();

        let mut debug_pin = cx.device.GPIOB.split().pb10.into_push_pull_output();

        // Schedule tasks

        algorithm_task::spawn().ok();
        millis_counter_task::spawn().ok();
        led_task::spawn().ok();

        // Initialize context

        (
            Shared {
                millis_counter: 0,
                wanted_led_state: true,
            },
            Local {
                led_pin: led_pin,
                state: false,
            }
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 1, shared = [millis_counter, wanted_led_state], local = [])]
    async fn algorithm_task(mut cx: algorithm_task::Context) {
        let mut brain = BrainState::new(0);
        let mut robot: Robot = Robot::new();

        let interval_ms = 1000 / sumobrain_common::UPS as u64;
        loop {
            let mut t0 = cx.shared.millis_counter.lock(|value|{ *value });

            brain.update(&mut robot);

            //cortex_m::asm::delay(16_000_000);
            //robot.wheel_speed_right += 1.0;

            info!("wheel_speed: {:?} {:?}", robot.wheel_speed_left, robot.wheel_speed_right);

            // Toggle LED for debugging
            cx.shared.wanted_led_state.lock(|value| { *value = !*value; });

            // Enforce minimum interval
            let t1 = cx.shared.millis_counter.lock(|value|{ *value });
            let additional_delay = t0 as i64 + interval_ms as i64 - t1 as i64;
            if additional_delay > 0 {
                Systick::delay((additional_delay as u32).millis()).await;
            }
        }
    }

    #[task(priority = 2, shared = [millis_counter], local = [])]
    async fn millis_counter_task(mut cx: millis_counter_task::Context) {
        loop {
            cx.shared.millis_counter.lock(|value| {
                *value = *value + 1;
            });
            Systick::delay(1.millis()).await;
        }
    }

    #[task(priority = 2, shared = [wanted_led_state], local = [led_pin, state])]
    async fn led_task(mut cx: led_task::Context) {
        loop {
            let wanted_state = cx.shared.wanted_led_state.lock(|value| {
                *value
            });
            cx.local.led_pin.set_state(wanted_state.into());
            Systick::delay(1.millis()).await;
        }
    }
}

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
