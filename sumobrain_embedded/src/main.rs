#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
//use panic_halt as _; // panic handler
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::{pac, prelude::*};
use arrayvec::ArrayVec;
use nalgebra::{Vector2, Point2, Rotation2};
use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED
        let gpioa = dp.GPIOA.split();
        let mut led = gpioa.pa8.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
		// TODO: 168MHz
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        loop {
            // On for 1s, off for 1s.
            led.toggle();
            delay.delay_ms(100_u32);
        }
    }
    loop {}
}

// TODO: Implement
struct Robot {
}

impl Robot {
    fn new() -> Self {
        Robot {
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
        // TODO
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
        let proximity_sensor_readings: ArrayVec<(f32, f32, bool), 6> = ArrayVec::new();
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

//#[entry]
#[cortex_m_rt::entry]
fn main() -> ! {
    let mut brain = BrainState::new(0);
    let mut robot: Robot = Robot::new();

    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED
        let gpioa = dp.GPIOA.split();
        let mut led = gpioa.pa8.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
		// TODO: 168MHz
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        loop {
            // On for 1s, off for 1s.
            led.toggle();
            // TODO
            brain.update(&mut robot);
            led.toggle();

            delay.delay_ms(10_u32);
        }
    }

    loop {}
}
