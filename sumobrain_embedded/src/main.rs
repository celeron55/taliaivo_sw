#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use stm32f4xx_hal as hal;
use stm32f4xx_hal::{prelude::*, interrupt, gpio};
use cortex_m_rt::{entry, exception};
use cortex_m::interrupt::{Mutex, CriticalSection};
use crate::hal::{pac, prelude::*};
use arrayvec::ArrayVec;
use nalgebra::{Vector2, Point2, Rotation2};
use core::cell::RefCell;
use core::ops::DerefMut;

use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

static GPIO_LED: Mutex<RefCell<Option<gpio::gpioa::PA8<gpio::Output<gpio::PushPull>>>>> =
        Mutex::new(RefCell::new(None));

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
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

#[entry]
fn main() -> ! {
    let mut brain = BrainState::new(0);
    let mut robot: Robot = Robot::new();

    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up i/o
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa8.into_push_pull_output();
    cortex_m::interrupt::free(|cs| {
        GPIO_LED.borrow(cs).replace(Some(led));
    });
    let gpiob = dp.GPIOB.split();
    let mut debug_pin = gpiob.pb10.into_push_pull_output();

    // Configure system clock
    let rcc = dp.RCC.constrain();
    //let clocks = rcc.cfgr.sysclk(48.MHz()).freeze(); // Internal at 48MHz
    let clocks = rcc.cfgr
        .use_hse(16.MHz()) // Use external crystal (HSE)
        .sysclk(168.MHz()) // Set system clock (SYSCLK)
        .freeze(); // Apply the configuration

    // Set up 10ms SysTick
    cp.SYST.set_reload(1680000-1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    //unsafe { cp.SYST.rvr.write(1680000) }; // Reload value
    //unsafe { cp.SYST.cvr.write(1680000) }; // Current value
    //unsafe { cp.SYST.csr.write(0x0105) }; // Enable timer, disable interrupt

    // Create a delay abstraction based on SysTick
    let mut delay = cp.SYST.delay(&clocks);

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = GPIO_LED.borrow(cs).borrow_mut().deref_mut() {
            led.set_high();
        }
    });

    loop {
        //led.set_high();
        //led.set_low();
        //led.set_state(true.into());

        brain.update(&mut robot);

        debug_pin.set_high();
        delay.delay_us(100_u32);
        debug_pin.set_low();
    }
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = GPIO_LED.borrow(cs).borrow_mut().deref_mut() {
            led.toggle();
        }
    });
}
