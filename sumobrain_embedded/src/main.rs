#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
//#![deny(unsafe_code)]
//#![allow(clippy::empty_loop)]

use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

use arrayvec::ArrayVec;
use nalgebra::{Vector2, Point2, Rotation2};
use core::cell::RefCell;
use core::ops::DerefMut;

//use stm32f4xx_hal as hal;
//use stm32f4xx_hal::{prelude::*, interrupt, gpio, otg_fs::{USB, UsbBus}};
//use crate::hal::{pac, prelude::*};
//use usb_device::{prelude::*};
//use usbd_serial::{SerialPort, USB_CLASS_CDC};

use cortex_m_rt::{entry, exception};
use cortex_m::interrupt::{Mutex, CriticalSection};
use embassy_stm32::Peripherals;
use core::sync::atomic::{AtomicU32, Ordering};
//use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use futures::future::join;
//use {defmt_rtt as _, panic_probe as _};

/*static LED_PIN: Mutex<RefCell<Option<gpio::gpioa::PA8<gpio::Output<gpio::PushPull>>>>> =
        Mutex::new(RefCell::new(None));
static DEBUG_PIN: Mutex<RefCell<Option<gpio::gpiob::PB10<gpio::Output<gpio::PushPull>>>>> =
        Mutex::new(RefCell::new(None));*/
/*static GLOBAL_TIME_MS: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

fn millis() -> u32 {
    cortex_m::interrupt::free(|cs| {
        *GLOBAL_TIME_MS.borrow(cs).borrow()
    })
}

fn timestamp_age(t0: u32) -> u32 {
    let t1 = millis();
    t1 - t0
}*/

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(16_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    let p = embassy_stm32::init(config);

    //info!("Hello World!");

    //let dp = pac::Peripherals::take().unwrap();
    //let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // I/O

    let mut led = Output::new(p.PA8, Level::High, Speed::Low);

    /*let gpioa = dp.GPIOA.split();
    let mut led_pin = gpioa.pa8.into_push_pull_output();
    led_pin.set_high();
    let mut debug_pin = dp.GPIOB.split().pb10.into_push_pull_output();*/
    /*cortex_m::interrupt::free(|cs| {
        LED_PIN.borrow(cs).replace(Some(led_pin));
        DEBUG_PIN.borrow(cs).replace(Some(debug_pin));
    });*/

    // System clock

    /*let rcc = dp.RCC.constrain();
    //let clocks = rcc.cfgr.sysclk(48.MHz()).freeze(); // Internal at 48MHz
    //let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
    // Results in about 39mA taken via USB with stlink not connected
    // - Powering via stlink adds about 34mA
    let clocks = rcc.cfgr
        .use_hse(16.MHz()) // Use external crystal (HSE)
        .hclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .sysclk(168.MHz()) // Set system clock (SYSCLK)
        .freeze(); // Apply the configuration*/

    // SysTick

    /*let systick_interval_ms = 1;
    //cp.SYST.set_reload(clocks.sysclk().to_Hz() / 1000 * systick_interval_ms - 1);
    // No idea what's going on, for some reason the value needs to be divided by
    // 8 in order to get correct timing
    cp.SYST.set_reload(clocks.sysclk().to_Hz() / 1000 * systick_interval_ms / 8 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();*/

    // Create a delay abstraction based on SysTick
    /*let mut delay = cp.SYST.delay(&clocks);
    loop {
        delay.delay_us(500000_u32);
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut led) = LED_PIN.borrow(cs).borrow_mut().deref_mut() {
                led.toggle();
            }
        });
    }*/

    // USB

    /*
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
    };
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    // Use https://pid.codes/1209/0001/
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .device_class(USB_CLASS_CDC)
        .strings(&[
            StringDescriptors::new(usb_device::descriptor::lang_id::LangID::EN)
                .manufacturer("8Dromeda Productions")
                .product("Taliaivo v1 (2023)")
                .serial_number("NO_SERIAL")
        ]).unwrap()
        .build();
    */

    // Main loop

    let mut brain = BrainState::new(0);
    let mut robot: Robot = Robot::new();

    //let mut last_led_toggle_timestamp = millis();

    loop {
        brain.update(&mut robot);

        //info!("high");
        led.set_high();
        Timer::after_millis(300).await;

        //info!("low");
        led.set_low();
        Timer::after_millis(300).await;

        /*cortex_m::interrupt::free(|cs| {
            if let Some(ref mut debug_pin) = DEBUG_PIN.borrow(cs).borrow_mut().deref_mut() {
                debug_pin.toggle();
            }
        });*/

        /*if timestamp_age(last_led_toggle_timestamp) >= 500 {
            last_led_toggle_timestamp = millis();

            cortex_m::interrupt::free(|cs| {
                if let Some(ref mut led) = LED_PIN.borrow(cs).borrow_mut().deref_mut() {
                    led.toggle();
                }
            });
        }*/
    }
}

/*#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        /*if let Some(ref mut led) = LED_PIN.borrow(cs).borrow_mut().deref_mut() {
            led.toggle();
        }*/
        /*if let Some(ref mut debug_pin) = DEBUG_PIN.borrow(cs).borrow_mut().deref_mut() {
            debug_pin.toggle();
        }*/
        let mut global_time_ms = GLOBAL_TIME_MS.borrow(cs).borrow_mut();
        *global_time_ms += 1;
    });
}*/
