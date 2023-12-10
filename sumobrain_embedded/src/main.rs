#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
//#![deny(unsafe_code)]
//#![allow(clippy::empty_loop)]

use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

use arrayvec::{ArrayVec, ArrayString};
use nalgebra::{Vector2, Point2, Rotation2};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::fmt::Write; // For ArrayString
use core::borrow::BorrowMut;
use core::sync::atomic::{Ordering, AtomicU32, AtomicBool};
use static_cell::StaticCell;

use cortex_m_rt::{entry, exception};
use cortex_m::interrupt::{Mutex, CriticalSection};
use embassy_stm32::Peripherals;
use embassy_executor::{Spawner, Executor, InterruptExecutor};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Output, Pin, Pull, Speed};
use embassy_stm32::gpio;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_time::{Duration, Timer, Instant, Ticker};
use embassy_usb::Builder;
use embassy_usb::driver::EndpointError;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::class::cdc_acm;
use futures::future::join;
use futures::join;
use log;
use log::{Record, Metadata, Log, info, warn};
use once_cell::race::OnceRef;
//use defmt::{panic, *};
//use {defmt_rtt as _, panic_probe as _};

static WANTED_LED_STATE: AtomicBool = AtomicBool::new(true);
static USB_LOGGING_ENABLED: AtomicBool = AtomicBool::new(false);

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

bind_interrupts!(struct Irqs {
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

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

struct UsbLogger {
    buffer: Mutex<RefCell<Option<ArrayString<1024>>>>,
}

impl UsbLogger {
    async fn flush_to_usb<'d, T: embassy_stm32::usb_otg::Instance>(
            &self, sender: &mut cdc_acm::Sender<'d, Driver<'d, T>>)
            -> Result<(), EndpointError> {
        let mut buf2: Option<ArrayString<1024>> = Some(ArrayString::new());
        cortex_m::interrupt::free(|cs| {
            // This replaces the logger buffer with an empty one, and we get the
            // possibly filled in one
            buf2 = self.buffer.borrow(cs).replace(buf2);
        });
        if let Some(ref mut buffer) = buf2 {
            if !buffer.is_empty() && USB_LOGGING_ENABLED.load(Ordering::Relaxed) {
                // Toggle LED
                //WANTED_LED_STATE.fetch_xor(true, Ordering::Relaxed); // Toggle
                // Write the buffer contents to the USB CDC and clear the buffer
                sender.write_packet(buffer.as_bytes()).await?;
            }
        }
        Ok(())
    }
}

impl Log for UsbLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::Level::Info // Adjust as needed
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            cortex_m::interrupt::free(|cs| { 
                if let Some(ref mut buffer) = self.buffer.borrow(cs).borrow_mut().deref_mut() {
                    let _ = writeln!(buffer, "[{}] {}\r", record.level(), record.args());
                }
            });
        }
    }

    fn flush(&self) {
        // Flushing is handled elsewhere
    }
}

static LOGGER: UsbLogger = UsbLogger {
    buffer: Mutex::new(RefCell::new(None)),
};

// Function to initialize the logger
fn init_logger() {
    cortex_m::interrupt::free(|cs| {
        LOGGER.buffer.borrow(cs).replace(Some(ArrayString::new()));
    });
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(log::LevelFilter::Info); // TODO: Adjust
}

async fn flush_log_to_usb<'d, T: Instance + 'd>(sender: &mut cdc_acm::Sender<'d, Driver<'d, T>>)
        -> Result<(), EndpointError> {
    loop {
        LOGGER.flush_to_usb(sender).await?;
        // TODO: Adjust delay
        Timer::after_millis(20).await;
    }
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

// TODO: Figure out how to make this work without a global static
static mut EP_OUT_BUFFER: [u8; 256] = [0u8; 256];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // System clock

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

    // Utilities

    init_logger();

    // I/O

    /*let gpioa = dp.GPIOA.split();
    let mut led_pin = gpioa.pa8.into_push_pull_output();
    led_pin.set_high();
    let mut debug_pin = dp.GPIOB.split().pb10.into_push_pull_output();*/
    /*cortex_m::interrupt::free(|cs| {
        LED_PIN.borrow(cs).replace(Some(led_pin));
        DEBUG_PIN.borrow(cs).replace(Some(debug_pin));
    });*/

    // USB

    // Create the driver, from the HAL.
    let mut config = embassy_stm32::usb_otg::Config::default();
    // NOTE: For this, VBUS would have to be connected to PA9
    //config.vbus_detection = true;
    config.vbus_detection = false;
    let driver = unsafe {
        // TODO: Figure out how to make this work without unsafe
        Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, &mut EP_OUT_BUFFER, config)
    };

    // Executors and tasks

    // High-priority executor: UART4, priority level 6
    interrupt::UART4.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::UART4);
    spawner.spawn(led_task(p.PA8.degrade())).unwrap();
    spawner.spawn(usb_task(driver)).unwrap();

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(background_processing_task()).unwrap();
    }); 
}

#[embassy_executor::task]
async fn usb_task(mut driver: Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>) {

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("8Dromeda Productions");
    config.product = Some("Taliaivo");
    config.serial_number = Some("1337");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut acm = CdcAcmClass::new(&mut builder, &mut state, 64);

    let (mut sender, mut receiver) = acm.split();
    //let (mut sender, mut receiver, mut control) = acm.split_with_control();

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Handle the ACM class
    let acm_sender_fut = async {
        loop {
            sender.wait_connection().await;
            let flusher = flush_log_to_usb(&mut sender);
            let input_handler = usb_serial_input_handler(&mut receiver);
            let (flusher_result, input_handler_result) = join!(flusher, input_handler);
            match flusher_result {
                Ok(_) => (),
                Err(_) => loop {
                    // TODO: Instead of this loop, reset USB in such a way that
                    //       a new connection can be made
                    USB_LOGGING_ENABLED.store(false, Ordering::Relaxed);
                    WANTED_LED_STATE.store(true, Ordering::Relaxed);
                    Timer::after_millis(125).await;
                    WANTED_LED_STATE.store(false, Ordering::Relaxed);
                    Timer::after_millis(125).await;
                },
            };
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using
    // separate tasks instead.
    join!(usb_fut, acm_sender_fut);
}

#[embassy_executor::task]
async fn background_processing_task() {
    let mut brain = BrainState::new(0);
    let mut robot: Robot = Robot::new();

    let interval_us = 1000000 / sumobrain_common::UPS as u64;
    let mut ticker = Ticker::every(Duration::from_micros(interval_us));
    loop {
        brain.update(&mut robot);

        info!("wheel_speed: {:?} {:?}", robot.wheel_speed_left, robot.wheel_speed_right);

        // Make sure other tasks get at least some time
        //Timer::after_millis(1).await;

        ticker.next().await;

        WANTED_LED_STATE.fetch_xor(true, Ordering::Relaxed); // Toggle LED

        /*info!("high");
        WANTED_LED_STATE.store(true, Ordering::Relaxed);
        Timer::after_millis(500).await;

        info!("low");
        WANTED_LED_STATE.store(false, Ordering::Relaxed);
        Timer::after_millis(500).await;*/
    }
}

#[embassy_executor::task]
async fn led_task(pin: AnyPin) {
    let mut led = Output::new(pin, gpio::Level::Low, Speed::Low);

    loop {
        let wanted_state = WANTED_LED_STATE.load(Ordering::Relaxed);
        led.set_level(wanted_state.into());
        Timer::after_millis(10).await;
    }
}

async fn usb_serial_input_handler<'d, T: Instance + 'd>(
        receiver: &mut cdc_acm::Receiver<'d, Driver<'d, T>>) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    loop {
        let n = receiver.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:?}", data);
        // Any received data starts USB logging
        USB_LOGGING_ENABLED.store(true, Ordering::Relaxed);
        // Some commands could be handled like this, altough buffering should be
        // added
        /*if data.len() >= 1 && data[0] == b'a' {
        }*/
    }
}
