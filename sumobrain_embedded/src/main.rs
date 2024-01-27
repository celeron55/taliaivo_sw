#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)] 
//#![deny(unsafe_code)]
//#![allow(clippy::empty_loop)]

use cortex_m;
use cortex_m::interrupt::{Mutex, CriticalSection};
//use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::gpio::{Output, PushPull, PA8};
use stm32f4xx_hal::gpio;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::serial::{config::Config, Event, Serial};
use stm32f4xx_hal::pac as pac;
use stm32f4xx_hal::otg_fs;
use stm32f4xx_hal as hal;
use rtic::app;
use rtic_monotonics::systick::*;
use usb_device::{prelude::*};
use usbd_serial::USB_CLASS_CDC;
use usbd_serial;

use arrayvec::{ArrayVec, ArrayString};
use nalgebra::{Vector2, Point2, Rotation2};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::fmt::Write; // For ArrayString
//use log;
use log::{Record, Metadata, Log, info, warn};
use ringbuffer::{RingBuffer, ConstGenericRingBuffer};
//use core::borrow::BorrowMut;
//use core::sync::atomic::{Ordering, AtomicU32, AtomicBool};
//use static_cell::StaticCell;
//use futures::future::join;
//use futures::join;
//use once_cell::race::OnceRef;
//use defmt::{panic, *};
//use {defmt_rtt as _, panic_probe as _};

use sumobrain_common::{RobotInterface, BrainState, Map};
use sumobrain_common::map::HoughLine;

mod command_accumulator;
use command_accumulator::CommandAccumulator;

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

// Log buffering system

struct MultiLogger {
    uart_buffer: Mutex<RefCell<Option<ArrayString<1024>>>>,
    usb_buffer: Mutex<RefCell<Option<ArrayString<1024>>>>,
}

impl MultiLogger {
    fn get_uart_buffer(&self) -> Option<ArrayString<1024>> {
        let mut buf2: Option<ArrayString<1024>> = Some(ArrayString::new());
        cortex_m::interrupt::free(|cs| {
            // This replaces the logger buffer with an empty one, and we get the
            // possibly filled in one
            buf2 = self.uart_buffer.borrow(cs).replace(buf2);
        });
        buf2
    }
    fn get_usb_buffer(&self) -> Option<ArrayString<1024>> {
        let mut buf2: Option<ArrayString<1024>> = Some(ArrayString::new());
        cortex_m::interrupt::free(|cs| {
            // This replaces the logger buffer with an empty one, and we get the
            // possibly filled in one
            buf2 = self.usb_buffer.borrow(cs).replace(buf2);
        });
        buf2
    }
}

impl Log for MultiLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::Level::Info // TODO: Adjust as needed
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            cortex_m::interrupt::free(|cs| { 
                if let Some(ref mut buffer) =
                        self.uart_buffer.borrow(cs).borrow_mut().deref_mut() {
                    let _ = buffer.write_fmt(format_args!("[{}] {}\r\n",
                            record.level(), record.args()));
                }
                if let Some(ref mut buffer) =
                        self.usb_buffer.borrow(cs).borrow_mut().deref_mut() {
                    let _ = buffer.write_fmt(format_args!("[{}] {}\r\n",
                            record.level(), record.args()));
                }
            });
            // Trigger write to hardware by triggering USART1 interrupt
            pac::NVIC::pend(pac::Interrupt::USART1);
            // Trigger write to hardware by triggering OTG_FS interrupt
            pac::NVIC::pend(pac::Interrupt::OTG_FS);
        }
    }

    fn flush(&self) {
        // Flushing is handled elsewhere
    }
}

static MULTI_LOGGER: MultiLogger = MultiLogger {
    uart_buffer: Mutex::new(RefCell::new(None)),
    usb_buffer: Mutex::new(RefCell::new(None)),
};

// Function to initialize the logger
fn init_logger() {
    cortex_m::interrupt::free(|cs| {
        MULTI_LOGGER.uart_buffer.borrow(cs).replace(Some(ArrayString::new()));
        MULTI_LOGGER.usb_buffer.borrow(cs).replace(Some(ArrayString::new()));
    });
    log::set_logger(&MULTI_LOGGER).unwrap();
    log::set_max_level(log::LevelFilter::Info); // TODO: Adjust as needed
}

// Main program

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [UART4, UART5, USART6])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        millis_counter: u64,
        wanted_led_state: bool,
        console_rxbuf: ConstGenericRingBuffer<u8, 1024>,
        usb_dev: UsbDevice<'static, otg_fs::UsbBusType>,
        usb_serial: usbd_serial::SerialPort<'static, otg_fs::UsbBusType>,
    }

    #[local]
    struct Local {
        led_pin: PA8<Output<PushPull>>,
        usart1_rx: stm32f4xx_hal::serial::Rx<stm32f4xx_hal::pac::USART1, u8>,
        usart1_tx: stm32f4xx_hal::serial::Tx<stm32f4xx_hal::pac::USART1, u8>,
        usart1_txbuf: ConstGenericRingBuffer<u8, 1024>,
        command_accumulator: CommandAccumulator<50>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<otg_fs::UsbBusType>> = None;

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

        // Software utilities

        init_logger();

        info!("-!- Taliaivo up and running");

        // I/O

        let gpioa = cx.device.GPIOA.split();
        let gpiod = cx.device.GPIOD.split();

        let mut led_pin = gpioa.pa8.into_push_pull_output();
        led_pin.set_high();

        let mut debug_pin = cx.device.GPIOB.split().pb10.into_push_pull_output();

        let mut motor_a_enable_pin = gpiod.pd10.into_push_pull_output();
        let mut motor_b_enable_pin = gpiod.pd11.into_push_pull_output();

        motor_a_enable_pin.set_low();
        motor_b_enable_pin.set_low();

        // NOTE: Motor A IN1,2 = TIM4 CH1,2 = PD12,13
        // NOTE: Motor B IN1,2 = TIM4 CH3,4 = PD14,15
        //hal::timer::Timer4::pwm_hz;
        let motor_a_ch1: hal::timer::ChannelBuilder<hal::pac::TIM4, 0, false> =
                hal::timer::Channel1::new(gpiod.pd12);
        let motor_a_ch2: hal::timer::ChannelBuilder<hal::pac::TIM4, 1, false> =
                hal::timer::Channel2::new(gpiod.pd13);
        let motor_b_ch1: hal::timer::ChannelBuilder<hal::pac::TIM4, 2, false> =
                hal::timer::Channel3::new(gpiod.pd14);
        let motor_b_ch2: hal::timer::ChannelBuilder<hal::pac::TIM4, 3, false> =
                hal::timer::Channel4::new(gpiod.pd15);
        let mut motor_pwm = cx.device.TIM4.pwm_hz(
                (motor_a_ch1, motor_a_ch2, motor_b_ch1, motor_b_ch2),
                5000.Hz(), &clocks);
        motor_pwm.enable(hal::timer::Channel::C1);
        motor_pwm.set_duty(hal::timer::Channel::C1,
                (motor_pwm.get_max_duty() as f32 * 0.25) as u16);

        // UART1

        let mut serial_usart1: Serial<stm32f4xx_hal::pac::USART1, u8> = Serial::new(
            cx.device.USART1,
            (gpioa.pa9.into_alternate::<7>(), gpioa.pa10.into_alternate::<7>()),
            Config::default().baudrate(115_200.bps()),
            &clocks
        ).unwrap();
        serial_usart1.listen(Event::RxNotEmpty | Event::TxEmpty);
        let (usart1_tx, usart1_rx) = serial_usart1.split();

        // USB

        let usb = otg_fs::USB::new(
            (cx.device.OTG_FS_GLOBAL, cx.device.OTG_FS_DEVICE, cx.device.OTG_FS_PWRCLK),
            (gpioa.pa11.into_alternate::<10>(), gpioa.pa12.into_alternate::<10>()),
            &clocks
        );

        unsafe {
            USB_BUS.replace(otg_fs::UsbBus::new(usb, &mut EP_MEMORY));
        }

        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        // Use https://pid.codes/1209/0001/
        let mut usb_dev = UsbDeviceBuilder::new(
                unsafe { USB_BUS.as_ref().unwrap() },
                UsbVidPid(0x1209, 0x0001))
            .device_class(USB_CLASS_CDC)
            .strings(&[
                StringDescriptors::new(usb_device::descriptor::lang_id::LangID::EN)
                    .manufacturer("8Dromeda Productions")
                    .product("Taliaivo v1.1 (2024)")
                    .serial_number("1337")
            ]).unwrap()
            .build();

        // Schedule tasks

        algorithm_task::spawn().ok();
        millis_counter_task::spawn().ok();
        led_task::spawn().ok();
        console_command_task::spawn().ok();

        // Initialize context

        (
            Shared {
                millis_counter: 0,
                wanted_led_state: true,
                console_rxbuf: ConstGenericRingBuffer::new(),
                usb_dev: usb_dev,
                usb_serial: usb_serial,
            },
            Local {
                led_pin: led_pin,
                usart1_rx: usart1_rx,
                usart1_tx: usart1_tx,
                usart1_txbuf: ConstGenericRingBuffer::new(),
                command_accumulator: CommandAccumulator::new(),
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
        //let interval_ms = 1000;
        loop {
            let t0 = cx.shared.millis_counter.lock(|value|{ *value });

            brain.update(&mut robot);

            //cortex_m::asm::delay(16_000_000);
            //robot.wheel_speed_right += 1.0;

            //info!("wheel_speed: {:?} {:?}", robot.wheel_speed_left, robot.wheel_speed_right);

            // Toggle LED for debugging
            //cx.shared.wanted_led_state.lock(|value| { *value = !*value; });

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

    #[task(priority = 2, shared = [wanted_led_state], local = [led_pin])]
    async fn led_task(mut cx: led_task::Context) {
        loop {
            let wanted_state = cx.shared.wanted_led_state.lock(|value| { *value });
            cx.local.led_pin.set_state(wanted_state.into());
            Systick::delay(1.millis()).await;
        }
    }

    #[task(
        priority = 1,
        shared = [console_rxbuf, wanted_led_state],
        local = [command_accumulator]
    )]
    async fn console_command_task(mut cx: console_command_task::Context) {
        loop {
            Systick::delay(1.millis()).await;
            let b_maybe = cx.shared.console_rxbuf.lock(|rxbuf| {
                rxbuf.dequeue()
            });
            if let Some(b) = b_maybe {
                //info!("Received byte: {:?} = {:?}", b, b as char);
                if let Some(command) = cx.local.command_accumulator.put(b as char) {
                    info!("Command: {:?}", command);
                    // TODO: Implement commands
                    // Toggle LED for debugging
                    cx.shared.wanted_led_state.lock(|value| { *value = !*value; });
                }
            }
        }
    }

    #[task(
        binds = USART1,
        shared = [wanted_led_state, console_rxbuf],
        local = [usart1_rx, usart1_tx, usart1_txbuf])
    ]
    fn usart1(mut cx: usart1::Context) {
        // Check if there is something to receive, and if so, receive it into
        // somewhere
        if let Ok(b) = cx.local.usart1_rx.read() {
            //info!("Received: {:?}", b);
            cx.shared.console_rxbuf.lock(|rxbuf| {
                rxbuf.push(b);
            });
        }
        if cx.local.usart1_txbuf.is_empty() {
            // Copy MULTI_LOGGER's buffer to txbuf
            // NOTE: This assumes there are only single-byte characters in the
            // buffer. Otherwise it won't fully fit in our byte-based txbuf
            let logger_txbuf_option = MULTI_LOGGER.get_uart_buffer();
            if let Some(mut logger_txbuf) = logger_txbuf_option {
                for b in logger_txbuf.bytes() {
                    cx.local.usart1_txbuf.push(b);
                }
            }
        }
        if let Some(b) = cx.local.usart1_txbuf.dequeue() {
            cx.local.usart1_tx.write(b);
        }
    }

    #[task(
        binds = OTG_FS,
        shared = [
            usb_dev,
            usb_serial,
            wanted_led_state,
            console_rxbuf
        ],
        local = []
    )]
    fn otg_fs_int(mut cx: otg_fs_int::Context) {
        let otg_fs_int::SharedResources {
            __rtic_internal_marker,
            mut usb_dev,
            mut usb_serial,
            mut wanted_led_state,
            mut console_rxbuf,
        } = cx.shared;

        // TODO: Maybe locking console_rxbuf here with usb_dev and usb_serial
        // isn't a good idea
        (&mut usb_dev, &mut usb_serial, &mut console_rxbuf).lock(
                |usb_dev, usb_serial, console_rxbuf| {
            // Write
            let logger_txbuf_option = MULTI_LOGGER.get_usb_buffer();
            if let Some(mut logger_txbuf) = logger_txbuf_option {
                // Convert from string to bytes
                let buf = logger_txbuf.as_bytes();
                // Just throw the buffer to the peripheral and leave. It'll send
                // what it can. We don't care how it went.
                let _ = usb_serial.write(&buf);
            }
            // Read
            if usb_dev.poll(&mut [usb_serial]) {
                let mut buf = [0u8; 64];
                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        for i in 0..count {
                            console_rxbuf.push(buf[i]);
                        }
                    }
                    _ => {}
                }
            }
        });
    }
}

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
