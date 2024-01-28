#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)] 
//#![deny(unsafe_code)]
//#![allow(clippy::empty_loop)]

use cortex_m;
use cortex_m::interrupt::{Mutex, CriticalSection};
//use cortex_m_rt::{entry, exception};
use stm32f4xx_hal as hal;
use stm32f4xx_hal::{
    gpio::{Output, PushPull, PA8},
    gpio,
    prelude::*,
    serial::{config::Config, Event, Serial},
    pac as pac,
    otg_fs,
    adc::{
        config::{AdcConfig, Clock, Dma, Resolution, SampleTime, Scan, Sequence},
        Adc,
    },
    dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
    pac::{ADC1, DMA2, TIM2, USART2},
};
use rtic::app;
use rtic_monotonics::systick::*;
use usb_device::{prelude::*};
use usbd_serial::USB_CLASS_CDC;
use usbd_serial;
use mpu6050::{Mpu6050, Mpu6050Error};

use arrayvec::{ArrayVec, ArrayString};
use nalgebra::{Vector2, Point2, Rotation2, Vector3};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::fmt::Write; // For ArrayString
//use log;
use log::{Record, Metadata, Log, info, warn};
use ringbuffer::{RingBuffer, ConstGenericRingBuffer};
use core::f64::consts::PI;
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

const MAX_PWM: f32 = 0.40;
const FRICTION_COMPENSATION_FACTOR: f32 = 1.2;
const MOTOR_CUTOFF_BATTERY_VOLTAGE: f32 = 9.6;
const SENSOR_MOUNT_RADIUS_CM: f32 = 4.0;

struct Robot {
    wheel_speed_left: f32,
    wheel_speed_right: f32,
    proximity_sensor_readings: ArrayVec<(f32, f32, bool), 6>,
    acc: Vector3<f32>,
    gyro: Vector3<f32>,
    battery_voltage: f32,
}

impl Robot {
    fn new() -> Self {
        Robot {
            wheel_speed_left: 0.0,
            wheel_speed_right: 0.0,
            proximity_sensor_readings: ArrayVec::new(),
            acc: Vector3::new(0.0, 0.0, 0.0),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            battery_voltage: 0.0,
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
        return self.proximity_sensor_readings.clone();
    }
    // X, Y, Z axis values
    fn get_gyroscope_reading(&self) -> (f32, f32, f32) {
        // TODO: Check polarity of each axis
        return (self.gyro.x, self.gyro.y, self.gyro.z);
    }
    // X, Y, Z axis values
    fn get_accelerometer_reading(&self) -> (f32, f32, f32) {
        // TODO: Check polarity and scaling of each axis
        return (self.acc.x, self.acc.y, self.acc.z);
    }
    // Voltages of individual cells
    fn get_battery_min_cell_voltage(&self) -> f32 {
        return self.battery_voltage / 3.0; // 3S
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

// ADC DMA

const ADC_NUM_CHANNELS: usize = 7;
type AdcDmaTransfer = Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory,
        &'static mut [u16; ADC_NUM_CHANNELS]>;

fn adc_scale_vbat(raw: u16) -> f32 {
    return raw as f32 / 4096.0 * (1.0+100.0/4.7) * 3.3;
}

// Motor control

type MotorPwm = hal::timer::PwmHz<hal::pac::TIM4, (
            hal::timer::ChannelBuilder<hal::pac::TIM4, 0, false>,
            hal::timer::ChannelBuilder<hal::pac::TIM4, 1, false>,
            hal::timer::ChannelBuilder<hal::pac::TIM4, 2, false>,
            hal::timer::ChannelBuilder<hal::pac::TIM4, 3, false>
        )>;

fn set_motor_speeds(
    pwm_left: f32,
    pwm_right: f32,
    motor_pwm: &mut MotorPwm,
){
    // Motor A (connector J1) (right side; C1 > C2 = forward)
    if pwm_right >= 0.0 {
        motor_pwm.set_duty(hal::timer::Channel::C1,
                (motor_pwm.get_max_duty() as f32 * (pwm_right).min(MAX_PWM)) as u16);
        motor_pwm.set_duty(hal::timer::Channel::C2, 0);
    } else {
        motor_pwm.set_duty(hal::timer::Channel::C1, 0);
        motor_pwm.set_duty(hal::timer::Channel::C2,
                (motor_pwm.get_max_duty() as f32 * (-pwm_right).min(MAX_PWM)) as u16);
    }
    // Motor B (connector J2) (left side; C3 > C4 = forward)
    if pwm_left >= 0.0 {
        motor_pwm.set_duty(hal::timer::Channel::C3,
                (motor_pwm.get_max_duty() as f32 * (pwm_left).min(MAX_PWM)) as u16);
        motor_pwm.set_duty(hal::timer::Channel::C4, 0);
    } else {
        motor_pwm.set_duty(hal::timer::Channel::C3, 0);
        motor_pwm.set_duty(hal::timer::Channel::C4,
                (motor_pwm.get_max_duty() as f32 * (-pwm_left).min(MAX_PWM)) as u16);
    }
}

// Main program

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [UART4, UART5, USART6])]
mod app {
    use super::*;

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum DriveMode {
        Normal,
        MotorTest,
        Stop,
    }

    #[shared]
    struct Shared {
        millis_counter: u64,
        wanted_led_state: bool,
        console_rxbuf: ConstGenericRingBuffer<u8, 1024>,
        usb_dev: UsbDevice<'static, otg_fs::UsbBusType>,
        usb_serial: usbd_serial::SerialPort<'static, otg_fs::UsbBusType>,
        adc_transfer: AdcDmaTransfer,
        adc_result: [u16; ADC_NUM_CHANNELS],
        drive_mode: DriveMode,
        vbat_lowpass: f32,
        log_sensors: bool,
    }

    #[local]
    struct Local {
        led_pin: PA8<Output<PushPull>>,
        usart1_rx: stm32f4xx_hal::serial::Rx<stm32f4xx_hal::pac::USART1, u8>,
        usart1_tx: stm32f4xx_hal::serial::Tx<stm32f4xx_hal::pac::USART1, u8>,
        usart1_txbuf: ConstGenericRingBuffer<u8, 1024>,
        command_accumulator: CommandAccumulator<50>,
        motor_pwm: MotorPwm,
        adc_buffer: Option<&'static mut [u16; ADC_NUM_CHANNELS]>,
        mpu: Mpu6050<hal::i2c::I2c<hal::pac::I2C1>>,
    }

    #[init(
        local = [
            adc_buffer1: [u16; ADC_NUM_CHANNELS] = [0; ADC_NUM_CHANNELS],
            adc_buffer2: [u16; ADC_NUM_CHANNELS] = [0; ADC_NUM_CHANNELS]
        ]
    )]
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

        // I/O

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();
        let gpiod = cx.device.GPIOD.split();

        let mut led_pin = gpioa.pa8.into_push_pull_output();
        led_pin.set_high();

        //let mut debug_pin = gpiob.pb10.into_push_pull_output();

        // Motor control

        let mut motor_a_enable_pin = gpiod.pd10.into_push_pull_output();
        let mut motor_b_enable_pin = gpiod.pd11.into_push_pull_output();
        motor_a_enable_pin.set_high();
        motor_b_enable_pin.set_high();

        // Motor A IN1,2 = TIM4 CH1,2 = PD12,13
        // Motor B IN1,2 = TIM4 CH3,4 = PD14,15
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
        // Motor A (connector J1) (right side; C1 > C2 = forward)
        motor_pwm.enable(hal::timer::Channel::C1);
        motor_pwm.enable(hal::timer::Channel::C2);
        motor_pwm.set_duty(hal::timer::Channel::C1, 0);
        motor_pwm.set_duty(hal::timer::Channel::C2, 0);
        /*motor_pwm.set_duty(hal::timer::Channel::C1,
                (motor_pwm.get_max_duty() as f32 * 0.1) as u16);
        motor_pwm.set_duty(hal::timer::Channel::C2,
                (motor_pwm.get_max_duty() as f32 * 0.0) as u16);*/
        // Motor B (connector J2) (left side; C3 > C4 = forward)
        motor_pwm.enable(hal::timer::Channel::C3);
        motor_pwm.enable(hal::timer::Channel::C4);
        motor_pwm.set_duty(hal::timer::Channel::C3, 0);
        motor_pwm.set_duty(hal::timer::Channel::C4, 0);
        /*motor_pwm.set_duty(hal::timer::Channel::C3,
                (motor_pwm.get_max_duty() as f32 * 0.1) as u16);
        motor_pwm.set_duty(hal::timer::Channel::C4,
                (motor_pwm.get_max_duty() as f32 * 0.0) as u16);*/

        // MPU6050
        // Accelerometer:
        // - Resting position: (0, 0, -1)
        // - Accelerating backwards: (-1, 0, -1)
        // - Accelerating left: (0, -1, -1)
        // Gyro:
        // - Turning left: (0, 0, -1) (Z=-1=left is what the algorithm expects)
        // - Turning right: (0, 0, 1)
        // - Flipping left: (-1, 0, 0)
        // - Flipping right: (1, 0, 0)
        // - Flipping forward: (0, -1, 0)
        // - Flipping backward: (0, 1, 0)

        let mut mpu_i2c = hal::i2c::I2c::new(
            cx.device.I2C1,
            (gpiob.pb6, gpiob.pb7),
            hal::i2c::Mode::Standard { frequency: 400.kHz() },
            &clocks
        );
        //let mut mpu = Mpu6050::new(mpu_i2c);
        let mut mpu = Mpu6050::new_with_addr(mpu_i2c, 0x69); // AD0 looks to be high
        let mut syst = {
            let mut delay = cortex_m::delay::Delay::new(cx.core.SYST, 168_000_000);

            // MPU6050 takes some time to boot after applying power
            // When tested, 0ms isn't enough, but 10ms was enough
            delay.delay_ms(20);

            mpu.init(&mut delay);

            delay.free() // Return SYST peripheral for other uses
        };
        mpu.set_clock_source(mpu6050::device::CLKSEL::GZAXIS);
        mpu.set_accel_range(mpu6050::device::AccelRange::G16);
        mpu.set_gyro_range(mpu6050::device::GyroRange::D2000);

        // SysTick

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(syst, 168_000_000, systick_token); // Finally eats SYST peripheral

        // Software utilities

        init_logger();

        info!("-!- Taliaivo up and running");

        let acc = mpu.get_acc();
        info!("MPU6050: acc: {:?}", acc);
        let gyro = mpu.get_gyro();
        info!("MPU6050: gyro: {:?}", gyro);

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

        // ADC

        let adc1_c0 = gpioa.pa0.into_analog();
        let adc1_c1 = gpioa.pa1.into_analog();
        let adc1_c2 = gpioa.pa2.into_analog();
        let adc1_c3 = gpioa.pa3.into_analog();
        let adc1_c4 = gpioa.pa4.into_analog();
        let adc1_c5 = gpioa.pa5.into_analog();
        let adc1_c14 = gpioc.pc4.into_analog();

        let adc_config = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled)
            .resolution(Resolution::Twelve)
            .clock(Clock::Pclk2_div_8);

        let mut adc = Adc::adc1(cx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc1_c0, Sequence::One, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c1, Sequence::Two, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c2, Sequence::Three, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c3, Sequence::Four, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c4, Sequence::Five, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c5, Sequence::Six, SampleTime::Cycles_480);
        adc.configure_channel(&adc1_c14, Sequence::Seven, SampleTime::Cycles_480);

        let dma = StreamsTuple::new(cx.device.DMA2);
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);

        let adc_transfer = Transfer::init_peripheral_to_memory(
            dma.0,
            adc,
            cx.local.adc_buffer1,
            None,
            dma_config
        );

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
                adc_transfer: adc_transfer,
                adc_result: [0; ADC_NUM_CHANNELS],
                drive_mode: DriveMode::Normal,
                vbat_lowpass: 0.0,
                log_sensors: false,
            },
            Local {
                led_pin: led_pin,
                usart1_rx: usart1_rx,
                usart1_tx: usart1_tx,
                usart1_txbuf: ConstGenericRingBuffer::new(),
                command_accumulator: CommandAccumulator::new(),
                motor_pwm: motor_pwm,
                adc_buffer: Some(cx.local.adc_buffer2),
                mpu: mpu,
            }
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 1,
        shared = [
            millis_counter,
            wanted_led_state,
            adc_transfer,
            adc_result,
            drive_mode,
            vbat_lowpass,
            log_sensors,
        ],
        local = [
            motor_pwm,
            mpu,
        ]
    )]
    async fn algorithm_task(mut cx: algorithm_task::Context) {
        // Figure out what drive mode to go into initially
        {
            // Wait for a bit
            Systick::delay(10.millis()).await;

            // Start ADC
            cx.shared.adc_transfer.lock(|transfer| {
                transfer.start(|adc| {
                    adc.start_conversion();
                });
            });

            // Wait for a bit
            Systick::delay(10.millis()).await;

            // Now we can check the battery voltage and determine whether we're
            // being powered by:
            // - a programming set-up
            //   -> Go into stop mode
            // - cold start by main switch
            //   -> Go into normal mode
            let vbat = cx.shared.adc_result.lock(|adc_result| {
                    adc_scale_vbat(adc_result[6]) });
            let initial_mode = if vbat < 6.0 {
                DriveMode::Stop
            } else {
                DriveMode::Normal
            };
            cx.shared.drive_mode.lock(|v| { *v = initial_mode; });
            info!("-!- Selected initial mode: {:?}", initial_mode);
        }

        let mut brain = BrainState::new(0);
        let mut robot: Robot = Robot::new();

        let interval_ms = 1000 / sumobrain_common::UPS as u64;
        //let interval_ms = 1000;
        loop {
            let t0 = cx.shared.millis_counter.lock(|value|{ *value });

            let mut vbat = 0.0;
            let mut vbat_lowpass = 0.0;

            cx.shared.adc_result.lock(|adc_result| {
                // Monitor Vbat
                // 100k:4.7k resistor divider, 3.3V reference
                vbat = adc_scale_vbat(adc_result[6]);
                vbat_lowpass = cx.shared.vbat_lowpass.lock(|vbat_lowpass| {
                    *vbat_lowpass = *vbat_lowpass * 0.95 + vbat * 0.05;
                    robot.battery_voltage = *vbat_lowpass;
                    //info!("Vbat = {:?} V", *vbat_lowpass);
                    *vbat_lowpass
                });

                // Convert proximity sensor readings
                robot.proximity_sensor_readings.clear();
                // NOTE: The algorithm currently assumes sensors to be in this
                //       exact order
                let proximity_sensor_angles =
                        [0.0, -45.0, 45.0, -90.0, 90.0, 180.0];
                let adc_indexes = [2, 3, 1, 4, 0, 5];
                for (i, angle_deg) in proximity_sensor_angles.iter().enumerate() {
                    let angle_rad: f32 = angle_deg / 180.0 * PI as f32;
                    let raw = adc_result[adc_indexes[i]];
                    let (distance_cm, detected) = {
                        if raw < 220 {
                            (90.0, false)
                        } else {
                            ((15000.0 / raw as f32).max(5.5), true)
                        }
                    };
                    robot.proximity_sensor_readings.push((
                            angle_rad as f32,
                            distance_cm + SENSOR_MOUNT_RADIUS_CM,
                            detected));
                }
            });

            let acc = cx.local.mpu.get_acc();
            //info!("MPU6050: acc: {:?}", acc);
            if let Ok(acc) = acc {
                robot.acc = Vector3::new(acc.x, acc.y, acc.z);
            }
            let gyro = cx.local.mpu.get_gyro();
            //info!("MPU6050: gyro: {:?}", gyro);
            if let Ok(gyro) = gyro {
                robot.gyro = Vector3::new(gyro.x, gyro.y, gyro.z);
            }

            let log_sensors = cx.shared.log_sensors.lock(|v|{ *v });
            if log_sensors {
                let mut values: [f32; 10] = [0.0; 10];
                if robot.proximity_sensor_readings.len() >= 6 {
                    values[0] = robot.proximity_sensor_readings[0].1;
                    values[1] = robot.proximity_sensor_readings[1].1;
                    values[2] = robot.proximity_sensor_readings[2].1;
                    values[3] = robot.proximity_sensor_readings[3].1;
                    values[4] = robot.proximity_sensor_readings[4].1;
                    values[5] = robot.proximity_sensor_readings[5].1;
                }
                values[6] = robot.gyro.z;
                values[7] = robot.wheel_speed_left;
                values[8] = robot.wheel_speed_right;
                values[9] = vbat;
                info!("S,{:.0},{:.0},{:.0},{:.0},{:.0},{:.0},{:.1},{:.0},{:.0},{:.1}",
                        values[0], values[1], values[2],
                        values[3], values[4], values[5],
                        values[6], values[7], values[8],
                        values[9]);
            }

            brain.update(&mut robot);

            //info!("wheel_speed: {:?} {:?}", robot.wheel_speed_left, robot.wheel_speed_right);

            let drive_mode = cx.shared.drive_mode.lock(|v| { *v });
            //info!("drive_mode: {:?}", drive_mode);

            if vbat_lowpass < MOTOR_CUTOFF_BATTERY_VOLTAGE {
                set_motor_speeds(0.0, 0.0, cx.local.motor_pwm);
            } else if drive_mode == DriveMode::MotorTest {
                // Theoretical basis for scaling:
                // -> Wheel speed is 514/60*pi*2*3.6 = 194cm/s @ 100% PWM
                // -> Test speed: Wheel should spin at 2r/s at 120/514 = 23.3% PWM.
                //    This should be the equivalent of 2*pi*2*3.6 = 45cm/s
                //    Tested to match reality on 2024-01-28
                //    * Comment: There isn't much torque, the wheels can almost stop
                //      rotating at some friction spots of the drivetrain
                let motor_pwm_left = 0.233;
                let motor_pwm_right = 0.233;
                set_motor_speeds(motor_pwm_left, motor_pwm_right, cx.local.motor_pwm);
            } else if drive_mode == DriveMode::Normal {
                // Theoretical basis for scaling:
                // * Wheel speeds are specified in cm/s
                // * Wheel diameters are 36mm
                // * Gearing from motor to wheel is 21:10
                // * Battery voltage is 11.0V
                // * Motor speed is 590rpm @ 6.0V @ 100% PWM
                // -> Motor speed is 11.0/6.0*590 =  1081rpm @ 11.0V @ 100% PWM
                // -> Wheel speed is 1081*10/21 = 514rpm @ 100% PWM
                // -> Wheel speed is 514/60*pi*2*3.6 = 194cm/s @ 100% PWM
                // -> Conversion factor from cm/s to PWM is 1.0/194 = 0.00515
                //    * This is boosted by a bit to overcome friction
                let cm_per_s_to_pwm = 1.0 / 194.0 * FRICTION_COMPENSATION_FACTOR;
                let motor_pwm_left = robot.wheel_speed_left * cm_per_s_to_pwm;
                let motor_pwm_right = robot.wheel_speed_right * cm_per_s_to_pwm;
                set_motor_speeds(motor_pwm_left, motor_pwm_right, cx.local.motor_pwm);
            } else {
                set_motor_speeds(0.0, 0.0, cx.local.motor_pwm);
            }

            // Toggle LED for debugging
            //cx.shared.wanted_led_state.lock(|value| { *value = !*value; });

            // Enforce minimum interval
            let t1 = cx.shared.millis_counter.lock(|value|{ *value });
            let additional_delay = t0 as i64 + interval_ms as i64 - t1 as i64;
            if additional_delay > 0 {
                Systick::delay((additional_delay as u32).millis()).await;
            }

            // Start ADC
            cx.shared.adc_transfer.lock(|transfer| {
                transfer.start(|adc| {
                    adc.start_conversion();
                });
            });
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

    #[task(priority = 2,
        shared = [
            wanted_led_state,
            vbat_lowpass,
            drive_mode
        ],
        local = [led_pin]
    )]
    async fn led_task(mut cx: led_task::Context) {
        loop {
            let vbat_lowpass = cx.shared.vbat_lowpass.lock(|v|{ *v });
            let drive_mode = cx.shared.drive_mode.lock(|v| { *v });
            if drive_mode == DriveMode::Normal {
                if vbat_lowpass < MOTOR_CUTOFF_BATTERY_VOLTAGE {
                    cx.local.led_pin.set_state(true.into());
                    Systick::delay(100.millis()).await;
                    cx.local.led_pin.set_state(false.into());
                    Systick::delay(100.millis()).await;
                } else {
                    cx.local.led_pin.set_state(true.into());
                    Systick::delay(1000.millis()).await;
                    cx.local.led_pin.set_state(false.into());
                    Systick::delay(1000.millis()).await;
                }
            } else if drive_mode == DriveMode::Stop {
                cx.local.led_pin.set_state(true.into());
                Systick::delay(100.millis()).await;
                cx.local.led_pin.set_state(false.into());
                Systick::delay(900.millis()).await;
            } else if drive_mode == DriveMode::MotorTest {
                cx.local.led_pin.set_state(true.into());
                Systick::delay(100.millis()).await;
                cx.local.led_pin.set_state(false.into());
                Systick::delay(100.millis()).await;
                cx.local.led_pin.set_state(true.into());
                Systick::delay(100.millis()).await;
                cx.local.led_pin.set_state(false.into());
                Systick::delay(700.millis()).await;
            } else {
                let wanted_state = cx.shared.wanted_led_state.lock(|value| { *value });
                cx.local.led_pin.set_state(wanted_state.into());
            }
            Systick::delay(1.millis()).await;
        }
    }

    #[task(
        priority = 1,
        shared = [
            console_rxbuf,
            wanted_led_state,
            drive_mode,
            vbat_lowpass,
            log_sensors,
        ],
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
                    if command == ArrayString::from("test").unwrap() {
                        cx.shared.drive_mode.lock(|v| { *v = DriveMode::MotorTest });
                        info!("Drive mode: {:?}", cx.shared.drive_mode.lock(|v| { *v }));
                    } else if command == ArrayString::from("normal").unwrap() {
                        cx.shared.drive_mode.lock(|v| { *v = DriveMode::Normal });
                        info!("Drive mode: {:?}", cx.shared.drive_mode.lock(|v| { *v }));
                    } else if command == ArrayString::from("stop").unwrap() {
                        cx.shared.drive_mode.lock(|v| { *v = DriveMode::Stop });
                        info!("Drive mode: {:?}", cx.shared.drive_mode.lock(|v| { *v }));
                    } else if command == ArrayString::from("bat").unwrap() {
                        let vbat_lowpass = cx.shared.vbat_lowpass.lock(|v|{ *v });
                        info!("Battery voltage: {:?}", cx.shared.vbat_lowpass.lock(|v| { *v }));
                    } else if command == ArrayString::from("log").unwrap() {
                        let log_sensors = cx.shared.log_sensors.lock(|v|{ *v = !*v; *v });
                        info!("Sensor logging: {:?}", log_sensors);
                        if log_sensors {
                            info!("S,P0,P1,P2,P3,P4,P5,GZ,WL,WR,VB");
                        }
                    } else {
                        info!("-> {:?} is an unknown command", command);
                        info!("Available commands:");
                        info!("  normal");
                        info!("  test");
                        info!("  stop");
                        info!("  log");
                        info!("  bat");
                    }
                    // Flash LED for debugging
                    cx.shared.wanted_led_state.lock(|value| { *value = false; });
                    Systick::delay(50.millis()).await;
                    cx.shared.wanted_led_state.lock(|value| { *value = true; });
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

    #[task(binds = DMA2_STREAM0, shared = [adc_transfer, adc_result], local = [adc_buffer])]
    fn adc_dma(mut cx: adc_dma::Context) {
        let buffer = cx.shared.adc_transfer.lock(|transfer| {
            let (buffer, _) = transfer
                .next_transfer(cx.local.adc_buffer.take().unwrap())
                .unwrap();
            buffer
        });

        // Read out values from buffer
        let result = buffer.clone();

        // Return buffer to resources pool for next transfer
        *cx.local.adc_buffer = Some(buffer);

        cx.shared.adc_result.lock(|shared_result| {
            *shared_result = result.clone();
        });

        /*info!("ADC result: {:?} {:?} {:?} {:?} {:?} {:?}",
                result[0], result[1], result[2],
                result[3], result[4], result[5]);*/
    }
}

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
