#![no_std]

extern crate arrayvec; // Use static arrays like the embedded code

use arrayvec::ArrayVec;
use libc_print::std_name::{println, eprintln, dbg};

pub trait RobotInterface {
    // Motor control
    fn set_motor_speed(&mut self, left_speed_cm_s: f32, right_speed_cm_s: f32);
    
    // Weapon control
    fn set_weapon_throttle(&mut self, throttle_percentage: f32); // -100 to +100

    // R/C Receiver Inputs
    fn get_rc_input_values(&self, values: &mut[&f32]); // Returns values from all R/C receiver channels

    // Sensor readings
    fn get_weapon_current(&self) -> f32; // Current in Amperes
    fn get_proximity_sensors(&self) -> ArrayVec<(f32, Option<f32>), 6>; // Returns a list of (angle (radians), distance (cm)) tuples for each sensor
    fn get_gyroscope_reading(&self) -> (f32, f32, f32); // X, Y, Z axis values
    fn get_accelerometer_reading(&self) -> (f32, f32, f32); // X, Y, Z axis values
    fn get_battery_cell_voltages(&self, values: &mut[&f32]); // Voltages of individual cells

    // LED control
    fn set_led_status(&mut self, status: bool);

    // Diagnostic data
    fn set_map(&mut self, map_width: i32, map_data: &[&i8]);
}

const UPS: u64 = 100; // Updates per second

pub struct BrainState {
    counter: u64,
}

impl BrainState {
    pub fn new() -> Self {
        BrainState {
            counter: 0,
        }
    }

    // Should be called at 10ms interval
    pub fn update(&mut self, robot: &mut dyn RobotInterface) {
        let wheel_speed_left = {
            let mut speed = 100.0;
            if (self.counter % (UPS * 6)) < (UPS * 3) {
                speed = -100.0;
            }
            speed
        };
        let wheel_speed_right = {
            let mut speed = 100.0;
            if (self.counter % (UPS * 6)) < (UPS * 3) {
                speed = -100.0;
            }
            speed
        };
        robot.set_motor_speed(wheel_speed_left, wheel_speed_right);

        robot.set_weapon_throttle(100.0);

        let proximity_sensor_readings = robot.get_proximity_sensors();

        println!("proximity_sensor_readings: {:?}", proximity_sensor_readings);

        self.counter += 1;
    }
}
