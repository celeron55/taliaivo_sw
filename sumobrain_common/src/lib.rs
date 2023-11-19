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
        let (gyro_x, gyro_y, gyro_z) = robot.get_gyroscope_reading();
        println!("gyro_z: {:?}", gyro_z);

        let mut wheel_speed_left = {
            let mut speed = 100.0;
            if (self.counter % (UPS * 6)) < (UPS * 3) {
                speed = -100.0;
            }
            speed
        };
        let mut wheel_speed_right = {
            let mut speed = 100.0;
            if (self.counter % (UPS * 6)) < (UPS * 3) {
                speed = -100.0;
            }
            speed
        };

        if gyro_z.abs() > 1.5 {
            wheel_speed_left *= 0.1;
            wheel_speed_right *= 0.1;
        }

        let proximity_sensor_readings = robot.get_proximity_sensors();

        println!("proximity_sensor_readings: {:?}", proximity_sensor_readings);

        if proximity_sensor_readings.len() >= 6 {
            if let Some(distance) = proximity_sensor_readings[0].1 {
                if distance < 20.0 {
                    wheel_speed_left = -10.0;
                    wheel_speed_right = -5.0;
                }
            }
            if let Some(distance) = proximity_sensor_readings[5].1 {
                if distance < 20.0 {
                    wheel_speed_left = 50.0;
                    wheel_speed_right = 50.0;
                }
            }
        }
            
        robot.set_motor_speed(wheel_speed_left, wheel_speed_right);

        let mut weapon_throttle = 100.0;
        if gyro_z.abs() > 1.5 {
            weapon_throttle = 0.0;
        }
        robot.set_weapon_throttle(weapon_throttle);

        self.counter += 1;
    }
}
