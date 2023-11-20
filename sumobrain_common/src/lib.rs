#![no_std]

extern crate arrayvec; // Use static arrays like the embedded code

use arrayvec::ArrayVec;
use libc_print::std_name::{println, eprintln, print, dbg};
use nalgebra::{Vector2, Point2, UnitComplex};
use core::f32::consts::PI;

pub trait RobotInterface {
    // Motor control
    fn set_motor_speed(&mut self, left_speed_cm_s: f32, right_speed_cm_s: f32);
    
    // Weapon control
    fn set_weapon_throttle(&mut self, throttle_percentage: f32); // -100 to +100

    // R/C Receiver Inputs
    fn get_rc_input_values(&self, values: &mut[&f32]); // Returns values from all R/C receiver channels

    // Sensor readings
    fn get_weapon_current(&self) -> f32; // Current in Amperes
    fn get_proximity_sensors(&self) -> ArrayVec<(f32, f32, bool), 6>; // Returns a list of (angle (radians), distance (cm), something_seen) tuples for each sensor
    fn get_gyroscope_reading(&self) -> (f32, f32, f32); // X, Y, Z axis values
    fn get_accelerometer_reading(&self) -> (f32, f32, f32); // X, Y, Z axis values
    fn get_battery_cell_voltages(&self, values: &mut[&f32]); // Voltages of individual cells

    // LED control
    fn set_led_status(&mut self, status: bool);

    // Diagnostic data
    fn report_map(&mut self, map: &Map, robot_p: Point2<f32>, robot_r: f32);
}

pub const UPS: u64 = 100; // Updates per second
const MAP_T: f32 = 5.0; // Map tile width and height in cm
const MAP_W_REAL: f32 = 200.0; // Map width in cm
const MAP_H_REAL: f32 = MAP_W_REAL;
const MAP_W: u32 = (MAP_W_REAL / MAP_T) as u32; // Map width in tiles
const MAP_H: u32 = MAP_W;
const MAP_SIZE: usize = (MAP_W * MAP_H) as usize;

#[derive(Clone)]
pub struct Map {
    pub tile_wh: f32,
    pub width: u32,
    pub height: u32,
    pub data: ArrayVec<f32, MAP_SIZE>,
}

impl Map {
    pub fn new() -> Self {
        let mut data = ArrayVec::new();
        // TODO: Figure out something better for filling the map (note: fill()
        // doesn't work)
        for _ in 0..MAP_SIZE {
            data.push(0.0);
        }
        Map {
            tile_wh: MAP_T,
            width: MAP_W,
            height: MAP_H,
            data: data,
        }
    }

    // something_seen: If nothing is found within sensor range, set this to
    // true, and set distance to the sensor maximum range
    pub fn paint_proximity_reading(&mut self, starting_position: Point2<f32>,
            angle_rad: f32, distance: f32, something_seen: bool) {
        // Calculate end point of the ray
        let direction: Vector2<f32> = Vector2::new(angle_rad.cos(), angle_rad.sin());
        let end_point = starting_position + direction * distance;

        // Convert to tile indices
        let mut x0 = (starting_position.x / self.tile_wh) as i32;
        let mut y0 = (starting_position.y / self.tile_wh) as i32;
        let x1 = (end_point.x / self.tile_wh) as i32;
        let y1 = (end_point.y / self.tile_wh) as i32;

        // Bresenham's line algorithm
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        loop {
            // Paint the current tile
            if let Some(tile) = self.data.get_mut((y0 as u32 * self.width + x0 as u32) as usize) {
                *tile = -100.0;
            }

            if x0 == x1 && y0 == y1 { break; }
            let e2 = 2 * err;
            if e2 >= dy { err += dy; x0 += sx; }
            if e2 <= dx { err += dx; y0 += sy; }
        }

        if something_seen {
            // Paint the end tile
            if let Some(tile) = self.data.get_mut((y1 as u32 * self.width + x1 as u32) as usize) {
                *tile = 100.0;
            }
        }
    }

    pub fn global_forget(&mut self, factor: f32) {
        for y in 0..self.height {
            for x in 0..self.width {
                let idx = (y * self.width + x) as usize;
                let tile_value = &mut self.data[idx];
                *tile_value *= factor;
            }
        }
    }

    pub fn translate(&mut self, dx: i32, dy: i32) {
        let mut new_data = ArrayVec::<f32, MAP_SIZE>::new();
        // TODO: Figure out something better for filling the map (note: fill()
        // doesn't work)
        for _ in 0..MAP_SIZE {
            new_data.push(0.0);
        }

        for y in 0..self.height {
            for x in 0..self.width {
                let new_x = x as i32 + dx;
                let new_y = y as i32 + dy;

                if new_x >= 0 && new_x < self.width as i32 && new_y >= 0 && new_y < self.height as i32 {
                    let old_idx = (y * self.width + x) as usize;
                    let new_idx = (new_y as u32 * self.width + new_x as u32) as usize;
                    new_data[new_idx] = self.data[old_idx];
                }
            }
        }

        self.data = new_data;
    }

    pub fn print(&self, robot_pos: Point2<f32>) {
        let robot_x = (robot_pos.x / self.tile_wh).round() as u32;
        let robot_y = (robot_pos.y / self.tile_wh).round() as u32;
        for y in 0..self.height {
            for x in 0..self.width {
                let idx = (y * self.width + x) as usize;
                let tile_value = self.data[idx];
                let symbol = if robot_x == x && robot_y == y {
                    "R"
                } else if tile_value < -50.0 {
                    " "
                } else if tile_value < -10.0 {
                    "."
                } else if tile_value < 10.0 {
                    "+"
                } else if tile_value < 50.0 {
                    "x"
                } else {
                    "X"
                };
                print!(" {}", symbol);
            }
            println!(); // New line at the end of each row
        }
    }
}

pub struct BrainState {
    counter: u64,
    map: Map,
    pos: Point2<f32>, // Position of robot on map
    rot: f32, // Angle of robot on map (radians)
    vel: Vector2<f32>, // Velocity of robot on map
}

impl BrainState {
    pub fn new() -> Self {
        BrainState {
            counter: 0,
            map: Map::new(),
            pos: Point2::new(100.0, 100.0),
            rot: 0.0,
            vel: Vector2::new(0.0, 0.0),
        }
    }

    // Should be called at 1.0s / UPS interval
    pub fn update(&mut self, robot: &mut dyn RobotInterface) {
        // Move robot closer to the center of map if it's near an edge
        let edge = 20.0;
        if (self.pos.x - MAP_W_REAL / 2.0).abs() > MAP_W_REAL / 4.0 ||
                (self.pos.y - MAP_H_REAL / 2.0).abs() > MAP_H_REAL / 4.0 {
            let ideal_translate = Point2::new(MAP_W_REAL/2.0, MAP_H_REAL/2.0) - self.pos;
            let dx_tiles = (ideal_translate.x / self.map.tile_wh).round() as i32;
            let dy_tiles = (ideal_translate.y / self.map.tile_wh).round() as i32;
            let final_translate = Vector2::new(dx_tiles as f32 * self.map.tile_wh, dy_tiles as f32 * self.map.tile_wh);
            self.pos += final_translate;
            // Transform the map when moving the robot closer to the center of
            // the map so that the map hopefully somewhat matches the
            // surroundings
            self.map.translate(dx_tiles, dy_tiles);
        }

        let (gyro_x, gyro_y, gyro_z) = robot.get_gyroscope_reading();
        println!("gyro_z: {:?}", gyro_z);

        let s = 60.0;
        let dur = 6;
        let mut wheel_speed_left = {
            let mut speed = s;
            if (self.counter % (UPS * dur)) < (UPS * dur / 2) {
                speed = -s;
            }
            speed
        };
        let mut wheel_speed_right = {
            let mut speed = s;
            if (self.counter % (UPS * dur)) < (UPS * dur / 2) {
                speed = -s;
            }
            speed
        };

        let proximity_sensor_readings = robot.get_proximity_sensors();

        //println!("proximity_sensor_readings: {:?}", proximity_sensor_readings);

        self.map.global_forget(0.998);

        if gyro_z.abs() > 5.0 {
            self.map.global_forget(0.9);
        }

        for reading in &proximity_sensor_readings {
            self.map.paint_proximity_reading(self.pos, reading.0 + self.rot, reading.1, reading.2);
        }

        robot.report_map(&self.map, self.pos, self.rot);

        if proximity_sensor_readings.len() >= 6 {
            let d0 = proximity_sensor_readings[0].1;
            let d1 = proximity_sensor_readings[1].1;
            let d2 = proximity_sensor_readings[2].1;
            let d3 = proximity_sensor_readings[3].1;
            let d4 = proximity_sensor_readings[4].1;
            let d5 = proximity_sensor_readings[5].1;
            // Assume the first 3 sensors are pointing somewhat forward and if they
            // all are showing short distance, don't try to push further
            // TODO: Make an exception when it has been determined that we are
            // pushing against the opponent instead of a wall
            if d0 < 10.0 && d1 < 15.0 && d2 < 15.0 {
                wheel_speed_left = -10.0;
                wheel_speed_right = -5.0;
            }
            // Assume sensor [5] is pointing rearwards. Don't try to reverse more if
            // it's detecting something.
            if d5 < 10.0 {
                wheel_speed_left = 5.0;
                wheel_speed_right = 5.0;
            }
            // Try to steer away from walls on the sides
            if d3 < 10.0 && d0 > 30.0 {
                wheel_speed_left = 15.0;
                wheel_speed_right = 30.0;
            }
            if d4 < 10.0 && d0 > 30.0 {
                wheel_speed_left = 30.0;
                wheel_speed_right = 15.0;
            }
        }

        // TODO: Limit wheel speed changes, i.e. limit acceleration

        // Modulate motor speeds a bit to generate better sensor data
        wheel_speed_left += (self.counter as f32 / UPS as f32 * 10.0).sin() * 5.0;
        wheel_speed_right += (self.counter as f32 / UPS as f32 * 10.0 + PI).sin() * 5.0;

        robot.set_motor_speed(wheel_speed_left, wheel_speed_right);

        let mut weapon_throttle = 100.0;
        if gyro_z.abs() > 5.0 {
            weapon_throttle = 0.0;
        }
        robot.set_weapon_throttle(weapon_throttle);

        // TODO: Make sure this is scaled appropriately
        self.rot += gyro_z / UPS as f32;

        // TODO: If gyro_z doesn't match rotation caused by wheel speeds, assume
        // a wheel is not touching the ground and act differently

        // TODO: Maintain self.rot via motor speeds

        // TODO: Maintain self.vel via accelerometer and self.rot

        // TODO: Maintain self.vel via motor speeds and self.rot
        let avg_wheel_speed = (wheel_speed_left + wheel_speed_right) / 2.0;
        self.vel.x = avg_wheel_speed * self.rot.cos();
        self.vel.y = avg_wheel_speed * self.rot.sin();

        // TODO: Maintain self.pos via self.vel
        self.pos.x += self.vel.x / UPS as f32;
        self.pos.y += self.vel.y / UPS as f32;

        self.counter += 1;
    }
}
