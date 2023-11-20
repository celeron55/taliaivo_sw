#![no_std]

extern crate arrayvec; // Use static arrays like the embedded code

use arrayvec::ArrayVec;
use libc_print::std_name::{println, eprintln, print, dbg};
use nalgebra::{Vector2, Point2, UnitComplex, Rotation2};
use core::f32::consts::PI;

pub trait RobotInterface {
    // Capabilities and dimensions
    fn get_track_width(&self) -> f32;

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
    fn report_map(&mut self, map: &Map, robot_p: Point2<f32>, robot_r: f32,
            attack_p: Option<Point2<f32>>, scan_p: Option<Point2<f32>>);
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

    pub fn find_pattern(&self, pattern: &[f32], pattern_width: u32, pattern_height: u32,
            ignore_map_below_significance: f32, score_for_ignore: f32)
            -> Option<(u32, u32, f32)> {
        let mut best_match = None;
        let mut best_score = f32::MAX;

        for y in 0..self.height.saturating_sub(pattern_height) {
            for x in 0..self.width.saturating_sub(pattern_width) {
                let score = self.calculate_match_score(x, y, pattern, pattern_width, pattern_height,
                        ignore_map_below_significance, score_for_ignore);
                if score < best_score {
                    best_score = score;
                    best_match = Some((x, y, score));
                }
            }
        }

        best_match
    }

    fn calculate_match_score(&self, x: u32, y: u32, pattern: &[f32], pattern_width: u32,
            pattern_height: u32, ignore_map_below_significance: f32, score_for_ignore: f32) -> f32 {
        let mut score = 0.0;

        for pattern_y in 0..pattern_height {
            for pattern_x in 0..pattern_width {
                let map_x = x + pattern_x;
                let map_y = y + pattern_y;
                let map_idx = (map_y * self.width + map_x) as usize;
                let pattern_idx = (pattern_y * pattern_width + pattern_x) as usize;

                if self.data[map_idx].abs() >= ignore_map_below_significance {
                    score += (self.data[map_idx] - pattern[pattern_idx]).abs();
                } else {
                    score += score_for_ignore;
                }
            }
        }

        score
    }
}

fn limit_acceleration(previous_value: f32, target_value: f32, max_change: f32) -> f32 {
    if target_value > previous_value {
        if previous_value + max_change > target_value {
            return target_value;
        }
        return previous_value + max_change;
    } else {
        if previous_value - max_change < target_value {
            return target_value;
        }
        return previous_value - max_change;
    }
}

pub struct BrainState {
    counter: u64,
    map: Map,
    pos: Point2<f32>, // Position of robot on map
    rot: f32, // Angle of robot on map (radians)
    vel: Vector2<f32>, // Velocity of robot on map
    applied_wheel_speed_left: f32,
    applied_wheel_speed_right: f32,
    proximity_sensor_readings: ArrayVec<(f32, f32, bool), 6>,
    attack_p: Option<Point2<f32>>, // For diagnostics
    scan_p: Option<Point2<f32>>, // For diagnostics
}

impl BrainState {
    pub fn new() -> Self {
        BrainState {
            counter: 0,
            map: Map::new(),
            pos: Point2::new(100.0, 100.0),
            rot: 0.0,
            vel: Vector2::new(0.0, 0.0),
            applied_wheel_speed_left: 0.0,
            applied_wheel_speed_right: 0.0,
            proximity_sensor_readings: ArrayVec::new(),
            attack_p: None,
            scan_p: None,
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

        // TODO: Make sure this is scaled appropriately
        self.rot += gyro_z / UPS as f32;

        // TODO: If gyro_z doesn't match rotation caused by wheel speeds, assume
        // a wheel is not touching the ground and act differently

        // TODO: Maintain self.rot via motor speeds

        // TODO: Maintain self.vel via accelerometer and self.rot

        // TODO: Maintain self.vel via motor speeds and self.rot
        let avg_wheel_speed = (self.applied_wheel_speed_left + self.applied_wheel_speed_right) / 2.0;
        self.vel.x = avg_wheel_speed * self.rot.cos();
        self.vel.y = avg_wheel_speed * self.rot.sin();

        // TODO: Maintain self.pos via self.vel
        self.pos.x += self.vel.x / UPS as f32;
        self.pos.y += self.vel.y / UPS as f32;

        self.proximity_sensor_readings = robot.get_proximity_sensors();

        //println!("proximity_sensor_readings: {:?}", self.proximity_sensor_readings);

        self.map.global_forget(0.998);

        if gyro_z.abs() > PI * 2.5 {
            self.map.global_forget(0.9);
        }

        for reading in &self.proximity_sensor_readings {
            self.map.paint_proximity_reading(self.pos, reading.0 + self.rot, reading.1, reading.2);
        }

        robot.report_map(&self.map, self.pos, self.rot, self.attack_p, self.scan_p);

        let mut wanted_linear_speed = 0.0;
        let mut wanted_rotation_speed = 0.0;

        // TODO: Remove
        let s = 60.0;
        let dur = 6;
        if (self.counter % (UPS * dur)) < (UPS * dur / 2) {
            wanted_linear_speed = -s;
        } else {
            wanted_linear_speed = s;
        }

        // TODO: Remove
        if self.proximity_sensor_readings.len() >= 6 {
            let d0 = self.proximity_sensor_readings[0].1;
            let d1 = self.proximity_sensor_readings[1].1;
            let d2 = self.proximity_sensor_readings[2].1;
            let d3 = self.proximity_sensor_readings[3].1;
            let d4 = self.proximity_sensor_readings[4].1;
            let d5 = self.proximity_sensor_readings[5].1;
            // Assume the first 3 sensors are pointing somewhat forward and if they
            // all are showing short distance, don't try to push further
            // TODO: Make an exception when it has been determined that we are
            // pushing against the opponent instead of a wall
            if d0 < 10.0 && d1 < 15.0 && d2 < 15.0 {
                wanted_linear_speed = -5.0;
                wanted_rotation_speed = 1.0;
            }
            // Assume sensor [5] is pointing rearwards. Don't try to reverse more if
            // it's detecting something.
            if d5 < 10.0 {
                wanted_linear_speed = 5.0;
                wanted_rotation_speed = 0.0;
            }
            // Try to steer away from walls on the sides
            if d3 < 10.0 && d0 > 30.0 {
                wanted_linear_speed = 22.25;
                wanted_rotation_speed = 1.0;
            }
            if d4 < 10.0 && d0 > 30.0 {
                wanted_linear_speed = 22.25;
                wanted_rotation_speed = -1.0;
            }
        }

        (wanted_linear_speed, wanted_rotation_speed) = self.create_motion();

        /*// TODO: Remove or replace
        let wanted_absolute_angle = PI * 0.5;
        let max_rotation_speed = PI * 3.0;
        let max_linear_speed = 30.0;
        let target_p = Point2::new(100.0, 100.0);
        //wanted_rotation_speed = self.steer_towards_absolute_angle(
        //        wanted_absolute_angle, max_rotation_speed);
        (wanted_linear_speed, wanted_rotation_speed) = self.drive_towards_absolute_position(
                target_p, max_linear_speed, max_rotation_speed);*/

        let track = robot.get_track_width();
        let wanted_wheel_speed_left = wanted_linear_speed - wanted_rotation_speed * (track / 2.0);
        let wanted_wheel_speed_right = wanted_linear_speed + wanted_rotation_speed * (track / 2.0);

        // Limit wheel speed changes, i.e. limit acceleration
        self.applied_wheel_speed_left = limit_acceleration(self.applied_wheel_speed_left,
                wanted_wheel_speed_left, 250.0 / UPS as f32);
        self.applied_wheel_speed_right = limit_acceleration(self.applied_wheel_speed_right,
                wanted_wheel_speed_right, 250.0 / UPS as f32);

        robot.set_motor_speed(self.applied_wheel_speed_left, self.applied_wheel_speed_right);

        let mut weapon_throttle = 100.0;
        if gyro_z.abs() > 5.0 {
            weapon_throttle = 0.0;
        }
        robot.set_weapon_throttle(weapon_throttle);

        self.counter += 1;
    }

    pub fn create_motion(&mut self) -> (f32, f32) {
        // Reset diagnostic values
        self.attack_p = None;
        self.scan_p = None;

        // See if the enemy can be reasonably found on the map
        /*let pattern_w: u32 = 4;
        let pattern_h: u32 = 4;
        let pattern = [
            -100.0, -100.0, -100.0, -100.0,
            -100.0,  100.0,  100.0, -100.0,
            -100.0,  100.0,  100.0, -100.0,
            -100.0, -100.0, -100.0, -100.0,
        ];*/
        let pattern_w: u32 = 6;
        let pattern_h: u32 = 6;
        let pattern = [
            -100.0, -100.0, -100.0, -100.0, -100.0, -100.0,
            -100.0, -100.0, -100.0, -100.0, -100.0, -100.0,
            -100.0, -100.0,  100.0,  100.0, -100.0, -100.0,
            -100.0, -100.0,  100.0,  100.0, -100.0, -100.0,
            -100.0, -100.0, -100.0, -100.0, -100.0, -100.0,
            -100.0, -100.0, -100.0, -100.0, -100.0, -100.0,
        ];
        let result_maybe = self.map.find_pattern(&pattern, pattern_w, pattern_h, 30.0, 50.0);
        if let Some(result) = result_maybe {
            let score = result.2;
            println!("score: {:?}", score);
            if score < 20.0 * pattern_w as f32 * pattern_h as f32 {
                // Target the center of the pattern
                let target_p = Point2::new(
                    (result.0 + pattern_w / 2) as f32 * self.map.tile_wh,
                    (result.1 + pattern_h / 2) as f32 * self.map.tile_wh,
                );
                //println!("target_p: {:?}", target_p);
                println!("attack {:?}", target_p);
                return self.create_attack_motion(target_p);
            }
        }

        println!("scan");
        return self.create_scanning_motion();
    }

    pub fn create_scanning_motion(&mut self) -> (f32, f32) {
        let max_linear_speed = 50.0;
        let max_rotation_speed = PI * 2.0;
        // Find a good spot on the map to investigate
        // TODO: Somehow fill in the behinds of walls on the map in such a way
        //       that the robot doesn't see them as places where it can go, or
        //       in some other way make it so that the robot doesn't try to
        //       drive through walls
        let pattern_w: u32 = 6;
        let pattern_h: u32 = 6;
        let pattern = [
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
            -50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
        ];
        let result_maybe = self.map.find_pattern(&pattern, pattern_w, pattern_h, 5.0, 30.0);
        if let Some(result) = result_maybe {
            // Target the center of the pattern
            let target_p = Point2::new(
                (result.0 + pattern_w / 2) as f32 * self.map.tile_wh,
                (result.1 + pattern_h / 2) as f32 * self.map.tile_wh,
            );
            //println!("target_p: {:?}", target_p);
            self.scan_p = Some(target_p);
            // Drive towards that spot
            let (mut wanted_linear_speed, mut wanted_rotation_speed) =
                        self.drive_towards_absolute_position(
                            target_p, max_linear_speed, max_rotation_speed);
            // Apply very strong motor speed modulation to get scanning data
            //wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 7.0).sin() * 2.0;
            return (wanted_linear_speed, wanted_rotation_speed);
        }
        let wanted_linear_speed = 100.0;
        let wanted_rotation_speed = PI * 1.0;
        return (wanted_linear_speed, wanted_rotation_speed);
    }

    pub fn create_attack_motion(&mut self, target_p: Point2<f32>) -> (f32, f32) {
        self.attack_p = Some(target_p);
        // Drive towards target
        let max_linear_speed = 100.0;
        let max_rotation_speed = PI * 4.0;
        let (mut wanted_linear_speed, mut wanted_rotation_speed) =
                    self.drive_towards_absolute_position(
                        target_p, max_linear_speed, max_rotation_speed);
        // Apply some motor speed modulation to get scanning data
        wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 4.0).sin() * 1.0;
        // Revert linear speed at an interval to allow the weapon to spin up
        if (self.counter % (UPS * 4)) < (UPS * 1) {
            wanted_linear_speed *= -1.0;
        }
        return (wanted_linear_speed, wanted_rotation_speed);
    }

    // Returns a value suitable for wanted_rotation_speed
    pub fn steer_towards_absolute_angle(&self, target_angle_rad: f32,
            max_rotation_speed: f32) -> f32 {
        let angle_diff = ((target_angle_rad - self.rot + PI) % (PI * 2.0)) - PI;
        //println!("angle_diff: {:?}", angle_diff);
        let speed_factor = 0.1 + (angle_diff.abs() / PI * 0.9);
        if angle_diff > 0.0 {
            return speed_factor * max_rotation_speed;
        } else {
            return speed_factor * -max_rotation_speed;
        }
    }

    // Returns a value suitable for wanted_linear_speed and wanted_rotation_speed
    pub fn drive_towards_absolute_position(&self, target_p: Point2<f32>,
            max_linear_speed: f32, max_rotation_speed: f32) -> (f32, f32) {
        let u = target_p - self.pos;
        let heading = Rotation2::rotation_between(&Vector2::x(), &u);
        let target_angle_rad = heading.angle();
        let speed_factor = (u.magnitude() / 20.0).clamp(0.0, 1.0);
        let wanted_linear_speed = max_linear_speed * speed_factor;
        let wanted_rotation_speed = self.steer_towards_absolute_angle(
                target_angle_rad, max_rotation_speed);
        return (wanted_linear_speed, wanted_rotation_speed)
    }
}
