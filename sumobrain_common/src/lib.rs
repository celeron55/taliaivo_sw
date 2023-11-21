#![no_std]

extern crate arrayvec; // Use static arrays like the embedded code
pub mod map;

use arrayvec::ArrayVec;
use libc_print::std_name::{println};
use nalgebra::{Vector2, Point2, Rotation2};
use core::f32::consts::PI;
pub use map::*;

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
            attack_p: Option<Point2<f32>>, scan_p: Option<Point2<f32>>,
            lines: &[HoughLine]);
}

pub const UPS: u32 = 100; // Updates per second

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
    seed: u32,
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
    attack_step_count: u32,
}

impl BrainState {
    pub fn new(seed: u32) -> Self {
        BrainState {
            seed: seed,
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
            attack_step_count: 0,
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
        //println!("gyro_z: {:?}", gyro_z);

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

        if gyro_z.abs() > PI * 5.0 {
            self.map.global_forget(0.9);
        }

        for reading in &self.proximity_sensor_readings {
            self.map.paint_proximity_reading(self.pos, reading.0 + self.rot, reading.1, reading.2);
        }

        let mut lines = self.map.hough_transform();
        for line in &lines {
            println!("HoughLine: angle={:?} distance={:?} votes={:?}",
                    line.angle, line.distance, line.votes);
        }

        robot.report_map(&self.map, self.pos, self.rot, self.attack_p, self.scan_p, &lines);

        // TODO
        // Avoid walls as found by hough_transform
        let robot_tile_position = self.pos.coords * (1.0 / self.map.tile_wh);
        let robot_direction_vector = Vector2::new(self.rot.cos(), self.rot.sin());
        let mut shortest_distance: f32 = f32::MAX;
        // TODO: Instead of or in addition to measuring head-on distance, just
        //       measure distance from walls to any direction?
        const AVOID_WALL_HEAD_ON_DISTANCE: f32 = 30.0; // cm
        const IMPORTANCE_CONSTANT: f32 = 1.0; // tiles
        let mut wall_avoidance_vector = Vector2::new(0.0, 0.0);
        //println!("At: p(tiles)={:?}, direction={:?}", robot_tile_position, robot_direction_vector);
        for line in lines {
            if let Some(intersection_point_tiles) = calculate_intersection(
                    robot_tile_position, robot_direction_vector, &line) {
                let distance_tiles = (robot_tile_position - intersection_point_tiles).magnitude();
                /*println!("On trajectory to hit: p(tiles)={:?}, distance(tiles)={:?}",
                        intersection_point_tiles, distance_tiles);*/
                let distance = distance_tiles * self.map.tile_wh;
                let intersection_point = intersection_point_tiles * self.map.tile_wh;
                if distance < shortest_distance {
                    shortest_distance = distance;
                }
            }

            let vector_from_wall_to_robot = line.vector_to_point(robot_tile_position);
            let distance_from_line_tiles = vector_from_wall_to_robot.magnitude();
            let importance = 1.0 / (distance_from_line_tiles + IMPORTANCE_CONSTANT);
            wall_avoidance_vector += vector_from_wall_to_robot * importance;
        }
        if shortest_distance < f32::MAX {
            println!("Shortest distance to wall collision: {:?}", shortest_distance);
        }
        if wall_avoidance_vector.magnitude() > 0.0 {
            println!("Walls would be best avoided by moving towards: {:?}", wall_avoidance_vector);
        }

        let mut wanted_linear_speed = 0.0;
        let mut wanted_rotation_speed = 0.0;

        (wanted_linear_speed, wanted_rotation_speed) = self.create_motion();

        let track = robot.get_track_width();
        let wanted_wheel_speed_left = wanted_linear_speed - wanted_rotation_speed * (track / 2.0);
        let wanted_wheel_speed_right = wanted_linear_speed + wanted_rotation_speed * (track / 2.0);

        // Limit wheel speed changes, i.e. limit acceleration
        self.applied_wheel_speed_left = limit_acceleration(self.applied_wheel_speed_left,
                wanted_wheel_speed_left, 250.0 / UPS as f32);
        self.applied_wheel_speed_right = limit_acceleration(self.applied_wheel_speed_right,
                wanted_wheel_speed_right, 250.0 / UPS as f32);

        // Rotation has to be prioritized, thus if the wanted wheel speed
        // difference wasn't applied, force it, ignoring acceleration
        // TODO: Rethink this
        /*let avg_applied_speed = (self.applied_wheel_speed_left + self.applied_wheel_speed_right) / 2.0;
        let wanted_difference = wanted_wheel_speed_left - wanted_wheel_speed_right;
        self.applied_wheel_speed_left = avg_applied_speed + wanted_difference / 2.0;
        self.applied_wheel_speed_right = avg_applied_speed - wanted_difference / 2.0;*/

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

        // TODO:
        // Detect if the robot is moving weapon first into arena wall, and if
        // so, prioritize a wall avoidance maneuver

        // See if the enemy can be reasonably found on the map

        // TODO: Improve enemy finding
        // - The enemy can take many shapes within 2x2 tiles
        // - The enemy can be against a wall
        // - The age of information about the enemy can vary

        //let score_requirement = 3.4;
        let score_requirement = 3.0;
        let pattern_w: u32 = 6;
        let pattern_h: u32 = 6;
        let pattern = [
            false, false, false, false, false, false,
            false, false, false, false, false, false,
            false, false, true , true , false, false,
            false, false, true , true , false, false,
            false, false, false, false, false, false,
            false, false, false, false, false, false,
        ];
        let weights = [
            0.1, 0.2, 0.2, 0.2, 0.2, 0.1,
            0.2, 0.8, 0.8, 0.8, 0.8, 0.2,
            0.2, 0.8, 1.0, 1.0, 0.8, 0.2,
            0.2, 0.8, 1.0, 1.0, 0.8, 0.2,
            0.2, 0.8, 0.8, 0.8, 0.8, 0.2,
            0.1, 0.2, 0.2, 0.2, 0.2, 0.1,
        ];
        let result_maybe = self.map.find_binary_pattern(
                &pattern, pattern_w, pattern_h, 30.0, 0.5, Some(&weights));
        if let Some(result) = result_maybe {
            let score = result.2;
            //println!("score: {:?}", score);
            if score < score_requirement {
                // Target the center of the pattern
                let target_p = Point2::new(
                    (result.0 + pattern_w / 2) as f32 * self.map.tile_wh,
                    (result.1 + pattern_h / 2) as f32 * self.map.tile_wh,
                );
                //println!("attack {:?}", target_p);
                return self.create_attack_motion(target_p);
            }
        }

        //println!("scan");
        self.attack_step_count = 0;
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
        let v = -40.0;
        let pattern = [
            v, v, v, v, v, v,
            v, v, v, v, v, v,
            v, v, v, v, v, v,
            v, v, v, v, v, v,
            v, v, v, v, v, v,
            v, v, v, v, v, v,
        ];
        let result_maybe = self.map.find_pattern(&pattern, pattern_w, pattern_h, 5.0, 100.0);
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
            // Apply motor speed modulation to get scanning data
            wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 10.0).sin() * 1.5;
            /*// Stupid logic
            if self.proximity_sensor_readings.len() >= 6 {
                let d0 = self.proximity_sensor_readings[0].1;
                let d1 = self.proximity_sensor_readings[1].1;
                let d2 = self.proximity_sensor_readings[2].1;
                let d3 = self.proximity_sensor_readings[3].1;
                let d4 = self.proximity_sensor_readings[4].1;
                let d5 = self.proximity_sensor_readings[5].1;
                // Assume the first 3 sensors are pointing somewhat forward and if they
                // all are showing short distance, don't try to push further
                if d0 < 10.0 && d1 < 15.0 && d2 < 15.0 {
                    wanted_linear_speed = -15.0;
                    wanted_rotation_speed = PI * 1.0;
                }
            }*/
            return (wanted_linear_speed, wanted_rotation_speed);
        }
        let wanted_linear_speed = 100.0;
        let wanted_rotation_speed = PI * 1.0;
        return (wanted_linear_speed, wanted_rotation_speed);
    }

    pub fn create_attack_motion(&mut self, target_p: Point2<f32>) -> (f32, f32) {
        self.attack_p = Some(target_p);
        // Drive towards target
        // TODO: Make it so that a higher speed like this is controllable
        /*let max_linear_speed = 200.0;
        let max_rotation_speed = PI * 4.0;*/
        let max_linear_speed = 100.0;
        let max_rotation_speed = PI * 4.0;
        let (mut wanted_linear_speed, mut wanted_rotation_speed) =
                    self.drive_towards_absolute_position(
                        target_p, max_linear_speed, max_rotation_speed);
        // Apply some motor speed modulation to get scanning data to help stay
        // on target
        wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 10.0).sin() * 1.5;
        // Revert linear speed at an interval to allow the weapon to spin up
        if self.attack_step_count > UPS * 2 &&
                ((self.attack_step_count as u32) % (UPS * 4)) < (UPS * 1) {
            wanted_linear_speed *= -1.0;
        }
        self.attack_step_count += 1;
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
    // Prioritizes steering first, then picks up speed
    pub fn drive_towards_absolute_position(&self, target_p: Point2<f32>,
            max_linear_speed: f32, max_rotation_speed: f32) -> (f32, f32) {
        let u = target_p - self.pos;
        let heading = Rotation2::rotation_between(&Vector2::x(), &u);
        let target_angle_rad = heading.angle();
        let angle_diff = ((target_angle_rad - self.rot + PI) % (PI * 2.0)) - PI;
        let mut speed_factor = ((u.magnitude() / 20.0).clamp(0.0, 1.0) - angle_diff / PI * 2.0).clamp(0.0, 1.0);
        let wanted_linear_speed = max_linear_speed * speed_factor;
        let wanted_rotation_speed = self.steer_towards_absolute_angle(
                target_angle_rad, max_rotation_speed);
        return (wanted_linear_speed, wanted_rotation_speed)
    }
}
