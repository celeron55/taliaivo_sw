#![no_std]

extern crate arrayvec; // Use static arrays like the embedded code
extern crate ringbuffer;
pub mod map;

use arrayvec::ArrayVec;

use nalgebra::{Vector2, Point2, Rotation2};
use core::f32::consts::PI;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
pub use map::*;

pub const UPS: u32 = 100; // Updates per second
const ENEMY_HISTORY_LENGTH: usize = 50;
const ARENA_DIMENSION: f32 = 125.0; // cm. Used to predict opposing walls.
const AGGRESSIVENESS: f32 = -1.0; // roughly -1.0...1.0, 0.0 = normal aggressiveness
const MAX_LINEAR_SPEED: f32 = 100.0;
const MAX_ROTATION_SPEED: f32 = PI * 4.0;

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
            attack_p: Option<Point2<f32>>,
            scan_p: Option<Point2<f32>>,
            wall_avoid_p: Option<Point2<f32>>,
            wall_lines: &[HoughLine]);
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

fn average_enemy_position_over_recent_ticks(
        buffer: &ConstGenericRingBuffer<(u64, Point2<f32>), ENEMY_HISTORY_LENGTH>,
        current_tick: u64, max_age: u64) -> Option<Point2<f32>> {
    let min_tick = current_tick - max_age;
    let mut sum_position = Point2::new(0.0, 0.0);
    let mut count = 0;

    for &(tick, position) in buffer.iter() {
        if tick >= min_tick {
            sum_position.coords += position.coords;
            count += 1;
        }
    }

    if count > 0 {
        Some(Point2::new(sum_position.x / count as f32, sum_position.y / count as f32))
    } else {
        None // No data for the specified range
    }
}

fn average_enemy_position_and_velocity_over_recent_ticks(
    buffer: &ConstGenericRingBuffer<(u64, Point2<f32>), ENEMY_HISTORY_LENGTH>,
    current_tick: u64,
    max_age: u64,
) -> Option<(Point2<f32>, Vector2<f32>)> {
    let min_tick = current_tick.saturating_sub(max_age);
    let mut sum_position = Point2::new(0.0, 0.0);
    let mut initial_position = None;
    let mut final_position = None;
    let mut count = 0;

    for &(tick, position) in buffer.iter() {
        if tick >= min_tick {
            sum_position.coords += position.coords;
            count += 1;
            final_position = Some((tick, position));
            initial_position = initial_position.or(Some((tick, position)));
        }
    }

    if count > 0 && initial_position.is_some() && final_position.is_some() {
        let (initial_tick, initial_pos) = initial_position.unwrap();
        let (final_tick, final_pos) = final_position.unwrap();
        let time_diff = final_tick as f32 - initial_tick as f32;
        if time_diff > 0.0 {
            let average_position = Point2::new(sum_position.x / count as f32, sum_position.y / count as f32);
            let velocity = Vector2::new(
                (final_pos.x - initial_pos.x) / time_diff,
                (final_pos.y - initial_pos.y) / time_diff,
            );
            Some((average_position, velocity))
        } else {
            None
        }
    } else {
        None
    }
}

// Works in tile units
fn tile_is_close_to_wall(p: Point2<f32>,
        wall_lines: &ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES>,
        threshold_distance: f32) -> bool {
    for line in wall_lines {
        if line.distance(p.coords) < threshold_distance {
            return true;
        }
    }
    return false;
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
    wall_avoid_p: Option<Point2<f32>>, // For diagnostics
    attack_step_count: u32,
    wall_lines: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES>,
    shortest_wall_head_on_distance: f32,
    wall_avoidance_vector: Vector2<f32>,
    shortest_wall_distance: f32,
    enemy_history: ConstGenericRingBuffer::<(u64, Point2<f32>), ENEMY_HISTORY_LENGTH>,
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
            wall_avoid_p: None,
            attack_step_count: 0,
            wall_lines: ArrayVec::new(),
            shortest_wall_head_on_distance: f32::MAX,
            wall_avoidance_vector: Vector2::new(0.0, 0.0),
            shortest_wall_distance: f32::MAX,
            enemy_history: ConstGenericRingBuffer::new(),
        }
    }

    // Should be called at 1.0s / UPS interval
    pub fn update(&mut self, robot: &mut dyn RobotInterface) {
        // Move robot closer to the center of map if it's near an edge
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

        let robot_tile_position = self.pos.coords * (1.0 / self.map.tile_wh);
        let robot_direction_vector = Vector2::new(self.rot.cos(), self.rot.sin());
        //println!("At: p(tiles)={:?}, direction={:?}", robot_tile_position, robot_direction_vector);

        let (_gyro_x, _gyro_y, gyro_z) = robot.get_gyroscope_reading();
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

        // 0.998 works for keeping up to date with a 125x125cm arena
        self.map.global_forget(0.998);

        if gyro_z.abs() > PI * 5.0 {
            self.map.global_forget(0.9);
        }

        for reading in &self.proximity_sensor_readings {
            let maybe_newly_occupied = self.map.paint_proximity_reading(
                    self.pos, reading.0 + self.rot, reading.1, reading.2, -40.0);
            if AGGRESSIVENESS >= 0.2 {
                if let Some(newly_occupied_p) = maybe_newly_occupied {
                    //println!("Newly occupied: {:?}", newly_occupied_p);
                    // Filter out point if it is close to a wall
                    let newly_occupied_p_f = Point2::new(
                            newly_occupied_p.x as f32,
                            newly_occupied_p.y as f32);
                    if !tile_is_close_to_wall(newly_occupied_p_f, &self.wall_lines, 3.0) {
                        // Detect if the newly occupied point behind us regarding to our
                        // movement direction. That would mean it is likely the newly
                        // occupied point was not caused by our movement and instead it
                        // is the enemy trying to catch up on us. If it is such a point,
                        // add it to enemy_history.
                        //let robot_rotation_vector = Vector2::new(self.rot.cos(), self.rot.sin());
                        let robot_direction_vector = self.vel;
                        let newly_occupied_p_world = Point2::new(
                                newly_occupied_p.x as f32 * self.map.tile_wh,
                                newly_occupied_p.y as f32 * self.map.tile_wh);
                        let point_direction_vector = newly_occupied_p_world - self.pos;
                        if robot_direction_vector.dot(&point_direction_vector) < 0.0 ||
                                AGGRESSIVENESS >= 0.6 {
                            self.enemy_history.push((self.counter, newly_occupied_p_world));
                        }
                        //println!("Isn't close to wall");
                    } else {
                        //println!("Is close to wall");
                    }
                }
            }
        }

        self.wall_lines = self.map.hough_transform();
        /*for line in &self.wall_lines {
            println!("HoughLine: angle={:?} distance={:?} votes={:?}",
                    line.angle, line.distance, line.votes);
        }*/

        // Guess opposing wall positions and add them also
        let arena_dimension_tiles = ARENA_DIMENSION / self.map.tile_wh;
        let mut new_wall_lines: ArrayVec<HoughLine, MAX_NUM_LINE_CANDIDATES> = ArrayVec::new();
        for line in &self.wall_lines {
            let line2 = line.move_towards_point(robot_tile_position, arena_dimension_tiles);
            new_wall_lines.push(line2);
        }
        self.wall_lines.extend(new_wall_lines);

        robot.report_map(&self.map, self.pos, self.rot, self.attack_p, self.scan_p,
                self.wall_avoid_p, &self.wall_lines);

        // Process wall lines

        self.shortest_wall_head_on_distance = f32::MAX;
        for line in &self.wall_lines {
            if let Some(intersection_point_tiles) = calculate_intersection(
                    robot_tile_position, robot_direction_vector, &line) {
                let distance_tiles = (robot_tile_position - intersection_point_tiles).magnitude();
                /*println!("On trajectory to hit: p(tiles)={:?}, distance(tiles)={:?}",
                        intersection_point_tiles, distance_tiles);*/
                let distance = distance_tiles * self.map.tile_wh;
                let _intersection_point = intersection_point_tiles * self.map.tile_wh;
                if distance < self.shortest_wall_head_on_distance {
                    self.shortest_wall_head_on_distance = distance;
                }
            }
        }
        /*if self.shortest_wall_head_on_distance < f32::MAX {
            println!("Shortest distance to head-on wall collision: {:?}",
                    self.shortest_wall_head_on_distance);
        }*/

        const IMPORTANCE_CONSTANT: f32 = 15.0;
        self.wall_avoidance_vector = Vector2::new(0.0, 0.0);
        self.shortest_wall_distance = f32::MAX;
        for line in &self.wall_lines {
            let vector_from_wall_to_robot = line.vector_to_point(robot_tile_position);
            let distance_from_line_tiles = vector_from_wall_to_robot.magnitude();
            let importance = 1.0 / (distance_from_line_tiles +
                    IMPORTANCE_CONSTANT / self.map.tile_wh);
            self.wall_avoidance_vector += vector_from_wall_to_robot.normalize() * importance;
            let distance = distance_from_line_tiles * self.map.tile_wh;
            if distance < self.shortest_wall_distance {
                self.shortest_wall_distance = distance;
            }
        }
        /*if self.wall_avoidance_vector.magnitude() > 0.0 {
            println!("Walls (shortest distance: {:?}) would be best avoided by moving towards: {:?}",
                    self.shortest_wall_distance, self.wall_avoidance_vector);
        }*/

        let mut wanted_linear_speed = 0.0;
        let mut wanted_rotation_speed = 0.0;

        (wanted_linear_speed, wanted_rotation_speed) = self.create_motion();

        let track = robot.get_track_width();
        let wanted_wheel_speed_left = wanted_linear_speed - wanted_rotation_speed * (track / 2.0);
        let wanted_wheel_speed_right = wanted_linear_speed + wanted_rotation_speed * (track / 2.0);

        // Limit wheel speed changes, i.e. limit acceleration
        let max_accel = 250.0 / UPS as f32;
        self.applied_wheel_speed_left = limit_acceleration(self.applied_wheel_speed_left,
                wanted_wheel_speed_left, max_accel);
        self.applied_wheel_speed_right = limit_acceleration(self.applied_wheel_speed_right,
                wanted_wheel_speed_right, max_accel);

        // Rotation has to be prioritized, thus if the wanted wheel speed
        // difference wasn't applied, force it, ignoring acceleration
        // TODO: Rethink this
        let avg_applied_speed = (self.applied_wheel_speed_left + self.applied_wheel_speed_right) / 2.0;
        let wanted_difference = wanted_wheel_speed_left - wanted_wheel_speed_right;
        self.applied_wheel_speed_left = avg_applied_speed + wanted_difference / 2.0;
        self.applied_wheel_speed_right = avg_applied_speed - wanted_difference / 2.0;

        robot.set_motor_speed(self.applied_wheel_speed_left, self.applied_wheel_speed_right);

        let weapon_throttle = 100.0;
        /*if gyro_z.abs() > 5.0 {
            weapon_throttle = 0.0;
        }*/
        robot.set_weapon_throttle(weapon_throttle);

        self.counter += 1;
    }

    pub fn get_wall_safe_linear_speed(&self) -> f32 {
        if self.proximity_sensor_readings.len() >= 6 {
            let d0 = self.proximity_sensor_readings[0].1;
            let d1 = self.proximity_sensor_readings[1].1;
            let d2 = self.proximity_sensor_readings[2].1;
            let _d3 = self.proximity_sensor_readings[3].1;
            let _d4 = self.proximity_sensor_readings[4].1;
            let _d5 = self.proximity_sensor_readings[5].1;
            // Assume the first 3 sensors are pointing somewhat forward and if they
            // all are showing short distance, don't try to push further
            let mut safe_linear_speed = 100.0;
            let L = 30.0;
            let L2 = 20.0;
            if d0 < L {
                safe_linear_speed *= 0.5;
            }
            if d0 < L2 {
                safe_linear_speed *= 0.5;
            }
            if d1 < L {
                safe_linear_speed *= 0.8;
            }
            if d1 < L2 {
                safe_linear_speed *= 0.8;
            }
            if d2 < L {
                safe_linear_speed *= 0.8;
            }
            if d2 < L2 {
                safe_linear_speed *= 0.8;
            }
            return safe_linear_speed;
        } else {
            return 50.0;
        }
    }

    pub fn create_motion(&mut self) -> (f32, f32) {
        // Reset diagnostic values
        self.attack_p = None;
        self.scan_p = None;
        self.wall_avoid_p = None;

        // See if the enemy can be reasonably found on the map and if so, add it
        // to the enemy history ringbuffer

        // NOTE: Higher score is easier match
        let score_requirement = 3.2 + AGGRESSIVENESS;
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
        // Filter out positions that are behind or close to walls
        let robot_tile = self.pos.coords * (1.0 / self.map.tile_wh);
        let wall_filter = |x: u32, y: u32| -> bool {
            // Take into account pattern size and use the middle as the target
            // point
            let point_tile = Vector2::new(x as f32 + pattern_w as f32 / 2.0,
                    y as f32 + pattern_h as f32 / 2.0);
            for line in &self.wall_lines {
                // If the distance from the point to the wall is smaller than a
                // set threshold, we don't want to investigate the point as it
                // would be unsafe
                let d_point_to_wall = line.distance(point_tile);
                if d_point_to_wall < 5.0 - AGGRESSIVENESS * 2.0 {
                    return false;
                }
                // If the distance from the robot to the wall is smaller than
                // the distance from the robot to the point, then the point is
                // beyond the wall
                let d_robot_to_wall = line.distance(robot_tile);
                let d_robot_to_point = (robot_tile - point_tile).magnitude();
                if d_robot_to_wall < d_robot_to_point {
                    return false;
                }
            }
            true
        };
        let result_maybe = self.map.find_binary_pattern(
                &pattern, pattern_w, pattern_h, 30.0, 0.5, Some(&weights), wall_filter);
        if let Some(result) = result_maybe {
            let score = result.2;
            //println!("score: {:?}", score);
            if score < score_requirement {
                // Target the center of the pattern
                let target_p = Point2::new(
                    (result.0 + pattern_w / 2) as f32 * self.map.tile_wh,
                    (result.1 + pattern_h / 2) as f32 * self.map.tile_wh,
                );
                self.enemy_history.push((self.counter, target_p.clone()));
                //println!("enemy_history.len(): {:?}", self.enemy_history.len());
                //println!("attack {:?}", target_p);
                //return self.create_attack_motion(target_p);
            }
        }

        // See if the enemy history ringbuffer looks such that we can determine
        // where the enemy is. If so, attack the enemy.
        let r = average_enemy_position_and_velocity_over_recent_ticks(
                &self.enemy_history, self.counter, (UPS as f32 * 0.10) as u64);
        if let Some((p, v)) = r {
            // Try to predict the enemy position a bit
            let target_p = p + v * (UPS as f32 * 0.05);
            return self.create_attack_motion(target_p);
        }

        // Avoid walls as a higher priority than scanning
        if self.shortest_wall_distance < 15.0 ||
                self.shortest_wall_head_on_distance < 40.0 {
            //println!("Avoiding walls by moving towards: {:?}", self.wall_avoidance_vector);
            let target_p = self.pos + self.wall_avoidance_vector.normalize() * 40.0;
            self.wall_avoid_p = Some(target_p);
            let max_linear_speed = self.get_wall_safe_linear_speed();
            let max_rotation_speed = MAX_ROTATION_SPEED * 0.75;
            let (mut wanted_linear_speed, mut wanted_rotation_speed) =
                        self.drive_towards_absolute_position(
                            target_p, max_linear_speed, max_rotation_speed, true);
            if self.shortest_wall_head_on_distance < 20.0 {
                wanted_linear_speed = -max_linear_speed * 0.2;
            } else if self.shortest_wall_head_on_distance < 40.0 {
                wanted_linear_speed = max_linear_speed * 0.05;
            }
            // Apply motor speed modulation to get scanning data
            wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 10.0).sin() * 1.5;
            return (wanted_linear_speed, wanted_rotation_speed);
        }

        // Avoid hits also with stupid logic. The wall detection is not always
        // fast enough as it requires a certain length of wall to be seen.
        if self.proximity_sensor_readings.len() >= 6 {
            let d0 = self.proximity_sensor_readings[0].1;
            let d1 = self.proximity_sensor_readings[1].1;
            let d2 = self.proximity_sensor_readings[2].1;
            let _d3 = self.proximity_sensor_readings[3].1;
            let _d4 = self.proximity_sensor_readings[4].1;
            let _d5 = self.proximity_sensor_readings[5].1;
            // Assume the first 3 sensors are pointing somewhat forward and if they
            // all are showing short distance, don't try to push further
            let mut wanted_linear_speed = 0.0;
            let mut wanted_rotation_speed = PI * 0.0;
            let d = 15.0;
            if d0 < d {
                //println!("Stupid hit avoidance logic: Reverse");
                wanted_linear_speed = -50.0;
            }
            if d1 < d {
                //println!("Stupid hit avoidance logic: Turn right");
                wanted_rotation_speed = PI * 2.0;
            }
            if d2 < d {
                //println!("Stupid hit avoidance logic: Turn left");
                wanted_rotation_speed = PI * -2.0;
            }
            if wanted_linear_speed != 0.0 || wanted_rotation_speed != 0.0 {
                return (wanted_linear_speed, wanted_rotation_speed);
            }
        }

        //println!("scan");
        self.attack_step_count = 0;
        return self.create_scanning_motion();
    }

    pub fn create_scanning_motion(&mut self) -> (f32, f32) {
        let max_linear_speed = self.get_wall_safe_linear_speed();
        let max_rotation_speed = MAX_ROTATION_SPEED * 0.5;
        // Find a good spot on the map to investigate
        let pattern_w: u32 = 6;
        let pattern_h: u32 = 6;
        let pattern = [
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
            -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
        ];
        // Filter out positions that are behind or close to walls
        let robot_tile = self.pos.coords * (1.0 / self.map.tile_wh);
        let wall_filter = |x: u32, y: u32| -> bool {
            // Take into account pattern size and use the middle as the target
            // point
            let point_tile = Vector2::new(x as f32 + pattern_w as f32 / 2.0,
                    y as f32 + pattern_h as f32 / 2.0);
            for line in &self.wall_lines {
                // We don't want to scan very close to ourselves
                let v_robot_to_point = point_tile - robot_tile;
                let d_robot_to_point = v_robot_to_point.magnitude();
                if d_robot_to_point < 4.0 {
                    return false;
                }

                // If the distance from the point to the wall is smaller than a
                // set threshold, we don't want to investigate the point as it
                // would be unsafe
                let d_point_to_wall = line.distance(point_tile);
                if d_point_to_wall < 6.0 {
                    return false;
                }

                // Check whether the wall is between the robot and the point
                if let Some(intersection_tile) = calculate_intersection(
                        robot_tile, v_robot_to_point, &line) {
                    let distance_tiles = (robot_tile - intersection_tile).magnitude();
                    if distance_tiles < d_robot_to_point {
                        return false
                    }
                }
            }
            true
        };
        // A bit more priority (negative score) could be put on forgotten area
        // in order to generate exploration targets
        let result_maybe = self.map.find_pattern(
                &pattern, pattern_w, pattern_h, 10.0, 0.1, wall_filter);
        if let Some(result) = result_maybe {
            // Target the center of the pattern
            let target_p = Point2::new(
                (result.0 + pattern_w / 2) as f32 * self.map.tile_wh,
                (result.1 + pattern_h / 2) as f32 * self.map.tile_wh,
            );
            //println!("target_p: {:?}", target_p);
            self.scan_p = Some(target_p);
            // Drive towards that spot
            let (wanted_linear_speed, mut wanted_rotation_speed) =
                        self.drive_towards_absolute_position(
                            target_p, max_linear_speed, max_rotation_speed, false);
            // Apply motor speed modulation to get scanning data
            wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 10.0).sin() * 1.5;
            return (wanted_linear_speed, wanted_rotation_speed);
        }

        //let wanted_linear_speed = 100.0;
        //let wanted_rotation_speed = PI * 1.0;
        let wanted_linear_speed = 0.0;
        let wanted_rotation_speed = PI * 3.0;
        return (wanted_linear_speed, wanted_rotation_speed);
    }

    pub fn create_attack_motion(&mut self, target_p: Point2<f32>) -> (f32, f32) {
        self.attack_p = Some(target_p);
        // Drive towards target
        let max_linear_speed = MAX_LINEAR_SPEED;
        let max_rotation_speed = MAX_ROTATION_SPEED;
        let (mut wanted_linear_speed, mut wanted_rotation_speed) =
                    self.drive_towards_absolute_position(
                        target_p, max_linear_speed, max_rotation_speed, true);
        // Apply some motor speed modulation to get scanning data to help stay
        // on target
        wanted_rotation_speed += (self.counter as f32 / UPS as f32 * 10.0).sin() * 1.5;
        // Revert linear speed at an interval to allow the weapon to spin up
        if self.attack_step_count > UPS * 2 &&
                ((self.attack_step_count as u32) % (UPS * 4)) < (UPS as f32 * 0.3) as u32 {
            wanted_linear_speed *= -0.2;
        }
        self.attack_step_count += 1;
        return (wanted_linear_speed, wanted_rotation_speed);
    }

    // Returns a value suitable for wanted_rotation_speed
    pub fn steer_towards_absolute_angle(&self, target_angle_rad: f32,
            max_rotation_speed: f32) -> f32 {
        let angle_diff = ((target_angle_rad - self.rot + PI) % (PI * 2.0)) - PI;
        //println!("angle_diff: {:?}", angle_diff);
        let speed_factor = 0.2 + (angle_diff.abs() / PI * 0.8);
        if angle_diff > 0.0 {
            return speed_factor * max_rotation_speed;
        } else {
            return speed_factor * -max_rotation_speed;
        }
    }

    // Returns a value suitable for wanted_linear_speed and wanted_rotation_speed
    // Prioritizes steering first, then picks up speed
    pub fn drive_towards_absolute_position(&self, target_p: Point2<f32>,
            max_linear_speed: f32, max_rotation_speed: f32,
            allow_overshoot: bool) -> (f32, f32) {
        let u = target_p - self.pos;
        let heading = Rotation2::rotation_between(&Vector2::x(), &u);
        let target_angle_rad = heading.angle();
        let angle_diff = ((target_angle_rad - self.rot + PI) % (PI * 2.0)) - PI;
        let speed_factor = if allow_overshoot {
            (1.0 - angle_diff / PI * 2.0).clamp(0.0, 1.0)
        } else {
            ((u.magnitude() / 20.0).clamp(0.0, 1.0) - angle_diff / PI * 2.0).clamp(0.0, 1.0)
        };
        let wanted_linear_speed = max_linear_speed * speed_factor;
        let wanted_rotation_speed = self.steer_towards_absolute_angle(
                target_angle_rad, max_rotation_speed);
        return (wanted_linear_speed, wanted_rotation_speed)
    }
}
