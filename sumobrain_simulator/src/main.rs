extern crate piston_window;
extern crate rapier2d;
extern crate sumobrain_common;
extern crate arrayvec; // Use static arrays like the embedded code

use piston_window::*;
use rapier2d::prelude::*;
use nalgebra::{Vector2, Point2, UnitComplex};
use std::f64::consts::PI;
use sumobrain_common::{RobotInterface, BrainState, Map};
use arrayvec::ArrayVec;
use sumobrain_common::map::HoughLine;
use rand::distributions::{Distribution, Uniform};

const FPS: u64 = 120;
const UPS: u64 = sumobrain_common::UPS as u64;
const PLAY_UPS: u64 = UPS; // Can be lowered for slow-mo effect
//const PLAY_UPS: u64 = 33; // Can be lowered for slow-mo effect
const DT: f32 = 1.0 / UPS as f32;

const ENABLE_ROBOT_WEAPON: [bool; 2] = [false, false];
const ENABLE_ROBOT_BRAIN: [bool; 2] = [true, false];
//const ROBOT_FRICTION_NORMAL_FORCE_PER_WHEEL: f32 = 9.81 * 0.45; // Not very stable
const ROBOT_FRICTION_NORMAL_FORCE_PER_WHEEL: f32 = 9.81 * 0.15;
const ROBOT_FRICTION_COEFFICIENT: f32 = 0.8;

const SIMULATE_LIDAR: bool = false;
const PROXIMITY_SENSOR_NOISE_MIN_CM: f32 = -2.0;
const PROXIMITY_SENSOR_NOISE_MAX_CM: f32 =  2.0;
const GYRO_SENSOR_NOISE_MIN: f32 = PI as f32 * -0.1;
const GYRO_SENSOR_NOISE_MAX: f32 = PI as f32 *  0.1;
const GYRO_SENSOR_ABSOLUTE_ERROR: f32 = PI as f32 * 0.025;
const GYRO_SENSOR_ABSOLUTE_ERROR_WITH_POLARITY: f32 = PI as f32 * 0.05;
const GYRO_SENSOR_ABSOLUTE_ERROR_POLARITY_INTERVAL: u64 = UPS * 4;

const GROUP_ARENA:         u32 = 0b00001000;
const GROUP_ROBOT0_BODY:   u32 = 0b00000001;
const GROUP_ROBOT1_BODY:   u32 = 0b00000010;
const GROUP_ROBOT0_WEAPON: u32 = 0b00000100;
const GROUP_ROBOT1_WEAPON: u32 = 0b00010000;

struct Robot {
    body_handle: RigidBodyHandle,
    blade_handle: Option<RigidBodyHandle>,
    left_wheel_position: Point2<f32>,
    right_wheel_position: Point2<f32>,
    wheel_speed_left: f32,
    wheel_speed_right: f32,
    weapon_throttle: f32, // -100 to +100
    proximity_sensor_readings: ArrayVec<(f32, f32, bool), 6>,
    gyro_z: f32,
    diagnostic_map: Map,
    diagnostic_robot_p: Point2<f32>,
    diagnostic_robot_r: f32,
    diagnostic_attack_p: Option<Point2<f32>>,
    diagnostic_scan_p: Option<Point2<f32>>,
    diagnostic_wall_avoid_p: Option<Point2<f32>>,
    diagnostic_wall_lines: Vec<HoughLine>,
    interaction_groups: InteractionGroups,
}

struct ArenaWall {
    body_handle: RigidBodyHandle,
}

impl RobotInterface for Robot {
    // Capabilities and dimensions
    fn get_track_width(&self) -> f32 {
        return self.right_wheel_position.x - self.left_wheel_position.x;
    }

    // Motor control
    fn set_motor_speed(&mut self, left_speed_cm_s: f32, right_speed_cm_s: f32) {
        self.wheel_speed_left = left_speed_cm_s;
        self.wheel_speed_right = right_speed_cm_s;
    }
    
    // Weapon control
    // -100 to +100
    fn set_weapon_throttle(&mut self, throttle_percentage: f32) {
        self.weapon_throttle = throttle_percentage;
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
        return (0.0, 0.0, self.gyro_z);
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
        self.diagnostic_map = map.clone();
        self.diagnostic_robot_p = robot_p;
        self.diagnostic_robot_r = robot_r;
        self.diagnostic_attack_p = attack_p;
        self.diagnostic_scan_p = scan_p;
        self.diagnostic_wall_avoid_p = wall_avoid_p;
        self.diagnostic_wall_lines = wall_lines.to_vec();
    }
}

impl Robot {
    fn new(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet, x: f32, y: f32,
            width: f32, height: f32, rotation: f32, velocity: Vector2<f32>, angular_velocity: f32,
            interaction_groups: InteractionGroups) -> Self {
        let box_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y])
            .rotation(rotation)
            .linvel(velocity)
            .angvel(angular_velocity)
            .build();
        let box_collider = ColliderBuilder::cuboid(width / 2.0, height / 2.0)
            .density(0.008) // About right for 11.5x10cm = 900g
            .collision_groups(interaction_groups)
            .build();
        let body_handle = rigid_body_set.insert(box_rigid_body);
        collider_set.insert_with_parent(box_collider, body_handle, rigid_body_set);
        Robot {
            body_handle,
            blade_handle: None,
            wheel_speed_left: 0.0,
            wheel_speed_right: 0.0,
            left_wheel_position: Point2::new(-5.0, 2.0), // -X=left, -Y=front
            right_wheel_position: Point2::new(5.0, 2.0),
            weapon_throttle: 0.0,
            proximity_sensor_readings: ArrayVec::new(),
            gyro_z: 0.0,
            diagnostic_map: Map::new(),
            diagnostic_robot_p: Point2::new(0.0, 0.0),
            diagnostic_robot_r: 0.0,
            diagnostic_attack_p: None,
            diagnostic_scan_p: None,
            diagnostic_wall_avoid_p: None,
            diagnostic_wall_lines: Vec::new(),
            interaction_groups: interaction_groups,
        }
    }

    fn attach_blade(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet,
            joint_set: &mut ImpulseJointSet, blade_width: f32, blade_height: f32,
            angular_velocity: f32, blade_axle_p_on_robot: Point2<f32>,
            interaction_groups: InteractionGroups) {
        let blade_rigid_body = RigidBodyBuilder::dynamic()
            .angvel(angular_velocity)
            .build();
        let blade_collider = ColliderBuilder::cuboid(blade_width / 2.0, blade_height / 2.0)
            .density(0.008) // About right for 11.5x10cm = 900g
            .collision_groups(interaction_groups)
            .build();
        let blade_handle = rigid_body_set.insert(blade_rigid_body);
        collider_set.insert_with_parent(blade_collider, blade_handle, rigid_body_set);

        // Create the joint
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(point![0.0, 0.0]) // Joint location on the blade
            .local_anchor2(blade_axle_p_on_robot) // Joint location on the robot
            .build();
        joint_set.insert(blade_handle, self.body_handle, joint, true);

        self.blade_handle = Some(blade_handle);
    }

    fn draw(&self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet, _c: &Context, g: &mut G2d, transform: &[[f64; 3]; 2]) {
        if let Some(blade_handle) = self.blade_handle {
            if let Some(blade_body) = rigid_body_set.get(blade_handle) {
                if let Some(collider) = collider_set.get(blade_body.colliders()[0]) {
                    if let Some(shape) = collider.shape().as_cuboid() {
                        let position = blade_body.position();
                        let half_extents = shape.half_extents;
                        let rotation = position.rotation.angle();
                        rectangle([0.2, 0.8, 0.2, 1.0], 
                                  [-half_extents.x as f64 * 2.0, -half_extents.y as f64 * 2.0, 
                                   half_extents.x as f64 * 2.0, half_extents.y as f64 * 2.0],
                                  transform
                                    .trans(position.translation.x as f64, position.translation.y as f64)
                                    .rot_rad(rotation as f64)
                                    .trans(half_extents.x as f64, half_extents.y as f64),
                                  g);
                    }
                }
            }
        }

        if let Some(body) = rigid_body_set.get(self.body_handle) {
            if let Some(collider) = collider_set.get(body.colliders()[0]) {
                if let Some(shape) = collider.shape().as_cuboid() {
                    let position = body.position();
                    let half_extents = shape.half_extents;
                    let rotation = position.rotation.angle();
                    let mut color = [0.7, 0.7, 0.7, 1.0];
                    if !self.diagnostic_attack_p.is_none() {
                        color = [0.6, 0.2, 0.2, 1.0];
                    } else if !self.diagnostic_scan_p.is_none() {
                        color = [0.2, 0.6, 0.2, 1.0];
                    } else if !self.diagnostic_wall_avoid_p.is_none() {
                        color = [0.2, 0.3, 0.6, 1.0];
                    }
                    rectangle(color, 
                              [-half_extents.x as f64 * 2.0, -half_extents.y as f64 * 2.0, 
                               half_extents.x as f64 * 2.0, half_extents.y as f64 * 2.0],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64)
                                .trans(half_extents.x as f64, half_extents.y as f64),
                              g);
                    rectangle([0.8, 0.8, 0.8, 1.0], 
                              [-0.75, half_extents.y as f64 - 3.0, 1.5, 1.5],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64),
                              g);
                    rectangle([0.8, 0.8, 0.2, 1.0], 
                              [half_extents.x as f64 - 3.0, half_extents.y as f64 - 3.0, 1.5, 1.5],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64),
                              g);
                }
            }
        }
    }

    fn update_movement(&mut self, rigid_body_set: &mut RigidBodySet, dt: f32) {
        if let Some(body) = rigid_body_set.get_mut(self.body_handle) {
            let robot_velocity = body.linvel();
            let robot_angular_velocity = body.angvel();
            let robot_orientation = body.position().rotation;
            let robot_inverse_orientation = robot_orientation.inverse();

            const NORMAL_FORCE: f32 = ROBOT_FRICTION_NORMAL_FORCE_PER_WHEEL;
            const KINETIC_FRICTION_COEFFICIENT: f32 = ROBOT_FRICTION_COEFFICIENT;
            // Have to multiply by 100 because we use centimeters instead of
            // meters
            let frictional_force_magnitude = KINETIC_FRICTION_COEFFICIENT * NORMAL_FORCE * 100.0;

            //println!("---");

            // Transform global velocity to robot's local space
            let robot_velocity_local = robot_inverse_orientation * robot_velocity;
            //println!("robot_velocity_local: L={:?}", robot_velocity_local);
            //println!("robot_velocity_world: L={:?}", robot_velocity);

            // Transform wheel positions
            //println!("wheel_position_local: L={:?}\tR={:?}", self.left_wheel_position, self.right_wheel_position);
            let left_wheel_position_world = robot_orientation * self.left_wheel_position + body.position().translation.vector;
            let right_wheel_position_world = robot_orientation * self.right_wheel_position + body.position().translation.vector;
            //println!("wheel_position_world: L={:?}\tR={:?}", left_wheel_position_world, right_wheel_position_world);

            // Local wheel velocities
            let left_wheel_driven_velocity_local = Vector2::new(0.0, self.wheel_speed_left);
            let right_wheel_driven_velocity_local = Vector2::new(0.0, self.wheel_speed_right);
            //println!("wheel_driven_velocity_local: L={:?}\tR={:?}", left_wheel_driven_velocity_local, right_wheel_driven_velocity_local);

            // Perpendicular velocity due to rotation (rotational velocity =
            // radius x angular velocity)
            let left_wheel_radius_vector = Vector2::new(self.left_wheel_position.x, self.left_wheel_position.y);
            let right_wheel_radius_vector = Vector2::new(self.right_wheel_position.x, self.right_wheel_position.y);
            let left_wheel_rotational_velocity = Vector2::new(-left_wheel_radius_vector.y, left_wheel_radius_vector.x) * robot_angular_velocity;
            let right_wheel_rotational_velocity = Vector2::new(-right_wheel_radius_vector.y, right_wheel_radius_vector.x) * robot_angular_velocity;

            // Total ground velocity at wheel positions
            let left_wheel_ground_velocity_local = robot_velocity_local + left_wheel_rotational_velocity;
            let right_wheel_ground_velocity_local = robot_velocity_local + right_wheel_rotational_velocity;
            //println!("wheel_ground_velocity_local: L={:?}\tR={:?}", left_wheel_ground_velocity_local, right_wheel_ground_velocity_local);

            // Calculate the speed differences in the local frame
            let left_velocity_diff = left_wheel_driven_velocity_local - left_wheel_ground_velocity_local;
            let right_velocity_diff = right_wheel_driven_velocity_local - right_wheel_ground_velocity_local;
            //println!("velocity_diff: L={:?}\tR={:?}", left_velocity_diff, right_velocity_diff);

            // Apply forces
            if left_velocity_diff.magnitude() > 0.001 {
                let left_friction_force_local = left_velocity_diff.normalize() * frictional_force_magnitude;
                let left_friction_force = robot_orientation * left_friction_force_local;
                //println!("left_friction_force: {:?}", left_friction_force);
                body.add_force_at_point(left_friction_force, left_wheel_position_world, true);
            }
            if right_velocity_diff.magnitude() > 0.001 {
                let right_friction_force_local = right_velocity_diff.normalize() * frictional_force_magnitude;
                let right_friction_force = robot_orientation * right_friction_force_local;
                //println!("right_friction_force: {:?}", right_friction_force);
                body.add_force_at_point(right_friction_force, right_wheel_position_world, true);
            }
        }

        if let Some(blade_handle) = self.blade_handle {
            if let Some(blade) = rigid_body_set.get_mut(blade_handle) {
                if blade.angvel().abs() < PI as f32 * 2.0 / 60.0 * 10000.0 * self.weapon_throttle / 100.0 {
                    blade.apply_torque_impulse(dt * 1000.0 * self.weapon_throttle / 100.0, true);
                } else {
                    if blade.angvel() > 0.0 {
                        blade.apply_torque_impulse(dt * 1000.0 * -0.2, true);
                    } else {
                        blade.apply_torque_impulse(dt * 1000.0 * 0.2, true);
                    }
                }
            }
        }
    }

    fn update_sensors(&mut self, query_pipeline: &mut QueryPipeline, collider_set: &ColliderSet,
            rigid_body_set: &RigidBodySet, tick_count: u64) {

        // Gyroscope
        if let Some(body) = rigid_body_set.get(self.body_handle) {
            let gyro_z_precise_value = body.angvel();
            let noise = Uniform::from(
                    GYRO_SENSOR_NOISE_MIN..GYRO_SENSOR_NOISE_MAX)
                    .sample(&mut rand::thread_rng());
            let add_error = GYRO_SENSOR_ABSOLUTE_ERROR + if
                    (tick_count % (GYRO_SENSOR_ABSOLUTE_ERROR_POLARITY_INTERVAL * 2))
                        < GYRO_SENSOR_ABSOLUTE_ERROR_POLARITY_INTERVAL {
                GYRO_SENSOR_ABSOLUTE_ERROR_WITH_POLARITY
            } else {
                -GYRO_SENSOR_ABSOLUTE_ERROR_WITH_POLARITY
            };
            self.gyro_z = gyro_z_precise_value + noise + add_error;
        }

        // Proximity sensors

        let sensor_mount_radius: f32 = 3.5;
        let max_detection_distance: f32 = 40.0;
        // Below a certain range sensor result is essentially random so it will
        // be clamped
        let min_detection_distance: f32 = 7.0;

        let position_sensor_angles = if SIMULATE_LIDAR {
            // Simulate a lidar turret (have to have the same number of entries)
            [
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0) % 360.0,
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0 + 180.0) % 360.0,
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0) % 360.0,
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0 + 180.0) % 360.0,
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0) % 360.0,
                (tick_count as f32 / UPS as f32 * 5.0 * 360.0 + 180.0) % 360.0,
            ]
        } else {
            [0.0, -45.0, 45.0, -90.0, 90.0, 180.0]
            //[0.0, -45.0, 45.0, -135.0, 135.0, 180.0]
        };

        if let Some(body) = rigid_body_set.get(self.body_handle) {
            let robot_orientation = body.position().rotation;

            self.proximity_sensor_readings.clear();

            for angle_deg in position_sensor_angles {
                let angle_rad: f32 = angle_deg / 180.0 * PI as f32;

                let shape = Ball::new(2.0);
                let shape_pos: Isometry<Real> = *body.position();
                let additional_rotation = UnitComplex::new(angle_rad as f32);
                let rotation = additional_rotation * robot_orientation;
                let shape_vel: Vector2<f32> = rotation * Vector2::new(0.0, 1.0);
                let max_toi = sensor_mount_radius + max_detection_distance;
                let stop_at_penetration = true;
                let filter = QueryFilter::new().groups(self.interaction_groups);

                if let Some((_handle, hit)) = query_pipeline.cast_shape(
                    &rigid_body_set, &collider_set, &shape_pos, &shape_vel, &shape, max_toi,
                    stop_at_penetration, filter
                ) {
                    // The first collider hit has the handle `handle`. The `hit` is a
                    // structure containing details about the hit configuration.
                    //println!("Hit the collider {:?} with the configuration: {:?}", handle, hit);
                    let reported_detection_distance = if hit.toi >=
                            sensor_mount_radius + min_detection_distance {
                        hit.toi
                    } else {
                        sensor_mount_radius + min_detection_distance
                    };
                    let noise = Uniform::from(
                            PROXIMITY_SENSOR_NOISE_MIN_CM..PROXIMITY_SENSOR_NOISE_MAX_CM)
                            .sample(&mut rand::thread_rng());
                    self.proximity_sensor_readings.push((angle_rad as f32,
                            reported_detection_distance + noise, true));
                } else {
                    self.proximity_sensor_readings.push((angle_rad as f32,
                            sensor_mount_radius + max_detection_distance, false));
                }
            }
		}
    }

    fn draw_map(&self, _c: &Context, g: &mut G2d, transform: &[[f64; 3]; 2], tile_size: f64) {
        let map = &self.diagnostic_map;
        for y in 0..map.height {
            for x in 0..map.width {
                let idx = (y * map.width + x) as usize;
                let tile = map.data[idx];
                rectangle([
                        (tile + 100.0) / 200.0,
                        (tile + 100.0) / 200.0,
                        (tile + 100.0) / 200.0,
                        1.0
                    ], 
                    [tile_size * x as f64, tile_size * y as f64, tile_size, tile_size],
                    *transform,
                    g);
            }
        }

        let p = self.diagnostic_robot_p;
        let r = self.diagnostic_robot_r;
        polygon([0.2, 0.8, 0.8, 1.0],
                &[
                    [tile_size * 0.6, 0.0],
                    [-tile_size * 0.6, -tile_size * 0.5],
                    [-tile_size * 0.6,  tile_size * 0.5],
                ],
                transform
                    .trans(tile_size * p.x as f64 / map.tile_wh as f64,
                            tile_size * p.y as f64 / map.tile_wh as f64)
                    .rot_rad(r as f64),
                g);

        if let Some(p) = self.diagnostic_attack_p {
            rectangle([0.8, 0.2, 0.2, 1.0],
                    [-tile_size/2.0, -tile_size/2.0, tile_size, tile_size],
                    transform
                        .trans(tile_size * p.x as f64 / map.tile_wh as f64,
                                tile_size * p.y as f64 / map.tile_wh as f64),
                    g);
        }
        if let Some(p) = self.diagnostic_scan_p {
            rectangle([0.2, 0.8, 0.2, 1.0],
                    [-tile_size/2.0, -tile_size/2.0, tile_size, tile_size],
                    transform
                        .trans(tile_size * p.x as f64 / map.tile_wh as f64,
                                tile_size * p.y as f64 / map.tile_wh as f64),
                    g);
        }
        if let Some(p) = self.diagnostic_wall_avoid_p {
            rectangle([0.2, 0.2, 0.8, 1.0],
                    [-tile_size/2.0, -tile_size/2.0, tile_size, tile_size],
                    transform
                        .trans(tile_size * p.x as f64 / map.tile_wh as f64,
                                tile_size * p.y as f64 / map.tile_wh as f64),
                    g);
        }

        //self.diagnostic_map.print(Point2::new(0.0, 0.0));

        for (_i, line) in self.diagnostic_wall_lines.iter().enumerate() {
            // These lines do not have a starting or ending point. We need to
            // create those ourselves. Use the map midpoint to project the
            // midpoint for each line and base their endpoints on that.
            //let robot_p_tiles = self.diagnostic_robot_p / map.tile_wh;
            //let vtp = line.vector_to_point(robot_p_tiles.coords);
            let map_midpoint_tiles = Point2::new(map.width as f32 / 2.0, map.height as f32 / 2.0);
            let vtp = line.vector_to_point(map_midpoint_tiles.coords);
            let line_midpoint = map_midpoint_tiles.coords - vtp;
            let angle_rad = line.angle.to_radians();
            //let normal_direction = Vector2::new(angle_rad.cos(), angle_rad.sin());
            let line_direction = Vector2::new((angle_rad + PI as f32 * 0.5).cos(),
                    (angle_rad + PI as f32 * 0.5).sin());
            let p1 = line_midpoint + line_direction * 15.0;
            let p2 = line_midpoint - line_direction * 15.0;
            piston_window::line([0.35, 0.45, 1.0, 1.0], tile_size * 0.2,
                    [tile_size * p1.x as f64, tile_size * p1.y as f64,
                        tile_size * p2.x as f64, tile_size * p2.y as f64],
                    *transform,
                    g);
        }
    }
}

impl ArenaWall {
    fn new(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet, x: f32, y: f32, width: f32, height: f32) -> Self {
        let wall_rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![x, y])
            .build();
        let wall_collider = ColliderBuilder::cuboid(width / 2.0, height / 2.0)
            .collision_groups(InteractionGroups::new(
                    (GROUP_ARENA).into(),
                    (GROUP_ROBOT0_BODY | GROUP_ROBOT1_BODY |
                        GROUP_ROBOT0_WEAPON | GROUP_ROBOT1_WEAPON).into()))
            .build();
        let wall_handle = rigid_body_set.insert(wall_rigid_body);
        collider_set.insert_with_parent(wall_collider, wall_handle, rigid_body_set);
        ArenaWall { body_handle: wall_handle }
    }

    fn draw(&self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet, _c: &Context, g: &mut G2d, transform: &[[f64; 3]; 2]) {
        if let Some(wall_body) = rigid_body_set.get(self.body_handle) {
            if let Some(collider) = collider_set.get(wall_body.colliders()[0]) {
                if let Some(shape) = collider.shape().as_cuboid() {
                    let position = wall_body.position();
                    let half_extents = shape.half_extents;
                    let rotation = position.rotation.angle();
                    rectangle([0.2, 0.2, 0.8, 1.0],
                              [-half_extents.x as f64 * 2.0, -half_extents.y as f64 * 2.0, 
                               half_extents.x as f64 * 2.0, half_extents.y as f64 * 2.0],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64)
                                .trans(half_extents.x as f64, half_extents.y as f64),
                              g);
                }
            }
        }
    }
}

struct KeyboardController {
    up: bool,
    down: bool,
    left: bool,
    right: bool,
    t_toggle: bool,
}

impl KeyboardController {
    fn new() -> Self {
        KeyboardController {
            up: false,
            down: false,
            left: false,
            right: false,
            t_toggle: true,
        }
    }

    fn event(&mut self, e: &piston_window::Event) {
        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                Key::Up => self.up = true,
                Key::Down => self.down = true,
                Key::Left => self.left = true,
                Key::Right => self.right = true,
                Key::T => self.t_toggle = !self.t_toggle,
                _ => {}
            }
        }

        if let Some(Button::Keyboard(key)) = e.release_args() {
            match key {
                Key::Up => self.up = false,
                Key::Down => self.down = false,
                Key::Left => self.left = false,
                Key::Right => self.right = false,
                _ => {}
            }
        }
    }

    fn control(&self, robot: &mut Robot) {
        let speed = 100.0;
        let turn_speed = 35.0;
        robot.wheel_speed_left = 0.0;
        robot.wheel_speed_right = 0.0;
        if self.up {
            robot.wheel_speed_left += speed;
            robot.wheel_speed_right += speed;
        }
        if self.down {
            robot.wheel_speed_left -= speed;
            robot.wheel_speed_right -= speed;
        }
        if self.left {
            // FIXME: This seems backwards
            robot.wheel_speed_left += turn_speed;
            robot.wheel_speed_right -= turn_speed;
        }
        if self.right {
            // FIXME: This seems backwards
            robot.wheel_speed_left -= turn_speed;
            robot.wheel_speed_right += turn_speed;
        }

        robot.weapon_throttle = if self.t_toggle { 100.0 } else { 0.0 };

        // Clear some diagnostic info
        robot.diagnostic_attack_p = None;
        robot.diagnostic_scan_p = None;
        robot.diagnostic_wall_avoid_p = None;
    }
}

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Sumobrain Simulator", [1200, 600])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let event_settings = EventSettings::new().max_fps(FPS).ups(PLAY_UPS);
    let mut events = Events::new(event_settings);
    window.events = events;

    let gravity = Vector2::new(0.0, 0.0); // No gravity in a top-down view
    let integration_parameters = {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = DT;
        integration_parameters
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    //let asize = 125.0 + 5.0;
    let asize = 63.0 + 5.0;
    let ad = 10.0;
    let at = 5.0;
    let arena_walls = vec![
        ArenaWall::new(&mut rigid_body_set, &mut collider_set, ad+asize/2.0, ad, asize, at),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, ad, ad+asize/2.0, at, asize),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, ad+asize, ad+asize/2.0, at, asize),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, ad+asize/2.0, ad+asize, asize, at),
    ];

    let mut robots = vec![
        Robot::new(&mut rigid_body_set, &mut collider_set,
                asize*0.8, asize*0.8, 10.0, 11.5, (PI*1.0) as f32, Vector2::new(0.0, 0.0), 0.0,
                InteractionGroups::new(
                        (GROUP_ROBOT0_BODY).into(),
                        (GROUP_ARENA | GROUP_ROBOT1_BODY | GROUP_ROBOT1_WEAPON).into())),
        Robot::new(&mut rigid_body_set, &mut collider_set,
                asize*0.96, asize*0.96, 8.0, 9.0, 3.0, Vector2::new(0.0, 0.0), 0.0,
                InteractionGroups::new(
                        (GROUP_ROBOT1_BODY).into(),
                        (GROUP_ARENA | GROUP_ROBOT0_WEAPON | GROUP_ROBOT0_BODY).into())),
    ];
    if ENABLE_ROBOT_WEAPON[0] {
        robots[0].attach_blade(&mut rigid_body_set, &mut collider_set, &mut impulse_joint_set,
                10.0, 2.0, 0.0, point![0.0, 4.0],
                InteractionGroups::new(
                        (GROUP_ROBOT0_WEAPON).into(), (GROUP_ROBOT1_BODY | GROUP_ARENA).into()));
    }
    if ENABLE_ROBOT_WEAPON[0] {
        robots[1].attach_blade(&mut rigid_body_set, &mut collider_set, &mut impulse_joint_set,
                8.0, 1.5, 0.0, point![0.0, 3.0],
                InteractionGroups::new(
                        (GROUP_ROBOT1_WEAPON).into(), (GROUP_ROBOT0_BODY | GROUP_ARENA).into()));
    }

    let mut brain = BrainState::new(0);
    let mut brain2 = BrainState::new(UPS as u32 * 2);
    let mut keyboard_controller = KeyboardController::new();
    enum RobotController {
        Brain,
        Keyboard,
    }
    let mut robot2_controller = RobotController::Brain;

    let mut counter: u64 = 0;

    let mut paused = false;
    let mut slow_motion = false;
    let mut slow_motion_counter: u64 = 0;

    while let Some(e) = window.next() {
        if e.render_args().is_some() {
            window.draw_2d(&e, |c, g, _| {
                clear([0.1; 4], g);

                let transform = c.transform
                    .zoom(4.0)
                    .trans(0.0, 0.0);
                    //.rot_rad(PI)
                    //.trans(-200.0, -200.0);

                // Draw robots and walls
                for robot in &robots {
                    robot.draw(&rigid_body_set, &collider_set, &c, g, &transform);
                }

                for wall in &arena_walls {
                    wall.draw(&rigid_body_set, &collider_set, &c, g, &transform);
                }

                // Draw origin dot
                rectangle([0.8, 0.8, 0.8, 1.0], 
                          [0.0, 0.0, 5.0, 5.0],
                          transform
                            .trans(10.0, 7.0)
                            .rot_rad(PI * 0.25),
                          g);

                let transform1 = transform.trans(180.0, 10.0);
                robots[0].draw_map(&c, g, &transform1, 2.2);
                //let transform2 = transform1.trans(0.0, 100.0);
                //robots[1].draw_map(&c, g, &transform2, 0.75);
            });
        }

        if e.update_args().is_some() && !paused {
            if !slow_motion || slow_motion_counter % 5 == 0 {
                if ENABLE_ROBOT_BRAIN[0] {
                    brain.update(&mut robots[0]);
                }

                match robot2_controller {
                    RobotController::Brain => {
                        if ENABLE_ROBOT_BRAIN[1] {
                            brain2.update(&mut robots[1]);
                        }
                    },
                    RobotController::Keyboard => {
                        keyboard_controller.control(&mut robots[1]);
                    },
                }

                for robot in &mut robots {
                    if let Some(body) = rigid_body_set.get_mut(robot.body_handle) {
                        body.reset_forces(true);
                        body.reset_torques(true);
                    }
                }

                for robot in &mut robots {
                    robot.update_movement(&mut rigid_body_set, integration_parameters.dt);
                    robot.update_sensors(&mut query_pipeline, &collider_set, &rigid_body_set, counter);
                }

                physics_pipeline.step(
                    &gravity,
                    &integration_parameters,
                    &mut island_manager,
                    &mut broad_phase,
                    &mut narrow_phase,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut impulse_joint_set,
                    &mut multibody_joint_set,
                    &mut ccd_solver,
                    Some(&mut query_pipeline),
                    &physics_hooks,
                    &event_handler,
                );

                counter += 1;
            }
            slow_motion_counter += 1;
        }

        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                Key::K => {
                    match robot2_controller {
                        RobotController::Brain => {
                            robot2_controller = RobotController::Keyboard;
                        },
                        RobotController::Keyboard => {
                            robot2_controller = RobotController::Brain;
                        },
                    }
                },
                Key::Space => {
                    paused = !paused;
                },
                Key::M => {
                    slow_motion = !slow_motion;
                },
                _ => {}
            }
        }

        keyboard_controller.event(&e);
    }
}
