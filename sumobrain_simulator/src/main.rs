extern crate piston_window;
extern crate rapier2d;
extern crate sumobrain_common;
extern crate arrayvec; // Use static arrays like the embedded code

use piston_window::*;
use rapier2d::prelude::*;
use nalgebra::{Vector2, Point2, UnitComplex};
use std::f64::consts::PI;
use sumobrain_common::{RobotInterface, BrainState};
use arrayvec::ArrayVec;

const FPS: u64 = 120;
const UPS: u64 = 120;
const DT: f32 = 1.0 / UPS as f32;

const GROUP_EGO: u32 = 0b0001;
const GROUP_ENEMY: u32 = 0b0010;
const GROUP_BLADE: u32 = 0b0100; 
const GROUP_ARENA: u32 = 0b1000; 

struct Robot {
    body_handle: RigidBodyHandle,
    blade_handle: Option<RigidBodyHandle>,
    left_wheel_position: Point2<f32>,
    right_wheel_position: Point2<f32>,
    wheel_speed_left: f32,
    wheel_speed_right: f32,
    weapon_throttle: f32, // -100 to +100
    proximity_sensor_readings: ArrayVec<(f32, Option<f32>), 6>,
    gyro_z: f32,
}

struct ArenaWall {
    body_handle: RigidBodyHandle,
}

impl RobotInterface for Robot {
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
    fn get_rc_input_values(&self, values: &mut[&f32]) {
        // TODO
    }

    // Sensor readings
    // Current in Amperes
    fn get_weapon_current(&self) -> f32 {
        // TODO
        return 0.0;
    }
    // Returns a list of (angle, distance (cm)) tuples for each sensor
    fn get_proximity_sensors(&self) -> ArrayVec<(f32, Option<f32>), 6> {
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
    fn get_battery_cell_voltages(&self, values: &mut[&f32]) {
        // TODO
    }

    // LED control
    fn set_led_status(&mut self, status: bool) {
        // TODO
    }

    // Diagnostic data
    fn set_map(&mut self, map_width: i32, map_data: &[&i8]) {
        // TODO
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
        let box_collider = ColliderBuilder::cuboid(width, height)
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
        }
    }

    fn attach_blade(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet,
            joint_set: &mut ImpulseJointSet, blade_width: f32, blade_height: f32,
            angular_velocity: f32, blade_axle_p_on_robot: Point2<f32>,
            interaction_groups: InteractionGroups) {
        let blade_rigid_body = RigidBodyBuilder::dynamic()
            .angvel(angular_velocity)
            .build();
        let blade_collider = ColliderBuilder::cuboid(blade_width, blade_height)
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

    fn draw(&self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet, c: &Context, g: &mut G2d, transform: &[[f64; 3]; 2]) {
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
                    rectangle([0.8, 0.2, 0.2, 1.0], 
                              [-half_extents.x as f64 * 2.0, -half_extents.y as f64 * 2.0, 
                               half_extents.x as f64 * 2.0, half_extents.y as f64 * 2.0],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64)
                                .trans(half_extents.x as f64, half_extents.y as f64),
                              g);
                    rectangle([0.8, 0.8, 0.8, 1.0], 
                              [-1.5, half_extents.y as f64 - 6.0, 3.0, 3.0],
                              transform
                                .trans(position.translation.x as f64, position.translation.y as f64)
                                .rot_rad(rotation as f64),
                              g);
                    rectangle([0.8, 0.8, 0.2, 1.0], 
                              [half_extents.x as f64 - 6.0, half_extents.y as f64 - 6.0, 3.0, 3.0],
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

            const MASS: f32 = 0.45; // Per wheel
            const NORMAL_FORCE: f32 = 9.81 * MASS;
            const KINETIC_FRICTION_COEFFICIENT: f32 = 0.8;
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

            // TODO: is this correct?
            let left_wheel_ground_velocity_local = robot_velocity_local + robot_angular_velocity * self.left_wheel_position.coords;
            let right_wheel_ground_velocity_local = robot_velocity_local + robot_angular_velocity * self.right_wheel_position.coords;
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
            rigid_body_set: &RigidBodySet) {

        // Gyroscope
        if let Some(body) = rigid_body_set.get(self.body_handle) {
            self.gyro_z = body.angvel();
        }

        // Proximity sensors
        let position_sensor_angles = [0.0, -45.0, 45.0, -90.0, 90.0, 180.0];
        let max_detection_distance: f32 = 40.0;

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
                let max_toi = max_detection_distance;
                let stop_at_penetration = true;
                let filter = QueryFilter::new().groups(InteractionGroups::new(
                        (GROUP_EGO).into(), (GROUP_ENEMY | GROUP_ARENA).into()));

                if let Some((handle, hit)) = query_pipeline.cast_shape(
                    &rigid_body_set, &collider_set, &shape_pos, &shape_vel, &shape, max_toi, stop_at_penetration, filter
                ) {
                    // The first collider hit has the handle `handle`. The `hit` is a
                    // structure containing details about the hit configuration.
                    //println!("Hit the collider {:?} with the configuration: {:?}", handle, hit);

                    self.proximity_sensor_readings.push((angle_rad as f32, Some(hit.toi)));
                } else {
                    self.proximity_sensor_readings.push((angle_rad as f32, None));
                }
            }
		}
    }
}

impl ArenaWall {
    fn new(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet, x: f32, y: f32, width: f32, height: f32) -> Self {
        let wall_rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![x, y])
            .build();
        let wall_collider = ColliderBuilder::cuboid(width, height)
            .collision_groups(InteractionGroups::new(
                    (GROUP_ARENA).into(), (GROUP_ENEMY | GROUP_EGO | GROUP_BLADE).into()))
            .build();
        let wall_handle = rigid_body_set.insert(wall_rigid_body);
        collider_set.insert_with_parent(wall_collider, wall_handle, rigid_body_set);
        ArenaWall { body_handle: wall_handle }
    }

    fn draw(&self, rigid_body_set: &RigidBodySet, collider_set: &ColliderSet, c: &Context, g: &mut G2d, transform: &[[f64; 3]; 2]) {
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
fn main() {
    let mut window: PistonWindow = WindowSettings::new("Sumobrain Simulator", [800, 600])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let event_settings = EventSettings::new().max_fps(FPS).ups(UPS);
    let mut events = Events::new(event_settings);

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

    let mut robots = vec![
        Robot::new(&mut rigid_body_set, &mut collider_set,
                35.0, 100.0, 10.0, 11.5, (PI*0.25) as f32, Vector2::new(0.0, 0.0), 0.0,
                InteractionGroups::new(
                        (GROUP_EGO).into(), (GROUP_ENEMY | GROUP_ARENA).into())),
        Robot::new(&mut rigid_body_set, &mut collider_set,
                150.0, 100.0, 8.0, 9.0, 3.0, Vector2::new(-1.0, 0.0), -0.2,
                InteractionGroups::new(
                        (GROUP_ENEMY).into(),
                        (GROUP_ENEMY | GROUP_ARENA | GROUP_BLADE | GROUP_EGO).into())),
    ];
    robots[0].attach_blade(&mut rigid_body_set, &mut collider_set, &mut impulse_joint_set,
            10.0, 2.0, 0.0, point![0.0, 10.0],
            InteractionGroups::new(
                    (GROUP_BLADE).into(), (GROUP_ENEMY | GROUP_ARENA).into()));

    let arena_walls = vec![
        ArenaWall::new(&mut rigid_body_set, &mut collider_set, 100.0, 10.0, 180.0 / 2.0, 5.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 10.0, 100.0, 5.0 / 2.0, 180.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 190.0, 100.0, 5.0 / 2.0, 180.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 100.0, 190.0, 180.0 / 2.0, 5.0 / 2.0),
        // Repeat for other walls
    ];

    let mut brain = BrainState::new();

    let mut counter: u64 = 0;

    while let Some(e) = events.next(&mut window) {
        if e.render_args().is_some() {
            window.draw_2d(&e, |c, g, _| {
                clear([0.1; 4], g);

                let transform = c.transform
                    .zoom(2.5)
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
            });
        }

        if e.update_args().is_some() {
            brain.update(&mut robots[0]);

            for robot in &mut robots {
                if let Some(body) = rigid_body_set.get_mut(robot.body_handle) {
                    body.reset_forces(true);
                    body.reset_torques(true);
                }
            }

            for robot in &mut robots {
                robot.update_movement(&mut rigid_body_set, integration_parameters.dt);
                robot.update_sensors(&mut query_pipeline, &collider_set, &rigid_body_set);
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
    }
}
