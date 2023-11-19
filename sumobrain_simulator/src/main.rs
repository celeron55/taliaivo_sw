extern crate piston_window;
extern crate rapier2d;

use piston_window::*;
use rapier2d::prelude::*;
use nalgebra::{Vector2, Point2};

const GROUP_EGO: u32 = 0b0001;
const GROUP_ENEMY: u32 = 0b0010;
const GROUP_BLADE: u32 = 0b0100; 
const GROUP_ARENA: u32 = 0b1000; 

struct Robot {
    body_handle: RigidBodyHandle,
    blade_handle: Option<RigidBodyHandle>,
}

struct ArenaWall {
    body_handle: RigidBodyHandle,
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
            .density(1.0)
            .collision_groups(interaction_groups)
            .build();
        let body_handle = rigid_body_set.insert(box_rigid_body);
        collider_set.insert_with_parent(box_collider, body_handle, rigid_body_set);
        Robot { body_handle, blade_handle: None }
    }

    fn attach_blade(&mut self, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet,
            joint_set: &mut ImpulseJointSet, blade_width: f32, blade_height: f32,
            angular_velocity: f32, blade_axle_p_on_robot: Point2<f32>,
            interaction_groups: InteractionGroups) {
        let blade_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 0.0]) // position will be set by the joint
            .angvel(angular_velocity)
            .build();
        let blade_collider = ColliderBuilder::cuboid(blade_width, blade_height)
            .density(1.0)
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

    let gravity = Vector2::new(0.0, 0.0); // No gravity in a top-down view
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    let mut robots = vec![
        Robot::new(&mut rigid_body_set, &mut collider_set,
                35.0, 100.0, 10.0, 11.5, -1.0, Vector2::new(5.0, 0.0), 0.0,
                InteractionGroups::new(
                        (GROUP_EGO).into(), (GROUP_ENEMY | GROUP_ARENA).into())),
        Robot::new(&mut rigid_body_set, &mut collider_set,
                150.0, 100.0, 8.0, 9.0, 3.0, Vector2::new(-1.0, 0.0), -0.2,
                InteractionGroups::new(
                        (GROUP_ENEMY).into(),
                        (GROUP_ENEMY | GROUP_ARENA | GROUP_BLADE | GROUP_EGO).into())),
    ];
    robots[0].attach_blade(&mut rigid_body_set, &mut collider_set, &mut impulse_joint_set,
            10.0, 2.0, 2.0, point![0.0, 10.0],
            InteractionGroups::new(
                    (GROUP_BLADE).into(), (GROUP_ENEMY | GROUP_ARENA).into()));

    let mut arena_walls = vec![
        ArenaWall::new(&mut rigid_body_set, &mut collider_set, 100.0, 10.0, 180.0 / 2.0, 5.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 10.0, 100.0, 5.0 / 2.0, 180.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 190.0, 100.0, 5.0 / 2.0, 180.0 / 2.0),
	    ArenaWall::new(&mut rigid_body_set, &mut collider_set, 100.0, 190.0, 180.0 / 2.0, 5.0 / 2.0),
        // Repeat for other walls
    ];

    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g, _| {
            clear([0.1; 4], g);

            let transform = c.transform
                .trans(0.0, 60.0)
                .zoom(2.5);

            // Draw robots and walls
            for robot in &robots {
                robot.draw(&rigid_body_set, &collider_set, &c, g, &transform);
            }

            for wall in &arena_walls {
                wall.draw(&rigid_body_set, &collider_set, &c, g, &transform);
            }

        });

        // Update physics
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
			None,
			&physics_hooks,
			&event_handler,
        );
    }
}
