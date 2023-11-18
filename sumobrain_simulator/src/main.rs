extern crate piston_window;
extern crate rapier2d;

use piston_window::*;
use rapier2d::prelude::*;
use nalgebra::Vector2;

const GROUP_EGO: u32 = 0b0001;
const GROUP_ENEMY: u32 = 0b0010;
const GROUP_BLADE: u32 = 0b0100; 
const GROUP_ARENA: u32 = 0b1000; 

fn create_wall(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet, x: f32, y: f32, width: f32, height: f32) -> RigidBodyHandle {
    let wall_rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![x, y])
        .build();
    let wall_collider = ColliderBuilder::cuboid(width, height).build();
    let wall_handle = rigid_body_set.insert(wall_rigid_body);
    collider_set.insert_with_parent(wall_collider, wall_handle, rigid_body_set);
    wall_handle
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

	let mut box_handles = Vec::new();
	let mut wall_handles = Vec::new();
	let mut blade_handles = Vec::new();

	// Create multiple boxes
	for c in [
	    ([30.0, 100.0, 10.0, 11.5, 0.1, 1.2],
	            GROUP_EGO, GROUP_ENEMY | GROUP_ARENA,
	            Vector2::new(5.0, 0.0),
	            4.0),
        ([150.0, 100.0, 8.0, 9.0, -0.2, 0.8],
                GROUP_ENEMY, GROUP_ENEMY | GROUP_ARENA | GROUP_BLADE | GROUP_EGO,
                Vector2::new(-1.0, 0.0),
                0.0),
	] {
	    let b = c.0;
		let x = b[0];
		let y = b[1];
		let width = b[2];
		let height = b[3];
		let angular_v = b[4];
		let density = b[5];
		let collision_groups_self = c.1;
		let collision_groups_collides = c.2;
		let velocity = c.3;
        let box_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y])
            .rotation(c.4)
            .linvel(velocity)
            .angvel(angular_v)
            .build();
        let box_collider = ColliderBuilder::cuboid(width, height)
            .density(density)
            .collision_groups(InteractionGroups::new(collision_groups_self.into(), collision_groups_collides.into()))
            .build();
        let box_handle = rigid_body_set.insert(box_rigid_body);
        collider_set.insert_with_parent(box_collider, box_handle, &mut rigid_body_set);
		box_handles.push(box_handle);
	}

    {
        let box_0_handle = box_handles[0];

        // Create the blade
        let blade_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![100.0, 100.0]) // Position the blade
            .angvel(2.0)
            .build();
        let blade_collider = ColliderBuilder::cuboid(10.0, 2.0)
            .density(4.0) // Compensate for non-circular shape
            .collision_groups(InteractionGroups::new(
                    (GROUP_BLADE).into(), (GROUP_ENEMY | GROUP_ARENA).into()))
            .build();
        let blade_handle = rigid_body_set.insert(blade_rigid_body);
        collider_set.insert_with_parent(blade_collider, blade_handle, &mut rigid_body_set);
        blade_handles.push(blade_handle);

        // Create the joint
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(point![0.0, 0.0]) // Joint location on the blade
            .local_anchor2(point![0.0, 10.0]); // Joint location on box 0 (+y edge)
        let joint_handle = impulse_joint_set.insert(blade_handle, box_0_handle, joint, true);
    }

    // Exclude collisions between the blade and box 0
    // You will need to set up collision groups or filters for this

	// Create multiple walls
	for w in [
	    [100.0, 10.0, 180.0/2.0, 5.0/2.0],
	    [10.0, 100.0, 5.0/2.0, 180.0/2.0],
	    [190.0, 100.0, 5.0/2.0, 180.0/2.0],
	    [100.0, 190.0, 180.0/2.0, 5.0/2.0],
	] {
		let handle = create_wall(&mut rigid_body_set, &mut collider_set, w[0], w[1], w[2], w[3]);
		wall_handles.push(handle);
	}

    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g, _| {
            clear([0.1; 4], g);

            let transform = c.transform
                .trans(0.0, 60.0)
                .zoom(2.5);

            for box_handle in &blade_handles {
				if let Some(box_body) = rigid_body_set.get(*box_handle) {
                    if let Some(collider) = collider_set.get(box_body.colliders()[0]) {
                        if let Some(shape) = collider.shape().as_cuboid() {
                            let position = box_body.position();
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

            for box_handle in &box_handles {
				if let Some(box_body) = rigid_body_set.get(*box_handle) {
                    if let Some(collider) = collider_set.get(box_body.colliders()[0]) {
                        if let Some(shape) = collider.shape().as_cuboid() {
                            let position = box_body.position();
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

            for wall_handle in &wall_handles {
                if let Some(wall_body) = rigid_body_set.get(*wall_handle) {
                    if let Some(collider) = collider_set.get(wall_body.colliders()[0]) {
                        if let Some(shape) = collider.shape().as_cuboid() {
                            let position = wall_body.position();
                            let half_extents = shape.half_extents;
                            rectangle([0.2, 0.2, 0.8, 1.0],
                                      [position.translation.x as f64 - half_extents.x as f64, 
                                       position.translation.y as f64 - half_extents.y as f64, 
                                       half_extents.x as f64 * 2.0, 
                                       half_extents.y as f64 * 2.0], // dimensions
                                      transform, g);
                        }
                    }
                }
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

