use avian2d::{math::*, prelude::*};
use bevy::{ecs::query::Has, prelude::*, window::PrimaryWindow};

use crate::Halt;

pub struct CharacterControllerPlugin;

impl Plugin for CharacterControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<MovementAction>()
            .add_systems(
                Update,
                (
                    keyboard_input,
                    gamepad_input,
                    update_grounded,
                    apply_gravity,
                    movement,
                    apply_movement_damping,
                    move_camera,
                )
                    .chain(),
            )
            .add_systems(
                // Run collision handling after collision detection.
                //
                // NOTE: The collision implementation here is very basic and a bit buggy.
                //       A collide-and-slide algorithm would likely work better.
                PostProcessCollisions,
                kinematic_controller_collisions,
            );
    }
}

/// An event sent for a movement input action.
#[derive(Event)]
pub enum MovementAction {
    Move(Scalar),
    Jump,
}

/// A marker component indicating that an entity is using a character controller.
#[derive(Component)]
pub struct CharacterController;

/// A marker component indicating that an entity is on the ground.
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct Grounded;
/// The acceleration used for character movement.
#[derive(Component)]
pub struct MovementAcceleration(Scalar);

/// The damping factor used for slowing down movement.
#[derive(Component)]
pub struct MovementDampingFactor(Scalar);

/// The strength of a jump.
#[derive(Component)]
pub struct JumpImpulse(Scalar);

/// The gravitational acceleration used for a character controller.
#[derive(Component)]
pub struct ControllerGravityStrengt(Scalar);

/// The gravitational acceleration used for a character controller.
#[derive(Component)]
pub struct CurrentControllerGravity(Vector);

/// The maximum angle a slope can have for a character controller
/// to be able to climb and jump. If the slope is steeper than this angle,
/// the character will slide down.
#[derive(Component)]
pub struct MaxSlopeAngle(Scalar);

/// A bundle that contains the components needed for a basic
/// kinematic character controller.
#[derive(Bundle)]
pub struct CharacterControllerBundle {
    character_controller: CharacterController,
    rigid_body: RigidBody,
    collider: Collider,
    ground_caster: ShapeCaster,
    gravity: CurrentControllerGravity,
    gravity_strength: ControllerGravityStrengt,
    movement: MovementBundle,
}

/// A bundle that contains components for character movement.
#[derive(Bundle)]
pub struct MovementBundle {
    acceleration: MovementAcceleration,
    damping: MovementDampingFactor,
    jump_impulse: JumpImpulse,
    max_slope_angle: MaxSlopeAngle,
}

impl MovementBundle {
    pub const fn new(
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        Self {
            acceleration: MovementAcceleration(acceleration),
            damping: MovementDampingFactor(damping),
            jump_impulse: JumpImpulse(jump_impulse),
            max_slope_angle: MaxSlopeAngle(max_slope_angle),
        }
    }
}

impl Default for MovementBundle {
    fn default() -> Self {
        Self::new(30.0, 0.9, 7.0, PI * 0.45)
    }
}

impl CharacterControllerBundle {
    pub fn new(collider: Collider, gravity: f32) -> Self {
        // Create shape caster as a slightly smaller version of collider
        let mut caster_shape = collider.clone();
        caster_shape.set_scale(Vector::ONE * 0.99, 10);

        Self {
            character_controller: CharacterController,
            rigid_body: RigidBody::Kinematic,
            collider,
            ground_caster: ShapeCaster::new(caster_shape, Vector::ZERO, 0.0, Dir2::NEG_Y)
                .with_max_time_of_impact(30.0),
            gravity_strength: ControllerGravityStrengt(gravity),
            gravity: CurrentControllerGravity(Vec2::NEG_Y),
            movement: MovementBundle::default(),
        }
    }

    pub fn with_movement(
        mut self,
        acceleration: Scalar,
        damping: Scalar,
        jump_impulse: Scalar,
        max_slope_angle: Scalar,
    ) -> Self {
        self.movement = MovementBundle::new(acceleration, damping, jump_impulse, max_slope_angle);
        self
    }
}

fn not_halted() -> impl Condition<()> {
    resource_exists::<Halt>.and_then(resource_equals(Halt(false)))
}

/// Sends [`MovementAction`] events based on keyboard input.
fn keyboard_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
    let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

    let horizontal = right as i8 - left as i8;
    let direction = horizontal as Scalar;

    if direction != 0.0 {
        movement_event_writer.send(MovementAction::Move(direction));
    }

    if keyboard_input.just_pressed(KeyCode::Space) {
        println!("SPACE");
        movement_event_writer.send(MovementAction::Jump);
    }
}

/// Sends [`MovementAction`] events based on gamepad input.
fn gamepad_input(
    mut movement_event_writer: EventWriter<MovementAction>,
    gamepads: Res<Gamepads>,
    axes: Res<Axis<GamepadAxis>>,
    buttons: Res<ButtonInput<GamepadButton>>,
) {
    for gamepad in gamepads.iter() {
        let axis_lx = GamepadAxis {
            gamepad,
            axis_type: GamepadAxisType::LeftStickX,
        };

        if let Some(x) = axes.get(axis_lx) {
            movement_event_writer.send(MovementAction::Move(x as Scalar));
        }

        let jump_button = GamepadButton {
            gamepad,
            button_type: GamepadButtonType::South,
        };

        if buttons.just_pressed(jump_button) {
            movement_event_writer.send(MovementAction::Jump);
        }
    }
}

/// Updates the [`Grounded`] status for character controllers.
fn update_grounded(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &ShapeHits,
            &Rotation,
            &CurrentControllerGravity,
            Option<&MaxSlopeAngle>,
        ),
        With<CharacterController>,
    >,
) {
    for (entity, hits, rotation, gravity, max_slope_angle) in &mut query {
        // The character is grounded if the shape caster has a hit with a normal
        // that isn't too steep.
        let is_grounded = hits.iter().any(|hit| {
            if let Some(angle) = max_slope_angle {
                let outcome = (rotation * -hit.normal2).angle_between(-gravity.0).abs() <= angle.0;
                println!("is grounded: {outcome}");
                outcome
            } else {
                true
            }
        });

        if is_grounded {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

/// Responds to [`MovementAction`] events and moves character controllers accordingly.
fn movement(
    time: Res<Time>,
    mut movement_event_reader: EventReader<MovementAction>,
    mut controllers: Query<(
        &MovementAcceleration,
        &CurrentControllerGravity,
        &JumpImpulse,
        &mut LinearVelocity,
        Has<Grounded>,
    )>,
    mut gizmos: Gizmos,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for event in movement_event_reader.read() {
        for (movement_acceleration, gravity, jump_impulse, mut linear_velocity, is_grounded) in
            &mut controllers
        {
            match event {
                MovementAction::Move(direction) => {
                    let direction = Vec2::from_angle(PI / 2.0).rotate(gravity.0)
                        * (*direction * movement_acceleration.0 * delta_time);
                    linear_velocity.0 += direction;
                    gizmos.arrow_2d(Vec2::ZERO, direction, Color::srgb(1.0, 0.0, 0.0));
                }
                MovementAction::Jump => {
                    println!("JUMP");
                    if is_grounded {
                        println!("velocity: {}", -gravity.0 * jump_impulse.0);
                        *linear_velocity = LinearVelocity(-gravity.0 * jump_impulse.0);
                    }
                }
            }
        }
    }
}

/// Applies [`ControllerGravity`] to character controllers.
fn apply_gravity(
    time: Res<Time>,
    mut controllers: Query<(
        &ControllerGravityStrengt,
        &Transform,
        &mut CurrentControllerGravity,
        &mut LinearVelocity,
        &mut ShapeCaster,
    )>,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    let delta_time = time.delta_seconds_f64().adjust_precision();

    for (gravity, transform, mut current_gravity, mut linear_velocity, mut shape_caster) in
        &mut controllers
    {
        let position = ((*transform) * Vec3::ZERO).xy();
        let direction = (position - Vec2::ZERO + Vec2::NEG_Y * 0.0001).normalize();

        *current_gravity = CurrentControllerGravity(direction);
        shape_caster.direction = Dir2::new_unchecked(direction);

        linear_velocity.0 += (direction * gravity.0).xy() * delta_time;
    }
}

fn move_camera(
    mut camera: Query<(&mut Transform, &Camera2d)>,
    mut controllers: Query<(&CurrentControllerGravity, &Position)>,
) {
    for (gravity, position) in &mut controllers {
        let mut camera_transform = camera.get_single_mut().unwrap().0;
        *camera_transform = camera_transform
            .with_rotation(Quat::from_rotation_z(-gravity.0.angle_between(Vec2::NEG_Y)).normalize())
            .with_translation(position.0.extend(0.0));
    }
}

/// Slows down movement in the X direction.
fn apply_movement_damping(
    mut query: Query<(
        &MovementDampingFactor,
        &mut LinearVelocity,
        &CurrentControllerGravity,
    )>,
) {
    for (damping_factor, mut linear_velocity, gravity) in &mut query {
        let tangent = gravity.0.perp();
        let projection = linear_velocity.project_onto(tangent);
        linear_velocity.0 -= projection * damping_factor.0;
    }
}

/// Kinematic bodies do not get pushed by collisions by default,
/// so it needs to be done manually.
///
/// This system handles collision response for kinematic character controllers
/// by pushing them along their contact normals by the current penetration depth,
/// and applying velocity corrections in order to snap to slopes, slide along walls,
/// and predict collisions using speculative contacts.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn kinematic_controller_collisions(
    collisions: Res<Collisions>,
    bodies: Query<&RigidBody>,
    collider_parents: Query<&ColliderParent, Without<Sensor>>,
    mut character_controllers: Query<
        (
            &mut Position,
            &Rotation,
            &mut LinearVelocity,
            Option<&MaxSlopeAngle>,
            &mut CurrentControllerGravity,
        ),
        (With<RigidBody>, With<CharacterController>),
    >,
    time: Res<Time>,
    mut halt: ResMut<Halt>,
    mut gizmos: Gizmos,
) {
    if collisions.iter().count() == 0 {
        halt.0 = false;
        println!("no contacts");
    }

    // Iterate through collisions and move the kinematic body to resolve penetration
    for contacts in collisions.iter() {
        // Get the rigid body entities of the colliders (colliders could be children)
        let Ok([collider_parent1, collider_parent2]) =
            collider_parents.get_many([contacts.entity1, contacts.entity2])
        else {
            continue;
        };

        // Get the body of the character controller and whether it is the first
        // or second entity in the collision.
        let is_first: bool;

        let character_rb: RigidBody;
        let is_other_dynamic: bool;

        let (mut position, rotation, mut linear_velocity, max_slope_angle, gravity) =
            if let Ok(character) = character_controllers.get_mut(collider_parent1.get()) {
                is_first = true;
                character_rb = *bodies.get(collider_parent1.get()).unwrap();
                is_other_dynamic = bodies
                    .get(collider_parent2.get())
                    .is_ok_and(|rb| rb.is_dynamic());
                character
            } else if let Ok(character) = character_controllers.get_mut(collider_parent2.get()) {
                is_first = false;
                character_rb = *bodies.get(collider_parent2.get()).unwrap();
                is_other_dynamic = bodies
                    .get(collider_parent1.get())
                    .is_ok_and(|rb| rb.is_dynamic());
                character
            } else {
                continue;
            };

        // gizmos.arrow_2d(
        //     **position,
        //     **position + gravity.0,
        //     Color::srgb(1.0, 0.0, 0.0),
        // );

        // This system only handles collision response for kinematic character controllers.
        if !character_rb.is_kinematic() {
            continue;
        }

        println!("-----------------");

        // Iterate through contact manifolds and their contacts.
        // Each contact in a single manifold shares the same contact normal.
        for manifold in contacts.manifolds.iter() {
            let normal = if is_first {
                println!("chose normal 1");
                -manifold.global_normal1(rotation)
            } else {
                println!("chose normal 2");
                -manifold.global_normal2(rotation)
            };

            println!("normal: {normal}");
            // gizmos.arrow_2d(
            //     **position,
            //     **position + normal * 60.0,
            //     Color::srgb(1.0, 1.0, 1.0),
            // );

            let mut deepest_penetration: Scalar = Scalar::MIN;

            const PEN_LIMIT: f32 = 3.0;

            // Solve each penetrating contact in the manifold.
            for contact in manifold.contacts.iter() {
                let mov = if contact.penetration > PEN_LIMIT {
                    println!("MOVE: pos");
                    normal * contact.penetration
                } else {
                    // -normal * contact.penetration
                    Vec2::ZERO
                };
                println!("move: {mov}");
                position.0 += mov;
                println!("penetration: {}", contact.penetration);
                deepest_penetration = deepest_penetration.max(contact.penetration);
            }

            // For now, this system only handles velocity corrections for collisions against static geometry.
            if is_other_dynamic {
                continue;
            }

            // Determine if the slope is climbable or if it's too steep to walk on.
            let slope_angle = normal.angle_between(-gravity.0);
            let climbable = max_slope_angle.is_some_and(|angle| slope_angle.abs() <= angle.0);

            println!("deepest penetration: {deepest_penetration}");

            if deepest_penetration > 0.0 {
                println!("reject from surface");

                if deepest_penetration < 1.0 {
                    continue;
                }
                println!("MOVE: reject");
                let impulse = linear_velocity.reject_from_normalized(normal);
                println!("impulse: {}", impulse.length());
                linear_velocity.0 = impulse;
            } else {
                if deepest_penetration > -PEN_LIMIT {
                    continue;
                }
                println!("no intersect yet");

                let normal_speed = linear_velocity.dot(normal);
                println!("normal speed: {normal_speed}");

                // Don't apply an impulse if the character is moving away from the surface.
                // Otherwise jumping is rather hard ;)
                if normal_speed > 0.0 {
                    continue;
                }

                println!("MOVE: impulse");

                let impulse_magnitude = normal_speed
                    - deepest_penetration / time.delta_seconds_f64().adjust_precision();
                let impulse = impulse_magnitude * normal;

                println!("impulse: {impulse}");

                // gizmos.arrow_2d(
                //     **position,
                //     **position + impulse * 600.0,
                //     Color::srgb(0.0, 0.0, 0.0),
                // );

                linear_velocity.0 -= impulse;
            }
        }
    }
}

#[test]
fn dot() {
    dbg!(Vec2::new(0.0, 1.0).dot(Vec2::new(0.5, 1.0)));
    panic!();
}
