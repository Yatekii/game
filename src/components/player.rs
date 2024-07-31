use avian2d::{collision::Collider, math::Scalar};
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};

use crate::plugins::character_controller::CharacterControllerBundle;

const PLAYER_DIAMETER: f32 = 30.0;
const PLAYER_COLOR: Color = Color::srgb(0.5, 0.5, 0.9);

#[derive(Component)]
pub struct Player {}

pub fn spawn_player(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    println!("Spawn player");

    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes.add(Circle::default()).into(),
            material: materials.add(PLAYER_COLOR),
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0))
                .with_scale(Vec2::splat(PLAYER_DIAMETER).extend(1.)),
            ..default()
        },
        Player {},
        CharacterControllerBundle::new(Collider::circle(0.5), 1500.0).with_movement(
            1250.0,
            0.92,
            800.0,
            (70.0 as Scalar).to_radians(),
        ),
    ));
}
