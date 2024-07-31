use std::f32::consts::PI;

use avian2d::{collision::Collider, dynamics::rigid_body::RigidBody};
use bevy::prelude::*;

const N_TILES: usize = 30;
const TILE_SIZE: f32 = 60.0;
const TILE_COLOR: Color = Color::srgb(0.7, 0.3, 0.3);

fn world_r() -> f32 {
    let phi = (2.0 * PI) / ((N_TILES as f32) * 2.0);
    TILE_SIZE / 2.0 / phi.sin() + TILE_SIZE / 2.0
}

#[derive(Resource, Deref, DerefMut)]
pub struct Gravity(pub Vec2);

#[derive(Component)]
pub struct Tile;

pub fn spawn_map(mut commands: Commands) {
    let world_r = world_r();
    println!(
        r"
r = {world_r}
tiles = {N_TILES}
    "
    );
    for x in 0..N_TILES {
        let r: f32 = world_r;
        let phi = 2.0 * PI * (x as f32) / (N_TILES as f32);
        let x = r * phi.cos();
        let y = r * phi.sin();
        commands.spawn((
            SpriteBundle {
                transform: Transform {
                    translation: Vec3::new(x, y, 0.0),
                    scale: Vec3::splat(TILE_SIZE) * 0.99,
                    rotation: Quat::from_rotation_z(phi),
                },
                sprite: Sprite {
                    color: TILE_COLOR,
                    ..default()
                },
                ..default()
            },
            RigidBody::Static,
            Collider::rectangle(1.0, 1.0),
            Tile,
        ));
    }
}
