use std::f32::consts::PI;

use avian2d::{collision::Collider, dynamics::rigid_body::RigidBody};
use bevy::prelude::*;
use bevy_ecs_tilemap::tiles::TileBundle;

const N_TILES: usize = 31;
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

pub fn spawn_map(mut commands: Commands, asset_server: Res<AssetServer>) {
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
                    rotation: Quat::from_rotation_z(phi + PI / 2.0),
                },
                sprite: Sprite {
                    // color: TILE_COLOR,
                    rect: Some(Rect::new(0.0, 30.0, 48.0, 64.0)),
                    custom_size: Some(Vec2::new(1.0, 1.0)),
                    ..default()
                },
                texture: asset_server.load("tile_jungle_ground_brown.png"),
                ..default()
            },
            // ImageScaleMode::Tiled {
            //     tile_x: true,
            //     tile_y: true,
            //     stretch_value: 0.5, // The image will tile every 128px
            // },
            RigidBody::Static,
            Collider::rectangle(1.0, 1.0),
            Tile,
        ));
    }

    #[allow(clippy::never_loop)]
    for x in 0..N_TILES / 2 {
        let r: f32 = world_r * 0.8;
        let phi = 2.0 * PI * (x as f32) / (N_TILES as f32);
        let x = r * phi.cos();
        let y = r * phi.sin();
        commands.spawn((
            SpriteBundle {
                transform: Transform {
                    translation: Vec3::new(x, y, 0.0),
                    scale: Vec3::splat(TILE_SIZE) * 0.99,
                    rotation: Quat::from_rotation_z(phi + PI / 2.0),
                },
                sprite: Sprite {
                    // color: TILE_COLOR,
                    rect: Some(Rect::new(0.0, 30.0, 48.0, 64.0)),
                    custom_size: Some(Vec2::new(1.0, 1.0)),
                    ..default()
                },
                texture: asset_server.load("tile_jungle_ground_brown.png"),
                ..default()
            },
            // ImageScaleMode::Tiled {
            //     tile_x: true,
            //     tile_y: true,
            //     stretch_value: 0.5, // The image will tile every 128px
            // },
            RigidBody::Static,
            Collider::rectangle(1.0, 1.0),
            Tile,
        ));
        return;
    }
}
