#![allow(clippy::type_complexity)]

pub mod components;
pub mod plugins;

use avian2d::{debug_render::PhysicsDebugPlugin, PhysicsPlugins};
use bevy::prelude::*;
use components::{player::spawn_player, tile::spawn_map};
use plugins::character_controller::CharacterControllerPlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins,))
        .add_plugins((
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 20 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(20.0),
            CharacterControllerPlugin,
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, (setup, spawn_map, spawn_player))
        // .insert_gizmo_config(
        //     PhysicsGizmos {
        //         aabb_color: Some(Color::WHITE),
        //         ..default()
        //     },
        //     GizmoConfig::default(),
        // )
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}
