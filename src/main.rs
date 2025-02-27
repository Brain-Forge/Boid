/*
 * Boid Flocking Simulation
 * 
 * This application simulates the flocking behavior of birds (boids) based on three main rules:
 * 1. Separation: Avoid crowding neighbors
 * 2. Alignment: Steer towards the average heading of neighbors
 * 3. Cohesion: Steer towards the average position of neighbors
 * 
 * The simulation includes interactive sliders to adjust parameters in real-time
 * and displays debug information about the current state.
 * 
 * Features:
 * - Dynamic window sizing based on user's screen
 * - Camera controls for zooming and panning
 * - Large simulation space that extends beyond the visible area
 * - Spatial partitioning for efficient neighbor lookups
 * - Parallel processing for performance optimization
 * - Modular code organization for better maintainability
 */

// Import the library modules
mod boid;
mod camera;
mod spatial_grid;
mod params;
mod debug;
mod app;
mod ui;
mod physics;
mod renderer;
mod culling;
mod input;

// Re-export constants
pub const BOID_SIZE: f32 = 6.0;

fn main() {
    nannou::app(app::model)
        .update(app::update)
        .run();
}
