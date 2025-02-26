/*
 * Boid Flocking Simulation - Module Definitions
 * 
 * This file defines the module structure for the boid simulation application.
 * It organizes the code into logical components for better maintainability.
 */

// Re-export key components for easier access
pub use boid::Boid;
pub use camera::Camera;
pub use spatial_grid::SpatialGrid;
pub use params::SimulationParams;
pub use debug::DebugInfo;
pub use app::Model;

// Define modules
pub mod boid;
pub mod camera;
pub mod spatial_grid;
pub mod params;
pub mod debug;
pub mod app;
pub mod ui;
pub mod physics;
pub mod renderer;
pub mod culling;
pub mod input;

// Constants
pub const BOID_SIZE: f32 = 6.0;
pub const WORLD_SIZE: f32 = 5000.0; 