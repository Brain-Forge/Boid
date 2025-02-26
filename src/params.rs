/*
 * Simulation Parameters Module
 * 
 * This module defines the SimulationParams struct that contains all the
 * adjustable parameters for the boid simulation. These parameters can be
 * modified through the UI.
 */

// Parameters for the simulation that can be adjusted via UI
pub struct SimulationParams {
    pub num_boids: usize,
    pub separation_weight: f32,
    pub alignment_weight: f32,
    pub cohesion_weight: f32,
    pub separation_radius: f32,
    pub alignment_radius: f32,
    pub cohesion_radius: f32,
    pub max_speed: f32,
    pub show_debug: bool,
    pub pause_simulation: bool,
    // Performance settings
    pub enable_parallel: bool,
    pub enable_spatial_grid: bool,
    pub cell_size_factor: f32,  // Multiplier for cell size relative to perception radius
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            num_boids: 500, // Increased default number of boids for the larger world
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            separation_radius: 25.0,
            alignment_radius: 50.0,
            cohesion_radius: 50.0,
            max_speed: 4.0,
            show_debug: false,
            pause_simulation: false,
            // Default performance settings
            enable_parallel: true,
            enable_spatial_grid: true,
            cell_size_factor: 1.0,
        }
    }
} 