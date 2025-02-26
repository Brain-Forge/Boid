/*
 * Simulation Parameters Module
 * 
 * This module defines the SimulationParams struct that contains all the
 * adjustable parameters for the boid simulation. These parameters can be
 * modified through the UI. It also provides methods for parameter change detection
 * and management to improve separation of concerns.
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
    
    // Internal state for tracking changes
    previous_values: Option<ParamSnapshot>,
}

// A snapshot of parameter values used for change detection
struct ParamSnapshot {
    num_boids: usize,
    separation_weight: f32,
    alignment_weight: f32,
    cohesion_weight: f32,
    separation_radius: f32,
    alignment_radius: f32,
    cohesion_radius: f32,
    max_speed: f32,
    show_debug: bool,
    pause_simulation: bool,
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
            // Initialize with no previous values
            previous_values: None,
        }
    }
}

impl SimulationParams {
    // Take a snapshot of current parameter values for change detection
    pub fn take_snapshot(&mut self) {
        self.previous_values = Some(ParamSnapshot {
            num_boids: self.num_boids,
            separation_weight: self.separation_weight,
            alignment_weight: self.alignment_weight,
            cohesion_weight: self.cohesion_weight,
            separation_radius: self.separation_radius,
            alignment_radius: self.alignment_radius,
            cohesion_radius: self.cohesion_radius,
            max_speed: self.max_speed,
            show_debug: self.show_debug,
            pause_simulation: self.pause_simulation,
        });
    }
    
    // Check if any parameters have changed since the last snapshot
    // Returns a tuple of (should_reset_boids, num_boids_changed, any_ui_changed)
    pub fn detect_changes(&self) -> (bool, bool, bool) {
        let mut num_boids_changed = false;
        let mut ui_changed = false;
        
        // If we don't have previous values, nothing has changed
        if let Some(prev) = &self.previous_values {
            // Check for number of boids change
            if self.num_boids != prev.num_boids {
                num_boids_changed = true;
                ui_changed = true;
            }
            
            // Check for other parameter changes
            if self.separation_weight != prev.separation_weight ||
               self.alignment_weight != prev.alignment_weight ||
               self.cohesion_weight != prev.cohesion_weight ||
               self.separation_radius != prev.separation_radius ||
               self.alignment_radius != prev.alignment_radius ||
               self.cohesion_radius != prev.cohesion_radius ||
               self.max_speed != prev.max_speed ||
               self.show_debug != prev.show_debug ||
               self.pause_simulation != prev.pause_simulation {
                ui_changed = true;
            }
        }
        
        // The first element (should_reset_boids) will be set by the UI when the reset button is clicked
        (false, num_boids_changed, ui_changed)
    }
    
    // Get parameter ranges for UI sliders
    pub fn get_num_boids_range() -> std::ops::RangeInclusive<usize> {
        10..=100000
    }
    
    pub fn get_max_speed_range() -> std::ops::RangeInclusive<f32> {
        1.0..=100.0
    }
    
    pub fn get_weight_range() -> std::ops::RangeInclusive<f32> {
        0.0..=3.0
    }
    
    pub fn get_radius_range() -> std::ops::RangeInclusive<f32> {
        10.0..=100.0
    }
    
    pub fn get_cell_size_factor_range() -> std::ops::RangeInclusive<f32> {
        0.05..=10.0
    }
} 