/*
 * Simulation Parameters Module
 * 
 * This module defines the SimulationParams struct that contains all the
 * adjustable parameters for the boid simulation. These parameters can be
 * modified through the UI. It also provides methods for parameter change detection
 * and management to improve separation of concerns.
 * 
 * Optimized for performance with spatial partitioning and adaptive settings.
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
    pub world_size: f32,  // Added world size parameter
    pub show_debug: bool,
    pub pause_simulation: bool,
    // Performance settings
    pub enable_parallel: bool,
    pub enable_spatial_grid: bool,
    pub cell_size_factor: f32,  // Multiplier for cell size relative to perception radius
    pub enable_squared_distance: bool, // Use squared distance calculations to avoid sqrt operations
    pub enable_frustum_culling: bool, // Enable frustum culling optimization
    pub adaptive_cell_sizing: bool, // Dynamically adjust cell size based on boid density
    // Timing settings
    pub fixed_physics_fps: f32, // Fixed physics update rate (updates per second)
    pub target_render_fps: f32, // Target rendering framerate (0 = unlimited)
    pub enable_interpolation: bool, // Enable interpolation between physics updates
    
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
    world_size: f32,  // Added world size parameter
    show_debug: bool,
    enable_squared_distance: bool,
    enable_frustum_culling: bool,
    adaptive_cell_sizing: bool,
    fixed_physics_fps: f32,
    target_render_fps: f32,
    enable_interpolation: bool,
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            num_boids: 500, // Increased default number of boids for the larger world
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            separation_radius: 50.0,
            alignment_radius: 200.0,
            cohesion_radius: 150.0,
            max_speed: 50.0,
            world_size: 5000.0, // Default world size (same as the constant)
            show_debug: false,
            pause_simulation: false,
            // Default performance settings
            enable_parallel: true,
            enable_spatial_grid: true,
            cell_size_factor: 0.1,
            enable_squared_distance: true, // Enable by default for better performance
            enable_frustum_culling: true,  // Enable frustum culling by default
            adaptive_cell_sizing: true,    // Enable adaptive cell sizing by default
            // Default timing settings
            fixed_physics_fps: 30.0, // 60 physics updates per second
            target_render_fps: 0.0,  // Unlimited rendering by default
            enable_interpolation: true, // Enable interpolation by default
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
            world_size: self.world_size,  // Added world size parameter
            show_debug: self.show_debug,
            enable_squared_distance: self.enable_squared_distance,
            enable_frustum_culling: self.enable_frustum_culling,
            adaptive_cell_sizing: self.adaptive_cell_sizing,
            fixed_physics_fps: self.fixed_physics_fps,
            target_render_fps: self.target_render_fps,
            enable_interpolation: self.enable_interpolation,
        });
    }
    
    // Detect changes in parameters and return flags for different types of changes
    // Returns (boids_changed, physics_changed, rendering_changed, world_size_changed)
    pub fn detect_changes(&self) -> (bool, bool, bool, bool) {
        if let Some(prev) = &self.previous_values {
            let boids_changed = self.num_boids != prev.num_boids;
            
            let physics_changed = 
                self.separation_weight != prev.separation_weight ||
                self.alignment_weight != prev.alignment_weight ||
                self.cohesion_weight != prev.cohesion_weight ||
                self.separation_radius != prev.separation_radius ||
                self.alignment_radius != prev.alignment_radius ||
                self.cohesion_radius != prev.cohesion_radius ||
                self.max_speed != prev.max_speed ||
                self.enable_squared_distance != prev.enable_squared_distance ||
                self.adaptive_cell_sizing != prev.adaptive_cell_sizing;
            
            let rendering_changed = 
                self.show_debug != prev.show_debug ||
                self.enable_frustum_culling != prev.enable_frustum_culling ||
                self.fixed_physics_fps != prev.fixed_physics_fps ||
                self.target_render_fps != prev.target_render_fps ||
                self.enable_interpolation != prev.enable_interpolation;
            
            let world_size_changed = self.world_size != prev.world_size;
            
            (boids_changed, physics_changed, rendering_changed, world_size_changed)
        } else {
            // If no previous values, consider everything changed
            (true, true, true, true)
        }
    }
    
    // Range getters for UI sliders
    
    pub fn get_num_boids_range() -> std::ops::RangeInclusive<usize> {
        10..=200000
    }
    
    pub fn get_max_speed_range() -> std::ops::RangeInclusive<f32> {
        1.0..=10.0
    }
    
    pub fn get_weight_range() -> std::ops::RangeInclusive<f32> {
        0.0..=3.0
    }
    
    pub fn get_radius_range() -> std::ops::RangeInclusive<f32> {
        5.0..=200.0
    }
    
    pub fn get_world_size_range() -> std::ops::RangeInclusive<f32> {
        1000.0..=50000.0
    }
    
    pub fn get_cell_size_factor_range() -> std::ops::RangeInclusive<f32> {
        0.01..=2.0
    }
    
    pub fn get_physics_fps_range() -> std::ops::RangeInclusive<f32> {
        30.0..=240.0
    }
    
    pub fn get_render_fps_range() -> std::ops::RangeInclusive<f32> {
        0.0..=240.0
    }
} 