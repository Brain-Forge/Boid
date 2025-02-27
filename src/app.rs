/*
 * Application Module
 * 
 * This module defines the main application model and coordinates the different
 * components of the boid simulation. It handles initialization and delegates
 * specific tasks to specialized modules.
 * 
 * The application is structured to optimize performance through:
 * - Modular code organization for better maintainability
 * - Fixed timestep physics with interpolated rendering
 * - Efficient spatial partitioning and frustum culling
 * - Adaptive cell sizing for optimal spatial grid performance
 */

use nannou::prelude::*;
use nannou_egui::Egui;
use rand::Rng;
use std::cell::UnsafeCell;
use std::time::{Duration, Instant};

use crate::boid::Boid;
use crate::camera::Camera;
use crate::spatial_grid::SpatialGrid;
use crate::params::SimulationParams;
use crate::debug::DebugInfo;
use crate::physics;
use crate::renderer;
use crate::input;
use crate::ui;

// Main model for the application
pub struct Model {
    pub boids: Vec<Boid>,
    pub params: SimulationParams,
    pub egui: Egui,
    pub debug_info: UnsafeCell<DebugInfo>,
    pub camera: Camera,
    pub mouse_position: Vec2,
    pub spatial_grid: SpatialGrid,
    pub cached_visible_boids: UnsafeCell<Option<Vec<usize>>>,
    pub render_needed: UnsafeCell<bool>,
    pub _last_camera_state: Option<(Vec2, f32)>, // Marked as intentionally unused
    // Fixed timestep physics variables
    pub physics_accumulator: Duration,
    pub physics_step_size: Duration,
    pub last_update_time: Instant,
    pub interpolation_alpha: f32,
    pub _last_render_time: Instant, // Marked as intentionally unused
    // Frustum culling optimization
    pub visible_area_cache: Option<Rect>,
    // Boid selection and following
    pub selected_boid_index: Option<usize>,
    // Adaptive cell sizing
    pub last_cell_size_update: Instant,
    pub cell_size_update_interval: Duration,
}

// Make Model safe to share across threads
unsafe impl Sync for Model {}

// Initialize the model
pub fn model(app: &App) -> Model {
    // Get the primary monitor's dimensions
    let monitor = app.primary_monitor().expect("Failed to get primary monitor");
    let monitor_size = monitor.size();
    
    // Calculate window size based on monitor size (80% of monitor size)
    let window_width = monitor_size.width as f32 * 0.8;
    let window_height = monitor_size.height as f32 * 0.8;
    
    // Create the main window with dynamic size
    let window_id = app
        .new_window()
        .title("Boid Flocking Simulation")
        .size(window_width as u32, window_height as u32)
        .view(renderer::view)
        .mouse_moved(input::mouse_moved)
        .mouse_pressed(input::mouse_pressed)
        .mouse_released(input::mouse_released)
        .mouse_wheel(input::mouse_wheel)
        .raw_event(input::raw_window_event)
        .build()
        .unwrap();
    
    // Get the window
    let window = app.window(window_id).unwrap();
    
    // Create the UI
    let egui = Egui::from_window(&window);
    
    // Create simulation parameters
    let params = SimulationParams::default();
    
    // Create camera
    let camera = Camera::new();
    
    // Create spatial grid, cell size should be at least as large as the largest perception radius
    let max_radius = f32::max(
        params.separation_radius,
        f32::max(params.alignment_radius, params.cohesion_radius)
    );
    
    // Use cell size factor to determine cell size
    let cell_size = max_radius * params.cell_size_factor;
    
    // Create spatial grid with the world size from params
    let spatial_grid = SpatialGrid::new(cell_size, params.world_size);
    
    // Create boids
    let mut boids = Vec::with_capacity(params.num_boids);
    let mut rng = rand::thread_rng();
    
    // Create boids with random positions within the world bounds
    for _ in 0..params.num_boids {
        let half_world = params.world_size / 2.0;
        let x = rng.gen_range(-half_world..half_world);
        let y = rng.gen_range(-half_world..half_world);
        boids.push(Boid::new(x, y));
    }
    
    // Set max speed for all boids
    for boid in &mut boids {
        boid.max_speed = params.max_speed;
    }
    
    // Calculate physics step size based on fixed FPS
    let physics_step_size = Duration::from_secs_f32(1.0 / params.fixed_physics_fps);
    
    // Create the model
    let mut model = Model {
        boids,
        params,
        egui,
        debug_info: UnsafeCell::new(DebugInfo::default()),
        camera,
        mouse_position: Vec2::ZERO,
        spatial_grid,
        cached_visible_boids: UnsafeCell::new(None),
        render_needed: UnsafeCell::new(true),
        _last_camera_state: None,
        physics_accumulator: Duration::from_secs(0),
        physics_step_size,
        last_update_time: Instant::now(),
        interpolation_alpha: 0.0,
        _last_render_time: Instant::now(),
        visible_area_cache: None,
        selected_boid_index: None,
        last_cell_size_update: Instant::now(),
        cell_size_update_interval: Duration::from_secs(1), // Update cell size every second
    };
    
    // Take initial snapshot of parameters
    model.params.take_snapshot();
    
    model
}

// Update the model
pub fn update(app: &App, model: &mut Model, update: Update) {
    // Update the UI
    let ui_response = ui::update_ui(app, model, &update);
    
    // Detect parameter changes
    let (boids_changed, physics_changed, rendering_changed, world_size_changed) = model.params.detect_changes();
    
    // Handle parameter changes
    if boids_changed {
        // Reset boids if the number has changed
        physics::reset_boids(model);
    }
    
    if physics_changed || world_size_changed {
        // Update max speed for all boids if it changed
        for boid in &mut model.boids {
            boid.max_speed = model.params.max_speed;
        }
        
        // Update physics step size if FPS changed
        model.physics_step_size = Duration::from_secs_f32(1.0 / model.params.fixed_physics_fps);
    }
    
    // If world size changed, we need to recreate the spatial grid
    if world_size_changed {
        // Recalculate cell size
        let max_radius = f32::max(
            model.params.separation_radius,
            f32::max(model.params.alignment_radius, model.params.cohesion_radius)
        );
        let cell_size = max_radius * model.params.cell_size_factor;
        
        // Create new spatial grid with updated world size
        model.spatial_grid = SpatialGrid::new(cell_size, model.params.world_size);
        
        // Ensure all boids are within the new world bounds
        let half_world = model.params.world_size / 2.0;
        for boid in &mut model.boids {
            // Wrap boids that are outside the new world bounds
            if boid.position.x > half_world || boid.position.x < -half_world ||
               boid.position.y > half_world || boid.position.y < -half_world {
                boid.wrap_edges(model.params.world_size);
            }
        }
    }
    
    // Take a snapshot of the current parameters for change detection in the next frame
    model.params.take_snapshot();
    
    // Skip physics updates if paused
    if !model.params.pause_simulation {
        // Calculate time since last update
        let now = Instant::now();
        let dt = now.duration_since(model.last_update_time);
        model.last_update_time = now;
        
        // Add to accumulator
        model.physics_accumulator += dt;
        
        // Store previous state for interpolation
        for boid in &mut model.boids {
            boid.store_previous_state();
        }
        
        // Perform fixed timestep updates
        while model.physics_accumulator >= model.physics_step_size {
            // Update boids
            physics::update_boids(model);
            
            // Subtract step size from accumulator
            model.physics_accumulator -= model.physics_step_size;
        }
        
        // Calculate interpolation alpha
        if model.params.enable_interpolation {
            model.interpolation_alpha = model.physics_accumulator.as_secs_f32() / model.physics_step_size.as_secs_f32();
        } else {
            model.interpolation_alpha = 0.0;
        }
        
        // Update camera position to follow selected boid if in follow mode
        if model.camera.follow_mode && model.selected_boid_index.is_some() {
            let boid_idx = model.selected_boid_index.unwrap();
            if boid_idx < model.boids.len() {
                // Get the interpolated position of the boid for smooth camera movement
                let boid_pos = if model.params.enable_interpolation {
                    model.boids[boid_idx].get_interpolated_position(model.interpolation_alpha)
                } else {
                    model.boids[boid_idx].position
                };
                
                // Update camera position to match the boid's position
                model.camera.position = Vec2::new(boid_pos.x, boid_pos.y);
                
                // Force re-render when following a boid
                unsafe { *model.render_needed.get() = true; }
                
                // Clear the cached visible boids when camera moves
                unsafe { *model.cached_visible_boids.get() = None; }
                
                // Clear the visible area cache
                model.visible_area_cache = None;
            }
        }
        
        // Update adaptive cell size if enabled
        if model.params.adaptive_cell_sizing && 
           now.duration_since(model.last_cell_size_update) >= model.cell_size_update_interval {
            update_adaptive_cell_size(model);
            model.last_cell_size_update = now;
        }
    }
    
    // Update debug info
    if model.params.show_debug {
        model.debug_info.get_mut().update_from_app(app);
        
        // Get the cached visible boids
        let cached_visible_boids = unsafe { &*model.cached_visible_boids.get() };
        
        model.debug_info.get_mut().update_from_model(
            model.selected_boid_index,
            model.camera.follow_mode,
            model.interpolation_alpha,
            cached_visible_boids,
            model.boids.len(),
            model.visible_area_cache,
            model.params.world_size
        );
    }
    
    // Mark that a render is needed
    unsafe {
        *model.render_needed.get() = true;
    }
}

// Update the spatial grid cell size based on boid density
fn update_adaptive_cell_size(model: &mut Model) {
    // Calculate the maximum perception radius
    let max_radius = f32::max(
        model.params.separation_radius,
        f32::max(model.params.alignment_radius, model.params.cohesion_radius)
    );
    
    // Calculate average number of neighbors per boid
    let mut total_neighbors = 0;
    let sample_size = model.boids.len().min(100); // Sample at most 100 boids for efficiency
    
    if sample_size == 0 {
        return; // No boids to sample
    }
    
    let step = model.boids.len() / sample_size;
    
    // Extract positions for the spatial grid's calculations
    let boid_positions: Vec<Point2> = model.boids.iter().map(|boid| boid.position).collect();
    
    for i in (0..model.boids.len()).step_by(step.max(1)) {
        if i >= model.boids.len() {
            break;
        }
        
        let nearby = model.spatial_grid.get_nearby_with_distances(
            model.boids[i].position,
            &boid_positions,
            model.params.world_size
        );
        
        total_neighbors += nearby.len();
    }
    
    let avg_neighbors = total_neighbors as f32 / sample_size as f32;
    
    // Adjust cell size based on average neighbors
    // Target: around 10-20 neighbors per cell for optimal performance
    let target_neighbors = 15.0;
    let current_cell_size = model.spatial_grid.cell_size;
    
    let mut new_cell_size = if avg_neighbors > target_neighbors * 1.5 {
        // Too many neighbors, decrease cell size
        current_cell_size * 0.9
    } else if avg_neighbors < target_neighbors * 0.5 {
        // Too few neighbors, increase cell size
        current_cell_size * 1.1
    } else {
        // Good range, keep current size
        current_cell_size
    };
    
    // Ensure cell size is at least the maximum perception radius
    new_cell_size = f32::max(new_cell_size, max_radius * model.params.cell_size_factor);
    
    // Only recreate grid if cell size changed significantly
    if (new_cell_size - current_cell_size).abs() > current_cell_size * 0.1 {
        model.spatial_grid = SpatialGrid::new(new_cell_size, model.params.world_size);
    }
} 