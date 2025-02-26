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
use crate::WORLD_SIZE;

// Main model for the application
pub struct Model {
    pub boids: Vec<Boid>,
    pub params: SimulationParams,
    pub egui: Egui,
    pub debug_info: DebugInfo,
    pub camera: Camera,
    pub mouse_position: Vec2,
    pub spatial_grid: SpatialGrid,
    pub cached_visible_boids: UnsafeCell<Option<Vec<usize>>>,
    pub render_needed: UnsafeCell<bool>,
    pub last_camera_state: Option<(Vec2, f32)>,
    // Fixed timestep physics variables
    pub physics_accumulator: Duration,
    pub physics_step_size: Duration,
    pub last_update_time: Instant,
    pub interpolation_alpha: f32,
    pub last_render_time: Instant,
    // Frustum culling optimization
    pub visible_area_cache: Option<Rect>,
    // Boid selection and following
    pub selected_boid_index: Option<usize>,
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
    let cell_size = f32::max(params.separation_radius, f32::max(params.alignment_radius, params.cohesion_radius));
    let spatial_grid = SpatialGrid::new(cell_size, WORLD_SIZE);
    
    // Create boids
    let mut boids = Vec::with_capacity(params.num_boids);
    let mut rng = rand::thread_rng();
    
    // Use the world size for boid positioning (much larger than the window)
    for _ in 0..params.num_boids {
        let x = rng.gen_range((-WORLD_SIZE / 2.0)..(WORLD_SIZE / 2.0));
        let y = rng.gen_range((-WORLD_SIZE / 2.0)..(WORLD_SIZE / 2.0));
        boids.push(Boid::new(x, y));
    }
    
    // Update max speed for all boids
    for boid in &mut boids {
        boid.max_speed = params.max_speed;
    }
    
    // Calculate physics step size based on default FPS
    let physics_step_size = Duration::from_secs_f32(1.0 / params.fixed_physics_fps);
    let now = Instant::now();
    
    // Return the model
    Model {
        boids,
        params,
        egui,
        debug_info: DebugInfo::default(),
        camera,
        mouse_position: Vec2::ZERO,
        spatial_grid,
        cached_visible_boids: UnsafeCell::new(None),
        render_needed: UnsafeCell::new(true),
        last_camera_state: None,
        physics_accumulator: Duration::ZERO,
        physics_step_size,
        last_update_time: now,
        interpolation_alpha: 0.0,
        last_render_time: now,
        visible_area_cache: None,
        selected_boid_index: None,
    }
}

// Update the model
pub fn update(app: &App, model: &mut Model, update: Update) {
    // Update debug info
    model.debug_info.fps = app.fps();
    model.debug_info.frame_time = update.since_last;
    model.debug_info.selected_boid_index = model.selected_boid_index;
    model.debug_info.follow_mode_active = model.camera.follow_mode;
    
    // Update UI and check if boids need to be reset
    let (should_reset_boids, num_boids_changed, ui_changed) = ui::update_ui(&mut model.egui, &mut model.params, &model.debug_info);
    
    // If UI changed, we need to re-render
    if ui_changed {
        unsafe { *model.render_needed.get() = true; }
        
        // Update physics step size if FPS changed
        model.physics_step_size = Duration::from_secs_f32(1.0 / model.params.fixed_physics_fps);
    }
    
    // Handle reset boids
    if should_reset_boids || num_boids_changed {
        physics::reset_boids(model);
        // Clear the caches when boids are reset
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
        
        // Clear selected boid when resetting
        model.selected_boid_index = None;
        model.camera.follow_mode = false;
    }
    
    // Get current time
    let current_time = Instant::now();
    
    // Calculate time since last update
    let frame_time = current_time.duration_since(model.last_update_time);
    model.last_update_time = current_time;
    
    // Add frame time to accumulator
    model.physics_accumulator += frame_time;
    
    // Only update boids if simulation is not paused
    if !model.params.pause_simulation {
        // Track number of physics updates in this frame
        let mut physics_updates_this_frame = 0;
        
        // Run fixed timestep updates
        while model.physics_accumulator >= model.physics_step_size {
            // Store previous state for interpolation
            for boid in &mut model.boids {
                boid.store_previous_state();
            }
            
            // Update physics
            physics::update_boids(model);
            
            // Subtract step size from accumulator
            model.physics_accumulator -= model.physics_step_size;
            
            // Increment counter
            physics_updates_this_frame += 1;
        }
        
        // Update debug info
        model.debug_info.physics_updates_per_frame = physics_updates_this_frame;
        
        // Calculate interpolation alpha
        if model.params.enable_interpolation {
            model.interpolation_alpha = model.physics_accumulator.as_secs_f32() / model.physics_step_size.as_secs_f32();
            model.interpolation_alpha = model.interpolation_alpha.clamp(0.0, 1.0);
        } else {
            model.interpolation_alpha = 1.0; // No interpolation, use current state
        }
        
        // Update debug info with interpolation alpha
        model.debug_info.interpolation_alpha = model.interpolation_alpha;
        
        // Clear the caches when simulation is running
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
    }
    
    // Update camera position if in follow mode
    if model.camera.follow_mode {
        if let Some(boid_idx) = model.selected_boid_index {
            if boid_idx < model.boids.len() {
                // Get the boid's interpolated position
                let boid_pos = model.boids[boid_idx].get_interpolated_position(model.interpolation_alpha);
                
                // Smoothly move the camera towards the boid
                let smoothing = 0.1; // Lower value = smoother/slower camera movement
                model.camera.position = model.camera.position.lerp(
                    Vec2::new(boid_pos.x, boid_pos.y), 
                    smoothing
                );
                
                // Force re-render when following
                unsafe { *model.render_needed.get() = true; }
            } else {
                // Selected boid no longer exists
                model.selected_boid_index = None;
                model.camera.follow_mode = false;
            }
        } else {
            // No boid selected, exit follow mode
            model.camera.follow_mode = false;
        }
    }
    
    // Check if camera has changed
    let current_camera_state = (model.camera.position, model.camera.zoom);
    if model.last_camera_state.is_none() || 
       model.last_camera_state.unwrap().0 != current_camera_state.0 || 
       model.last_camera_state.unwrap().1 != current_camera_state.1 {
        // Camera has changed, force a re-render
        unsafe { *model.render_needed.get() = true; }
        model.last_camera_state = Some(current_camera_state);
    }
    
    // Handle frame rate limiting for rendering
    if model.params.target_render_fps > 0.0 {
        let target_frame_duration = Duration::from_secs_f32(1.0 / model.params.target_render_fps);
        let time_since_last_render = current_time.duration_since(model.last_render_time);
        
        if time_since_last_render < target_frame_duration {
            // Not time to render yet
            unsafe { *model.render_needed.get() = false; }
        } else {
            // Time to render
            model.last_render_time = current_time;
            unsafe { *model.render_needed.get() = true; }
        }
    }
} 