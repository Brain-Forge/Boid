/*
 * Application Module
 * 
 * This module defines the main application model and logic for the boid simulation.
 * It handles the initialization, update, and rendering of the simulation.
 * 
 * Optimized for performance by:
 * - Combining forces before applying them to reduce vector operations
 * - Avoiding unnecessary vector instantiations and normalizations
 * - Using squared distances where possible
 * - Caching intermediate calculations
 * - Using fixed timestep for physics with interpolated rendering
 */

use nannou::prelude::*;
use nannou::winit::event::{MouseButton, MouseScrollDelta, TouchPhase};
use nannou_egui::Egui;
use rand::Rng;
use rayon::prelude::*;
use std::cell::UnsafeCell;
use std::time::{Duration, Instant};

use crate::boid::Boid;
use crate::camera::Camera;
use crate::spatial_grid::SpatialGrid;
use crate::params::SimulationParams;
use crate::debug::DebugInfo;
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
        .view(view)
        .mouse_moved(mouse_moved)
        .mouse_pressed(mouse_pressed)
        .mouse_released(mouse_released)
        .mouse_wheel(mouse_wheel)
        .raw_event(raw_window_event)
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
        // Fixed timestep physics variables
        physics_accumulator: Duration::ZERO,
        physics_step_size,
        last_update_time: now,
        interpolation_alpha: 0.0,
        last_render_time: now,
    }
}

// Update the model
pub fn update(app: &App, model: &mut Model, update: Update) {
    // Update debug info
    model.debug_info.fps = app.fps();
    model.debug_info.frame_time = update.since_last;
    
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
        reset_boids(model);
        // Clear the caches when boids are reset
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
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
            update_boids(model);
            
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

// Reset boids to random positions
fn reset_boids(model: &mut Model) {
    let mut rng = rand::thread_rng();
    
    // Resize the boids vector if needed
    model.boids.resize_with(model.params.num_boids, || {
        // Use the world size for boid positioning
        let x = rng.gen_range((-WORLD_SIZE / 2.0)..(WORLD_SIZE / 2.0));
        let y = rng.gen_range((-WORLD_SIZE / 2.0)..(WORLD_SIZE / 2.0));
        Boid::new(x, y)
    });
    
    // Update max speed for all boids
    for boid in &mut model.boids {
        boid.max_speed = model.params.max_speed;
    }
}

// Update boid positions and behaviors
fn update_boids(model: &mut Model) {
    // Only use spatial grid if enabled
    if model.params.enable_spatial_grid {
        update_boids_with_spatial_grid(model);
    } else {
        update_boids_without_spatial_grid(model);
    }
}

// Update boids using spatial grid for optimization
fn update_boids_with_spatial_grid(model: &mut Model) {
    // Ensure the spatial grid has appropriate cell size
    let max_radius = f32::max(
        model.params.separation_radius,
        f32::max(model.params.alignment_radius, model.params.cohesion_radius)
    );
    
    // Apply the cell size factor
    let cell_size = max_radius * model.params.cell_size_factor;
    
    // Recreate grid if perception radii have changed significantly
    if (cell_size - model.spatial_grid.cell_size).abs() > 5.0 {
        model.spatial_grid = SpatialGrid::new(cell_size, WORLD_SIZE);
    }
    
    // Clear the spatial grid
    model.spatial_grid.clear();
    
    // Add boids to the spatial grid
    for (i, boid) in model.boids.iter().enumerate() {
        model.spatial_grid.insert(i, boid.position, WORLD_SIZE);
    }
    
    // Pre-calculate weights to avoid multiplication in the inner loop
    let separation_weight = model.params.separation_weight;
    let alignment_weight = model.params.alignment_weight;
    let cohesion_weight = model.params.cohesion_weight;
    
    // Extract positions and velocities for the spatial grid's calculations
    let boid_positions: Vec<Point2> = model.boids.iter().map(|boid| boid.position).collect();
    let boid_velocities: Vec<Vec2> = model.boids.iter().map(|boid| boid.velocity).collect();
    
    // Pre-calculate squared radii
    let sep_radius_sq = model.params.separation_radius * model.params.separation_radius;
    let align_radius_sq = model.params.alignment_radius * model.params.alignment_radius;
    let cohesion_radius_sq = model.params.cohesion_radius * model.params.cohesion_radius;
    
    // Choose between parallel and sequential processing based on the setting
    if model.params.enable_parallel {
        // For parallel processing, we need to pre-compute all neighbor data
        let mut neighbor_data = Vec::with_capacity(model.boids.len());
        
        // First pass: gather all neighbor data
        for (_i, boid) in model.boids.iter().enumerate() {
            let nearby_with_distances = model.spatial_grid.get_nearby_with_distances(
                boid.position, 
                &boid_positions, 
                WORLD_SIZE
            );
            
            // Clone the data to avoid borrowing issues
            let neighbors: Vec<_> = nearby_with_distances.iter().map(|&n| n).collect();
            neighbor_data.push(neighbors);
        }
        
        // Second pass: calculate and apply forces in parallel
        // Use par_chunks_mut instead of par_iter_mut.enumerate() to reduce synchronization overhead
        // This processes boids in chunks, reducing the number of parallel tasks and synchronization points
        let chunk_size = std::cmp::max(model.boids.len() / rayon::current_num_threads(), 1);
        
        // Update debug info with chunk size if debug is enabled
        if model.params.show_debug {
            model.debug_info.chunk_size = chunk_size;
        }
        
        model.boids.par_chunks_mut(chunk_size).enumerate().for_each(|(chunk_idx, boid_chunk)| {
            // Process each boid in the chunk sequentially
            for (i_in_chunk, boid) in boid_chunk.iter_mut().enumerate() {
                let i = chunk_idx * chunk_size + i_in_chunk;
                if i >= neighbor_data.len() {
                    break; // Safety check for the last chunk which might be smaller
                }
                
                let neighbors = &neighbor_data[i];
                
                // Calculate forces
                let mut separation = Vec2::ZERO;
                let mut alignment = Vec2::ZERO;
                let mut cohesion = Vec2::ZERO;
                let mut sep_count = 0;
                let mut align_count = 0;
                let mut cohesion_count = 0;
                
                // Process all neighbors in a single pass
                for &neighbor in neighbors {
                    let d_squared = neighbor.distance_squared;
                    let other_idx = neighbor.index;
                    
                    // Separation
                    if d_squared < sep_radius_sq {
                        // Calculate vector pointing away from neighbor
                        let dx = boid.position.x - boid_positions[other_idx].x;
                        let dy = boid.position.y - boid_positions[other_idx].y;
                        
                        // Only calculate actual distance if needed for weighting
                        let d = d_squared.sqrt();
                        
                        // Weight by distance (closer boids have more influence)
                        separation.x += (dx / d) / d;
                        separation.y += (dy / d) / d;
                        sep_count += 1;
                    }
                    
                    // Alignment
                    if d_squared < align_radius_sq {
                        alignment += boid_velocities[other_idx];
                        align_count += 1;
                    }
                    
                    // Cohesion
                    if d_squared < cohesion_radius_sq {
                        cohesion.x += boid_positions[other_idx].x;
                        cohesion.y += boid_positions[other_idx].y;
                        cohesion_count += 1;
                    }
                }
                
                // Process separation
                if sep_count > 0 {
                    separation /= sep_count as f32;
                    
                    let separation_length_squared = separation.length_squared();
                    if separation_length_squared > 0.0 {
                        // Implement Reynolds: Steering = Desired - Velocity
                        let separation_length = separation_length_squared.sqrt();
                        let desired = separation * (boid.max_speed / separation_length);
                        
                        separation = desired - boid.velocity;
                        
                        // Limit force
                        let force_squared = separation.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            separation *= boid.max_force / force_length;
                        }
                    }
                }
                
                // Process alignment
                if align_count > 0 {
                    alignment /= align_count as f32;
                    
                    let alignment_length_squared = alignment.length_squared();
                    if alignment_length_squared > 0.0 {
                        // Implement Reynolds: Steering = Desired - Velocity
                        let alignment_length = alignment_length_squared.sqrt();
                        let desired = alignment * (boid.max_speed / alignment_length);
                        
                        alignment = desired - boid.velocity;
                        
                        // Limit force
                        let force_squared = alignment.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            alignment *= boid.max_force / force_length;
                        }
                    }
                }
                
                // Process cohesion
                if cohesion_count > 0 {
                    cohesion /= cohesion_count as f32;
                    
                    // Create desired velocity towards target
                    let desired = cohesion - Vec2::new(boid.position.x, boid.position.y);
                    
                    let desired_length_squared = desired.length_squared();
                    if desired_length_squared > 0.0 {
                        // Scale to maximum speed (only normalize if needed)
                        let desired_length = desired_length_squared.sqrt();
                        let desired_normalized = desired * (boid.max_speed / desired_length);
                        
                        // Implement Reynolds: Steering = Desired - Velocity
                        let mut steering = desired_normalized - boid.velocity;
                        
                        // Limit force
                        let force_squared = steering.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            steering *= boid.max_force / force_length;
                        }
                        
                        cohesion = steering;
                    }
                }
                
                // Combine forces with weights
                let mut combined_force = Vec2::ZERO;
                combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
                combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
                
                // Apply the calculated acceleration
                boid.apply_force(combined_force);
                
                // Update position
                boid.update();
                
                // Wrap around edges
                boid.wrap_edges(WORLD_SIZE);
            }
        });
    } else {
        // Sequential processing for better cache locality
        for i in 0..model.boids.len() {
            let boid_position = model.boids[i].position;
            
            // Get nearby boids with pre-computed distances
            let nearby_with_distances = model.spatial_grid.get_nearby_with_distances(boid_position, &boid_positions, WORLD_SIZE);
            
            // Calculate forces
            let mut separation = Vec2::ZERO;
            let mut alignment = Vec2::ZERO;
            let mut cohesion = Vec2::ZERO;
            let mut sep_count = 0;
            let mut align_count = 0;
            let mut cohesion_count = 0;
            
            // Process all neighbors in a single pass
            for neighbor in nearby_with_distances {
                let other_idx = neighbor.index;
                let d_squared = neighbor.distance_squared;
                
                // Separation
                if d_squared < sep_radius_sq {
                    // Calculate vector pointing away from neighbor
                    let dx = boid_position.x - boid_positions[other_idx].x;
                    let dy = boid_position.y - boid_positions[other_idx].y;
                    
                    // Only calculate actual distance if needed for weighting
                    let d = d_squared.sqrt();
                    
                    // Weight by distance (closer boids have more influence)
                    separation.x += (dx / d) / d;
                    separation.y += (dy / d) / d;
                    sep_count += 1;
                }
                
                // Alignment
                if d_squared < align_radius_sq {
                    alignment += boid_velocities[other_idx];
                    align_count += 1;
                }
                
                // Cohesion
                if d_squared < cohesion_radius_sq {
                    cohesion.x += boid_positions[other_idx].x;
                    cohesion.y += boid_positions[other_idx].y;
                    cohesion_count += 1;
                }
            }
            
            // Process separation
            if sep_count > 0 {
                separation /= sep_count as f32;
                
                let separation_length_squared = separation.length_squared();
                if separation_length_squared > 0.0 {
                    // Implement Reynolds: Steering = Desired - Velocity
                    let separation_length = separation_length_squared.sqrt();
                    let desired = separation * (model.boids[i].max_speed / separation_length);
                    
                    separation = desired - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = separation.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        separation *= model.boids[i].max_force / force_length;
                    }
                }
            }
            
            // Process alignment
            if align_count > 0 {
                alignment /= align_count as f32;
                
                let alignment_length_squared = alignment.length_squared();
                if alignment_length_squared > 0.0 {
                    // Implement Reynolds: Steering = Desired - Velocity
                    let alignment_length = alignment_length_squared.sqrt();
                    let desired = alignment * (model.boids[i].max_speed / alignment_length);
                    
                    alignment = desired - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = alignment.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        alignment *= model.boids[i].max_force / force_length;
                    }
                }
            }
            
            // Process cohesion
            if cohesion_count > 0 {
                cohesion /= cohesion_count as f32;
                
                // Create desired velocity towards target
                let desired = cohesion - Vec2::new(boid_position.x, boid_position.y);
                
                let desired_length_squared = desired.length_squared();
                if desired_length_squared > 0.0 {
                    // Scale to maximum speed (only normalize if needed)
                    let desired_length = desired_length_squared.sqrt();
                    let desired_normalized = desired * (model.boids[i].max_speed / desired_length);
                    
                    // Implement Reynolds: Steering = Desired - Velocity
                    let mut steering = desired_normalized - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = steering.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        steering *= model.boids[i].max_force / force_length;
                    }
                    
                    cohesion = steering;
                }
            }
            
            // Combine forces with weights
            let mut combined_force = Vec2::ZERO;
            combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
            combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
            
            // Apply the calculated acceleration
            model.boids[i].apply_force(combined_force);
            
            // Update position
            model.boids[i].update();
            
            // Wrap around edges
            model.boids[i].wrap_edges(WORLD_SIZE);
        }
    }
}

// Update boids without spatial grid (original O(n²) approach)
fn update_boids_without_spatial_grid(model: &mut Model) {
    // Create a copy of boids for the calculations
    let boids_clone = model.boids.clone();
    
    // Pre-calculate weights to avoid multiplication in the inner loop
    let separation_weight = model.params.separation_weight;
    let alignment_weight = model.params.alignment_weight;
    let cohesion_weight = model.params.cohesion_weight;
    
    // Use parallel processing if enabled
    if model.params.enable_parallel {
        // Calculate optimal chunk size based on available threads
        let chunk_size = std::cmp::max(model.boids.len() / rayon::current_num_threads(), 1);
        
        // Update debug info with chunk size if debug is enabled
        if model.params.show_debug {
            model.debug_info.chunk_size = chunk_size;
        }
        
        // Process boids in parallel chunks to reduce synchronization overhead
        model.boids.par_chunks_mut(chunk_size).for_each(|boid_chunk| {
            for boid in boid_chunk {
                // Calculate forces
                let separation = boid.separation_original(&boids_clone, model.params.separation_radius, model.params.enable_squared_distance);
                let alignment = boid.alignment_original(&boids_clone, model.params.alignment_radius, model.params.enable_squared_distance);
                let cohesion = boid.cohesion_original(&boids_clone, model.params.cohesion_radius, model.params.enable_squared_distance);
                
                // Combine forces with weights (avoid creating intermediate vectors)
                let mut combined_force = Vec2::ZERO;
                combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
                combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
                
                // Apply the calculated acceleration
                boid.apply_force(combined_force);
                
                // Update position
                boid.update();
                
                // Wrap around edges
                boid.wrap_edges(WORLD_SIZE);
            }
        });
    } else {
        // Sequential processing for when parallel is disabled
        for boid in &mut model.boids {
            // Calculate forces
            let separation = boid.separation_original(&boids_clone, model.params.separation_radius, model.params.enable_squared_distance);
            let alignment = boid.alignment_original(&boids_clone, model.params.alignment_radius, model.params.enable_squared_distance);
            let cohesion = boid.cohesion_original(&boids_clone, model.params.cohesion_radius, model.params.enable_squared_distance);
            
            // Combine forces with weights (avoid creating intermediate vectors)
            let mut combined_force = Vec2::ZERO;
            combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
            combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
            
            // Apply the calculated acceleration
            boid.apply_force(combined_force);
            
            // Update position
            boid.update();
            
            // Wrap around edges
            boid.wrap_edges(WORLD_SIZE);
        }
    }
}

// Render the model
pub fn view(app: &App, model: &Model, frame: Frame) {
    // Skip rendering if not needed (when paused and nothing has changed)
    let render_needed = unsafe { *model.render_needed.get() };
    if !render_needed {
        // Only draw the UI
        model.egui.draw_to_frame(&frame).unwrap();
        return;
    }
    
    // Begin drawing
    let draw = app.draw();
    
    // Clear the background
    draw.background().color(BLACK);
    
    // Get the window rectangle
    let window_rect = app.window_rect();
    
    // Draw world boundary to show the simulation limits
    let world_top_left = model.camera.world_to_screen(vec2(-WORLD_SIZE/2.0, -WORLD_SIZE/2.0), window_rect);
    let world_bottom_right = model.camera.world_to_screen(vec2(WORLD_SIZE/2.0, WORLD_SIZE/2.0), window_rect);
    
    let world_rect = Rect::from_corners(
        pt2(world_top_left.x, world_top_left.y),
        pt2(world_bottom_right.x, world_bottom_right.y)
    );
    
    draw.rect()
        .xy(world_rect.xy())
        .wh(world_rect.wh())
        .no_fill()
        .stroke_weight(1.0)
        .stroke(rgba(0.3, 0.3, 0.3, 1.0));
    
    // Calculate the visible area in world space for culling
    let visible_area = Rect::from_corners(
        pt2(
            model.camera.screen_to_world(pt2(window_rect.left(), window_rect.bottom()), window_rect).x,
            model.camera.screen_to_world(pt2(window_rect.left(), window_rect.bottom()), window_rect).y
        ),
        pt2(
            model.camera.screen_to_world(pt2(window_rect.right(), window_rect.top()), window_rect).x,
            model.camera.screen_to_world(pt2(window_rect.right(), window_rect.top()), window_rect).y
        )
    );
    
    // Add a margin to the visible area (scaled by zoom level)
    let margin = crate::BOID_SIZE * 2.0 / model.camera.zoom;
    let visible_area_with_margin = Rect::from_corners(
        pt2(visible_area.left() - margin, visible_area.bottom() - margin),
        pt2(visible_area.right() + margin, visible_area.top() + margin)
    );
    
    // Use cached visible boids if available and simulation is paused
    let visible_boids_indices = if model.params.pause_simulation {
        unsafe {
            if let Some(cached_indices) = &*model.cached_visible_boids.get() {
                cached_indices.clone()
            } else {
                // Filter boids that are in the visible area
                let indices: Vec<usize> = model.boids.iter()
                    .enumerate()
                    .filter(|(_, boid)| {
                        let pos = Vec2::new(boid.position.x, boid.position.y);
                        visible_area_with_margin.contains(pos)
                    })
                    .map(|(i, _)| i)
                    .collect();
                
                // Cache the indices
                *model.cached_visible_boids.get() = Some(indices.clone());
                
                indices
            }
        }
    } else {
        // When not paused, always recalculate visible boids
        // Use interpolated positions for culling if interpolation is enabled
        if model.params.enable_interpolation {
            model.boids.iter()
                .enumerate()
                .filter(|(_, boid)| {
                    let interpolated_pos = boid.get_interpolated_position(model.interpolation_alpha);
                    let pos = Vec2::new(interpolated_pos.x, interpolated_pos.y);
                    visible_area_with_margin.contains(pos)
                })
                .map(|(i, _)| i)
                .collect()
        } else {
            model.boids.iter()
                .enumerate()
                .filter(|(_, boid)| {
                    let pos = Vec2::new(boid.position.x, boid.position.y);
                    visible_area_with_margin.contains(pos)
                })
                .map(|(i, _)| i)
                .collect()
        }
    };
    
    // Track visible boid count for debug info
    if model.params.show_debug {
        let mut visible_boids_count = model.debug_info.visible_boids.lock().unwrap();
        *visible_boids_count = visible_boids_indices.len();
    }
    
    // Draw each visible boid with interpolation
    for &i in &visible_boids_indices {
        model.boids[i].draw(&draw, &model.camera, window_rect, model.interpolation_alpha);
    }
    
    // Draw debug visualization if enabled
    if model.params.show_debug {
        // Draw perception radius for the first boid if it's visible
        if !model.boids.is_empty() {
            let first_boid = &model.boids[0];
            
            // Get interpolated position for debug visualization
            let interpolated_pos = if model.params.enable_interpolation {
                first_boid.get_interpolated_position(model.interpolation_alpha)
            } else {
                first_boid.position
            };
            
            if visible_area_with_margin.contains(Vec2::new(interpolated_pos.x, interpolated_pos.y)) {
                let screen_pos = model.camera.world_to_screen(Vec2::new(interpolated_pos.x, interpolated_pos.y), window_rect);
                
                // Scale radii based on zoom level
                let sep_radius = model.params.separation_radius * model.camera.zoom;
                let align_radius = model.params.alignment_radius * model.camera.zoom;
                let cohesion_radius = model.params.cohesion_radius * model.camera.zoom;
                
                // Separation radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(sep_radius)
                    .no_fill()
                    .stroke(RED)
                    .stroke_weight(1.0);
                
                // Alignment radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(align_radius)
                    .no_fill()
                    .stroke(GREEN)
                    .stroke_weight(1.0);
                
                // Cohesion radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(cohesion_radius)
                    .no_fill()
                    .stroke(BLUE)
                    .stroke_weight(1.0);
                
                // Get interpolated velocity for debug visualization
                let interpolated_vel = if model.params.enable_interpolation {
                    first_boid.get_interpolated_velocity(model.interpolation_alpha)
                } else {
                    first_boid.velocity
                };
                
                // Velocity vector
                draw.arrow()
                    .start(pt2(screen_pos.x, screen_pos.y))
                    .end(pt2(
                        screen_pos.x + interpolated_vel.x * 5.0 * model.camera.zoom,
                        screen_pos.y + interpolated_vel.y * 5.0 * model.camera.zoom
                    ))
                    .color(YELLOW)
                    .stroke_weight(2.0);
            }
        }
        
        // Draw debug info
        ui::draw_debug_info(&draw, &model.debug_info, window_rect, model.boids.len(), model.camera.zoom, WORLD_SIZE);
    }
    
    // Finish drawing
    draw.to_frame(app, &frame).unwrap();
    
    // If simulation is paused, mark rendering as complete
    if model.params.pause_simulation {
        unsafe { *model.render_needed.get() = false; }
    }
    
    // Draw the egui UI
    model.egui.draw_to_frame(&frame).unwrap();
}

// Mouse moved event handler
pub fn mouse_moved(_app: &App, model: &mut Model, pos: Point2) {
    let new_pos = Vec2::new(pos.x, pos.y);
    
    // Update camera drag if we're dragging
    if model.camera.is_dragging {
        model.camera.drag(new_pos);
        // Clear the cached visible boids and force re-render when panning
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
    }
    
    // Always update the stored mouse position
    model.mouse_position = new_pos;
}

// Mouse pressed event handler
pub fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        // Check if the click is on the UI before starting camera drag
        if !model.egui.ctx().is_pointer_over_area() {
            model.camera.start_drag(model.mouse_position);
        }
    }
}

// Mouse released event handler
pub fn mouse_released(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        model.camera.end_drag();
    }
}

// Mouse wheel event handler for zooming
pub fn mouse_wheel(_app: &App, model: &mut Model, delta: MouseScrollDelta, _phase: TouchPhase) {
    match delta {
        MouseScrollDelta::LineDelta(x, y) => {
            // Handle trackpad pinch gestures and mouse wheel
            let window_rect = _app.window_rect();
            model.camera.zoom(vec2(x, y), model.mouse_position, window_rect);
        },
        MouseScrollDelta::PixelDelta(pos) => {
            // Handle pixel delta (less common)
            let window_rect = _app.window_rect();
            model.camera.zoom(vec2(pos.x as f32, pos.y as f32) * 0.01, model.mouse_position, window_rect);
        },
    }
    
    // Clear the cached visible boids and force re-render when zooming
    unsafe { *model.cached_visible_boids.get() = None; }
    unsafe { *model.render_needed.get() = true; }
}

// Handle raw window events for egui and camera dragging
pub fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Pass events to egui
    model.egui.handle_raw_event(event);
    
    // Force re-render when UI is interacted with
    if let nannou::winit::event::WindowEvent::MouseInput { .. } = event {
        unsafe { *model.render_needed.get() = true; }
    }
} 