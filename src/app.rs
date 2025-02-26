/*
 * Application Module
 * 
 * This module defines the main application model and logic for the boid simulation.
 * It handles the initialization, update, and rendering of the simulation.
 */

use nannou::prelude::*;
use nannou::winit::event::{MouseButton, MouseScrollDelta, TouchPhase};
use nannou_egui::Egui;
use rand::Rng;
use rayon::prelude::*;
use std::cell::UnsafeCell;

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
    }
    
    // Handle reset boids
    if should_reset_boids || num_boids_changed {
        reset_boids(model);
        // Clear the caches when boids are reset
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
    }
    
    // Only update boids if simulation is not paused
    if !model.params.pause_simulation {
        update_boids(model);
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
    
    // Process each boid with spatial partitioning
    let boids = &model.boids;
    let params = &model.params;
    let spatial_grid = &model.spatial_grid;
    
    // Collect accelerations first to avoid borrow checker issues
    let mut accelerations = Vec::with_capacity(boids.len());
    
    // Choose between parallel and sequential processing based on the setting
    if model.params.enable_parallel {
        // Phase 1: Calculate accelerations in parallel
        accelerations = (0..boids.len()).into_par_iter().map(|i| {
            // Get nearby boids from the spatial grid
            let nearby_indices = spatial_grid.get_nearby_indices(boids[i].position, WORLD_SIZE);
            
            // Calculate forces
            let separation = boids[i].separation(boids, &nearby_indices, params.separation_radius) * params.separation_weight;
            let alignment = boids[i].alignment(boids, &nearby_indices, params.alignment_radius) * params.alignment_weight;
            let cohesion = boids[i].cohesion(boids, &nearby_indices, params.cohesion_radius) * params.cohesion_weight;
            
            // Return combined forces
            separation + alignment + cohesion
        }).collect();
    } else {
        // Sequential processing
        for i in 0..boids.len() {
            // Get nearby boids from the spatial grid
            let nearby_indices = spatial_grid.get_nearby_indices(boids[i].position, WORLD_SIZE);
            
            // Calculate forces
            let separation = boids[i].separation(boids, &nearby_indices, params.separation_radius) * params.separation_weight;
            let alignment = boids[i].alignment(boids, &nearby_indices, params.alignment_radius) * params.alignment_weight;
            let cohesion = boids[i].cohesion(boids, &nearby_indices, params.cohesion_radius) * params.cohesion_weight;
            
            // Store combined forces
            accelerations.push(separation + alignment + cohesion);
        }
    }
    
    // Phase 2: Apply accelerations and update positions
    for i in 0..model.boids.len() {
        let boid = &mut model.boids[i];
        
        // Apply the precalculated acceleration
        boid.apply_force(accelerations[i]);
        
        // Update position based on velocity and acceleration
        boid.update();
        boid.wrap_edges(WORLD_SIZE);
    }
}

// Update boids without spatial grid (original O(n²) approach)
fn update_boids_without_spatial_grid(model: &mut Model) {
    // Create a copy of boids for the calculations
    let boids_clone = model.boids.clone();
    
    // Update each boid
    for boid in &mut model.boids {
        boid.flock(&boids_clone, &model.params);
        boid.update();
        boid.wrap_edges(WORLD_SIZE);
    }
}

// Render the model
pub fn view(app: &App, model: &Model, frame: Frame) {
    // Skip rendering if not needed (when paused and nothing has changed)
    let render_needed = unsafe { *model.render_needed.get() };
    if model.params.pause_simulation && !render_needed {
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
        model.boids.iter()
            .enumerate()
            .filter(|(_, boid)| {
                let pos = Vec2::new(boid.position.x, boid.position.y);
                visible_area_with_margin.contains(pos)
            })
            .map(|(i, _)| i)
            .collect()
    };
    
    // Track visible boid count for debug info
    if model.params.show_debug {
        let mut visible_boids_count = model.debug_info.visible_boids.lock().unwrap();
        *visible_boids_count = visible_boids_indices.len();
    }
    
    // Draw each visible boid
    for &i in &visible_boids_indices {
        model.boids[i].draw(&draw, &model.camera, window_rect);
    }
    
    // Draw debug visualization if enabled
    if model.params.show_debug {
        // Draw perception radius for the first boid if it's visible
        if !model.boids.is_empty() {
            let first_boid = &model.boids[0];
            
            if visible_area_with_margin.contains(Vec2::new(first_boid.position.x, first_boid.position.y)) {
                let screen_pos = model.camera.world_to_screen(Vec2::new(first_boid.position.x, first_boid.position.y), window_rect);
                
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
                
                // Velocity vector
                draw.arrow()
                    .start(pt2(screen_pos.x, screen_pos.y))
                    .end(pt2(
                        screen_pos.x + first_boid.velocity.x * 5.0 * model.camera.zoom,
                        screen_pos.y + first_boid.velocity.y * 5.0 * model.camera.zoom
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