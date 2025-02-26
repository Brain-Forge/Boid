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
 * Optimized for Apple Silicon (M1/M2) with memory efficiency and adaptive screen sizing.
 */

use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use rand::Rng;
use std::time::Duration;

// Constants
const BOID_SIZE: f32 = 6.0;
const DEFAULT_NUM_BOIDS: usize = 120; // Reduced default for better performance
const MAX_NUM_BOIDS: usize = 300;     // Capped maximum for memory efficiency

// Boid struct representing an individual agent in the simulation
#[derive(Clone)]
struct Boid {
    position: Point2,
    velocity: Vec2,
    acceleration: Vec2,
    max_speed: f32,
    max_force: f32,
    color: Rgb<u8>,
}

impl Boid {
    fn new(x: f32, y: f32) -> Self {
        let mut rng = rand::thread_rng();
        
        // Random initial velocity
        let vx = rng.gen_range(-1.0..1.0);
        let vy = rng.gen_range(-1.0..1.0);
        let velocity = vec2(vx, vy).normalize() * 2.0;
        
        Self {
            position: pt2(x, y),
            velocity,
            acceleration: Vec2::ZERO,
            max_speed: 4.0,
            max_force: 0.1,
            color: rgb(220, 220, 220),
        }
    }
    
    // Apply a force to the boid
    fn apply_force(&mut self, force: Vec2) {
        self.acceleration += force;
    }
    
    // Update the boid's position based on its velocity and acceleration
    fn update(&mut self) {
        // Update velocity
        self.velocity += self.acceleration;
        
        // Limit speed
        if self.velocity.length() > self.max_speed {
            self.velocity = self.velocity.normalize() * self.max_speed;
        }
        
        // Update position
        self.position += self.velocity;
        
        // Reset acceleration
        self.acceleration = Vec2::ZERO;
    }
    
    // Wrap the boid around the screen edges
    fn wrap_edges(&mut self, width: f32, height: f32) {
        let half_width = width / 2.0;
        let half_height = height / 2.0;
        
        if self.position.x > half_width {
            self.position.x = -half_width;
        } else if self.position.x < -half_width {
            self.position.x = half_width;
        }
        
        if self.position.y > half_height {
            self.position.y = -half_height;
        } else if self.position.y < -half_height {
            self.position.y = half_height;
        }
    }
    
    // Calculate separation force (avoid crowding neighbors)
    fn separation(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for other in boids {
            let d = self.position.distance(other.position);
            
            // If this is not the same boid and it's within perception radius
            if d > 0.0 && d < perception_radius {
                // Calculate vector pointing away from neighbor
                let mut diff = self.position - other.position;
                diff = diff.normalize() / d;  // Weight by distance
                steering += diff;
                count += 1;
            }
        }
        
        if count > 0 {
            steering /= count as f32;
            
            if steering.length() > 0.0 {
                // Implement Reynolds: Steering = Desired - Velocity
                steering = steering.normalize() * self.max_speed - self.velocity;
                
                if steering.length() > self.max_force {
                    steering = steering.normalize() * self.max_force;
                }
            }
        }
        
        steering
    }
    
    // Calculate alignment force (steer towards average heading of neighbors)
    fn alignment(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for other in boids {
            let d = self.position.distance(other.position);
            
            // If this is not the same boid and it's within perception radius
            if d > 0.0 && d < perception_radius {
                steering += other.velocity;
                count += 1;
            }
        }
        
        if count > 0 {
            steering /= count as f32;
            
            // Implement Reynolds: Steering = Desired - Velocity
            steering = steering.normalize() * self.max_speed - self.velocity;
            
            if steering.length() > self.max_force {
                steering = steering.normalize() * self.max_force;
            }
        }
        
        steering
    }
    
    // Calculate cohesion force (steer towards average position of neighbors)
    fn cohesion(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for other in boids {
            let d = self.position.distance(other.position);
            
            // If this is not the same boid and it's within perception radius
            if d > 0.0 && d < perception_radius {
                steering += Vec2::new(other.position.x, other.position.y);
                count += 1;
            }
        }
        
        if count > 0 {
            steering /= count as f32;
            
            // Create desired velocity towards target
            let desired = steering - Vec2::new(self.position.x, self.position.y);
            
            if desired.length() > 0.0 {
                // Scale to maximum speed
                let desired = desired.normalize() * self.max_speed;
                
                // Implement Reynolds: Steering = Desired - Velocity
                let mut steering = desired - self.velocity;
                
                if steering.length() > self.max_force {
                    steering = steering.normalize() * self.max_force;
                }
                
                return steering;
            }
        }
        
        Vec2::ZERO
    }
    
    // Apply all flocking behaviors
    fn flock(&mut self, boids: &[Boid], params: &SimulationParams) {
        let separation = self.separation(boids, params.separation_radius) * params.separation_weight;
        let alignment = self.alignment(boids, params.alignment_radius) * params.alignment_weight;
        let cohesion = self.cohesion(boids, params.cohesion_radius) * params.cohesion_weight;
        
        self.apply_force(separation);
        self.apply_force(alignment);
        self.apply_force(cohesion);
    }
    
    // Draw the boid
    fn draw(&self, draw: &Draw) {
        // Calculate the angle of the velocity
        let angle = self.velocity.y.atan2(self.velocity.x);
        
        // Create a triangle shape for the boid
        let points = [
            pt2(BOID_SIZE, 0.0),
            pt2(-BOID_SIZE, BOID_SIZE / 2.0),
            pt2(-BOID_SIZE, -BOID_SIZE / 2.0),
        ];
        
        draw.polygon()
            .color(self.color)
            .points(points)
            .xy(self.position)
            .rotate(angle);
    }
}

// Parameters for the simulation that can be adjusted via UI
struct SimulationParams {
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
    spatial_optimization: bool,
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            num_boids: DEFAULT_NUM_BOIDS,
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            separation_radius: 25.0,
            alignment_radius: 50.0,
            cohesion_radius: 50.0,
            max_speed: 4.0,
            show_debug: false,
            pause_simulation: false,
            spatial_optimization: true,
        }
    }
}

// Debug information to display
struct DebugInfo {
    fps: f32,
    frame_time: Duration,
    screen_width: f32,
    screen_height: f32,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::from_secs(0),
            screen_width: 0.0,
            screen_height: 0.0,
        }
    }
}

// Main model for the application
struct Model {
    boids: Vec<Boid>,
    params: SimulationParams,
    egui: Egui,
    debug_info: DebugInfo,
    window_rect: Rect,
}

fn main() {
    nannou::app(model)
        .update(update)
        .run();
}

fn model(app: &App) -> Model {
    // Get the primary monitor's dimensions
    let monitor = app.primary_monitor().expect("Failed to get primary monitor");
    let monitor_size = monitor.size();
    
    // Calculate window size (90% of monitor size)
    let window_width = (monitor_size.width as f32 * 0.9) as u32;
    let window_height = (monitor_size.height as f32 * 0.9) as u32;
    
    // Create the main window
    let window_id = app
        .new_window()
        .title("Boid Flocking Simulation")
        .size(window_width, window_height)
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    
    // Get the window
    let window = app.window(window_id).unwrap();
    let window_rect = app.window_rect();
    
    // Create the UI
    let egui = Egui::from_window(&window);
    
    // Create simulation parameters
    let params = SimulationParams::default();
    
    // Create boids
    let mut boids = Vec::with_capacity(params.num_boids);
    let mut rng = rand::thread_rng();
    
    let half_width = window_rect.w() / 2.0;
    let half_height = window_rect.h() / 2.0;
    
    for _ in 0..params.num_boids {
        let x = rng.gen_range(-half_width..half_width);
        let y = rng.gen_range(-half_height..half_height);
        boids.push(Boid::new(x, y));
    }
    
    // Update max speed for all boids
    for boid in &mut boids {
        boid.max_speed = params.max_speed;
    }
    
    // Create debug info
    let mut debug_info = DebugInfo::default();
    debug_info.screen_width = window_rect.w();
    debug_info.screen_height = window_rect.h();
    
    Model {
        boids,
        params,
        egui,
        debug_info,
        window_rect,
    }
}

fn update(app: &App, model: &mut Model, update: Update) {
    // Update window rect in case of resize
    model.window_rect = app.window_rect();
    
    // Update debug info
    model.debug_info.fps = app.fps();
    model.debug_info.frame_time = update.since_last;
    model.debug_info.screen_width = model.window_rect.w();
    model.debug_info.screen_height = model.window_rect.h();
    
    // Track if we need to reset boids
    let mut should_reset_boids = false;
    let mut num_boids_changed = false;
    let old_num_boids = model.params.num_boids;
    
    // Update UI
    {
        let ctx = model.egui.begin_frame();
        
        egui::Window::new("Simulation Controls")
            .default_pos([10.0, 10.0])
            .show(&ctx, |ui| {
                ui.collapsing("Boid Parameters", |ui| {
                    ui.add(egui::Slider::new(&mut model.params.num_boids, 10..=MAX_NUM_BOIDS).text("Number of Boids"));
                    if model.params.num_boids != old_num_boids {
                        num_boids_changed = true;
                    }
                    
                    if ui.button("Reset Boids").clicked() {
                        should_reset_boids = true;
                    }
                    
                    ui.add(egui::Slider::new(&mut model.params.max_speed, 1.0..=10.0).text("Max Speed"));
                    
                    // Update max speed for all boids when changed
                    for boid in &mut model.boids {
                        boid.max_speed = model.params.max_speed;
                    }
                    
                    ui.checkbox(&mut model.params.spatial_optimization, "Spatial Optimization");
                    ui.label("Enables spatial partitioning for better performance with many boids");
                });
                
                ui.collapsing("Flocking Behavior", |ui| {
                    ui.add(egui::Slider::new(&mut model.params.separation_weight, 0.0..=3.0).text("Separation Weight"));
                    ui.add(egui::Slider::new(&mut model.params.alignment_weight, 0.0..=3.0).text("Alignment Weight"));
                    ui.add(egui::Slider::new(&mut model.params.cohesion_weight, 0.0..=3.0).text("Cohesion Weight"));
                    
                    ui.add(egui::Slider::new(&mut model.params.separation_radius, 10.0..=100.0).text("Separation Radius"));
                    ui.add(egui::Slider::new(&mut model.params.alignment_radius, 10.0..=100.0).text("Alignment Radius"));
                    ui.add(egui::Slider::new(&mut model.params.cohesion_radius, 10.0..=100.0).text("Cohesion Radius"));
                });
                
                ui.checkbox(&mut model.params.show_debug, "Show Debug Info");
                ui.checkbox(&mut model.params.pause_simulation, "Pause Simulation");
            });
    } // End of UI scope - ctx is dropped here
    
    // Handle reset boids outside of the UI closure
    if should_reset_boids || num_boids_changed {
        let mut rng = rand::thread_rng();
        
        let half_width = model.window_rect.w() / 2.0;
        let half_height = model.window_rect.h() / 2.0;
        
        // Resize the boids vector if needed
        model.boids.resize_with(model.params.num_boids, || {
            let x = rng.gen_range(-half_width..half_width);
            let y = rng.gen_range(-half_height..half_height);
            Boid::new(x, y)
        });
        
        // Update max speed for all boids
        for boid in &mut model.boids {
            boid.max_speed = model.params.max_speed;
        }
    }
    
    // Only update boids if simulation is not paused
    if !model.params.pause_simulation {
        // Optimization for M1: Use spatial partitioning for large numbers of boids
        if model.params.spatial_optimization && model.boids.len() > 50 {
            // Simple spatial partitioning - divide the screen into a grid
            let grid_size = model.params.cohesion_radius.max(model.params.alignment_radius).max(model.params.separation_radius);
            let width = model.window_rect.w();
            let height = model.window_rect.h();
            
            let cols = (width / grid_size).ceil() as usize;
            let rows = (height / grid_size).ceil() as usize;
            
            // Create a grid of boid indices
            let mut grid = vec![Vec::new(); cols * rows];
            
            // Assign boids to grid cells
            for (i, boid) in model.boids.iter().enumerate() {
                let col = ((boid.position.x + width / 2.0) / grid_size).floor() as usize;
                let row = ((boid.position.y + height / 2.0) / grid_size).floor() as usize;
                
                let col = col.min(cols - 1);
                let row = row.min(rows - 1);
                
                let cell_index = row * cols + col;
                if cell_index < grid.len() {
                    grid[cell_index].push(i);
                }
            }
            
            // Clone boids for reading while updating
            let boids_clone = model.boids.clone();
            
            // Update each boid using only nearby boids
            for (_i, boid) in model.boids.iter_mut().enumerate() {
                let col = ((boid.position.x + width / 2.0) / grid_size).floor() as usize;
                let row = ((boid.position.y + height / 2.0) / grid_size).floor() as usize;
                
                let col = col.min(cols - 1);
                let row = row.min(rows - 1);
                
                // Get nearby boids from surrounding cells
                let mut nearby_boids = Vec::new();
                
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let neighbor_col = col as isize + dx;
                        let neighbor_row = row as isize + dy;
                        
                        if neighbor_col >= 0 && neighbor_col < cols as isize && 
                           neighbor_row >= 0 && neighbor_row < rows as isize {
                            let cell_index = (neighbor_row as usize) * cols + (neighbor_col as usize);
                            
                            for &boid_index in &grid[cell_index] {
                                nearby_boids.push(boids_clone[boid_index].clone());
                            }
                        }
                    }
                }
                
                // Apply flocking behavior with only nearby boids
                boid.flock(&nearby_boids, &model.params);
                boid.update();
                boid.wrap_edges(width, height);
            }
        } else {
            // Standard update for small numbers of boids
            let boids_clone = model.boids.clone();
            
            for boid in &mut model.boids {
                boid.flock(&boids_clone, &model.params);
                boid.update();
                boid.wrap_edges(model.window_rect.w(), model.window_rect.h());
            }
        }
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    // Begin drawing
    let draw = app.draw();
    
    // Clear the background
    draw.background().color(BLACK);
    
    // Draw each boid
    for boid in &model.boids {
        boid.draw(&draw);
    }
    
    // Draw debug visualization if enabled
    if model.params.show_debug {
        // Draw perception radius for the first boid
        if !model.boids.is_empty() {
            let first_boid = &model.boids[0];
            
            // Separation radius
            draw.ellipse()
                .xy(first_boid.position)
                .radius(model.params.separation_radius)
                .no_fill()
                .stroke(RED)
                .stroke_weight(1.0);
            
            // Alignment radius
            draw.ellipse()
                .xy(first_boid.position)
                .radius(model.params.alignment_radius)
                .no_fill()
                .stroke(GREEN)
                .stroke_weight(1.0);
            
            // Cohesion radius
            draw.ellipse()
                .xy(first_boid.position)
                .radius(model.params.cohesion_radius)
                .no_fill()
                .stroke(BLUE)
                .stroke_weight(1.0);
            
            // Velocity vector
            draw.arrow()
                .start(first_boid.position)
                .end(first_boid.position + first_boid.velocity * 5.0)
                .color(YELLOW)
                .stroke_weight(2.0);
        }
        
        // Draw FPS and other debug info
        let text_x = (-model.window_rect.w() / 2.0) + 100.0;
        let text_y_start = (model.window_rect.h() / 2.0) - 20.0;
        let line_height = 20.0;
        
        draw.text(&format!("FPS: {:.1}", model.debug_info.fps))
            .x_y(text_x, text_y_start)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Frame time: {:.2} ms", model.debug_info.frame_time.as_secs_f64() * 1000.0))
            .x_y(text_x, text_y_start - line_height)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Boids: {}", model.boids.len()))
            .x_y(text_x, text_y_start - line_height * 2.0)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Screen: {:.0}x{:.0}", model.debug_info.screen_width, model.debug_info.screen_height))
            .x_y(text_x, text_y_start - line_height * 3.0)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Optimization: {}", if model.params.spatial_optimization { "On" } else { "Off" }))
            .x_y(text_x, text_y_start - line_height * 4.0)
            .color(WHITE)
            .font_size(14);
    }
    
    // Finish drawing
    draw.to_frame(app, &frame).unwrap();
    
    // Draw the egui UI
    model.egui.draw_to_frame(&frame).unwrap();
}

// Handle raw window events for egui
fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.egui.handle_raw_event(event);
}
