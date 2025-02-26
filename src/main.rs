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
 */

use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use rand::Rng;
use std::time::Duration;

// Only keep the boid size as a constant
const BOID_SIZE: f32 = 6.0;

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
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            num_boids: 150,
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            separation_radius: 25.0,
            alignment_radius: 50.0,
            cohesion_radius: 50.0,
            max_speed: 4.0,
            show_debug: false,
            pause_simulation: false,
        }
    }
}

// Debug information to display
struct DebugInfo {
    fps: f32,
    frame_time: Duration,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::from_secs(0),
        }
    }
}

// Main model for the application
struct Model {
    boids: Vec<Boid>,
    params: SimulationParams,
    egui: Egui,
    debug_info: DebugInfo,
    // Add window dimensions to the model
    window_width: f32,
    window_height: f32,
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
    
    // Calculate window size based on monitor size (80% of monitor size)
    let window_width = monitor_size.width as f32 * 0.8;
    let window_height = monitor_size.height as f32 * 0.8;
    
    // Create the main window with dynamic size
    let window_id = app
        .new_window()
        .title("Boid Flocking Simulation")
        .size(window_width as u32, window_height as u32)
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    
    // Get the window
    let window = app.window(window_id).unwrap();
    
    // Create the UI
    let egui = Egui::from_window(&window);
    
    // Create simulation parameters
    let params = SimulationParams::default();
    
    // Create boids
    let mut boids = Vec::with_capacity(params.num_boids);
    let mut rng = rand::thread_rng();
    
    // Use the window dimensions for boid positioning
    for _ in 0..params.num_boids {
        let x = rng.gen_range((-window_width / 2.0)..(window_width / 2.0));
        let y = rng.gen_range((-window_height / 2.0)..(window_height / 2.0));
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
        window_width,
        window_height,
    }
}

fn update(app: &App, model: &mut Model, update: Update) {
    // Update debug info
    model.debug_info.fps = app.fps();
    model.debug_info.frame_time = update.since_last;
    
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
                    ui.add(egui::Slider::new(&mut model.params.num_boids, 10..=500).text("Number of Boids"));
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
        
        // Resize the boids vector if needed
        model.boids.resize_with(model.params.num_boids, || {
            // Use the window dimensions for boid positioning
            let x = rng.gen_range((-model.window_width / 2.0)..(model.window_width / 2.0));
            let y = rng.gen_range((-model.window_height / 2.0)..(model.window_height / 2.0));
            Boid::new(x, y)
        });
        
        // Update max speed for all boids
        for boid in &mut model.boids {
            boid.max_speed = model.params.max_speed;
        }
    }
    
    // Only update boids if simulation is not paused
    if !model.params.pause_simulation {
        // Update each boid
        let boids_clone = model.boids.clone(); // Clone to avoid borrow checker issues
        
        for boid in &mut model.boids {
            boid.flock(&boids_clone, &model.params);
            boid.update();
            // Use the window dimensions for wrapping
            boid.wrap_edges(model.window_width, model.window_height);
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
        draw.text(&format!("FPS: {:.1}", model.debug_info.fps))
            .x_y((-model.window_width / 2.0) + 100.0, (model.window_height / 2.0) - 20.0)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Frame time: {:.2} ms", model.debug_info.frame_time.as_secs_f64() * 1000.0))
            .x_y((-model.window_width / 2.0) + 100.0, (model.window_height / 2.0) - 40.0)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Boids: {}", model.boids.len()))
            .x_y((-model.window_width / 2.0) + 100.0, (model.window_height / 2.0) - 60.0)
            .color(WHITE)
            .font_size(14);
            
        // Add window size to debug info
        draw.text(&format!("Window: {:.0}x{:.0}", model.window_width, model.window_height))
            .x_y((-model.window_width / 2.0) + 100.0, (model.window_height / 2.0) - 80.0)
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
