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
 * Features:
 * - Dynamic window sizing based on user's screen
 * - Camera controls for zooming and panning
 * - Large simulation space that extends beyond the visible area
 */

use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use rand::Rng;
use std::time::Duration;

// Only keep the boid size as a constant
const BOID_SIZE: f32 = 6.0;

// Define the simulation world size (much larger than the visible area)
const WORLD_SIZE: f32 = 5000.0;

// Camera struct to handle zooming and panning
struct Camera {
    position: Vec2,
    zoom: f32,
    drag_start: Option<Vec2>,
    min_zoom: f32,
    max_zoom: f32,
}

impl Camera {
    fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            zoom: 1.0,
            drag_start: None,
            min_zoom: 0.1,
            max_zoom: 5.0,
        }
    }

    // Convert a point from world space to screen space
    fn world_to_screen(&self, point: Vec2, window_rect: Rect) -> Vec2 {
        // Apply zoom and translation
        let zoomed = (point - self.position) * self.zoom;
        // Convert to screen coordinates
        zoomed + window_rect.xy()
    }

    // Convert a point from screen space to world space
    fn screen_to_world(&self, point: Vec2, window_rect: Rect) -> Vec2 {
        // Convert from screen coordinates
        let centered = point - window_rect.xy();
        // Apply inverse zoom and translation
        centered / self.zoom + self.position
    }

    // Handle mouse wheel events for zooming
    fn zoom(&mut self, scroll_delta: Vec2, cursor_position: Vec2, window_rect: Rect) {
        // Calculate zoom factor based on scroll amount
        let zoom_factor = 1.0 + scroll_delta.y * 0.1;
        
        // Calculate cursor position in world space before zoom
        let cursor_world_before = self.screen_to_world(cursor_position, window_rect);
        
        // Apply zoom, clamping to min/max values
        self.zoom = (self.zoom * zoom_factor).clamp(self.min_zoom, self.max_zoom);
        
        // Calculate cursor position in world space after zoom
        let cursor_world_after = self.screen_to_world(cursor_position, window_rect);
        
        // Adjust camera position to keep cursor over the same world point
        self.position += cursor_world_before - cursor_world_after;
    }

    // Start dragging the camera
    fn start_drag(&mut self, position: Vec2) {
        self.drag_start = Some(position);
    }

    // Update camera position while dragging
    fn drag(&mut self, position: Vec2) {
        if let Some(start) = self.drag_start {
            // Calculate drag delta and apply it to camera position
            let delta = position - start;
            self.position -= delta / self.zoom;
            self.drag_start = Some(position);
        }
    }

    // End dragging
    fn end_drag(&mut self) {
        self.drag_start = None;
    }
}

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
    
    // Wrap the boid around the world edges
    fn wrap_edges(&mut self, world_size: f32) {
        let half_size = world_size / 2.0;
        
        if self.position.x > half_size {
            self.position.x = -half_size;
        } else if self.position.x < -half_size {
            self.position.x = half_size;
        }
        
        if self.position.y > half_size {
            self.position.y = -half_size;
        } else if self.position.y < -half_size {
            self.position.y = half_size;
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
    fn draw(&self, draw: &Draw, camera: &Camera, window_rect: Rect) {
        // Convert boid position from world space to screen space
        let screen_pos = camera.world_to_screen(Vec2::new(self.position.x, self.position.y), window_rect);
        
        // Calculate the angle of the velocity
        let angle = self.velocity.y.atan2(self.velocity.x);
        
        // Scale the boid size based on zoom level
        let scaled_size = BOID_SIZE * camera.zoom;
        
        // Create a triangle shape for the boid
        let points = [
            pt2(scaled_size, 0.0),
            pt2(-scaled_size, scaled_size / 2.0),
            pt2(-scaled_size, -scaled_size / 2.0),
        ];
        
        draw.polygon()
            .color(self.color)
            .points(points)
            .xy(pt2(screen_pos.x, screen_pos.y))
            .rotate(angle);
    }
    
    // Check if the boid is visible in the current view
    fn is_visible(&self, camera: &Camera, window_rect: Rect) -> bool {
        let screen_pos = camera.world_to_screen(Vec2::new(self.position.x, self.position.y), window_rect);
        let scaled_size = BOID_SIZE * camera.zoom * 2.0; // Add some margin
        
        // Check if the boid is within the visible area
        screen_pos.x + scaled_size >= window_rect.left() &&
        screen_pos.x - scaled_size <= window_rect.right() &&
        screen_pos.y + scaled_size >= window_rect.bottom() &&
        screen_pos.y - scaled_size <= window_rect.top()
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
        }
    }
}

// Debug information to display
struct DebugInfo {
    fps: f32,
    frame_time: Duration,
    visible_boids: usize,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::from_secs(0),
            visible_boids: 0,
        }
    }
}

// Main model for the application
struct Model {
    boids: Vec<Boid>,
    params: SimulationParams,
    egui: Egui,
    debug_info: DebugInfo,
    camera: Camera,
    mouse_position: Vec2,
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
                    ui.add(egui::Slider::new(&mut model.params.num_boids, 10..=2000).text("Number of Boids"));
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
                
                ui.collapsing("Camera Controls", |ui| {
                    ui.label("Zoom: Use mouse wheel or trackpad pinch gesture");
                    ui.label("Pan: Click and drag or use trackpad with two fingers");
                    if ui.button("Reset Camera").clicked() {
                        model.camera.position = Vec2::ZERO;
                        model.camera.zoom = 1.0;
                    }
                    ui.label(format!("Zoom Level: {:.2}x", model.camera.zoom));
                    ui.label(format!("Camera Position: ({:.0}, {:.0})", model.camera.position.x, model.camera.position.y));
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
    
    // Only update boids if simulation is not paused
    if !model.params.pause_simulation {
        // Update each boid
        let boids_clone = model.boids.clone(); // Clone to avoid borrow checker issues
        
        for boid in &mut model.boids {
            boid.flock(&boids_clone, &model.params);
            boid.update();
            // Use the world size for wrapping
            boid.wrap_edges(WORLD_SIZE);
        }
    }
    
    // Count visible boids for debug info
    if model.params.show_debug {
        let window_rect = app.window_rect();
        model.debug_info.visible_boids = model.boids
            .iter()
            .filter(|boid| boid.is_visible(&model.camera, window_rect))
            .count();
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
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
    
    // Draw each boid (only if visible in the current view)
    for boid in &model.boids {
        if boid.is_visible(&model.camera, window_rect) {
            boid.draw(&draw, &model.camera, window_rect);
        }
    }
    
    // Draw debug visualization if enabled
    if model.params.show_debug {
        // Draw perception radius for the first boid if it's visible
        if !model.boids.is_empty() {
            let first_boid = &model.boids[0];
            
            if first_boid.is_visible(&model.camera, window_rect) {
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
        
        // Draw FPS and other debug info (fixed position in screen space)
        let text_x = -window_rect.w() / 2.0 + 10.0;
        let text_y = window_rect.h() / 2.0 - 20.0;
        let line_height = 20.0;
        
        draw.text(&format!("FPS: {:.1}", model.debug_info.fps))
            .x_y(text_x, text_y)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Frame time: {:.2} ms", model.debug_info.frame_time.as_secs_f64() * 1000.0))
            .x_y(text_x, text_y - line_height)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Total Boids: {}", model.boids.len()))
            .x_y(text_x, text_y - line_height * 2.0)
            .color(WHITE)
            .font_size(14);
        
        draw.text(&format!("Visible Boids: {}", model.debug_info.visible_boids))
            .x_y(text_x, text_y - line_height * 3.0)
            .color(WHITE)
            .font_size(14);
            
        draw.text(&format!("Zoom: {:.2}x", model.camera.zoom))
            .x_y(text_x, text_y - line_height * 4.0)
            .color(WHITE)
            .font_size(14);
            
        draw.text(&format!("World Size: {:.0}x{:.0}", WORLD_SIZE, WORLD_SIZE))
            .x_y(text_x, text_y - line_height * 5.0)
            .color(WHITE)
            .font_size(14);
    }
    
    // Finish drawing
    draw.to_frame(app, &frame).unwrap();
    
    // Draw the egui UI
    model.egui.draw_to_frame(&frame).unwrap();
}

// Mouse moved event handler
fn mouse_moved(_app: &App, model: &mut Model, pos: Point2) {
    model.mouse_position = Vec2::new(pos.x, pos.y);
}

// Mouse pressed event handler
fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        model.camera.start_drag(model.mouse_position);
    }
}

// Mouse released event handler
fn mouse_released(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        model.camera.end_drag();
    }
}

// Mouse wheel event handler for zooming
fn mouse_wheel(_app: &App, model: &mut Model, delta: MouseScrollDelta, _phase: TouchPhase) {
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
}

// Handle raw window events for egui and camera dragging
fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Pass events to egui first
    model.egui.handle_raw_event(event);
    
    // Handle mouse move events for camera dragging
    if let nannou::winit::event::WindowEvent::CursorMoved { position, .. } = event {
        if model.camera.drag_start.is_some() {
            let pos = vec2(position.x as f32, position.y as f32);
            model.camera.drag(pos);
        }
    }
}
