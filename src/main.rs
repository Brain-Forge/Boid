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
use rayon::prelude::*;
use std::sync::{Arc, Mutex};

// Only keep the boid size as a constant
const BOID_SIZE: f32 = 6.0;

// Define the simulation world size (much larger than the visible area)
const WORLD_SIZE: f32 = 5000.0;

// Spatial partitioning grid for efficient neighbor lookups
struct SpatialGrid {
    cell_size: f32,
    grid: Vec<Vec<usize>>,
    grid_size: usize,
}

impl SpatialGrid {
    fn new(cell_size: f32, world_size: f32) -> Self {
        let grid_size = (world_size / cell_size).ceil() as usize;
        let mut grid = Vec::with_capacity(grid_size * grid_size);
        
        // Initialize an empty grid
        for _ in 0..(grid_size * grid_size) {
            grid.push(Vec::new());
        }
        
        Self {
            cell_size,
            grid,
            grid_size,
        }
    }
    
    // Convert world coordinates to grid cell index
    fn pos_to_cell_index(&self, pos: Point2, world_size: f32) -> usize {
        let half_world = world_size / 2.0;
        // Convert from world space to grid space (0 to grid_size)
        let grid_x = ((pos.x + half_world) / self.cell_size).clamp(0.0, self.grid_size as f32 - 1.0) as usize;
        let grid_y = ((pos.y + half_world) / self.cell_size).clamp(0.0, self.grid_size as f32 - 1.0) as usize;
        
        // Convert 2D coordinates to 1D index
        grid_y * self.grid_size + grid_x
    }
    
    // Clear the grid
    fn clear(&mut self) {
        for cell in &mut self.grid {
            cell.clear();
        }
    }
    
    // Insert a boid into the grid
    fn insert(&mut self, boid_index: usize, position: Point2, world_size: f32) {
        let cell_index = self.pos_to_cell_index(position, world_size);
        self.grid[cell_index].push(boid_index);
    }
    
    // Get boid indices within and adjacent to the cell containing the given position
    fn get_nearby_indices(&self, position: Point2, world_size: f32) -> Vec<usize> {
        let half_world = world_size / 2.0;
        
        // Get the cell coordinates
        let grid_x = ((position.x + half_world) / self.cell_size).floor() as isize;
        let grid_y = ((position.y + half_world) / self.cell_size).floor() as isize;
        
        let mut result = Vec::new();
        
        // Check the cell and its neighbors (3x3 grid)
        for y_offset in -1..=1 {
            for x_offset in -1..=1 {
                let check_x = grid_x + x_offset;
                let check_y = grid_y + y_offset;
                
                // Skip if outside grid
                if check_x < 0 || check_y < 0 || 
                   check_x >= self.grid_size as isize || 
                   check_y >= self.grid_size as isize {
                    continue;
                }
                
                let cell_index = (check_y as usize) * self.grid_size + (check_x as usize);
                
                // Add all boids in this cell to the result
                for &boid_index in &self.grid[cell_index] {
                    result.push(boid_index);
                }
            }
        }
        
        result
    }
}

// Camera struct to handle zooming and panning
struct Camera {
    position: Vec2,
    zoom: f32,
    drag_start: Option<Vec2>,
    min_zoom: f32,
    max_zoom: f32,
    is_dragging: bool,  // Add a flag to track if we're actively dragging
    last_cursor_pos: Vec2, // Store the last cursor position to avoid jumps
}

impl Camera {
    fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            zoom: 1.0,
            drag_start: None,
            min_zoom: 0.1,
            max_zoom: 5.0,
            is_dragging: false,
            last_cursor_pos: Vec2::ZERO,
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
        // Only set the drag start position, don't move the camera yet
        self.drag_start = Some(position);
        self.last_cursor_pos = position;
        self.is_dragging = true;
    }

    // Update camera position while dragging
    fn drag(&mut self, position: Vec2) {
        if self.is_dragging {
            // Calculate drag delta from the last position (not the start position)
            let delta = position - self.last_cursor_pos;
            
            // Only apply movement if there's actually a change
            if delta.length_squared() > 0.0 {
                self.position -= delta / self.zoom;
                self.last_cursor_pos = position;
            }
        }
    }

    // End dragging
    fn end_drag(&mut self) {
        self.drag_start = None;
        self.is_dragging = false;
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
    fn separation(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for &i in neighbor_indices {
            let other = &boids[i];
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
    fn alignment(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for &i in neighbor_indices {
            let other = &boids[i];
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
    fn cohesion(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        for &i in neighbor_indices {
            let other = &boids[i];
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
    
    // Original flock method for backward compatibility (without spatial grid)
    fn flock(&mut self, boids: &[Boid], params: &SimulationParams) {
        let separation = self.separation_original(boids, params.separation_radius) * params.separation_weight;
        let alignment = self.alignment_original(boids, params.alignment_radius) * params.alignment_weight;
        let cohesion = self.cohesion_original(boids, params.cohesion_radius) * params.cohesion_weight;
        
        self.apply_force(separation);
        self.apply_force(alignment);
        self.apply_force(cohesion);
    }
    
    // Original versions of the flocking behaviors (without spatial grid)
    fn separation_original(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
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
    
    fn alignment_original(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
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
    
    fn cohesion_original(&self, boids: &[Boid], perception_radius: f32) -> Vec2 {
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
    // Performance settings
    enable_parallel: bool,
    enable_spatial_grid: bool,
    cell_size_factor: f32,  // Multiplier for cell size relative to perception radius
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

// Debug information to display
struct DebugInfo {
    fps: f32,
    frame_time: Duration,
    visible_boids: Arc<Mutex<usize>>,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::from_secs(0),
            visible_boids: Arc::new(Mutex::new(0)),
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
    spatial_grid: SpatialGrid,
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
    
    // Create spatial grid, cell size should be at least as large as the largest perception radius
    // A typical good value is the maximum perception radius
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
                
                ui.collapsing("Performance Tuning", |ui| {
                    ui.checkbox(&mut model.params.enable_parallel, "Enable Parallel Processing");
                    ui.checkbox(&mut model.params.enable_spatial_grid, "Enable Spatial Grid");
                    ui.add(egui::Slider::new(&mut model.params.cell_size_factor, 0.5..=2.0).text("Cell Size Factor"));
                    
                    ui.separator();
                    
                    // Performance metrics
                    ui.label(format!("FPS: {:.1}", model.debug_info.fps));
                    ui.label(format!("Frame time: {:.2} ms", model.debug_info.frame_time.as_secs_f64() * 1000.0));
                    ui.label(format!("Total Boids: {}", model.boids.len()));
                    ui.label(format!("Visible Boids: {}", *model.debug_info.visible_boids.lock().unwrap()));
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
        // Only use spatial grid if enabled
        if model.params.enable_spatial_grid {
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
            
            // Phase 3: Rebuild the spatial grid with updated positions
            model.spatial_grid.clear();
            for (i, boid) in model.boids.iter().enumerate() {
                model.spatial_grid.insert(i, boid.position, WORLD_SIZE);
            }
        } else {
            // Fallback to the original O(n²) approach without spatial partitioning
            // Create a copy of boids for the calculations
            let boids_clone = model.boids.clone();
            
            // Update each boid
            for boid in &mut model.boids {
                boid.flock(&boids_clone, &model.params);
                boid.update();
                boid.wrap_edges(WORLD_SIZE);
            }
        }
    }
    
    // We'll calculate visible boids during view function now
    // No need to count visible boids here as we do it in the view function
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
    let margin = BOID_SIZE * 2.0 / model.camera.zoom;
    let visible_area_with_margin = Rect::from_corners(
        pt2(visible_area.left() - margin, visible_area.bottom() - margin),
        pt2(visible_area.right() + margin, visible_area.top() + margin)
    );
    
    // Use frustum culling: only draw boids that are in the visible area
    let visible_boids: Vec<&Boid> = model.boids.iter()
        .filter(|boid| {
            let pos = Vec2::new(boid.position.x, boid.position.y);
            visible_area_with_margin.contains(pos)
        })
        .collect();
    
    // Track visible boid count for debug info
    if model.params.show_debug {
        let mut visible_boids_count = model.debug_info.visible_boids.lock().unwrap();
        *visible_boids_count = visible_boids.len();
    }
    
    // Draw each visible boid
    for boid in visible_boids {
        boid.draw(&draw, &model.camera, window_rect);
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
        
        // Draw FPS and other debug info in the top-left corner
        let margin = 20.0;
        let line_height = 20.0;
        
        // Create a background panel in the top-left corner
        let panel_width = 200.0;
        let panel_height = line_height * 6.0 + margin;
        let panel_x = window_rect.left() + panel_width / 2.0;
        let panel_y = window_rect.top() - panel_height / 2.0;
        
        // Draw the background panel
        draw.rect()
            .x_y(panel_x, panel_y)
            .w_h(panel_width, panel_height)
            .color(rgba(0.0, 0.0, 0.0, 0.7));
        
        // For left-aligned text in nannou, we need to position each text element
        // at the left edge of our panel plus half the text's width
        // Since we don't know the exact width of each text, we'll use a fixed offset
        // from the left edge of the panel
        let text_x = window_rect.left() + margin;
        let text_y = window_rect.top() - margin;
        
        // Draw each line of text
        let debug_texts = [
            format!("FPS: {:.1}", model.debug_info.fps),
            format!("Frame time: {:.2} ms", model.debug_info.frame_time.as_secs_f64() * 1000.0),
            format!("Total Boids: {}", model.boids.len()),
            format!("Visible Boids: {}", model.debug_info.visible_boids.lock().unwrap()),
            format!("Zoom: {:.2}x", model.camera.zoom),
            format!("World Size: {:.0}x{:.0}", WORLD_SIZE, WORLD_SIZE),
        ];
        
        for (i, text) in debug_texts.iter().enumerate() {
            let y = text_y - (i as f32 * line_height);
            
            // Position the text with a fixed offset from the left edge
            // This won't be perfect left alignment but should be close enough
            draw.text(text)
                .x_y(text_x + 70.0, y)
                .color(WHITE)
                .font_size(14);
        }
    }
    
    // Finish drawing
    draw.to_frame(app, &frame).unwrap();
    
    // Draw the egui UI
    model.egui.draw_to_frame(&frame).unwrap();
}

// Mouse moved event handler
fn mouse_moved(_app: &App, model: &mut Model, pos: Point2) {
    let new_pos = Vec2::new(pos.x, pos.y);
    
    // Update camera drag if we're dragging
    if model.camera.is_dragging {
        model.camera.drag(new_pos);
    }
    
    // Always update the stored mouse position
    model.mouse_position = new_pos;
}

// Mouse pressed event handler
fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        // Check if the click is on the UI before starting camera drag
        if !model.egui.ctx().is_pointer_over_area() {
            model.camera.start_drag(model.mouse_position);
        }
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
    
    // We'll handle cursor movement in the mouse_moved callback instead
    // This avoids coordinate system mismatches between the two event systems
}
