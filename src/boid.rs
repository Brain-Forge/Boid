/*
 * Boid Module
 * 
 * This module defines the Boid struct and its behavior.
 * Each boid follows three main rules:
 * 1. Separation: Avoid crowding neighbors
 * 2. Alignment: Steer towards the average heading of neighbors
 * 3. Cohesion: Steer towards the average position of neighbors
 * 
 * Optimized for performance by:
 * - Normalizing vectors only when necessary
 * - Avoiding unnecessary vector instantiations
 * - Using squared distances where possible
 * - Using a spatial grid for efficient neighbor lookups
 * - Tracking visibility for efficient culling
 * 
 * Supports interpolation between physics updates for smooth rendering at any framerate.
 */

use nannou::prelude::*;
use crate::camera::Camera;
use crate::BOID_SIZE;
use rand::Rng;

#[derive(Clone)]
pub struct Boid {
    pub position: Point2,      // Current physics position
    pub velocity: Vec2,        // Current physics velocity
    pub acceleration: Vec2,    // Current physics acceleration
    pub prev_position: Point2, // Previous physics position (for interpolation)
    pub prev_velocity: Vec2,   // Previous physics velocity (for interpolation)
    pub max_speed: f32,
    pub max_force: f32,
    pub color: Rgb<u8>,
    pub is_visible: bool,      // Visibility flag for culling optimization
}

impl Boid {
    pub fn new(x: f32, y: f32) -> Self {
        let mut rng = rand::thread_rng();
        
        // Random initial velocity
        let vx = rng.gen_range(-1.0..1.0);
        let vy = rng.gen_range(-1.0..1.0);
        
        // Only normalize if needed (avoid division by zero)
        let velocity = if vx == 0.0 && vy == 0.0 {
            vec2(1.0, 0.0) * 2.0
        } else {
            let length = (vx * vx + vy * vy).sqrt();
            vec2(vx / length, vy / length) * 2.0
        };
        
        let position = pt2(x, y);
        
        Self {
            position,
            velocity,
            acceleration: Vec2::ZERO,
            prev_position: position, // Initialize previous to current
            prev_velocity: velocity, // Initialize previous to current
            max_speed: 4.0,
            max_force: 0.1,
            color: rgb(220, 220, 220),
            is_visible: false, // Initially not visible
        }
    }
    
    // Apply a force to the boid
    pub fn apply_force(&mut self, force: Vec2) {
        self.acceleration += force;
    }
    
    // Store current state as previous state before updating
    pub fn store_previous_state(&mut self) {
        self.prev_position = self.position;
        self.prev_velocity = self.velocity;
    }
    
    // Update the boid's position based on its velocity and acceleration
    pub fn update(&mut self) {
        // Update velocity
        self.velocity += self.acceleration;
        
        // Limit speed (only normalize if exceeding max_speed)
        let speed_squared = self.velocity.length_squared();
        let max_speed_squared = self.max_speed * self.max_speed;
        
        if speed_squared > max_speed_squared {
            let speed = speed_squared.sqrt();
            self.velocity *= self.max_speed / speed;
        }
        
        // Update position
        self.position += self.velocity;
        
        // Reset acceleration
        self.acceleration = Vec2::ZERO;
    }
    
    // Get interpolated position between previous and current state
    pub fn get_interpolated_position(&self, alpha: f32) -> Point2 {
        pt2(
            self.prev_position.x + (self.position.x - self.prev_position.x) * alpha,
            self.prev_position.y + (self.position.y - self.prev_position.y) * alpha
        )
    }
    
    // Get interpolated velocity between previous and current state
    pub fn get_interpolated_velocity(&self, alpha: f32) -> Vec2 {
        vec2(
            self.prev_velocity.x + (self.velocity.x - self.prev_velocity.x) * alpha,
            self.prev_velocity.y + (self.velocity.y - self.prev_velocity.y) * alpha
        )
    }
    
    // Wrap the boid around the world edges
    pub fn wrap_edges(&mut self, world_size: f32) {
        let half_size = world_size / 2.0;
        
        if self.position.x > half_size {
            self.position.x = -half_size;
            self.prev_position.x = -half_size; // Update previous position too to avoid interpolation issues
        } else if self.position.x < -half_size {
            self.position.x = half_size;
            self.prev_position.x = half_size; // Update previous position too to avoid interpolation issues
        }
        
        if self.position.y > half_size {
            self.position.y = -half_size;
            self.prev_position.y = -half_size; // Update previous position too to avoid interpolation issues
        } else if self.position.y < -half_size {
            self.position.y = half_size;
            self.prev_position.y = half_size; // Update previous position too to avoid interpolation issues
        }
    }
    
    // Original versions of the flocking behaviors (without spatial grid)
    pub fn separation_original(&self, boids: &[Boid], perception_radius: f32, _use_squared_distance: bool) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        // Pre-calculate squared radius for optimization
        let radius_squared = perception_radius * perception_radius;
        
        for other in boids {
            // Calculate squared distance directly
            let dx = self.position.x - other.position.x;
            let dy = self.position.y - other.position.y;
            let d_squared = dx * dx + dy * dy;
            
            // Skip if it's the same boid or outside perception radius
            if d_squared <= 0.0 || d_squared >= radius_squared {
                continue;
            }
            
            // Calculate vector pointing away from neighbor
            // Only calculate actual distance if needed for weighting
            let d = d_squared.sqrt();
            
            // Weight by distance (closer boids have more influence)
            // Reuse dx and dy instead of creating a new vector
            steering.x += (dx / d) / d;
            steering.y += (dy / d) / d;
            count += 1;
        }
        
        if count > 0 {
            steering /= count as f32;
            
            let steering_length_squared = steering.length_squared();
            if steering_length_squared > 0.0 {
                // Implement Reynolds: Steering = Desired - Velocity
                // Only normalize if needed
                let steering_length = steering_length_squared.sqrt();
                let desired = steering * (self.max_speed / steering_length);
                
                steering = desired - self.velocity;
                
                // Limit force
                let force_squared = steering.length_squared();
                let max_force_squared = self.max_force * self.max_force;
                
                if force_squared > max_force_squared {
                    let force_length = force_squared.sqrt();
                    steering *= self.max_force / force_length;
                }
            }
        }
        
        steering
    }
    
    // Original versions of the flocking behaviors (without spatial grid)
    pub fn alignment_original(&self, boids: &[Boid], perception_radius: f32, _use_squared_distance: bool) -> Vec2 {
        let mut steering = Vec2::ZERO;
        let mut count = 0;
        
        // Pre-calculate squared radius for optimization
        let radius_squared = perception_radius * perception_radius;
        
        for other in boids {
            // Calculate squared distance directly
            let dx = self.position.x - other.position.x;
            let dy = self.position.y - other.position.y;
            let d_squared = dx * dx + dy * dy;
            
            // Skip if it's the same boid or outside perception radius
            if d_squared <= 0.0 || d_squared >= radius_squared {
                continue;
            }
            
            // Accumulate velocities
            steering += other.velocity;
            count += 1;
        }
        
        if count > 0 {
            steering /= count as f32;
            
            // Only normalize if the steering vector has magnitude
            let steering_length_squared = steering.length_squared();
            if steering_length_squared > 0.0 {
                // Implement Reynolds: Steering = Desired - Velocity
                let steering_length = steering_length_squared.sqrt();
                let desired = steering * (self.max_speed / steering_length);
                
                steering = desired - self.velocity;
                
                // Limit force
                let force_squared = steering.length_squared();
                let max_force_squared = self.max_force * self.max_force;
                
                if force_squared > max_force_squared {
                    let force_length = force_squared.sqrt();
                    steering *= self.max_force / force_length;
                }
            }
        }
        
        steering
    }
    
    // Original versions of the flocking behaviors (without spatial grid)
    pub fn cohesion_original(&self, boids: &[Boid], perception_radius: f32, _use_squared_distance: bool) -> Vec2 {
        let mut sum_position = Vec2::ZERO;
        let mut count = 0;
        
        // Pre-calculate squared radius for optimization
        let radius_squared = perception_radius * perception_radius;
        
        for other in boids {
            // Calculate squared distance directly
            let dx = self.position.x - other.position.x;
            let dy = self.position.y - other.position.y;
            let d_squared = dx * dx + dy * dy;
            
            // Skip if it's the same boid or outside perception radius
            if d_squared <= 0.0 || d_squared >= radius_squared {
                continue;
            }
            
            // Accumulate positions (reuse existing Vec2 from position)
            sum_position.x += other.position.x;
            sum_position.y += other.position.y;
            count += 1;
        }
        
        if count > 0 {
            sum_position /= count as f32;
            
            // Create desired velocity towards target
            let desired = sum_position - Vec2::new(self.position.x, self.position.y);
            
            let desired_length_squared = desired.length_squared();
            if desired_length_squared > 0.0 {
                // Scale to maximum speed (only normalize if needed)
                let desired_length = desired_length_squared.sqrt();
                let desired_normalized = desired * (self.max_speed / desired_length);
                
                // Implement Reynolds: Steering = Desired - Velocity
                let mut steering = desired_normalized - self.velocity;
                
                // Limit force
                let force_squared = steering.length_squared();
                let max_force_squared = self.max_force * self.max_force;
                
                if force_squared > max_force_squared {
                    let force_length = force_squared.sqrt();
                    steering *= self.max_force / force_length;
                }
                
                return steering;
            }
        }
        
        Vec2::ZERO
    }
    
    // Draw the boid
    pub fn draw(&self, draw: &Draw, camera: &Camera, window_rect: Rect, alpha: f32) {
        // Get interpolated position and velocity
        let interpolated_position = self.get_interpolated_position(alpha);
        let interpolated_velocity = self.get_interpolated_velocity(alpha);
        
        // Convert boid position from world space to screen space
        let screen_pos = camera.world_to_screen(Vec2::new(interpolated_position.x, interpolated_position.y), window_rect);
        
        // Calculate the angle of the velocity
        let angle = interpolated_velocity.y.atan2(interpolated_velocity.x);
        
        // Scale the boid size based on zoom level
        let scaled_size = BOID_SIZE * camera.zoom;
        
        // Use thread-local storage for caching the triangle points
        thread_local! {
            static LAST_SIZE: std::cell::Cell<f32> = std::cell::Cell::new(0.0);
            static CACHED_POINTS: std::cell::RefCell<[Point2; 3]> = std::cell::RefCell::new([
                pt2(0.0, 0.0), pt2(0.0, 0.0), pt2(0.0, 0.0)
            ]);
        }
        
        // Only recalculate points if the size has changed
        LAST_SIZE.with(|last_size| {
            if (last_size.get() - scaled_size).abs() > 0.01 {
                last_size.set(scaled_size);
                
                let new_points = [
                    pt2(scaled_size, 0.0),
                    pt2(-scaled_size, scaled_size / 2.0),
                    pt2(-scaled_size, -scaled_size / 2.0),
                ];
                
                CACHED_POINTS.with(|points| {
                    *points.borrow_mut() = new_points;
                });
            }
        });
        
        // Draw the boid using the cached points
        CACHED_POINTS.with(|points| {
            let points = points.borrow();
            draw.polygon()
                .color(self.color)
                .points(points.clone())
                .xy(pt2(screen_pos.x, screen_pos.y))
                .rotate(angle);
        });
    }
} 