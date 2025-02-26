/*
 * Boid Module
 * 
 * This module defines the Boid struct and its behavior.
 * Each boid follows three main rules:
 * 1. Separation: Avoid crowding neighbors
 * 2. Alignment: Steer towards the average heading of neighbors
 * 3. Cohesion: Steer towards the average position of neighbors
 */

use nannou::prelude::*;
use crate::params::SimulationParams;
use crate::camera::Camera;
use crate::BOID_SIZE;
use rand::Rng;

#[derive(Clone)]
pub struct Boid {
    pub position: Point2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    pub max_speed: f32,
    pub max_force: f32,
    pub color: Rgb<u8>,
}

impl Boid {
    pub fn new(x: f32, y: f32) -> Self {
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
    pub fn apply_force(&mut self, force: Vec2) {
        self.acceleration += force;
    }
    
    // Update the boid's position based on its velocity and acceleration
    pub fn update(&mut self) {
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
    pub fn wrap_edges(&mut self, world_size: f32) {
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
    pub fn separation(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
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
    pub fn alignment(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
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
    pub fn cohesion(&self, boids: &[Boid], neighbor_indices: &[usize], perception_radius: f32) -> Vec2 {
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
    pub fn flock(&mut self, boids: &[Boid], params: &SimulationParams) {
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
    pub fn draw(&self, draw: &Draw, camera: &Camera, window_rect: Rect) {
        // Convert boid position from world space to screen space
        let screen_pos = camera.world_to_screen(Vec2::new(self.position.x, self.position.y), window_rect);
        
        // Calculate the angle of the velocity
        let angle = self.velocity.y.atan2(self.velocity.x);
        
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
                .points(points.iter().cloned())
                .xy(pt2(screen_pos.x, screen_pos.y))
                .rotate(angle);
        });
    }
} 