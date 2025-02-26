/*
 * Culling Module
 * 
 * This module handles frustum culling and visibility optimization for the boid simulation.
 * It determines which boids are visible in the current view and should be rendered.
 * 
 * Optimized for performance by:
 * - Using spatial grid for efficient culling when available
 * - Caching visible boids when the simulation is paused
 * - Using visibility flags to avoid duplicate processing
 */

use nannou::prelude::*;

use crate::app::Model;
use crate::boid::Boid;
use crate::WORLD_SIZE;

// Efficient function to get visible boids using the best available method
pub fn get_visible_boids(model: &Model, visible_area: Rect) -> Vec<usize> {
    // Use cached visible boids if available and simulation is paused
    if model.params.pause_simulation {
        unsafe {
            if let Some(cached_indices) = &*model.cached_visible_boids.get() {
                return cached_indices.clone();
            }
        }
    }
    
    // Choose the most efficient culling method based on available optimizations
    let indices = if model.params.enable_spatial_grid {
        // Use spatial grid for efficient culling
        cull_with_spatial_grid(model, visible_area)
    } else {
        // Use brute force culling
        cull_brute_force(model, visible_area)
    };
    
    // Cache the indices if simulation is paused
    if model.params.pause_simulation {
        unsafe {
            *model.cached_visible_boids.get() = Some(indices.clone());
        }
    }
    
    indices
}

// Brute force culling method
pub fn cull_brute_force(model: &Model, visible_area: Rect) -> Vec<usize> {
    let mut visible_indices = Vec::new();
    
    // Reset visibility flags for all boids
    for boid in &model.boids {
        unsafe {
            // Use raw pointer to modify the boid without borrowing issues
            let boid_ptr = boid as *const Boid as *mut Boid;
            (*boid_ptr).is_visible = false;
        }
    }
    
    // Check each boid for visibility
    for (i, boid) in model.boids.iter().enumerate() {
        let pos = if model.params.enable_interpolation {
            let interpolated_pos = boid.get_interpolated_position(model.interpolation_alpha);
            Vec2::new(interpolated_pos.x, interpolated_pos.y)
        } else {
            Vec2::new(boid.position.x, boid.position.y)
        };
        
        if visible_area.contains(pos) {
            visible_indices.push(i);
            
            // Mark as visible
            unsafe {
                let boid_ptr = boid as *const Boid as *mut Boid;
                (*boid_ptr).is_visible = true;
            }
        }
    }
    
    visible_indices
}

// Use spatial grid for efficient culling
pub fn cull_with_spatial_grid(model: &Model, visible_area: Rect) -> Vec<usize> {
    // Reset visibility flags for all boids
    for boid in &model.boids {
        unsafe {
            // Use raw pointer to modify the boid without borrowing issues
            let boid_ptr = boid as *const Boid as *mut Boid;
            (*boid_ptr).is_visible = false;
        }
    }
    
    // Convert visible area to grid cells
    let half_world = WORLD_SIZE / 2.0;
    let cell_size = model.spatial_grid.cell_size;
    let grid_size = model.spatial_grid.grid_size;
    
    // Calculate grid cell ranges that overlap with the visible area
    let min_grid_x = ((visible_area.left() + half_world) / cell_size).floor() as isize;
    let min_grid_y = ((visible_area.bottom() + half_world) / cell_size).floor() as isize;
    let max_grid_x = ((visible_area.right() + half_world) / cell_size).ceil() as isize;
    let max_grid_y = ((visible_area.top() + half_world) / cell_size).ceil() as isize;
    
    // Clamp to grid boundaries
    let min_grid_x = min_grid_x.clamp(0, grid_size as isize - 1);
    let min_grid_y = min_grid_y.clamp(0, grid_size as isize - 1);
    let max_grid_x = max_grid_x.clamp(0, grid_size as isize - 1);
    let max_grid_y = max_grid_y.clamp(0, grid_size as isize - 1);
    
    // Collect boids from all cells that overlap with the visible area
    let mut visible_indices = Vec::with_capacity(
        ((max_grid_x - min_grid_x + 1) * (max_grid_y - min_grid_y + 1) * 10) as usize
    );
    
    for grid_y in min_grid_y..=max_grid_y {
        let y_index = grid_y as usize * grid_size;
        
        for grid_x in min_grid_x..=max_grid_x {
            let cell_index = y_index + grid_x as usize;
            
            // Add all boids in this cell
            if cell_index < model.spatial_grid.grid.len() {
                for &boid_index in &model.spatial_grid.grid[cell_index] {
                    // Skip if already marked as visible
                    if model.boids[boid_index].is_visible {
                        continue;
                    }
                    
                    // For cells at the boundary, we need to check if the boid is actually visible
                    if boid_index < model.boids.len() {
                        let is_visible = if model.params.enable_interpolation {
                            let interpolated_pos = model.boids[boid_index].get_interpolated_position(model.interpolation_alpha);
                            let pos = Vec2::new(interpolated_pos.x, interpolated_pos.y);
                            visible_area.contains(pos)
                        } else {
                            let pos = Vec2::new(model.boids[boid_index].position.x, model.boids[boid_index].position.y);
                            visible_area.contains(pos)
                        };
                        
                        if is_visible {
                            visible_indices.push(boid_index);
                            
                            // Mark as visible
                            unsafe {
                                let boid_ptr = &model.boids[boid_index] as *const Boid as *mut Boid;
                                (*boid_ptr).is_visible = true;
                            }
                        }
                    }
                }
            }
        }
    }
    
    visible_indices
} 