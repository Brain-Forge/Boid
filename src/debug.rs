/*
 * Debug Module
 * 
 * This module provides debugging information and metrics for the boid simulation.
 * It tracks performance statistics, rendering information, and simulation state
 * to help with optimization and debugging.
 */

use nannou::prelude::*;
use std::time::Duration;

// Debug information for the simulation
pub struct DebugInfo {
    pub fps: f32,
    pub frame_time: Duration,
    pub physics_updates_per_frame: Option<usize>,
    pub interpolation_alpha: Option<f32>,
    pub visible_boids_count: Option<usize>,
    pub chunk_size: Option<usize>,
    pub selected_boid_index: Option<usize>,
    pub follow_mode_active: bool,
    pub culling_efficiency: Option<f32>,
    pub frustum_area_ratio: Option<f32>,
    // Spatial grid statistics
    pub grid_occupied_cells: Option<usize>,
    pub grid_total_cells: Option<usize>,
    pub grid_occupancy_percentage: Option<f32>,
    pub grid_max_cell_population: Option<usize>,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::ZERO,
            physics_updates_per_frame: None,
            interpolation_alpha: None,
            visible_boids_count: None,
            chunk_size: None,
            selected_boid_index: None,
            follow_mode_active: false,
            culling_efficiency: None,
            frustum_area_ratio: None,
            // Initialize grid statistics
            grid_occupied_cells: None,
            grid_total_cells: None,
            grid_occupancy_percentage: None,
            grid_max_cell_population: None,
        }
    }
}

impl DebugInfo {
    // Update debug information from the model
    pub fn update_from_app(&mut self, app: &App) {
        // Basic performance metrics
        self.fps = app.fps();
        self.frame_time = app.duration.since_prev_update;
    }
    
    // Update debug information from model fields
    pub fn update_from_model(&mut self, 
                            selected_boid_index: Option<usize>,
                            follow_mode_active: bool,
                            interpolation_alpha: f32,
                            cached_visible_boids: &Option<Vec<usize>>,
                            boids_len: usize,
                            visible_area_cache: Option<Rect>,
                            world_size: f32) {
        // Boid selection and camera state
        self.selected_boid_index = selected_boid_index;
        self.follow_mode_active = follow_mode_active;
        
        // Interpolation state
        self.interpolation_alpha = Some(interpolation_alpha);
        
        // Calculate visible boids count
        if let Some(visible_boids) = cached_visible_boids {
            self.visible_boids_count = Some(visible_boids.len());
            
            // Calculate culling efficiency (percentage of boids culled)
            if boids_len > 0 {
                let culling_efficiency = 100.0 * (1.0 - (visible_boids.len() as f32 / boids_len as f32));
                self.culling_efficiency = Some(culling_efficiency);
            }
        }
        
        // Calculate frustum area ratio if we have a visible area
        if let Some(visible_area) = visible_area_cache {
            let visible_area_size = visible_area.w() * visible_area.h();
            let world_area = world_size * world_size;
            self.frustum_area_ratio = Some(visible_area_size / world_area);
        }
    }
    
    // Update spatial grid statistics
    pub fn update_grid_stats(&mut self, occupied_cells: usize, total_cells: usize, 
                            occupancy_percentage: f32, max_cell_population: usize) {
        self.grid_occupied_cells = Some(occupied_cells);
        self.grid_total_cells = Some(total_cells);
        self.grid_occupancy_percentage = Some(occupancy_percentage);
        self.grid_max_cell_population = Some(max_cell_population);
    }
} 