/*
 * Spatial Grid Module
 * 
 * This module defines the SpatialGrid struct for efficient neighbor lookups.
 * It divides the simulation space into a grid of cells, allowing for O(1) 
 * neighbor queries instead of O(n) linear searches.
 */

use nannou::prelude::*;

pub struct SpatialGrid {
    pub cell_size: f32,
    pub grid: Vec<Vec<usize>>,
    pub grid_size: usize,
}

impl SpatialGrid {
    pub fn new(cell_size: f32, world_size: f32) -> Self {
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
    pub fn pos_to_cell_index(&self, pos: Point2, world_size: f32) -> usize {
        let half_world = world_size / 2.0;
        // Convert from world space to grid space (0 to grid_size)
        let grid_x = ((pos.x + half_world) / self.cell_size).clamp(0.0, self.grid_size as f32 - 1.0) as usize;
        let grid_y = ((pos.y + half_world) / self.cell_size).clamp(0.0, self.grid_size as f32 - 1.0) as usize;
        
        // Convert 2D coordinates to 1D index
        grid_y * self.grid_size + grid_x
    }
    
    // Clear the grid
    pub fn clear(&mut self) {
        for cell in &mut self.grid {
            cell.clear();
        }
    }
    
    // Insert a boid into the grid
    pub fn insert(&mut self, boid_index: usize, position: Point2, world_size: f32) {
        let cell_index = self.pos_to_cell_index(position, world_size);
        self.grid[cell_index].push(boid_index);
    }
    
    // Get boid indices within and adjacent to the cell containing the given position
    pub fn get_nearby_indices(&self, position: Point2, world_size: f32) -> Vec<usize> {
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