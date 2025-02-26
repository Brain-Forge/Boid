/*
 * Spatial Grid Module
 * 
 * This module defines the SpatialGrid struct for efficient neighbor lookups.
 * It divides the simulation space into a grid of cells, allowing for O(1) 
 * neighbor queries instead of O(n) linear searches.
 * 
 * Optimized for performance by:
 * - Using direct coordinate calculations instead of vector operations
 * - Pre-allocating memory for results to avoid reallocations
 * - Using integer arithmetic where possible
 * - Avoiding unnecessary bounds checks with clamping
 */

use nannou::prelude::*;

pub struct SpatialGrid {
    pub cell_size: f32,
    pub grid: Vec<Vec<usize>>,
    pub grid_size: usize,
    // Cache for nearby indices to avoid reallocations
    nearby_indices_cache: Vec<usize>,
}

impl SpatialGrid {
    pub fn new(cell_size: f32, world_size: f32) -> Self {
        let grid_size = (world_size / cell_size).ceil() as usize;
        let mut grid = Vec::with_capacity(grid_size * grid_size);
        
        // Initialize an empty grid
        for _ in 0..(grid_size * grid_size) {
            grid.push(Vec::new());
        }
        
        // Pre-allocate cache for nearby indices (9 cells * estimated boids per cell)
        let estimated_capacity = 9 * 10; // Assuming ~10 boids per cell on average
        
        Self {
            cell_size,
            grid,
            grid_size,
            nearby_indices_cache: Vec::with_capacity(estimated_capacity),
        }
    }
    
    // Convert world coordinates to grid cell index
    #[inline]
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
    #[inline]
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
        
        // Create a new result vector or reuse the cached one
        // Note: In a multi-threaded context, we'd need to clone this instead
        let mut result = Vec::with_capacity(self.nearby_indices_cache.capacity());
        
        // Pre-calculate grid size as isize to avoid repeated casts
        let grid_size_isize = self.grid_size as isize;
        
        // Check the cell and its neighbors (3x3 grid)
        for y_offset in -1..=1 {
            let check_y = grid_y + y_offset;
            
            // Skip if y is outside grid
            if check_y < 0 || check_y >= grid_size_isize {
                continue;
            }
            
            let y_index = check_y as usize * self.grid_size;
            
            for x_offset in -1..=1 {
                let check_x = grid_x + x_offset;
                
                // Skip if x is outside grid
                if check_x < 0 || check_x >= grid_size_isize {
                    continue;
                }
                
                let cell_index = y_index + check_x as usize;
                
                // Add all boids in this cell to the result
                result.extend_from_slice(&self.grid[cell_index]);
            }
        }
        
        result
    }
} 