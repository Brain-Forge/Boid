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
 * - Providing pre-computed distances for better performance
 */

use nannou::prelude::*;

/// A neighbor entry with pre-computed distance information
#[derive(Clone, Copy)]
pub struct NeighborEntry {
    pub index: usize,
    pub distance_squared: f32,
}

pub struct SpatialGrid {
    pub cell_size: f32,
    pub grid: Vec<Vec<usize>>,
    pub grid_size: usize,
    // Cache for nearby indices with distances to avoid reallocations
    nearby_with_distance_cache: Vec<NeighborEntry>,
}

impl SpatialGrid {
    pub fn new(cell_size: f32, world_size: f32) -> Self {
        let grid_size = (world_size / cell_size).ceil() as usize;
        
        // Pre-allocate grid with capacity
        let mut grid = Vec::with_capacity(grid_size * grid_size);
        
        // Initialize an empty grid with pre-allocated capacity for each cell
        // Estimate average boids per cell based on typical distribution
        let estimated_boids_per_cell = 10;
        for _ in 0..(grid_size * grid_size) {
            let mut cell = Vec::with_capacity(estimated_boids_per_cell);
            cell.clear(); // Ensure it's empty but with capacity
            grid.push(cell);
        }
        
        // Pre-allocate caches for nearby indices (9 cells * estimated boids per cell)
        let estimated_capacity = 9 * estimated_boids_per_cell;
        
        Self {
            cell_size,
            grid,
            grid_size,
            nearby_with_distance_cache: Vec::with_capacity(estimated_capacity),
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
        // No need to clear caches as they'll be overwritten
    }
    
    // Insert a boid into the grid
    #[inline]
    pub fn insert(&mut self, boid_index: usize, position: Point2, world_size: f32) {
        let cell_index = self.pos_to_cell_index(position, world_size);
        self.grid[cell_index].push(boid_index);
    }
    
    // Get boid indices with pre-computed squared distances
    // This avoids redundant distance calculations in the force computations
    pub fn get_nearby_with_distances(&mut self, position: Point2, boids: &[nannou::prelude::Point2], world_size: f32) -> &[NeighborEntry] {
        let half_world = world_size / 2.0;
        
        // Clear the cache but keep its capacity
        self.nearby_with_distance_cache.clear();
        
        // Get the cell coordinates
        let grid_x = ((position.x + half_world) / self.cell_size).floor() as isize;
        let grid_y = ((position.y + half_world) / self.cell_size).floor() as isize;
        
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
                
                // Add all boids in this cell to the result with pre-computed distances
                for &boid_index in &self.grid[cell_index] {
                    if boid_index < boids.len() {
                        let other_pos = boids[boid_index];
                        
                        // Skip if it's the same boid
                        if position == other_pos {
                            continue;
                        }
                        
                        // Calculate squared distance
                        let dx = position.x - other_pos.x;
                        let dy = position.y - other_pos.y;
                        let distance_squared = dx * dx + dy * dy;
                        
                        self.nearby_with_distance_cache.push(NeighborEntry {
                            index: boid_index,
                            distance_squared,
                        });
                    }
                }
            }
        }
        
        &self.nearby_with_distance_cache
    }
} 