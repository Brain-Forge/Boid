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
 * - Supporting world wrapping for seamless edge transitions
 * - Using a more efficient cell lookup strategy for wrapped worlds
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
    // Lookup table for wrapped cell coordinates to avoid repeated calculations
    wrapped_cell_lookup: Vec<(isize, isize)>,
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
        
        // Pre-compute wrapped cell offsets for a 3x3 neighborhood
        let mut wrapped_cell_lookup = Vec::with_capacity(9);
        for y_offset in -1..=1 {
            for x_offset in -1..=1 {
                wrapped_cell_lookup.push((x_offset, y_offset));
            }
        }
        
        Self {
            cell_size,
            grid,
            grid_size,
            nearby_with_distance_cache: Vec::with_capacity(estimated_capacity),
            wrapped_cell_lookup,
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
    
    // Convert world coordinates to grid cell coordinates
    #[inline]
    pub fn pos_to_cell_coords(&self, pos: Point2, world_size: f32) -> (isize, isize) {
        let half_world = world_size / 2.0;
        // Convert from world space to grid space (0 to grid_size)
        let grid_x = ((pos.x + half_world) / self.cell_size).floor() as isize;
        let grid_y = ((pos.y + half_world) / self.cell_size).floor() as isize;
        
        (grid_x, grid_y)
    }
    
    // Convert grid cell coordinates to 1D index, handling wrapping
    #[inline]
    pub fn cell_coords_to_index(&self, x: isize, y: isize) -> usize {
        // Handle wrapping by using modulo arithmetic
        let grid_size = self.grid_size as isize;
        let wrapped_x = ((x % grid_size) + grid_size) % grid_size;
        let wrapped_y = ((y % grid_size) + grid_size) % grid_size;
        
        (wrapped_y as usize) * self.grid_size + (wrapped_x as usize)
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
        if cell_index < self.grid.len() {  // Add bounds check for cell_index
            self.grid[cell_index].push(boid_index);
        }
    }
    
    // Calculate the squared distance between two points, accounting for world wrapping
    #[inline]
    fn wrapped_distance_squared(p1: Point2, p2: Point2, world_size: f32) -> f32 {
        let half_size = world_size / 2.0;
        
        // Calculate direct distance components
        let mut dx = (p1.x - p2.x).abs();
        let mut dy = (p1.y - p2.y).abs();
        
        // Check if wrapping around provides a shorter path
        if dx > half_size {
            dx = world_size - dx;
        }
        
        if dy > half_size {
            dy = world_size - dy;
        }
        
        // Return squared distance
        dx * dx + dy * dy
    }
    
    // Get boid indices with pre-computed squared distances
    // This avoids redundant distance calculations in the force computations
    pub fn get_nearby_with_distances(&mut self, position: Point2, boids: &[nannou::prelude::Point2], world_size: f32) -> &[NeighborEntry] {
        // Clear the cache but keep its capacity
        self.nearby_with_distance_cache.clear();
        
        // Get the cell coordinates
        let (grid_x, grid_y) = self.pos_to_cell_coords(position, world_size);
        
        // Check the cell and its neighbors (3x3 grid)
        for &(x_offset, y_offset) in &self.wrapped_cell_lookup {
            let check_x = grid_x + x_offset;
            let check_y = grid_y + y_offset;
            
            // Get the cell index with wrapping
            let cell_index = self.cell_coords_to_index(check_x, check_y);
            
            // Add all boids in this cell to the result with pre-computed distances
            if cell_index < self.grid.len() {
                for &boid_index in &self.grid[cell_index] {
                    if boid_index < boids.len() {
                        let other_pos = boids[boid_index];
                        
                        // Skip if it's the same boid
                        if position == other_pos {
                            continue;
                        }
                        
                        // Calculate squared distance with wrapping
                        let distance_squared = Self::wrapped_distance_squared(position, other_pos, world_size);
                        
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