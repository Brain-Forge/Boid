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
 * - Optimizing empty cell handling with occupancy tracking
 * - Using adaptive cell checking based on local density
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
    // Lookup table for dense areas (smaller neighborhood)
    dense_area_lookup: Vec<(isize, isize)>,
    // Track which cells are occupied for quick empty cell checks
    cell_occupancy: Vec<bool>,
    // Statistics for adaptive optimization
    empty_cell_count: usize,
    max_cell_population: usize,
    avg_cell_population: f32,
}

impl SpatialGrid {
    pub fn new(cell_size: f32, world_size: f32) -> Self {
        let grid_size = (world_size / cell_size).ceil() as usize;
        let total_cells = grid_size * grid_size;
        
        // Pre-allocate grid with capacity
        let mut grid = Vec::with_capacity(total_cells);
        
        // Initialize an empty grid with pre-allocated capacity for each cell
        // Estimate average boids per cell based on typical distribution
        let estimated_boids_per_cell = 10;
        for _ in 0..total_cells {
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
        
        // Pre-compute dense area lookup (smaller neighborhood - just adjacent cells)
        let mut dense_area_lookup = Vec::with_capacity(5);
        dense_area_lookup.push((0, 0)); // Center cell
        dense_area_lookup.push((1, 0)); // Right
        dense_area_lookup.push((-1, 0)); // Left
        dense_area_lookup.push((0, 1)); // Top
        dense_area_lookup.push((0, -1)); // Bottom
        
        // Initialize cell occupancy tracking
        let cell_occupancy = vec![false; total_cells];
        
        Self {
            cell_size,
            grid,
            grid_size,
            nearby_with_distance_cache: Vec::with_capacity(estimated_capacity),
            wrapped_cell_lookup,
            dense_area_lookup,
            cell_occupancy,
            empty_cell_count: total_cells,
            max_cell_population: 0,
            avg_cell_population: 0.0,
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
        // Reset statistics
        let total_cells = self.grid.len();
        self.empty_cell_count = total_cells;
        self.max_cell_population = 0;
        self.avg_cell_population = 0.0;
        
        // Clear all cells and reset occupancy
        for (i, cell) in self.grid.iter_mut().enumerate() {
            cell.clear();
            self.cell_occupancy[i] = false;
        }
    }
    
    // Insert a boid into the grid
    #[inline]
    pub fn insert(&mut self, boid_index: usize, position: Point2, world_size: f32) {
        let cell_index = self.pos_to_cell_index(position, world_size);
        if cell_index < self.grid.len() {
            let cell = &mut self.grid[cell_index];
            
            // Update occupancy tracking
            if cell.is_empty() && !self.cell_occupancy[cell_index] {
                self.empty_cell_count -= 1;
            }
            
            cell.push(boid_index);
            self.cell_occupancy[cell_index] = true;
            
            // Update statistics
            self.max_cell_population = self.max_cell_population.max(cell.len());
        }
    }
    
    // Update statistics after all insertions
    pub fn update_statistics(&mut self) {
        let total_cells = self.grid.len();
        let occupied_cells = total_cells - self.empty_cell_count;
        
        if occupied_cells > 0 {
            let mut total_boids = 0;
            for cell in &self.grid {
                total_boids += cell.len();
            }
            
            self.avg_cell_population = total_boids as f32 / occupied_cells as f32;
        } else {
            self.avg_cell_population = 0.0;
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
    
    // Process a single cell and add its boids to the result
    #[inline]
    fn process_cell(&mut self, cell_index: usize, position: Point2, boids: &[Point2], world_size: f32) -> bool {
        if cell_index >= self.grid.len() || !self.cell_occupancy[cell_index] {
            return false; // Cell is out of bounds or empty
        }
        
        let cell = &self.grid[cell_index];
        if cell.is_empty() {
            return false; // Double-check that cell is actually empty
        }
        
        for &boid_index in cell {
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
        
        true // Cell had boids
    }
    
    // Get boid indices with pre-computed squared distances
    // This avoids redundant distance calculations in the force computations
    pub fn get_nearby_with_distances(&mut self, position: Point2, boids: &[nannou::prelude::Point2], world_size: f32) -> &[NeighborEntry] {
        // Clear the cache but keep its capacity
        self.nearby_with_distance_cache.clear();
        
        // Get the cell coordinates
        let (grid_x, grid_y) = self.pos_to_cell_coords(position, world_size);
        
        // Check center cell first
        let center_index = self.cell_coords_to_index(grid_x, grid_y);
        let center_has_boids = self.process_cell(center_index, position, boids, world_size);
        
        // Choose search pattern based on local density
        // If center cell is dense, use smaller neighborhood to reduce checks
        let search_pattern = if center_has_boids && 
            self.grid.get(center_index).map_or(0, |cell| cell.len()) > (self.avg_cell_population as usize * 2) {
            &self.dense_area_lookup
        } else {
            &self.wrapped_cell_lookup
        };
        
        // Clone the search pattern to avoid borrowing issues
        let search_pattern: Vec<(isize, isize)> = search_pattern.iter().cloned().collect();
        
        // Check the cell and its neighbors based on the selected pattern
        for &(x_offset, y_offset) in &search_pattern {
            // Skip center cell as we already processed it
            if x_offset == 0 && y_offset == 0 {
                continue;
            }
            
            let check_x = grid_x + x_offset;
            let check_y = grid_y + y_offset;
            
            // Get the cell index with wrapping
            let cell_index = self.cell_coords_to_index(check_x, check_y);
            
            // Process cell if it's not empty (early skip)
            self.process_cell(cell_index, position, boids, world_size);
        }
        
        &self.nearby_with_distance_cache
    }
    
    // Get statistics about the grid for debugging and optimization
    pub fn get_statistics(&self) -> (usize, usize, f32, usize) {
        let total_cells = self.grid.len();
        let occupied_cells = total_cells - self.empty_cell_count;
        let occupancy_percentage = (occupied_cells as f32 / total_cells as f32) * 100.0;
        
        (occupied_cells, total_cells, occupancy_percentage, self.max_cell_population)
    }
} 