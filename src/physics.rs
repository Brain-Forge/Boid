/*
 * Physics Module
 * 
 * This module handles the physics simulation for the boid flocking behavior.
 * It contains functions for updating boid positions and applying the flocking rules:
 * separation, alignment, and cohesion.
 * 
 * Optimized for performance by:
 * - Using spatial partitioning for efficient neighbor lookups
 * - Combining forces before applying them to reduce vector operations
 * - Using squared distances where possible
 * - Parallel processing for large numbers of boids
 */

use nannou::prelude::*;
use rand::Rng;
use rayon::prelude::*;

use crate::app::Model;
use crate::boid::Boid;
use crate::spatial_grid::SpatialGrid;
use crate::WORLD_SIZE;

// Reset boids to random positions
pub fn reset_boids(model: &mut Model) {
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

// Update boid positions and behaviors
pub fn update_boids(model: &mut Model) {
    // Only use spatial grid if enabled
    if model.params.enable_spatial_grid {
        update_boids_with_spatial_grid(model);
    } else {
        update_boids_without_spatial_grid(model);
    }
}

// Update boids using spatial grid for optimization
fn update_boids_with_spatial_grid(model: &mut Model) {
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
    
    // Pre-calculate weights to avoid multiplication in the inner loop
    let separation_weight = model.params.separation_weight;
    let alignment_weight = model.params.alignment_weight;
    let cohesion_weight = model.params.cohesion_weight;
    
    // Extract positions and velocities for the spatial grid's calculations
    let boid_positions: Vec<Point2> = model.boids.iter().map(|boid| boid.position).collect();
    let boid_velocities: Vec<Vec2> = model.boids.iter().map(|boid| boid.velocity).collect();
    
    // Pre-calculate squared radii
    let sep_radius_sq = model.params.separation_radius * model.params.separation_radius;
    let align_radius_sq = model.params.alignment_radius * model.params.alignment_radius;
    let cohesion_radius_sq = model.params.cohesion_radius * model.params.cohesion_radius;
    
    // Choose between parallel and sequential processing based on the setting
    if model.params.enable_parallel {
        // For parallel processing, we need to pre-compute all neighbor data
        let mut neighbor_data = Vec::with_capacity(model.boids.len());
        
        // First pass: gather all neighbor data
        for (_i, boid) in model.boids.iter().enumerate() {
            let nearby_with_distances = model.spatial_grid.get_nearby_with_distances(
                boid.position, 
                &boid_positions, 
                WORLD_SIZE
            );
            
            // Clone the data to avoid borrowing issues
            let neighbors: Vec<_> = nearby_with_distances.iter().map(|&n| n).collect();
            neighbor_data.push(neighbors);
        }
        
        // Second pass: calculate and apply forces in parallel
        // Use par_chunks_mut instead of par_iter_mut.enumerate() to reduce synchronization overhead
        // This processes boids in chunks, reducing the number of parallel tasks and synchronization points
        let chunk_size = std::cmp::max(model.boids.len() / rayon::current_num_threads(), 1);
        
        // Update debug info with chunk size if debug is enabled
        if model.params.show_debug {
            model.debug_info.chunk_size = chunk_size;
        }
        
        model.boids.par_chunks_mut(chunk_size).enumerate().for_each(|(chunk_idx, boid_chunk)| {
            // Process each boid in the chunk sequentially
            for (i_in_chunk, boid) in boid_chunk.iter_mut().enumerate() {
                let i = chunk_idx * chunk_size + i_in_chunk;
                if i >= neighbor_data.len() {
                    break; // Safety check for the last chunk which might be smaller
                }
                
                let neighbors = &neighbor_data[i];
                
                // Calculate forces
                let mut separation = Vec2::ZERO;
                let mut alignment = Vec2::ZERO;
                let mut cohesion = Vec2::ZERO;
                let mut sep_count = 0;
                let mut align_count = 0;
                let mut cohesion_count = 0;
                
                // Process all neighbors in a single pass
                for &neighbor in neighbors {
                    let d_squared = neighbor.distance_squared;
                    let other_idx = neighbor.index;
                    
                    // Separation
                    if d_squared < sep_radius_sq {
                        // Calculate vector pointing away from neighbor
                        let dx = boid.position.x - boid_positions[other_idx].x;
                        let dy = boid.position.y - boid_positions[other_idx].y;
                        
                        // Only calculate actual distance if needed for weighting
                        let d = d_squared.sqrt();
                        
                        // Weight by distance (closer boids have more influence)
                        separation.x += (dx / d) / d;
                        separation.y += (dy / d) / d;
                        sep_count += 1;
                    }
                    
                    // Alignment
                    if d_squared < align_radius_sq {
                        alignment += boid_velocities[other_idx];
                        align_count += 1;
                    }
                    
                    // Cohesion
                    if d_squared < cohesion_radius_sq {
                        cohesion.x += boid_positions[other_idx].x;
                        cohesion.y += boid_positions[other_idx].y;
                        cohesion_count += 1;
                    }
                }
                
                // Process separation
                if sep_count > 0 {
                    separation /= sep_count as f32;
                    
                    let separation_length_squared = separation.length_squared();
                    if separation_length_squared > 0.0 {
                        // Implement Reynolds: Steering = Desired - Velocity
                        let separation_length = separation_length_squared.sqrt();
                        let desired = separation * (boid.max_speed / separation_length);
                        
                        separation = desired - boid.velocity;
                        
                        // Limit force
                        let force_squared = separation.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            separation *= boid.max_force / force_length;
                        }
                    }
                }
                
                // Process alignment
                if align_count > 0 {
                    alignment /= align_count as f32;
                    
                    let alignment_length_squared = alignment.length_squared();
                    if alignment_length_squared > 0.0 {
                        // Implement Reynolds: Steering = Desired - Velocity
                        let alignment_length = alignment_length_squared.sqrt();
                        let desired = alignment * (boid.max_speed / alignment_length);
                        
                        alignment = desired - boid.velocity;
                        
                        // Limit force
                        let force_squared = alignment.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            alignment *= boid.max_force / force_length;
                        }
                    }
                }
                
                // Process cohesion
                if cohesion_count > 0 {
                    cohesion /= cohesion_count as f32;
                    
                    // Create desired velocity towards target
                    let desired = cohesion - Vec2::new(boid.position.x, boid.position.y);
                    
                    let desired_length_squared = desired.length_squared();
                    if desired_length_squared > 0.0 {
                        // Scale to maximum speed (only normalize if needed)
                        let desired_length = desired_length_squared.sqrt();
                        let desired_normalized = desired * (boid.max_speed / desired_length);
                        
                        // Implement Reynolds: Steering = Desired - Velocity
                        let mut steering = desired_normalized - boid.velocity;
                        
                        // Limit force
                        let force_squared = steering.length_squared();
                        let max_force_squared = boid.max_force * boid.max_force;
                        
                        if force_squared > max_force_squared {
                            let force_length = force_squared.sqrt();
                            steering *= boid.max_force / force_length;
                        }
                        
                        cohesion = steering;
                    }
                }
                
                // Combine forces with weights
                let mut combined_force = Vec2::ZERO;
                combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
                combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
                
                // Apply the calculated acceleration
                boid.apply_force(combined_force);
                
                // Update position
                boid.update();
                
                // Wrap around edges
                boid.wrap_edges(WORLD_SIZE);
            }
        });
    } else {
        // Sequential processing for better cache locality
        for i in 0..model.boids.len() {
            let boid_position = model.boids[i].position;
            
            // Get nearby boids with pre-computed distances
            let nearby_with_distances = model.spatial_grid.get_nearby_with_distances(boid_position, &boid_positions, WORLD_SIZE);
            
            // Calculate forces
            let mut separation = Vec2::ZERO;
            let mut alignment = Vec2::ZERO;
            let mut cohesion = Vec2::ZERO;
            let mut sep_count = 0;
            let mut align_count = 0;
            let mut cohesion_count = 0;
            
            // Process all neighbors in a single pass
            for neighbor in nearby_with_distances {
                let other_idx = neighbor.index;
                let d_squared = neighbor.distance_squared;
                
                // Separation
                if d_squared < sep_radius_sq {
                    // Calculate vector pointing away from neighbor
                    let dx = boid_position.x - boid_positions[other_idx].x;
                    let dy = boid_position.y - boid_positions[other_idx].y;
                    
                    // Only calculate actual distance if needed for weighting
                    let d = d_squared.sqrt();
                    
                    // Weight by distance (closer boids have more influence)
                    separation.x += (dx / d) / d;
                    separation.y += (dy / d) / d;
                    sep_count += 1;
                }
                
                // Alignment
                if d_squared < align_radius_sq {
                    alignment += boid_velocities[other_idx];
                    align_count += 1;
                }
                
                // Cohesion
                if d_squared < cohesion_radius_sq {
                    cohesion.x += boid_positions[other_idx].x;
                    cohesion.y += boid_positions[other_idx].y;
                    cohesion_count += 1;
                }
            }
            
            // Process separation
            if sep_count > 0 {
                separation /= sep_count as f32;
                
                let separation_length_squared = separation.length_squared();
                if separation_length_squared > 0.0 {
                    // Implement Reynolds: Steering = Desired - Velocity
                    let separation_length = separation_length_squared.sqrt();
                    let desired = separation * (model.boids[i].max_speed / separation_length);
                    
                    separation = desired - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = separation.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        separation *= model.boids[i].max_force / force_length;
                    }
                }
            }
            
            // Process alignment
            if align_count > 0 {
                alignment /= align_count as f32;
                
                let alignment_length_squared = alignment.length_squared();
                if alignment_length_squared > 0.0 {
                    // Implement Reynolds: Steering = Desired - Velocity
                    let alignment_length = alignment_length_squared.sqrt();
                    let desired = alignment * (model.boids[i].max_speed / alignment_length);
                    
                    alignment = desired - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = alignment.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        alignment *= model.boids[i].max_force / force_length;
                    }
                }
            }
            
            // Process cohesion
            if cohesion_count > 0 {
                cohesion /= cohesion_count as f32;
                
                // Create desired velocity towards target
                let desired = cohesion - Vec2::new(boid_position.x, boid_position.y);
                
                let desired_length_squared = desired.length_squared();
                if desired_length_squared > 0.0 {
                    // Scale to maximum speed (only normalize if needed)
                    let desired_length = desired_length_squared.sqrt();
                    let desired_normalized = desired * (model.boids[i].max_speed / desired_length);
                    
                    // Implement Reynolds: Steering = Desired - Velocity
                    let mut steering = desired_normalized - model.boids[i].velocity;
                    
                    // Limit force
                    let force_squared = steering.length_squared();
                    let max_force_squared = model.boids[i].max_force * model.boids[i].max_force;
                    
                    if force_squared > max_force_squared {
                        let force_length = force_squared.sqrt();
                        steering *= model.boids[i].max_force / force_length;
                    }
                    
                    cohesion = steering;
                }
            }
            
            // Combine forces with weights
            let mut combined_force = Vec2::ZERO;
            combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
            combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
            
            // Apply the calculated acceleration
            model.boids[i].apply_force(combined_force);
            
            // Update position
            model.boids[i].update();
            
            // Wrap around edges
            model.boids[i].wrap_edges(WORLD_SIZE);
        }
    }
}

// Update boids without spatial grid (original O(n²) approach)
fn update_boids_without_spatial_grid(model: &mut Model) {
    // Create a copy of boids for the calculations
    let boids_clone = model.boids.clone();
    
    // Pre-calculate weights to avoid multiplication in the inner loop
    let separation_weight = model.params.separation_weight;
    let alignment_weight = model.params.alignment_weight;
    let cohesion_weight = model.params.cohesion_weight;
    
    // Use parallel processing if enabled
    if model.params.enable_parallel {
        // Calculate optimal chunk size based on available threads
        let chunk_size = std::cmp::max(model.boids.len() / rayon::current_num_threads(), 1);
        
        // Update debug info with chunk size if debug is enabled
        if model.params.show_debug {
            model.debug_info.chunk_size = chunk_size;
        }
        
        // Process boids in parallel chunks to reduce synchronization overhead
        model.boids.par_chunks_mut(chunk_size).for_each(|boid_chunk| {
            for boid in boid_chunk {
                // Calculate forces
                let separation = boid.separation_original(&boids_clone, model.params.separation_radius, model.params.enable_squared_distance);
                let alignment = boid.alignment_original(&boids_clone, model.params.alignment_radius, model.params.enable_squared_distance);
                let cohesion = boid.cohesion_original(&boids_clone, model.params.cohesion_radius, model.params.enable_squared_distance);
                
                // Combine forces with weights (avoid creating intermediate vectors)
                let mut combined_force = Vec2::ZERO;
                combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
                combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
                
                // Apply the calculated acceleration
                boid.apply_force(combined_force);
                
                // Update position
                boid.update();
                
                // Wrap around edges
                boid.wrap_edges(WORLD_SIZE);
            }
        });
    } else {
        // Sequential processing for when parallel is disabled
        for boid in &mut model.boids {
            // Calculate forces
            let separation = boid.separation_original(&boids_clone, model.params.separation_radius, model.params.enable_squared_distance);
            let alignment = boid.alignment_original(&boids_clone, model.params.alignment_radius, model.params.enable_squared_distance);
            let cohesion = boid.cohesion_original(&boids_clone, model.params.cohesion_radius, model.params.enable_squared_distance);
            
            // Combine forces with weights (avoid creating intermediate vectors)
            let mut combined_force = Vec2::ZERO;
            combined_force.x = separation.x * separation_weight + alignment.x * alignment_weight + cohesion.x * cohesion_weight;
            combined_force.y = separation.y * separation_weight + alignment.y * alignment_weight + cohesion.y * cohesion_weight;
            
            // Apply the calculated acceleration
            boid.apply_force(combined_force);
            
            // Update position
            boid.update();
            
            // Wrap around edges
            boid.wrap_edges(WORLD_SIZE);
        }
    }
} 