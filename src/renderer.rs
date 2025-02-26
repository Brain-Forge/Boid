/*
 * Renderer Module
 * 
 * This module handles the rendering of the boid simulation.
 * It draws the boids, world boundaries, and debug information.
 * 
 * Optimized for performance by:
 * - Only rendering visible boids (frustum culling)
 * - Skipping rendering when nothing has changed
 * - Using interpolation for smooth animation
 */

use nannou::prelude::*;

use crate::app::Model;
use crate::culling;
use crate::ui;
use crate::WORLD_SIZE;

// Render the model
pub fn view(app: &App, model: &Model, frame: Frame) {
    // Skip rendering if not needed (when paused and nothing has changed)
    let render_needed = unsafe { *model.render_needed.get() };
    if !render_needed {
        // Only draw the UI
        model.egui.draw_to_frame(&frame).unwrap();
        return;
    }
    
    // Begin drawing
    let draw = app.draw();
    
    // Clear the background
    draw.background().color(BLACK);
    
    // Get the window rectangle
    let window_rect = app.window_rect();
    
    // Draw world boundary to show the simulation limits
    let world_top_left = model.camera.world_to_screen(vec2(-WORLD_SIZE/2.0, -WORLD_SIZE/2.0), window_rect);
    let world_bottom_right = model.camera.world_to_screen(vec2(WORLD_SIZE/2.0, WORLD_SIZE/2.0), window_rect);
    
    let world_rect = Rect::from_corners(
        pt2(world_top_left.x, world_top_left.y),
        pt2(world_bottom_right.x, world_bottom_right.y)
    );
    
    draw.rect()
        .xy(world_rect.xy())
        .wh(world_rect.wh())
        .no_fill()
        .stroke_weight(1.0)
        .stroke(rgba(0.3, 0.3, 0.3, 1.0));
    
    // Calculate the visible area in world space for culling
    let visible_area = Rect::from_corners(
        pt2(
            model.camera.screen_to_world(pt2(window_rect.left(), window_rect.bottom()), window_rect).x,
            model.camera.screen_to_world(pt2(window_rect.left(), window_rect.bottom()), window_rect).y
        ),
        pt2(
            model.camera.screen_to_world(pt2(window_rect.right(), window_rect.top()), window_rect).x,
            model.camera.screen_to_world(pt2(window_rect.right(), window_rect.top()), window_rect).y
        )
    );
    
    // Add a margin to the visible area (scaled by zoom level)
    let margin = crate::BOID_SIZE * 2.0 / model.camera.zoom;
    let visible_area_with_margin = Rect::from_corners(
        pt2(visible_area.left() - margin, visible_area.bottom() - margin),
        pt2(visible_area.right() + margin, visible_area.top() + margin)
    );
    
    // Calculate frustum area ratio for debug info
    if model.params.show_debug {
        let world_area = WORLD_SIZE * WORLD_SIZE;
        let frustum_area = visible_area_with_margin.w() * visible_area_with_margin.h();
        let area_ratio = frustum_area / world_area;
        
        let mut frustum_area_ratio = model.debug_info.frustum_area_ratio.lock().unwrap();
        *frustum_area_ratio = area_ratio;
    }
    
    // Get visible boids based on culling settings
    let visible_boids_indices = if model.params.enable_frustum_culling {
        // Get visible boids using the most efficient method available
        culling::get_visible_boids(model, visible_area_with_margin)
    } else {
        // If culling is disabled, render all boids
        (0..model.boids.len()).collect()
    };
    
    // Track visible boid count and calculate culling efficiency for debug info
    if model.params.show_debug {
        let visible_count = visible_boids_indices.len();
        let total_count = model.boids.len();
        
        // Update visible boid count
        let mut visible_boids_count = model.debug_info.visible_boids.lock().unwrap();
        *visible_boids_count = visible_count;
        
        // Calculate and update culling efficiency
        if total_count > 0 {
            let efficiency = (1.0 - (visible_count as f32 / total_count as f32)) * 100.0;
            let mut culling_efficiency = model.debug_info.culling_efficiency.lock().unwrap();
            *culling_efficiency = efficiency;
        }
    }
    
    // Draw each visible boid with interpolation
    for &i in &visible_boids_indices {
        // Check if this is the selected boid
        let is_selected = model.selected_boid_index.map_or(false, |selected| selected == i);
        
        // Draw the boid, passing the selection state
        model.boids[i].draw(&draw, &model.camera, window_rect, model.interpolation_alpha, is_selected);
    }
    
    // Draw debug visualization if enabled
    if model.params.show_debug {
        // Draw frustum culling visualization if enabled
        if model.params.enable_frustum_culling {
            // Convert the visible area with margin to screen space for visualization
            let top_left = model.camera.world_to_screen(
                vec2(visible_area_with_margin.left(), visible_area_with_margin.top()), 
                window_rect
            );
            let bottom_right = model.camera.world_to_screen(
                vec2(visible_area_with_margin.right(), visible_area_with_margin.bottom()), 
                window_rect
            );
            
            // Draw the frustum culling boundary
            draw.rect()
                .xy(pt2((top_left.x + bottom_right.x) / 2.0, (top_left.y + bottom_right.y) / 2.0))
                .wh(vec2(bottom_right.x - top_left.x, bottom_right.y - top_left.y))
                .no_fill()
                .stroke_weight(2.0)
                .stroke(rgba(1.0, 0.5, 0.0, 0.7)); // Orange for frustum boundary
        }
        
        // Draw perception radius for the first boid if it's visible
        if !model.boids.is_empty() {
            let first_boid = &model.boids[0];
            
            // Get interpolated position for debug visualization
            let interpolated_pos = if model.params.enable_interpolation {
                first_boid.get_interpolated_position(model.interpolation_alpha)
            } else {
                first_boid.position
            };
            
            if visible_area_with_margin.contains(Vec2::new(interpolated_pos.x, interpolated_pos.y)) {
                let screen_pos = model.camera.world_to_screen(Vec2::new(interpolated_pos.x, interpolated_pos.y), window_rect);
                
                // Scale radii based on zoom level
                let sep_radius = model.params.separation_radius * model.camera.zoom;
                let align_radius = model.params.alignment_radius * model.camera.zoom;
                let cohesion_radius = model.params.cohesion_radius * model.camera.zoom;
                
                // Separation radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(sep_radius)
                    .no_fill()
                    .stroke(RED)
                    .stroke_weight(1.0);
                
                // Alignment radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(align_radius)
                    .no_fill()
                    .stroke(GREEN)
                    .stroke_weight(1.0);
                
                // Cohesion radius
                draw.ellipse()
                    .xy(pt2(screen_pos.x, screen_pos.y))
                    .radius(cohesion_radius)
                    .no_fill()
                    .stroke(BLUE)
                    .stroke_weight(1.0);
                
                // Get interpolated velocity for debug visualization
                let interpolated_vel = if model.params.enable_interpolation {
                    first_boid.get_interpolated_velocity(model.interpolation_alpha)
                } else {
                    first_boid.velocity
                };
                
                // Velocity vector
                draw.arrow()
                    .start(pt2(screen_pos.x, screen_pos.y))
                    .end(pt2(
                        screen_pos.x + interpolated_vel.x * 5.0 * model.camera.zoom,
                        screen_pos.y + interpolated_vel.y * 5.0 * model.camera.zoom
                    ))
                    .color(YELLOW)
                    .stroke_weight(2.0);
            }
        }
        
        // Draw debug info
        ui::draw_debug_info(&draw, &model.debug_info, window_rect, model.boids.len(), model.camera.zoom, WORLD_SIZE);
    }
    
    // Finish drawing
    draw.to_frame(app, &frame).unwrap();
    
    // If simulation is paused, mark rendering as complete
    if model.params.pause_simulation {
        unsafe { *model.render_needed.get() = false; }
    }
    
    // Draw the egui UI
    model.egui.draw_to_frame(&frame).unwrap();
} 