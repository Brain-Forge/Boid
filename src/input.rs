/*
 * Input Module
 * 
 * This module handles user input events for the boid simulation.
 * It processes mouse movements, clicks, and wheel events for camera control.
 * 
 * Features:
 * - Camera panning with mouse drag
 * - Camera zooming with mouse wheel
 * - Handling UI interaction
 * - Boid selection and camera following
 */

use nannou::prelude::*;
use nannou::winit::event::{MouseButton, MouseScrollDelta, TouchPhase};

use crate::app::Model;
use crate::BOID_SIZE;

// Mouse moved event handler
pub fn mouse_moved(_app: &App, model: &mut Model, pos: Point2) {
    let new_pos = Vec2::new(pos.x, pos.y);
    
    // Update camera drag if we're dragging
    if model.camera.is_dragging {
        model.camera.drag(new_pos);
        // Clear the cached visible boids and force re-render when panning
        unsafe { *model.cached_visible_boids.get() = None; }
        unsafe { *model.render_needed.get() = true; }
    }
    
    // Always update the stored mouse position
    model.mouse_position = new_pos;
}

// Mouse pressed event handler
pub fn mouse_pressed(app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        // Check if the click is on the UI before handling it
        if !model.egui.ctx().is_pointer_over_area() {
            // Get the window rectangle for coordinate transformations
            let window_rect = app.window_rect();
            
            // Convert mouse position from screen space to world space
            let world_pos = model.camera.screen_to_world(model.mouse_position, window_rect);
            
            // Check if we clicked on a boid
            let mut clicked_boid = None;
            let selection_radius = BOID_SIZE * 2.0; // Make the selection area a bit larger than the boid
            
            // Get visible boids to check for selection
            let visible_boids = if let Some(cached) = unsafe { &*model.cached_visible_boids.get() } {
                cached.clone()
            } else {
                // If no cached visible boids, check all boids
                (0..model.boids.len()).collect()
            };
            
            // Check each visible boid
            for &boid_idx in &visible_boids {
                let boid = &model.boids[boid_idx];
                
                // Get interpolated position for accurate selection
                let boid_pos = boid.get_interpolated_position(model.interpolation_alpha);
                let distance_squared = (boid_pos.x - world_pos.x).powi(2) + (boid_pos.y - world_pos.y).powi(2);
                
                // Check if the click is within the selection radius
                if distance_squared <= selection_radius.powi(2) {
                    clicked_boid = Some(boid_idx);
                    break;
                }
            }
            
            if let Some(boid_idx) = clicked_boid {
                // We clicked on a boid
                model.selected_boid_index = Some(boid_idx);
                model.camera.follow_mode = true;
                
                // Force re-render to show the selection
                unsafe { *model.render_needed.get() = true; }
            } else {
                // We didn't click on a boid, start camera drag
                model.camera.start_drag(model.mouse_position);
                
                // If we were following a boid, stop following
                if model.camera.follow_mode {
                    model.camera.follow_mode = false;
                    // Keep the selected boid highlighted but don't follow it
                }
            }
        }
    }
}

// Mouse released event handler
pub fn mouse_released(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        model.camera.end_drag();
    }
}

// Mouse wheel event handler for zooming
pub fn mouse_wheel(_app: &App, model: &mut Model, delta: MouseScrollDelta, _phase: TouchPhase) {
    match delta {
        MouseScrollDelta::LineDelta(x, y) => {
            // Handle trackpad pinch gestures and mouse wheel
            let window_rect = _app.window_rect();
            model.camera.zoom(vec2(x, y), model.mouse_position, window_rect);
        },
        MouseScrollDelta::PixelDelta(pos) => {
            // Handle pixel delta (less common)
            let window_rect = _app.window_rect();
            model.camera.zoom(vec2(pos.x as f32, pos.y as f32) * 0.01, model.mouse_position, window_rect);
        },
    }
    
    // Clear the cached visible boids and force re-render when zooming
    unsafe { *model.cached_visible_boids.get() = None; }
    unsafe { *model.render_needed.get() = true; }
    // Also clear the visible area cache
    model.visible_area_cache = None;
}

// Handle raw window events for egui and camera dragging
pub fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Pass events to egui
    model.egui.handle_raw_event(event);
    
    // Force re-render when UI is interacted with
    if let nannou::winit::event::WindowEvent::MouseInput { .. } = event {
        unsafe { *model.render_needed.get() = true; }
    }
} 