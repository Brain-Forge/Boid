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
 */

use nannou::prelude::*;
use nannou::winit::event::{MouseButton, MouseScrollDelta, TouchPhase};

use crate::app::Model;

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
pub fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
    if button == MouseButton::Left {
        // Check if the click is on the UI before starting camera drag
        if !model.egui.ctx().is_pointer_over_area() {
            model.camera.start_drag(model.mouse_position);
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