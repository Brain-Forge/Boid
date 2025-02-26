/*
 * Camera Module
 * 
 * This module defines the Camera struct that handles zooming and panning
 * in the simulation. It provides coordinate transformations between
 * world space and screen space.
 */

use nannou::prelude::*;

pub struct Camera {
    pub position: Vec2,
    pub zoom: f32,
    pub drag_start: Option<Vec2>,
    pub min_zoom: f32,
    pub max_zoom: f32,
    pub is_dragging: bool,
    pub last_cursor_pos: Vec2,
}

impl Camera {
    pub fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            zoom: 1.0,
            drag_start: None,
            min_zoom: 0.1,
            max_zoom: 5.0,
            is_dragging: false,
            last_cursor_pos: Vec2::ZERO,
        }
    }

    // Convert a point from world space to screen space
    pub fn world_to_screen(&self, point: Vec2, window_rect: Rect) -> Vec2 {
        // Apply zoom and translation
        let zoomed = (point - self.position) * self.zoom;
        // Convert to screen coordinates
        zoomed + window_rect.xy()
    }

    // Convert a point from screen space to world space
    pub fn screen_to_world(&self, point: Vec2, window_rect: Rect) -> Vec2 {
        // Convert from screen coordinates
        let centered = point - window_rect.xy();
        // Apply inverse zoom and translation
        centered / self.zoom + self.position
    }

    // Handle mouse wheel events for zooming
    pub fn zoom(&mut self, scroll_delta: Vec2, cursor_position: Vec2, window_rect: Rect) {
        // Calculate zoom factor based on scroll amount
        let zoom_factor = 1.0 + scroll_delta.y * 0.1;
        
        // Calculate cursor position in world space before zoom
        let cursor_world_before = self.screen_to_world(cursor_position, window_rect);
        
        // Apply zoom, clamping to min/max values
        self.zoom = (self.zoom * zoom_factor).clamp(self.min_zoom, self.max_zoom);
        
        // Calculate cursor position in world space after zoom
        let cursor_world_after = self.screen_to_world(cursor_position, window_rect);
        
        // Adjust camera position to keep cursor over the same world point
        self.position += cursor_world_before - cursor_world_after;
    }

    // Start dragging the camera
    pub fn start_drag(&mut self, position: Vec2) {
        // Only set the drag start position, don't move the camera yet
        self.drag_start = Some(position);
        self.last_cursor_pos = position;
        self.is_dragging = true;
    }

    // Update camera position while dragging
    pub fn drag(&mut self, position: Vec2) {
        if self.is_dragging {
            // Calculate drag delta from the last position (not the start position)
            let delta = position - self.last_cursor_pos;
            
            // Only apply movement if there's actually a change
            if delta.length_squared() > 0.0 {
                self.position -= delta / self.zoom;
                self.last_cursor_pos = position;
            }
        }
    }

    // End dragging
    pub fn end_drag(&mut self) {
        self.drag_start = None;
        self.is_dragging = false;
    }
} 