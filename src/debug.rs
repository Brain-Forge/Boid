/*
 * Debug Information Module
 * 
 * This module defines the DebugInfo struct that contains performance metrics
 * and other debug information to be displayed in the UI.
 * 
 * Includes metrics for:
 * - FPS (frames per second)
 * - Frame time
 * - Number of visible boids
 * - Parallel processing chunk size
 */

use std::time::Duration;
use std::sync::{Arc, Mutex};

// Debug information to display
pub struct DebugInfo {
    pub fps: f32,
    pub frame_time: Duration,
    pub visible_boids: Arc<Mutex<usize>>,
    pub chunk_size: usize,
    pub physics_updates_per_frame: usize,
    pub interpolation_alpha: f32,
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::ZERO,
            visible_boids: Arc::new(Mutex::new(0)),
            chunk_size: 0,
            physics_updates_per_frame: 0,
            interpolation_alpha: 0.0,
        }
    }
} 