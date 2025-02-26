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
}

impl Default for DebugInfo {
    fn default() -> Self {
        Self {
            fps: 0.0,
            frame_time: Duration::from_secs(0),
            visible_boids: Arc::new(Mutex::new(0)),
            chunk_size: 0,
        }
    }
} 