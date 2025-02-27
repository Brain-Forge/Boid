/*
 * UI Module
 * 
 * This module handles the user interface for the boid simulation.
 * It provides controls for adjusting simulation parameters and displays
 * debug information when enabled.
 */

use nannou::prelude::*;
use nannou_egui::egui;
use crate::app::Model;
use crate::params::SimulationParams;
use crate::debug::DebugInfo;

// UI response structure
pub struct UiResponse {
    // reset_boids field removed
}

// Update the UI
pub fn update_ui(app: &App, model: &mut Model, update: &Update) -> UiResponse {
    // reset_boids variable removed
    
    // Begin UI frame
    let ctx = model.egui.begin_frame();
    
    // Create a window for the UI
    egui::Window::new("Simulation Controls")
        .default_pos([20.0, 20.0])
        .show(&ctx, |ui| {
            ui.heading("Boid Parameters");
            
            // Number of boids slider - direct control without mapping
            ui.add(egui::Slider::new(&mut model.params.num_boids, *SimulationParams::get_num_boids_range().start()..=*SimulationParams::get_num_boids_range().end())
                .text("Number of Boids")
                .clamp_to_range(true));
            
            // Weights
            ui.add(egui::Slider::new(&mut model.params.separation_weight, SimulationParams::get_weight_range())
                .text("Separation Weight")
                .clamp_to_range(true));
            
            ui.add(egui::Slider::new(&mut model.params.alignment_weight, SimulationParams::get_weight_range())
                .text("Alignment Weight")
                .clamp_to_range(true));
            
            ui.add(egui::Slider::new(&mut model.params.cohesion_weight, SimulationParams::get_weight_range())
                .text("Cohesion Weight")
                .clamp_to_range(true));
            
            // Perception radii
            ui.add(egui::Slider::new(&mut model.params.separation_radius, SimulationParams::get_radius_range())
                .text("Separation Radius")
                .clamp_to_range(true));
            
            ui.add(egui::Slider::new(&mut model.params.alignment_radius, SimulationParams::get_radius_range())
                .text("Alignment Radius")
                .clamp_to_range(true));
            
            ui.add(egui::Slider::new(&mut model.params.cohesion_radius, SimulationParams::get_radius_range())
                .text("Cohesion Radius")
                .clamp_to_range(true));
            
            // Max speed
            ui.add(egui::Slider::new(&mut model.params.max_speed, SimulationParams::get_max_speed_range())
                .text("Max Speed")
                .clamp_to_range(true));
            
            // Reset button removed
            
            ui.separator();
            
            ui.heading("Performance Settings");
            
            // Spatial grid toggle
            ui.checkbox(&mut model.params.enable_spatial_grid, "Enable Spatial Grid");
            
            if model.params.enable_spatial_grid {
                // Cell size factor
                ui.add(egui::Slider::new(&mut model.params.cell_size_factor, SimulationParams::get_cell_size_factor_range())
                    .text("Cell Size Factor")
                    .clamp_to_range(true));
                
                // Adaptive cell sizing
                ui.checkbox(&mut model.params.adaptive_cell_sizing, "Adaptive Cell Sizing");
                
                if model.params.adaptive_cell_sizing {
                    ui.label(format!("Current Cell Size: {:.1}", model.spatial_grid.cell_size));
                }
            }
            
            // Parallel processing toggle
            ui.checkbox(&mut model.params.enable_parallel, "Enable Parallel Processing");
            
            // Squared distance toggle
            ui.checkbox(&mut model.params.enable_squared_distance, "Use Squared Distances");
            
            // Frustum culling toggle
            ui.checkbox(&mut model.params.enable_frustum_culling, "Enable Frustum Culling");
            
            ui.separator();
            
            ui.heading("Timing Settings");
            
            // Physics FPS
            ui.add(egui::Slider::new(&mut model.params.fixed_physics_fps, SimulationParams::get_physics_fps_range())
                .text("Physics FPS")
                .clamp_to_range(true));
            
            // Target render FPS
            ui.add(egui::Slider::new(&mut model.params.target_render_fps, SimulationParams::get_render_fps_range())
                .text("Target Render FPS (0 = unlimited)")
                .clamp_to_range(true));
            
            // Interpolation toggle
            ui.checkbox(&mut model.params.enable_interpolation, "Enable Interpolation");
            
            ui.separator();
            
            // Debug info toggle
            ui.checkbox(&mut model.params.show_debug, "Show Debug Info");
            
            // Pause toggle
            ui.checkbox(&mut model.params.pause_simulation, "Pause Simulation");
            
            // Display debug info if enabled
            if model.params.show_debug {
                ui.separator();
                ui.heading("Debug Info");
                
                ui.label(format!("FPS: {:.1}", app.fps()));
                ui.label(format!("Frame Time: {:.2} ms", update.since_last.as_secs_f32() * 1000.0));
                
                let debug_info = unsafe { &*model.debug_info.get() };
                
                if let Some(chunk_size) = debug_info.chunk_size {
                    ui.label(format!("Chunk Size: {}", chunk_size));
                }
                
                if let Some(selected_boid) = debug_info.selected_boid_index {
                    ui.label(format!("Selected Boid: {}", selected_boid));
                    
                    if debug_info.follow_mode_active {
                        ui.label("Follow Mode: Active");
                    }
                }
                
                if let Some(visible_count) = debug_info.visible_boids_count {
                    ui.label(format!("Visible Boids: {}/{}", visible_count, model.boids.len()));
                }
                
                if let Some(physics_updates) = debug_info.physics_updates_per_frame {
                    ui.label(format!("Physics Updates: {}/frame", physics_updates));
                }
                
                if let Some(alpha) = debug_info.interpolation_alpha {
                    ui.label(format!("Interpolation: {:.3}", alpha));
                }
                
                if let Some(culling_efficiency) = debug_info.culling_efficiency {
                    ui.label(format!("Culling Efficiency: {:.1}%", culling_efficiency));
                }
                
                if let Some(frustum_ratio) = debug_info.frustum_area_ratio {
                    ui.label(format!("Frustum/World Ratio: {:.2}%", frustum_ratio * 100.0));
                }
            }
        });
    
    UiResponse {
        // reset_boids field removed
    }
}

// Draw debug information on the screen
pub fn draw_debug_info(
    draw: &nannou::Draw, 
    debug_info: &DebugInfo, 
    window_rect: nannou::geom::Rect, 
    boids_len: usize,
    camera_zoom: f32,
    world_size: f32
) {
    // Create a background panel in the top-left corner
    let margin = 20.0;
    let line_height = 20.0;
    let panel_width = 250.0;
    let panel_height = line_height * 13.0 + margin; // Increased for additional lines
    let panel_x = window_rect.left() + panel_width / 2.0;
    let panel_y = window_rect.top() - panel_height / 2.0;
    
    // Draw the background panel
    draw.rect()
        .x_y(panel_x, panel_y)
        .w_h(panel_width, panel_height)
        .color(nannou::color::rgba(0.0, 0.0, 0.0, 0.7));
    
    // For left-aligned text in nannou, we need to position each text element
    // at the left edge of our panel plus half the text's width
    let text_x = window_rect.left() + margin;
    let text_y = window_rect.top() - margin;
    
    // Get culling metrics
    let culling_efficiency = debug_info.culling_efficiency.unwrap_or(0.0);
    let frustum_area_ratio = debug_info.frustum_area_ratio.unwrap_or(0.0);
    
    // Draw each line of text
    let mut debug_texts = vec![
        format!("FPS: {:.1}", debug_info.fps),
        format!("Frame time: {:.2} ms", debug_info.frame_time.as_secs_f64() * 1000.0),
        format!("Physics updates: {}", debug_info.physics_updates_per_frame.unwrap_or(0)),
        format!("Interpolation: {:.3}", debug_info.interpolation_alpha.unwrap_or(0.0)),
        format!("Total Boids: {}", boids_len),
        format!("Visible Boids: {}", debug_info.visible_boids_count.unwrap_or(0)),
        format!("Culling Efficiency: {:.1}%", culling_efficiency),
        format!("Frustum/World Ratio: {:.2}%", frustum_area_ratio * 100.0),
        format!("Camera Zoom: {:.2}x", camera_zoom),
        format!("World Size: {:.0}x{:.0}", world_size, world_size),
    ];
    
    // Add selected boid information
    if let Some(boid_idx) = debug_info.selected_boid_index {
        debug_texts.push(format!("Selected Boid: #{}", boid_idx));
        debug_texts.push(if debug_info.follow_mode_active {
            "Camera: Following boid".to_string()
        } else {
            "Camera: Free movement".to_string()
        });
    } else {
        debug_texts.push("No boid selected".to_string());
        debug_texts.push("Click on a boid to select it".to_string());
    }
    
    // Draw all debug text lines
    for (i, text) in debug_texts.iter().enumerate() {
        draw.text(text)
            .left_justify()
            .x_y(text_x, text_y - i as f32 * line_height)
            .color(nannou::color::WHITE);
    }
} 