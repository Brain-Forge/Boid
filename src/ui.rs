/*
 * UI Module
 * 
 * This module contains functions for creating and updating the user interface
 * using nannou_egui. It provides controls for adjusting simulation parameters.
 */

use nannou_egui::{egui, Egui};
use crate::params::SimulationParams;
use crate::debug::DebugInfo;

// Update the UI and return whether boids should be reset, number of boids changed, and if any UI changes occurred
pub fn update_ui(
    egui: &mut Egui, 
    params: &mut SimulationParams, 
    debug_info: &DebugInfo
) -> (bool, bool, bool) {
    let mut should_reset_boids = false;
    let mut num_boids_changed = false;
    let mut ui_changed = false;
    
    let old_num_boids = params.num_boids;
    let old_show_debug = params.show_debug;
    let old_pause_simulation = params.pause_simulation;
    let old_separation_weight = params.separation_weight;
    let old_alignment_weight = params.alignment_weight;
    let old_cohesion_weight = params.cohesion_weight;
    let old_separation_radius = params.separation_radius;
    let old_alignment_radius = params.alignment_radius;
    let old_cohesion_radius = params.cohesion_radius;
    let old_max_speed = params.max_speed;
    
    let ctx = egui.begin_frame();
    
    egui::Window::new("Simulation Controls")
        .default_pos([10.0, 10.0])
        .show(&ctx, |ui| {
            ui.collapsing("Boid Parameters", |ui| {
                ui.add(egui::Slider::new(&mut params.num_boids, 10..=10000).text("Number of Boids"));
                if params.num_boids != old_num_boids {
                    num_boids_changed = true;
                    ui_changed = true;
                }
                
                if ui.button("Reset Boids").clicked() {
                    should_reset_boids = true;
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.max_speed, 1.0..=100.0).text("Max Speed"));
                if params.max_speed != old_max_speed {
                    ui_changed = true;
                }
            });
            
            ui.collapsing("Flocking Behavior", |ui| {
                ui.add(egui::Slider::new(&mut params.separation_weight, 0.0..=3.0).text("Separation Weight"));
                if params.separation_weight != old_separation_weight {
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.alignment_weight, 0.0..=3.0).text("Alignment Weight"));
                if params.alignment_weight != old_alignment_weight {
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.cohesion_weight, 0.0..=3.0).text("Cohesion Weight"));
                if params.cohesion_weight != old_cohesion_weight {
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.separation_radius, 10.0..=100.0).text("Separation Radius"));
                if params.separation_radius != old_separation_radius {
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.alignment_radius, 10.0..=100.0).text("Alignment Radius"));
                if params.alignment_radius != old_alignment_radius {
                    ui_changed = true;
                }
                
                ui.add(egui::Slider::new(&mut params.cohesion_radius, 10.0..=100.0).text("Cohesion Radius"));
                if params.cohesion_radius != old_cohesion_radius {
                    ui_changed = true;
                }
            });
            
            ui.collapsing("Camera Controls", |ui| {
                ui.label("Zoom: Use mouse wheel or trackpad pinch gesture");
                ui.label("Pan: Click and drag or use trackpad with two fingers");
                if ui.button("Reset Camera").clicked() {
                    // We'll handle this in the app module
                }
                ui.label(format!("Zoom Level: {:.2}x", 1.0)); // Placeholder, will be updated in app
                ui.label(format!("Camera Position: ({:.0}, {:.0})", 0.0, 0.0)); // Placeholder
            });
            
            ui.collapsing("Performance Tuning", |ui| {
                ui.checkbox(&mut params.enable_parallel, "Enable Parallel Processing");
                ui.checkbox(&mut params.enable_spatial_grid, "Enable Spatial Grid");
                ui.add(egui::Slider::new(&mut params.cell_size_factor, 0.5..=2.0).text("Cell Size Factor"));
                
                ui.separator();
                
                // Performance metrics
                ui.label(format!("FPS: {:.1}", debug_info.fps));
                ui.label(format!("Frame time: {:.2} ms", debug_info.frame_time.as_secs_f64() * 1000.0));
                ui.label(format!("Total Boids: {}", params.num_boids));
                ui.label(format!("Visible Boids: {}", *debug_info.visible_boids.lock().unwrap()));
            });
            
            ui.checkbox(&mut params.show_debug, "Show Debug Info");
            if params.show_debug != old_show_debug {
                ui_changed = true;
            }
            
            ui.checkbox(&mut params.pause_simulation, "Pause Simulation");
            if params.pause_simulation != old_pause_simulation {
                ui_changed = true;
            }
        });
    
    (should_reset_boids, num_boids_changed, ui_changed)
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
    let panel_width = 200.0;
    let panel_height = line_height * 6.0 + margin;
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
    
    // Draw each line of text
    let debug_texts = [
        format!("FPS: {:.1}", debug_info.fps),
        format!("Frame time: {:.2} ms", debug_info.frame_time.as_secs_f64() * 1000.0),
        format!("Total Boids: {}", boids_len),
        format!("Visible Boids: {}", *debug_info.visible_boids.lock().unwrap()),
        format!("Zoom: {:.2}x", camera_zoom),
        format!("World Size: {:.0}x{:.0}", world_size, world_size),
    ];
    
    for (i, text) in debug_texts.iter().enumerate() {
        let y = text_y - (i as f32 * line_height);
        
        // Position the text with a fixed offset from the left edge
        draw.text(text)
            .x_y(text_x + 70.0, y)
            .color(nannou::color::WHITE)
            .font_size(14);
    }
} 