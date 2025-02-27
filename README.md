# Boid Flocking Simulation

A high-performance Rust implementation of Craig Reynolds' Boid flocking algorithm with interactive controls, spatial partitioning, and advanced optimization techniques.

## Overview

This application simulates the flocking behavior of birds (boids) based on three main rules:
1. **Separation**: Avoid crowding neighbors
2. **Alignment**: Steer towards the average heading of neighbors
3. **Cohesion**: Steer towards the average position of neighbors

The simulation is designed with performance in mind, capable of handling tens of thousands of boids through efficient spatial partitioning, parallel processing, and various optimization techniques.

## Features

- Real-time visualization of boid flocking behavior
- Interactive UI with sliders to adjust simulation parameters:
  - Number of boids (supports up to 200,000)
  - Separation, alignment, and cohesion weights
  - Perception radii for each behavior
  - Maximum speed
  - World size (1,000 to 50,000 units)
- Advanced performance optimizations:
  - Spatial partitioning grid for efficient neighbor lookups
  - Parallel processing using Rayon
  - Squared distance calculations to avoid expensive sqrt operations
  - Frustum culling to skip processing off-screen boids
  - Adaptive cell sizing based on boid density
  - Selective rendering (only render when changes occur)
  - Pre-computed distance information for neighbor calculations
  - Optimized empty cell handling with occupancy tracking
  - Lookup tables for wrapped cell coordinates
- Timing and rendering controls:
  - Configurable fixed physics update rate (30-240 FPS)
  - Configurable target rendering framerate
  - Interpolation between physics updates for smooth animation
- Camera controls for zooming and panning
- Boid selection and following:
  - Click on any boid to select it
  - Camera can follow selected boids automatically
  - Detailed information about selected boids
- Seamless world wrapping at boundaries
- Debug visualization showing:
  - Perception radii for the first boid
  - Velocity vector
  - FPS and frame time
  - Number of boids
  - Spatial grid statistics (occupied cells, max population)
  - Culling efficiency metrics
  - Physics update information
- Pause/resume functionality
- Reset boids to random positions

## Requirements

- Rust (latest stable version recommended)
- Cargo

## Installation

1. Clone this repository
2. Navigate to the project directory
3. Run the simulation:

```bash
cargo run --release
```

For maximum performance, use the release build with optimizations:

```bash
RUSTFLAGS="-C target-cpu=native" cargo run --release
```

## Controls

- **Mouse Controls**:
  - Scroll wheel: Zoom in/out
  - Click and drag: Pan camera
  - Click on a boid: Select and follow that boid
- **UI Controls**:
  - Use the sliders to adjust simulation parameters
  - Toggle "Show Debug Info" to display debug visualization
  - Toggle "Pause Simulation" to pause/resume the simulation
  - Toggle performance optimizations (parallel processing, spatial grid, etc.)
  - Click "Reset Boids" to randomize boid positions

## Architecture

The simulation is built with a modular architecture for better maintainability:

- **boid.rs**: Represents individual agents in the simulation with interpolation support
- **spatial_grid.rs**: Implements spatial partitioning for efficient neighbor lookups with adaptive optimization
- **params.rs**: Contains adjustable parameters with change detection for efficient updates
- **physics.rs**: Handles the physics update loop and force calculations with parallel processing
- **renderer.rs**: Manages rendering of boids and debug information with selective rendering
- **camera.rs**: Implements camera controls for zooming, panning, and boid following
- **culling.rs**: Implements frustum culling for performance optimization
- **ui.rs**: Manages the user interface using egui
- **debug.rs**: Handles debug visualization and performance metrics
- **input.rs**: Processes user input for camera control and boid selection
- **app.rs**: Manages the application state and main loop

## Performance Benchmarking

The project includes benchmarks to measure performance of key operations:

```bash
cargo bench
```

This will run benchmarks for:
- Spatial grid operations
- Force calculations (separation, alignment, cohesion)
- Overall update loop

The benchmarks test different numbers of boids (100, 500, 1000, 2000) to measure how the simulation scales.

## Dependencies

- **nannou**: A creative coding framework for Rust
- **nannou_egui**: Integration of egui with nannou for the UI
- **rand**: For random number generation
- **rayon**: For parallel processing
- **criterion**: For benchmarking (dev dependency)

## License

MIT 