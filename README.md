# Boid Flocking Simulation

A Rust implementation of Craig Reynolds' Boid flocking algorithm with interactive controls.

## Overview

This application simulates the flocking behavior of birds (boids) based on three main rules:
1. **Separation**: Avoid crowding neighbors
2. **Alignment**: Steer towards the average heading of neighbors
3. **Cohesion**: Steer towards the average position of neighbors

The simulation includes interactive sliders to adjust parameters in real-time and displays debug information about the current state.

## Features

- Real-time visualization of boid flocking behavior
- Interactive UI with sliders to adjust simulation parameters:
  - Number of boids
  - Separation, alignment, and cohesion weights
  - Perception radii for each behavior
  - Maximum speed
- Debug visualization showing:
  - Perception radii for the first boid
  - Velocity vector
  - FPS and frame time
  - Number of boids
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

## Controls

- Use the UI sliders to adjust simulation parameters
- Toggle "Show Debug Info" to display debug visualization
- Toggle "Pause Simulation" to pause/resume the simulation
- Click "Reset Boids" to randomize boid positions

## Implementation Details

The simulation is built using:
- **nannou**: A creative coding framework for Rust
- **nannou_egui**: Integration of egui with nannou for the UI
- **rand**: For random number generation

The code is structured around:
- `Boid`: Represents an individual agent in the simulation
- `SimulationParams`: Contains adjustable parameters for the simulation
- `Model`: Main application state
- `DebugInfo`: Information displayed when debug mode is enabled

## License

MIT 