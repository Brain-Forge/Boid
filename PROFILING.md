# Profiling the Boid Simulation

This document provides instructions for profiling the Boid simulation to identify performance bottlenecks.

## Built-in Performance Metrics

The simulation includes built-in performance metrics that are displayed in the UI:

- **FPS**: Frames per second
- **Frame Time**: Time taken to process a single frame
- **Update Time**: Time taken to update the simulation state
- **Spatial Grid Time**: Time taken to build and maintain the spatial grid
- **Force Calculation Time**: Time taken to calculate forces (separation, alignment, cohesion)
- **Position Update Time**: Time taken to update boid positions
- **Render Time**: Time taken to render the boids
- **Total Boids**: Total number of boids in the simulation
- **Visible Boids**: Number of boids currently visible on screen
- **Avg Neighbors**: Average number of neighbors per boid

These metrics can be viewed in the "Performance Tuning" section of the UI.

## Profiling with macOS Instruments

macOS Instruments is a powerful profiling tool that can help identify CPU, memory, and other performance bottlenecks.

### Running Instruments

1. Make sure you have Xcode and Xcode Command Line Tools installed.
2. Run the provided script:

```bash
./run_instruments.sh
```

This will:
- Build the project in release mode
- Launch Instruments with the Time Profiler template
- Start profiling the application

3. Use the application normally to generate profiling data.
4. When done, press Ctrl+C in Instruments to stop profiling.
5. The profiling data will be saved in the `profiling` directory.

### Analyzing Instruments Results

1. Open the generated trace file in Instruments.
2. Use the Time Profiler to identify functions that consume the most CPU time.
3. Look for:
   - Hot spots in the code (functions that consume a lot of CPU time)
   - Unexpected CPU usage in functions that should be efficient
   - Patterns of inefficient function calls

## Profiling with Flamegraph

Flamegraph provides a visual representation of CPU usage, making it easy to identify hot spots in the code.

### Running Flamegraph

1. Make sure you have the `flamegraph` tool installed (`cargo install flamegraph`).
2. Run the provided script:

```bash
./run_flamegraph.sh
```

This will:
- Build the project in release mode
- Launch the application with profiling enabled
- Generate a flamegraph SVG file

3. Use the application normally for a minute or so, then close it.
4. The flamegraph will be saved in the `profiling` directory.

### Analyzing Flamegraph Results

1. Open the generated SVG file in a web browser.
2. The x-axis represents the stack profile population, sorted alphabetically.
3. The y-axis shows the stack depth.
4. Each rectangle represents a function in the stack.
5. The wider the rectangle, the more time was spent in that function.
6. Look for wide rectangles, especially at the bottom of the graph, as these represent hot spots in the code.

## Benchmarking with Criterion

The project includes benchmarks using Criterion to measure the performance of specific components.

### Running Benchmarks

```bash
cargo bench
```

This will run the benchmarks and generate HTML reports in the `target/criterion` directory.

### Analyzing Benchmark Results

1. Open the HTML reports in a web browser.
2. Look for:
   - Performance differences between different numbers of boids
   - Performance differences between different algorithms (spatial grid vs. brute force)
   - Unexpected performance regressions

## Common Performance Bottlenecks

Based on the nature of the boid simulation, here are some common performance bottlenecks to look for:

1. **Neighbor Lookup**: The O(n²) complexity of finding neighbors without spatial partitioning.
2. **Force Calculations**: The calculations for separation, alignment, and cohesion can be expensive.
3. **Memory Allocation**: Excessive allocation and deallocation of memory during the simulation.
4. **Rendering**: Drawing a large number of boids can be expensive, especially with complex shapes.
5. **UI Updates**: Updating the UI with debug information can impact performance.

## Optimization Strategies

After identifying bottlenecks, consider these optimization strategies:

1. **Spatial Partitioning**: Ensure the spatial grid is properly tuned (cell size, etc.).
2. **Parallel Processing**: Use Rayon for parallel processing of boids.
3. **View Frustum Culling**: Only process boids that are visible or near the visible area.
4. **Memory Pooling**: Reuse memory allocations instead of allocating new memory each frame.
5. **SIMD Instructions**: Use SIMD instructions for vector operations.
6. **Reduce Debug Overhead**: Minimize the amount of debug information collected and displayed.
7. **Optimize Rendering**: Use simpler shapes or instanced rendering for boids. 