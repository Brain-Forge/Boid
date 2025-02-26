/*
 * Boid Simulation Benchmark
 * 
 * This file contains benchmarks for the boid simulation to identify performance bottlenecks.
 * It measures the performance of key operations like spatial partitioning, force calculations,
 * and the overall update loop.
 */

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rand::Rng;
use std::time::Duration;

// Import the necessary types from the main crate
// Note: We need to make these public in the main.rs file
// This is a placeholder - you'll need to modify main.rs to expose these types
// extern crate boids;

// Benchmark the spatial grid operations
fn bench_spatial_grid(c: &mut Criterion) {
    let mut group = c.benchmark_group("spatial_grid");
    
    // Benchmark different numbers of boids
    for num_boids in [100, 500, 1000, 2000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(num_boids), num_boids, |b, &n| {
            // Setup code here - create boids and spatial grid
            // This is a placeholder - you'll need to modify this to use your actual types
            let mut rng = rand::thread_rng();
            let world_size = 5000.0;
            
            // Create boids with random positions
            let boids: Vec<(f32, f32)> = (0..n)
                .map(|_| {
                    let x = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    let y = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    (x, y)
                })
                .collect();
            
            b.iter(|| {
                // Benchmark the spatial grid operations
                // This is a placeholder - you'll need to modify this to use your actual code
                black_box(boids.clone());
            });
        });
    }
    
    group.finish();
}

// Benchmark the force calculations (separation, alignment, cohesion)
fn bench_force_calculations(c: &mut Criterion) {
    let mut group = c.benchmark_group("force_calculations");
    
    // Benchmark different numbers of boids
    for num_boids in [100, 500, 1000, 2000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(num_boids), num_boids, |b, &n| {
            // Setup code here - create boids
            // This is a placeholder - you'll need to modify this to use your actual types
            let mut rng = rand::thread_rng();
            let world_size = 5000.0;
            
            // Create boids with random positions
            let boids: Vec<(f32, f32)> = (0..n)
                .map(|_| {
                    let x = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    let y = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    (x, y)
                })
                .collect();
            
            b.iter(|| {
                // Benchmark the force calculations
                // This is a placeholder - you'll need to modify this to use your actual code
                black_box(boids.clone());
            });
        });
    }
    
    group.finish();
}

// Benchmark the overall update loop
fn bench_update_loop(c: &mut Criterion) {
    let mut group = c.benchmark_group("update_loop");
    
    // Benchmark different numbers of boids
    for num_boids in [100, 500, 1000, 2000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(num_boids), num_boids, |b, &n| {
            // Setup code here - create model
            // This is a placeholder - you'll need to modify this to use your actual types
            let mut rng = rand::thread_rng();
            let world_size = 5000.0;
            
            // Create boids with random positions
            let boids: Vec<(f32, f32)> = (0..n)
                .map(|_| {
                    let x = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    let y = rng.gen_range((-world_size / 2.0)..(world_size / 2.0));
                    (x, y)
                })
                .collect();
            
            b.iter(|| {
                // Benchmark the update loop
                // This is a placeholder - you'll need to modify this to use your actual code
                black_box(boids.clone());
            });
        });
    }
    
    group.finish();
}

// Configure the benchmarks
criterion_group! {
    name = benches;
    config = Criterion::default()
        .sample_size(10)
        .measurement_time(Duration::from_secs(5))
        .warm_up_time(Duration::from_secs(1));
    targets = bench_spatial_grid, bench_force_calculations, bench_update_loop
}

criterion_main!(benches); 