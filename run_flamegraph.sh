#!/bin/bash

# Script to generate a flamegraph for the Boid simulation
# This script builds the project in release mode and then runs flamegraph

# Set variables
OUTPUT_DIR="profiling"
OUTPUT_NAME="boid_flamegraph_$(date +%Y%m%d_%H%M%S).svg"

# Create profiling directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Check if flamegraph is installed
if ! command -v cargo-flamegraph &> /dev/null; then
    echo "cargo-flamegraph not found. Installing..."
    cargo install flamegraph
fi

# Run flamegraph
echo "Running flamegraph..."
echo "The application will start and profiling will begin."
echo "Use the application normally for a minute or so, then close it."
echo "You will be prompted for your password to run with elevated permissions."

# Run flamegraph with the release build using --root flag
cargo flamegraph --root --bin boids -o "$OUTPUT_DIR/$OUTPUT_NAME"

echo "Flamegraph generation complete. Output saved to $OUTPUT_DIR/$OUTPUT_NAME"
echo "Open the SVG file in a browser to view the flamegraph." 