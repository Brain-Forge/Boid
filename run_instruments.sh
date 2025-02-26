#!/bin/bash

# Script to run macOS Instruments on the Boid simulation
# This script builds the project in release mode and then runs Instruments

# Set variables
APP_NAME="boids"
BUILD_DIR="target/release"
INSTRUMENTS_APP="/Applications/Xcode.app/Contents/Applications/Instruments.app"
TRACE_DIR="profiling"

# Create profiling directory if it doesn't exist
mkdir -p "$TRACE_DIR"

# Build the project in release mode
echo "Building project in release mode..."
cargo build --release

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# Check if Instruments is available
if [ ! -d "$INSTRUMENTS_APP" ]; then
    echo "Instruments application not found at $INSTRUMENTS_APP"
    echo "Please make sure Xcode is installed."
    exit 1
fi

# Get the full path to the executable
EXECUTABLE_PATH="$(pwd)/$BUILD_DIR/$APP_NAME"

echo "Running Instruments..."
echo "Please follow these steps:"
echo "1. Instruments will open. Select 'Time Profiler' from the templates."
echo "2. Click 'Choose Target' and select 'Choose Target...' from the dropdown."
echo "3. Navigate to: $EXECUTABLE_PATH"
echo "4. Click 'Record' to start profiling."
echo "5. Use the application normally for a minute or so."
echo "6. Click 'Stop' to end profiling."
echo "7. Save the trace file to the '$TRACE_DIR' directory."

# Open Instruments
open "$INSTRUMENTS_APP"

echo "When you're done profiling, save the trace file to the '$TRACE_DIR' directory." 