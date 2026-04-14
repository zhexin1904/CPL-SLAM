#!/bin/bash

# Define input and output folders
INPUT_FOLDER="${1:-/home/jason/research/TRO_sub/baseline/landmark/CPL-SLAM/landmark}"
BASE_OUTPUT_FOLDER="4_13"
ITER_INFO_FOLDER="${BASE_OUTPUT_FOLDER}/iteration_info"
TRAJECTORY_FOLDER="${BASE_OUTPUT_FOLDER}/trajectory"

# Create output folders if they don't exist
mkdir -p "$ITER_INFO_FOLDER"
mkdir -p "$TRAJECTORY_FOLDER"

# Path to the cpl_slam executable
EXECUTABLE="./cmake-build-release/bin/cpl_slam"

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found. Please build the project first."
    exit 1
fi

# Find all .pyfg (or .g2o if appropriate) files and run the executable
# Note: The codebase seems to use read_g2o_file but the user mentioned .pyfg.
# CPL-SLAM often uses .pyfg files which are similar to g2o.
find "$INPUT_FOLDER" -name "*.pyfg" -o -name "*.g2o" | while read -r input_file; do
    # Create a unique filename based on the relative path to avoid collisions
    abs_input_folder=$(realpath "$INPUT_FOLDER")
    abs_input_file=$(realpath "$input_file")
    
    if [[ -d "$INPUT_FOLDER" ]]; then
        rel_path="${abs_input_file#$abs_input_folder/}"
    else
        rel_path=$(basename "$abs_input_file")
    fi
    
    result_base="${rel_path//\//_}"
    result_base="${result_base%.pyfg}"
    result_base="${result_base%.g2o}"
    
    iter_info_file="${ITER_INFO_FOLDER}/${result_base}.csv"
    trajectory_file="${TRAJECTORY_FOLDER}/${result_base}.g2o"

    echo "Processing $input_file"
    echo "  Iteration info: $iter_info_file"
    echo "  Trajectory:     $trajectory_file"
    
    # Run the cpl_slam executable with "SE" mode as requested
    "$EXECUTABLE" "$input_file" "SE" "$iter_info_file" "$trajectory_file"
    
    if [ $? -eq 0 ]; then
        echo "Successfully processed $input_file"
    else
        echo "Failed to process $input_file"
    fi
done

echo "Benchmarking complete. Results saved in $BASE_OUTPUT_FOLDER"
