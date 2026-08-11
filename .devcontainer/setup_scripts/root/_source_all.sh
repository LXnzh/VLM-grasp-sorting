#!/bin/bash
set -e
set -x

# Get the absolute path of the directory containing this script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

echo "Sourcing scripts with root privileges: $SCRIPT_DIR"

# Change to the script directory
cd "$SCRIPT_DIR"

# Loop through the found scripts and source each one
while IFS= read -r -d '' SCRIPT; do
    source "$SCRIPT"
    # Ensure the next script is resolved from the expected directory.
    cd "$SCRIPT_DIR"
done < <(find "$SCRIPT_DIR" -maxdepth 1 -type f -name '*.sh' ! -name "$(basename "$0")" -print0)
