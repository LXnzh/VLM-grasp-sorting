#!/bin/bash

# Define the source file and the destination directory
SOURCE_FILE="$1"
DEST_DIR="$HOME/.config/tmuxinator"

# Check if the source file is provided and exists
if [ -z "$SOURCE_FILE" ]; then
    echo "Usage: $0 <source-file>"
    exit 1
fi

if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: Source file '$SOURCE_FILE' does not exist."
    exit 1
fi

# Check if the destination directory exists, create if not
if [ ! -d "$DEST_DIR" ]; then
    echo "Destination directory '$DEST_DIR' does not exist. Creating it now."
    mkdir -p "$DEST_DIR"
fi

# Copy the file to the destination directory
cp "$SOURCE_FILE" "$DEST_DIR"

# Verify the copy operation
if [ $? -eq 0 ]; then
    echo "File '$SOURCE_FILE' has been successfully copied to '$DEST_DIR'."
else
    echo "Error: Failed to copy file."
    exit 1
fi
