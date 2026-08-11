#!/bin/bash
set -e

CONDA_DIR="/opt/conda"

# Download Miniconda installer
wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh

# Install Miniconda
bash /tmp/miniconda.sh -b -p "$CONDA_DIR"

# Remove installer
rm /tmp/miniconda.sh

# Accept terms of service for required channels
"$CONDA_DIR/bin/conda" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
"$CONDA_DIR/bin/conda" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main

# Update conda
"$CONDA_DIR/bin/conda" update -y conda

# Link conda.sh to profile.d
ln -s "$CONDA_DIR/etc/profile.d/conda.sh" /etc/profile.d/conda.sh

# Clean up
"$CONDA_DIR/bin/conda" clean -afy

# Add conda to PATH for current session
export PATH="$CONDA_DIR/bin:$PATH"