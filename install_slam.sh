#!/bin/bash
# install_slam.sh
# Studio 16: Robot Integration
#
# Installs breezyslam and textual into the existing virtual environment.
# This script REQUIRES the env/ folder created by setup_environment.sh to
# already be present.  Run it from the same directory as your pi_sensor.py.
#
# Usage (from the folder that contains pi_sensor.py and env/):
#   bash install_slam.sh

set -e

echo '======================================='
echo '= Studio 16: Install breezyslam       ='
echo '======================================='
echo ''

# Change to the directory containing this script so relative paths work.
cd "$(dirname "$0")"
echo "Working directory: $(pwd)"
echo ''

# ------------------------------------------------------------------
# Require an existing virtual environment
# ------------------------------------------------------------------
# We do NOT create a new environment here.  The env/ folder must already
# exist (created by setup_environment.sh from the Sensor Mini-Project).

if [ ! -d "env" ]; then
    echo "ERROR: No 'env/' virtual environment found in $(pwd)."
    echo ''
    echo "This script must be run from the folder that contains your"
    echo "pi_sensor.py and env/ (created by setup_environment.sh)."
    echo ''
    echo "If you have not run setup_environment.sh yet, do that first:"
    echo "  bash setup_environment.sh"
    echo "then come back and run this script."
    exit 1
fi

# ------------------------------------------------------------------
# Activate the environment
# ------------------------------------------------------------------
echo "Activating virtual environment..."
source env/bin/activate

if [ -z "$VIRTUAL_ENV" ]; then
    echo "ERROR: Could not activate env/. Try running:"
    echo "  source env/bin/activate"
    echo "manually and check for errors."
    exit 1
fi

echo "Activated: $VIRTUAL_ENV"
echo ''

# ------------------------------------------------------------------
# Install BreezySLAM
# ------------------------------------------------------------------
# BreezySLAM is a lightweight SLAM library with a C extension.
# It is not reliably available on PyPI, so we install it from GitHub.

if ! python3 -c "import distutils" &>/dev/null; then
    echo "distutils not found, installing python3-distutils..."
    if command -v sudo &>/dev/null; then
        sudo apt-get update
        sudo apt-get install -y python3-distutils
    else
        apt-get update
        apt-get install -y python3-distutils
    fi
    echo ''
fi

# Ensure git exists because pip installs from a git+https URL.
if ! command -v git &>/dev/null; then
    echo "git not found, installing git..."
    if command -v sudo &>/dev/null; then
        sudo apt-get update
        sudo apt-get install -y git
    else
        apt-get update
        apt-get install -y git
    fi
    echo ''
fi

echo "Ensuring Python packages are available..."
python3 -m pip install --upgrade pip setuptools wheel

if python3 -c "import breezyslam" &>/dev/null; then
    echo "BreezySLAM is already installed."
else
    echo "Installing BreezySLAM from GitHub..."
    python3 -m pip install "git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python"
    echo ''
    echo "BreezySLAM installed successfully."
fi

if python3 -c "import textual" &>/dev/null; then
    echo "Textual is already installed."
else
    echo "Installing Textual..."
    python3 -m pip install textual
    echo ''
    echo "Textual installed successfully."
fi

echo ''
echo '======================================='
echo '= Done!                               ='
echo '======================================='
echo ''
echo "Activate the environment in any new terminal with:"
echo "  source env/bin/activate"
echo ''
echo "Then start the SLAM visualiser with:"
echo "  cd slam"
echo "  python3 slam.py"
