#!/bin/bash

LOGFILE="setup_environment.log"

exec > >(tee -i $LOGFILE)
exec 2>&1

############################################
############# CHECKING FOLDERS #############
############################################
# This script is meant to be run from the code folder of the Sensor Mini-Project studio (13_SensorMiniProject).
# It checks if the pyrplidar folder exists (required to install the library).
# If it doesn't, it exits with an error message.

echo '==============================='
echo '= Setting up the environment ='
echo '==============================='
echo ''

echo "Changing directory to the location of the script..."
cd "$(dirname "$0")"
echo "Directory changed to $(pwd)"

# Make sure the pyrplidar library folder is present
if [ ! -d "pyrplidar" ]; then
    echo "ERROR: The pyrplidar folder does not exist. Is this script in the right location?"
    echo "Expected: $(pwd)/pyrplidar"
    echo "Exiting..."
    exit 1
fi

echo ''
echo ''
echo ''

############################################
############# INSTALLING PYTHON ############
############################################
# We check if python and the python3-dev python3-venv packages are installed.
# If they are not, we install them.

echo '==============================='
echo '= Installing Python          ='
echo '==============================='
echo ''

# Skip the python install if it is already installed
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Installing..."
    sudo apt-get install -y python3
    echo ''
fi

# Install pip
# Skip the python3-pip install if it is already installed
if ! dpkg -s python3-pip &> /dev/null; then
    echo "python3-pip is not installed. Installing..."
    sudo apt-get install -y python3-pip
    echo ''
fi

# Skip the python3-venv install if it is already installed
if ! dpkg -s python3-venv &> /dev/null; then
    echo "python3-venv is not installed. Installing..."
    sudo apt-get install -y python3-venv
    echo ''
fi

# Skip the python3-dev install if it is already installed
if ! dpkg -s python3-dev &> /dev/null; then
    echo "python3-dev is not installed. Installing..."
    sudo apt-get install -y python3-dev
    echo ''
fi

# Install TK addon for PIL
if ! dpkg -s python3-pil.imagetk &> /dev/null; then
    echo "python3-pil.imagetk is not installed. Installing..."
    sudo apt-get install -y python3-pil.imagetk
    echo ''
fi

echo ''
echo ''
echo ''

##################################################
############ Virtual Environment Setup ###########
##################################################
# We create a virtual environment in the code folder.
# We provide access to the system site packages to allow for the use of -dev packages
# (e.g. picamera2, which is pre-installed on Raspberry Pi OS).

echo '==============================='
echo '= Installing Dependencies    ='
echo '==============================='
echo ''

# Check if the env folder exists; if it does, skip creation
if [ ! -d "env" ]; then
    echo "Creating virtual environment..."
    python3 -m venv ./env --system-site-packages
    echo "Virtual environment created."
    echo ''
fi

# Activate the virtual environment
source env/bin/activate

# Check if in the virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "ERROR: Virtual environment not activated. Exiting..."
    exit 1
fi
# Update pip to the latest version
python -m pip install --upgrade pip
echo ''

##################################################
############ INSTALLING PYTHON DEPS ##############
##################################################
# Install dependencies in the virtual environment.
#
# NOTE: numpy and picamera2 are intentionally NOT installed here via pip.
# picamera2 is a system package on Raspberry Pi OS and its C extensions
# (e.g. simplejpeg) are compiled against the system numpy.  Installing a
# different numpy version via pip causes a binary incompatibility:
#   ValueError: numpy.dtype size changed, may indicate binary incompatibility.
# Because the venv was created with --system-site-packages, both numpy and
# picamera2 are already available from the system installation.
pip install matplotlib pyserial
echo ''

##################################################
############ INSTALLING EXTERNAL DEPS ############
##################################################
# Install the pyrplidar library from the local copy.
if ! pip show pyrplidar &> /dev/null; then
    echo "Installing pyrplidar..."
    pip install -e ./pyrplidar --config-settings editable_mode=compat
    echo ''
fi


# If numpy/picamera2 were accidentally installed into the venv with pip,
# remove them so the system ABI-compatible builds are used instead.
for pkg in numpy picamera2; do
    pkg_location=$(pip show "$pkg" 2>/dev/null | awk -F': ' '/^Location:/{print $2}')
    if [ -n "$pkg_location" ] && [[ "$pkg_location" == "$VIRTUAL_ENV"* ]]; then
        echo "Removing venv-installed $pkg to avoid binary incompatibility..."
        pip uninstall -y "$pkg"
        echo ''
    fi
done


# Return control to the user
echo "Setup complete. To activate the environment, run:"
echo "  source env/bin/activate"
exit 0
