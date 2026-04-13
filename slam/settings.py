#!/usr/bin/env python3
"""
settings.py - All user-configurable settings for the SLAM system.

Change the values in this file to tune the SLAM system for your robot.
The most common settings to change are LIDAR_PORT and LIDAR_OFFSET_DEG.
"""

# ===========================================================================
# LIDAR hardware settings
# ===========================================================================

LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

# ===========================================================================
# SLAM map settings
# ===========================================================================

MAP_SIZE_PIXELS = 8000
MAP_SIZE_METERS = 6
MAP_QUALITY = 4
HOLE_WIDTH_MM = 400

# ===========================================================================
# Scan settings
# ===========================================================================

SCAN_SIZE = 360
SCAN_RATE_HZ = 5
DETECTION_ANGLE = 360
MAX_DISTANCE_MM = 8000

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================

# Rotate all LIDAR readings by this many degrees before feeding them to SLAM.
# Direction convention (counter-clockwise, viewed from above):
#   LIDAR_OFFSET_DEG = 0    - LIDAR forward = robot forward
#   LIDAR_OFFSET_DEG = 90   - LIDAR connector faces robot's right side
#   LIDAR_OFFSET_DEG = 180  - LIDAR is mounted backwards
#   LIDAR_OFFSET_DEG = -90  - LIDAR forward is 90 deg CW from robot forward
LIDAR_OFFSET_DEG = 0

# ===========================================================================
# Scan quality thresholds
# ===========================================================================

MIN_VALID_POINTS = 150
INITIAL_ROUNDS_SKIP = 5

# ===========================================================================
# UI and rendering settings
# ===========================================================================

# How many times per second the terminal map refreshes.
# 6 Hz gives smooth visual updates without overwhelming the Pi's CPU.
UI_REFRESH_HZ = 6

# Maximum width and height of the rendered map in terminal cells.
MAX_RENDER_COLS = 120
MAX_RENDER_ROWS = 45

# How often the map is copied from the SLAM process (times per second).
# 4 Hz gives near-realtime map updates.  The map copy is now much cheaper
# because we use a zero-copy numpy view instead of bytes().
MAP_UPDATE_HZ = 4.0
MAP_UPDATE_INTERVAL = 1.0 / MAP_UPDATE_HZ

DEFAULT_ZOOM = 0
PAN_STEP_FRACTION = 0.20
UNKNOWN_BYTE = 127

ZOOM_HALF_M = [
    None,
    MAP_SIZE_METERS / 2.0,
    MAP_SIZE_METERS / 3.0,
    MAP_SIZE_METERS / 5.0,
    MAP_SIZE_METERS / 8.0,
]

# ===========================================================================
# Path trail settings
# ===========================================================================

MAX_PATH_POINTS = 5000
PATH_MIN_DISTANCE_MM = 15.0

# ===========================================================================
# Auto-save settings
# ===========================================================================

SAVE_DIR = 'saved_maps'
AUTOSAVE_INTERVAL_S = 30
SAVE_ON_EXIT = True
