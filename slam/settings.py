#!/usr/bin/env python3
"""
settings.py - All user-configurable settings for the SLAM system.

Change the values in this file to tune the SLAM system for your robot.
The most common settings to change are LIDAR_PORT and LIDAR_OFFSET_DEG.
"""

# ===========================================================================
# LIDAR hardware settings
# ===========================================================================

# Serial port the RPLidar is plugged into.
# On the Raspberry Pi with a USB adapter this is usually /dev/ttyUSB0.
# If you have other USB devices connected it might be /dev/ttyUSB1, etc.
LIDAR_PORT = '/dev/ttyUSB0'

# Baud rate for the RPLidar A1M8. Do not change this.
LIDAR_BAUD = 115200

# ===========================================================================
# SLAM map settings
# ===========================================================================

# Side length of the square occupancy map in pixels.
# Larger values give finer spatial resolution but use more memory.
MAP_SIZE_PIXELS = 3000

# Real-world area the map covers, in metres.
# The map will span MAP_SIZE_METERS x MAP_SIZE_METERS metres.
MAP_SIZE_METERS = 8

# How aggressively new LIDAR scans update the map (1 = slow, 10 = fast).
# Lower values produce smoother maps; higher values react faster to changes.
MAP_QUALITY = 7

# Maximum gap (in mm) that BreezySLAM treats as a continuous wall.
HOLE_WIDTH_MM = 100

# ===========================================================================
# Scan settings
# ===========================================================================

# Number of angle bins BreezySLAM expects per 360-degree rotation.
SCAN_SIZE = 360

# Approximate rotation rate of the RPLidar A1M8 in scans per second.
SCAN_RATE_HZ = 5

# Field of view of the LIDAR in degrees (360 for a full-rotation sensor).
DETECTION_ANGLE = 360

# Readings beyond this distance (in mm) are treated as misses (no return).
MAX_DISTANCE_MM = 12000

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================

# Rotate all LIDAR readings by this many degrees before feeding them to SLAM.
# Use this to align the LIDAR's "forward" direction with the robot's forward
# direction.
#
# The raw RPLidar angles (clockwise) are first negated to convert to the
# counter-clockwise convention used by BreezySLAM, then this offset is added.
#
# Direction convention (counter-clockwise, viewed from above):
#   LIDAR_OFFSET_DEG = 0    - LIDAR forward (0 deg) = robot forward
#   LIDAR_OFFSET_DEG = 90   - LIDAR forward is 90 deg CCW from robot forward
#                              (LIDAR connector faces robot's right side)
#   LIDAR_OFFSET_DEG = 180  - LIDAR is mounted backwards
#   LIDAR_OFFSET_DEG = -90  - LIDAR forward is 90 deg CW from robot forward
#
# To find the correct value: stand behind the robot looking forward, and
# measure the CCW angle from the robot's forward to the LIDAR's forward.
#
# The default of 0 assumes the LIDAR's forward direction matches the robot's.
LIDAR_OFFSET_DEG = 0

# ===========================================================================
# Scan quality thresholds
# ===========================================================================

# Minimum number of valid distance readings in a scan for it to be used for
# a full SLAM update.  If a scan has fewer valid points we reuse the previous
# good scan instead.  Increase this if the map is noisy; decrease it if the
# robot is in a sparse environment.
MIN_VALID_POINTS = 150

# Number of scans to skip at startup.  The LIDAR motor needs a few rotations
# to reach full speed; early scans are noisier than steady-state scans.
INITIAL_ROUNDS_SKIP = 5

# ===========================================================================
# UI and rendering settings
# ===========================================================================

# How many times per second the terminal map refreshes.
# Lower values reduce CPU load; higher values give a smoother display.
UI_REFRESH_HZ = 2

# Maximum width and height of the rendered map in terminal cells.
# Reduce these if the display is too slow on your terminal emulator.
MAX_RENDER_COLS = 120
MAX_RENDER_ROWS = 45

# How often the map is copied from the SLAM process (times per second).
# Copying 1 MB of map data is relatively expensive; keep this low.
MAP_UPDATE_HZ = 1.0
MAP_UPDATE_INTERVAL = 1.0 / MAP_UPDATE_HZ

# Default zoom level index into ZOOM_HALF_M (0 = full map).
DEFAULT_ZOOM = 0

# How far to pan per key-press, as a fraction of the current view half-width.
PAN_STEP_FRACTION = 0.20

# Byte value written to uninitialised map cells.
# BreezySLAM uses 0 = wall, 127 = unknown, 255 = free.
UNKNOWN_BYTE = 127

# Zoom levels: None means show the full map.
# Otherwise the value is the half-width of the view window in metres,
# so zoom level 1 shows a MAP_SIZE_METERS/2 x MAP_SIZE_METERS/2 m window.
ZOOM_HALF_M = [
    None,
    MAP_SIZE_METERS / 2.0,
    MAP_SIZE_METERS / 3.0,
    MAP_SIZE_METERS / 5.0,
    MAP_SIZE_METERS / 8.0,
]
