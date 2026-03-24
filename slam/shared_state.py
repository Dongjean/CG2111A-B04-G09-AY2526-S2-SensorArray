#!/usr/bin/env python3
"""
shared_state.py - Shared state objects used between the SLAM process and the UI.

ProcessSharedState holds the occupancy map and robot pose in structures that
can be safely read and written by two separate Python processes:
  - The SLAM process writes new pose estimates and map data as they are computed.
  - The UI process reads them to render the map in the terminal.

The map is stored in a multiprocessing.shared_memory block (1 MB for a
1000x1000 map) to avoid copying it across the process boundary on every frame.
All other values use multiprocessing.Value and multiprocessing.Array primitives
which are backed by shared memory and are safe to read/write from both processes.
"""

import ctypes
import multiprocessing
import multiprocessing.shared_memory

from settings import MAP_SIZE_PIXELS, UNKNOWN_BYTE


class ProcessSharedState:
    """Shared state between the SLAM process and the UI process.

    Create one instance in the UI (parent) process before spawning the SLAM
    (child) process, and pass it as an argument to the SLAM process function.
    Both processes can then read and write the fields directly.

    Fields
    ------
    shm          - shared memory buffer holding MAP_SIZE_PIXELS^2 bytes
                   (0=wall, 127=unknown, 255=free per BreezySLAM convention)
    x_mm         - robot x position in mm
    y_mm         - robot y position in mm
    theta_deg    - robot heading in degrees (BreezySLAM convention: CCW from +x)
    valid_points - number of valid LIDAR readings in the most recent scan
    rounds_seen  - total number of LIDAR rotations processed so far
    map_version  - incremented each time the map is updated (use to skip redraws)
    pose_version - incremented each time the pose is updated
    connected    - True once the LIDAR is connected and SLAM is running
    stopped      - True once the SLAM process has exited
    paused       - set to True from the UI to pause SLAM updates
    status_note  - short human-readable status string (up to 127 bytes)
    error_message - error description if the SLAM process encountered a problem
    stop_event   - set this from the UI to ask the SLAM process to exit
    """

    def __init__(self):
        # Allocate shared memory for the occupancy map.
        self.shm = multiprocessing.shared_memory.SharedMemory(
            create=True, size=MAP_SIZE_PIXELS * MAP_SIZE_PIXELS,
        )
        # Initialise all cells to "unknown".
        for i in range(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS):
            self.shm.buf[i] = UNKNOWN_BYTE

        # Robot pose (doubles for sub-mm precision).
        self.x_mm = multiprocessing.Value(ctypes.c_double, 0.0)
        self.y_mm = multiprocessing.Value(ctypes.c_double, 0.0)
        self.theta_deg = multiprocessing.Value(ctypes.c_double, 0.0)

        # Scan and update counters.
        self.valid_points = multiprocessing.Value(ctypes.c_int, 0)
        self.rounds_seen = multiprocessing.Value(ctypes.c_int, 0)
        self.map_version = multiprocessing.Value(ctypes.c_int, 0)
        self.pose_version = multiprocessing.Value(ctypes.c_int, 0)

        # Status flags.
        self.connected = multiprocessing.Value(ctypes.c_bool, False)
        self.stopped = multiprocessing.Value(ctypes.c_bool, False)
        self.paused = multiprocessing.Value(ctypes.c_bool, False)

        # Short text fields (fixed-size byte arrays for IPC safety).
        self.status_note = multiprocessing.Array(ctypes.c_char, 128)
        self.error_message = multiprocessing.Array(ctypes.c_char, 256)

        # Signal from the UI to ask the SLAM process to stop cleanly.
        self.stop_event = multiprocessing.Event()

    def set_status(self, msg: str):
        """Write a status string (truncated to 127 bytes)."""
        self.status_note.value = msg.encode('utf-8')[:127]

    def get_status(self) -> str:
        """Read the current status string."""
        return self.status_note.value.decode('utf-8', errors='replace')

    def set_error(self, msg: str):
        """Write an error string (truncated to 255 bytes)."""
        self.error_message.value = msg.encode('utf-8')[:255]

    def get_error(self) -> str:
        """Read the current error string (empty string if no error)."""
        val = self.error_message.value
        return val.decode('utf-8', errors='replace') if val else ''

    def cleanup(self):
        """Release the shared memory block.  Call this in the UI process
        after the SLAM process has exited."""
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass
