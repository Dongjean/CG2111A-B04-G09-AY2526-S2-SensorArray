#!/usr/bin/env python3
"""
shared_state.py - Shared state objects used between the SLAM process and the UI.

ProcessSharedState holds the occupancy map, robot pose, and path trail in
structures that can be safely read and written by two separate Python processes.

Performance optimizations:
  - Map shared memory initialized with numpy (memset) instead of byte-by-byte
  - Path points read via numpy for bulk deserialization
  - Zero-copy map access via numpy view of shared memory buffer
"""

import ctypes
import multiprocessing
import multiprocessing.shared_memory
import struct

import numpy as np

from settings import MAP_SIZE_PIXELS, UNKNOWN_BYTE, MAX_PATH_POINTS


class ProcessSharedState:
    """Shared state between the SLAM process and the UI process.

    Create one instance in the UI (parent) process before spawning the SLAM
    (child) process, and pass it as an argument to the SLAM process function.
    Both processes can then read and write the fields directly.
    """

    # Each path point is two doubles (x_mm, y_mm) = 16 bytes.
    _PATH_POINT_SIZE = struct.calcsize('dd')

    def __init__(self):
        map_size = MAP_SIZE_PIXELS * MAP_SIZE_PIXELS

        # Allocate shared memory for the occupancy map.
        self.shm = multiprocessing.shared_memory.SharedMemory(
            create=True, size=map_size,
        )
        # Fast initialization using numpy (memset-equivalent).
        np.ndarray(map_size, dtype=np.uint8, buffer=self.shm.buf)[:] = UNKNOWN_BYTE

        # Allocate shared memory for the path trail.
        self.path_shm = multiprocessing.shared_memory.SharedMemory(
            create=True, size=MAX_PATH_POINTS * self._PATH_POINT_SIZE,
        )
        self.path_count = multiprocessing.Value(ctypes.c_int, 0)

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

    def add_path_point(self, x_mm: float, y_mm: float):
        """Append a path point to the trail buffer.

        If the buffer is full, the oldest half of points are discarded to
        make room (keeps the most recent trail).
        """
        count = self.path_count.value
        if count >= MAX_PATH_POINTS:
            half = MAX_PATH_POINTS // 2
            src_offset = half * self._PATH_POINT_SIZE
            length = (MAX_PATH_POINTS - half) * self._PATH_POINT_SIZE
            data = bytes(self.path_shm.buf[src_offset:src_offset + length])
            self.path_shm.buf[:length] = data
            count = MAX_PATH_POINTS - half
        offset = count * self._PATH_POINT_SIZE
        struct.pack_into('dd', self.path_shm.buf, offset, x_mm, y_mm)
        self.path_count.value = count + 1

    def get_path_points_numpy(self) -> np.ndarray:
        """Read all path points as a numpy array of shape (N, 2).

        Returns a float64 array of [[x_mm, y_mm], ...].
        Much faster than the Python-loop version for large point counts.
        Returns an empty (0, 2) array if no points.
        """
        count = self.path_count.value
        if count <= 0:
            return np.empty((0, 2), dtype=np.float64)
        nbytes = count * self._PATH_POINT_SIZE
        raw = np.frombuffer(
            bytes(self.path_shm.buf[:nbytes]), dtype=np.float64
        )
        return raw.reshape(count, 2)

    def get_path_points(self) -> list[tuple[float, float]]:
        """Read all path points from the trail buffer (list version).

        Used by the save routines where a list is more convenient.
        """
        arr = self.get_path_points_numpy()
        if arr.shape[0] == 0:
            return []
        return [(float(r[0]), float(r[1])) for r in arr]

    def get_map_numpy_view(self) -> np.ndarray:
        """Return a numpy view directly into the shared map memory.

        This is ZERO-COPY — no 9MB memcpy.  The returned array is only
        valid while the shared memory is alive.  Read what you need quickly.

        Shape: (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype uint8.
        """
        return np.ndarray(
            (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS),
            dtype=np.uint8,
            buffer=self.shm.buf,
        )

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
        """Release the shared memory blocks.  Call this in the UI process
        after the SLAM process has exited."""
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass
        try:
            self.path_shm.close()
            self.path_shm.unlink()
        except Exception:
            pass
