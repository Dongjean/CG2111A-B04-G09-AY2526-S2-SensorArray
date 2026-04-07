#!/usr/bin/env python3
"""
slam_process.py - SLAM background process.

Runs in a dedicated child process to perform SLAM so it has its own GIL.

Performance optimizations:
  - Scan resampling uses numpy for vectorized binning (no Python loop over
    hundreds of raw measurements per scan)
  - Map copy to shared memory done via numpy slice assignment (memcpy-speed)
"""

from __future__ import annotations

import math
import os
import signal
import time
from datetime import datetime
from typing import Optional

import numpy as np

from settings import (
    SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM,
    MAP_SIZE_PIXELS, MAP_SIZE_METERS, HOLE_WIDTH_MM, MAP_QUALITY,
    LIDAR_OFFSET_DEG, MIN_VALID_POINTS, INITIAL_ROUNDS_SKIP,
    MAP_UPDATE_INTERVAL, PATH_MIN_DISTANCE_MM,
    SAVE_DIR, AUTOSAVE_INTERVAL_S, SAVE_ON_EXIT,
)
from shared_state import ProcessSharedState


_SCAN_ANGLES: list[float] = [
    float(i * DETECTION_ANGLE / SCAN_SIZE)
    for i in range(SCAN_SIZE)
]


def _resample_scan(
    raw_angles: list[float],
    raw_distances: list[float],
) -> tuple[list[int], int]:
    """Resample raw LIDAR readings into SCAN_SIZE equal-angle bins.

    Vectorized with numpy: converts the raw lists to arrays, computes
    bin indices in bulk, and uses np.bincount for accumulation.
    """
    a = np.array(raw_angles, dtype=np.float64)
    d = np.array(raw_distances, dtype=np.float64)

    # Filter out invalid (zero/negative) distances.
    valid_mask = d > 0
    a = a[valid_mask]
    d = d[valid_mask]

    if a.size == 0:
        return [MAX_DISTANCE_MM] * SCAN_SIZE, 0

    # Convert CW RPLidar angles to CCW BreezySLAM convention + offset.
    ccw = -a + LIDAR_OFFSET_DEG
    bins = np.round(ccw).astype(np.intp) % SCAN_SIZE

    # Accumulate sums and counts per bin.
    bin_sums = np.bincount(bins, weights=d, minlength=SCAN_SIZE)
    bin_counts = np.bincount(bins, minlength=SCAN_SIZE)

    # Build the output: average where we have data, MAX_DISTANCE_MM otherwise.
    has_data = bin_counts > 0
    avg = np.where(has_data, bin_sums / np.maximum(bin_counts, 1), MAX_DISTANCE_MM)
    capped = np.where(avg >= MAX_DISTANCE_MM, MAX_DISTANCE_MM, avg)
    scan_distances = capped.astype(np.int64).tolist()

    # Valid = bins with data AND distance below max.
    valid = int(np.sum(has_data & (avg < MAX_DISTANCE_MM)))

    return scan_distances, valid


def _ensure_save_dir() -> str:
    """Create the save directory if it doesn't exist.  Returns the path."""
    base = os.path.dirname(os.path.abspath(__file__))
    save_path = os.path.join(base, SAVE_DIR)
    os.makedirs(save_path, exist_ok=True)
    return save_path


def _save_map_pgm(mapbytes: bytearray, save_path: str, tag: str,
                   x_mm: float = 0, y_mm: float = 0, theta_deg: float = 0,
                   path_points: list[tuple[float, float]] | None = None):
    """Save the occupancy map as a PGM file with a companion metadata .txt."""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    basename = f'map_{tag}_{timestamp}'

    pgm_path = os.path.join(save_path, f'{basename}.pgm')
    try:
        with open(pgm_path, 'wb') as f:
            header = f'P5\n{MAP_SIZE_PIXELS} {MAP_SIZE_PIXELS}\n255\n'
            f.write(header.encode('ascii'))
            f.write(mapbytes)
    except Exception as exc:
        print(f'[slam] Warning: could not save map to {pgm_path}: {exc}')
        return

    meta_path = os.path.join(save_path, f'{basename}.txt')
    try:
        with open(meta_path, 'w') as f:
            f.write(f'timestamp: {timestamp}\n')
            f.write(f'tag: {tag}\n')
            f.write(f'map_size_pixels: {MAP_SIZE_PIXELS}\n')
            f.write(f'map_size_meters: {MAP_SIZE_METERS}\n')
            f.write(f'robot_x_mm: {x_mm:.1f}\n')
            f.write(f'robot_y_mm: {y_mm:.1f}\n')
            f.write(f'robot_theta_deg: {theta_deg:.1f}\n')
            if path_points:
                f.write(f'path_points: {len(path_points)}\n')
                for px, py in path_points:
                    f.write(f'  {px:.1f},{py:.1f}\n')
    except Exception:
        pass

    print(f'[slam] Map saved: {pgm_path}')


def run_slam_process(pss: ProcessSharedState) -> None:
    """Entry point for the SLAM child process."""
    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
    except ImportError:
        pss.set_error('BreezySLAM not installed. Run: bash install_slam.sh')
        pss.stopped.value = True
        return

    try:
        import lidar as lidar_driver
    except ImportError:
        pss.set_error('lidar.py not found in the slam/ directory')
        pss.stopped.value = True
        return

    lidar = lidar_driver.connect()
    if lidar is None:
        from settings import LIDAR_PORT
        pss.set_error(f'Could not connect to LIDAR on {LIDAR_PORT}')
        pss.stopped.value = True
        return

    scan_mode = lidar_driver.get_scan_mode(lidar)

    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam = RMHC_SLAM(
        laser,
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        hole_width_mm=HOLE_WIDTH_MM,
        map_quality=MAP_QUALITY,
    )
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create a numpy view into the shared map memory for fast bulk copies.
    shm_array = np.ndarray(
        MAP_SIZE_PIXELS * MAP_SIZE_PIXELS,
        dtype=np.uint8,
        buffer=pss.shm.buf,
    )

    pss.set_status(f'connected (mode {scan_mode})')
    pss.connected.value = True

    save_path = _ensure_save_dir() if (AUTOSAVE_INTERVAL_S > 0 or SAVE_ON_EXIT) else None

    last_path_x = last_path_y = None

    def _exit_save(signum=None, frame=None):
        if save_path and SAVE_ON_EXIT:
            try:
                slam.getmap(mapbytes)
                path_pts = pss.get_path_points()
                _save_map_pgm(mapbytes, save_path, 'crash_recovery',
                              pss.x_mm.value, pss.y_mm.value,
                              pss.theta_deg.value, path_pts)
            except Exception:
                pass

    signal.signal(signal.SIGTERM, _exit_save)

    previous_distances: Optional[list[int]] = None
    round_num = 0
    last_map_update = time.monotonic()
    last_autosave = time.monotonic()

    try:
        for raw_angles, raw_distances in lidar_driver.scan_rounds(lidar, scan_mode):
            if pss.stop_event.is_set():
                break

            round_num += 1
            pss.rounds_seen.value = round_num

            if round_num <= INITIAL_ROUNDS_SKIP:
                pss.valid_points.value = 0
                pss.set_status(f'warming up {round_num}/{INITIAL_ROUNDS_SKIP}')
                continue

            if pss.paused.value:
                pss.set_status('paused')
                continue

            scan_distances, valid = _resample_scan(raw_angles, raw_distances)
            pss.valid_points.value = valid

            if valid >= MIN_VALID_POINTS:
                slam.update(scan_distances, scan_angles_degrees=_SCAN_ANGLES)
                previous_distances = list(scan_distances)
                note = f'live ({valid} pts)'
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=_SCAN_ANGLES)
                note = f'reusing previous ({valid} pts)'
            else:
                pss.set_status(f'waiting ({valid} pts)')
                continue

            x_mm, y_mm, theta_deg = slam.getpos()
            pss.x_mm.value = x_mm
            pss.y_mm.value = y_mm
            pss.theta_deg.value = theta_deg
            pss.pose_version.value += 1

            # Record path trail.
            if last_path_x is None:
                pss.add_path_point(x_mm, y_mm)
                last_path_x, last_path_y = x_mm, y_mm
            else:
                dx = x_mm - last_path_x
                dy = y_mm - last_path_y
                if dx * dx + dy * dy >= PATH_MIN_DISTANCE_MM * PATH_MIN_DISTANCE_MM:
                    pss.add_path_point(x_mm, y_mm)
                    last_path_x, last_path_y = x_mm, y_mm

            # Copy map to shared memory at throttled rate using fast numpy copy.
            now = time.monotonic()
            if now - last_map_update >= MAP_UPDATE_INTERVAL:
                slam.getmap(mapbytes)
                # numpy bulk copy — much faster than slice assignment on memoryview
                np.copyto(
                    shm_array,
                    np.frombuffer(mapbytes, dtype=np.uint8),
                )
                pss.map_version.value += 1
                last_map_update = now

            # Auto-save.
            if (save_path and AUTOSAVE_INTERVAL_S > 0
                    and now - last_autosave >= AUTOSAVE_INTERVAL_S):
                path_pts = pss.get_path_points()
                _save_map_pgm(mapbytes, save_path, 'autosave',
                              x_mm, y_mm, theta_deg, path_pts)
                last_autosave = now

            pss.set_status(note)

    except Exception as exc:
        pss.set_error(f'SLAM process error: {exc}')
        if save_path and SAVE_ON_EXIT:
            try:
                slam.getmap(mapbytes)
                path_pts = pss.get_path_points()
                _save_map_pgm(mapbytes, save_path, 'crash',
                              pss.x_mm.value, pss.y_mm.value,
                              pss.theta_deg.value, path_pts)
            except Exception:
                pass
    finally:
        if save_path and SAVE_ON_EXIT:
            try:
                slam.getmap(mapbytes)
                path_pts = pss.get_path_points()
                _save_map_pgm(mapbytes, save_path, 'exit',
                              pss.x_mm.value, pss.y_mm.value,
                              pss.theta_deg.value, path_pts)
            except Exception:
                pass
        try:
            lidar_driver.disconnect(lidar)
        except Exception:
            pass
        pss.connected.value = False
        pss.stopped.value = True
