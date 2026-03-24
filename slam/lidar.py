#!/usr/bin/env python3
"""
lidar.py - LIDAR hardware driver for the RPLidar A1M8.

Provides three functions used by the SLAM process:
  connect()      - open the serial port, reset the sensor, and start the motor
  get_scan_mode() - return the recommended scan mode for this sensor model
  scan_rounds()  - yield one complete 360-degree scan per motor rotation
  disconnect()   - stop the motor and close the serial port

The LIDAR_PORT and LIDAR_BAUD settings live in settings.py.
"""

import time
from pyrplidar import PyRPlidar

from settings import LIDAR_PORT, LIDAR_BAUD


def connect(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Open the LIDAR serial port, reset the sensor, and start the motor.

    Performing a reset before starting ensures the sensor is in a clean state
    even if it was left running from a previous session.

    Returns the PyRPlidar object on success, or None if the connection fails
    (e.g. wrong port or the device is not plugged in).
    """
    try:
        lidar = PyRPlidar()
        # Connect, reset, wait for the sensor to restart, then reconnect.
        lidar.connect(port=port, baudrate=baudrate, timeout=10)
        lidar.reset()
        time.sleep(2)
        lidar.disconnect()
        lidar.connect(port=port, baudrate=baudrate, timeout=10)
        lidar.set_motor_pwm(500)
        return lidar
    except Exception as exc:
        print(f'[lidar] Could not connect on {port}: {exc}')
        return None


def get_scan_mode(lidar):
    """Return the recommended scan mode index for this LIDAR model.

    Falls back to mode 2 (a safe default for the A1M8) if the query fails.
    """
    try:
        return lidar.get_scan_mode_typical()
    except Exception:
        return 2


def scan_rounds(lidar, mode):
    """Yield one complete 360-degree scan per motor rotation.

    Each yielded value is a (angles, distances) tuple containing two
    parallel lists:
      angles    - float degrees, 0.0 to 360.0
      distances - float mm (0 means no return / out of range)

    The generator runs until the LIDAR is disconnected or an exception is
    raised by the underlying PyRPlidar library.
    """
    buff = []
    started = False
    for meas in lidar.start_scan_express(mode)():
        if meas.start_flag:
            # A start_flag marks the beginning of a new rotation.
            # Yield the completed buffer from the previous rotation.
            if started and buff:
                yield [m.angle for m in buff], [m.distance for m in buff]
            buff = [meas]
            started = True
        elif started:
            buff.append(meas)


def disconnect(lidar):
    """Stop the LIDAR motor and close the serial connection."""
    try:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
    except Exception:
        pass
