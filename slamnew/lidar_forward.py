#!/usr/bin/env python3
"""
lidar_forward.py - TCP server that streams resampled LiDAR scan data to clients.

Runs as a daemon thread alongside slam.py. Reads the resampled scan from
shared_state (written by slam_process.py) and forwards it as JSON to any
connected client on port 5002.

Client receives: {"angles": [0..359], "distances": [int, ...], "version": int}
"""

import json
import socket
import threading
import time

from settings import SCAN_SIZE

FORWARD_PORT = 5002
FORWARD_HOST = ''   # listen on all interfaces


def _angles():
    """Return the fixed angle list (0..359 degrees)."""
    return list(range(SCAN_SIZE))


def start_forwarder(pss):
    """Start the LiDAR forwarder TCP server in a daemon thread.

    Args:
        pss: ProcessSharedState instance (must already be initialised).
    """
    t = threading.Thread(target=_serve, args=(pss,), daemon=True, name='lidar-forward')
    t.start()
    return t


def _serve(pss):
    angles = _angles()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((FORWARD_HOST, FORWARD_PORT))
        server.listen(1)
        server.settimeout(1.0)
        print(f'[lidar_forward] Listening on port {FORWARD_PORT}')
    except OSError as e:
        print(f'[lidar_forward] Could not bind port {FORWARD_PORT}: {e}')
        return

    while not pss.stop_event.is_set():
        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        except Exception:
            break
        print(f'[lidar_forward] Client connected from {addr}')
        _handle_client(conn, pss, angles)
        print(f'[lidar_forward] Client disconnected')

    server.close()


def _handle_client(conn, pss, angles):
    last_version = -1
    try:
        while not pss.stop_event.is_set():
            version = pss.scan_version.value
            if version == last_version:
                time.sleep(0.05)
                continue
            last_version = version
            distances = list(pss.scan_distances)
            payload = json.dumps({
                'angles':    angles,
                'distances': distances,
                'version':   version,
            }) + '\n'
            conn.sendall(payload.encode('utf-8'))
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        conn.close()