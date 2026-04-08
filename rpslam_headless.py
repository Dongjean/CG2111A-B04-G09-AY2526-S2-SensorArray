#!/usr/bin/env python3
"""
rpslam_headless.py  -  BreezySLAM for CG2111A Alex robot, NO display needed.

Streams the live map to your laptop browser. Nothing is written to disk.

Dependencies
------------
  pip install breezyslam pyrplidar pillow

Run
---
  source env/bin/activate
  python3 rpslam_headless.py
  # then open http://<pi-ip>:8080 on your laptop
"""

import time
import math
import argparse
import threading
import io
import socket

from pyrplidar import PyRPlidar
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from http.server import HTTPServer, BaseHTTPRequestHandler
from PIL import Image, ImageDraw

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_PORT     = '/dev/ttyUSB0'
BAUDRATE         = 115200
MAP_SIZE_PIXELS  = 500
MAP_SIZE_METERS  = 10
MIN_VALID_POINTS = 200
HTTP_PORT        = 8080
REFRESH_INTERVAL = 2   # seconds between browser auto-refresh

# ── Shared state ──────────────────────────────────────────────────────────────
latest_scan = []
scan_lock   = threading.Lock()

latest_png  = b''
png_lock    = threading.Lock()

stop_event  = threading.Event()


def measurements_to_distances(measurements):
    distances = [0] * 360
    for m in measurements:
        idx  = int(m.angle) % 360
        dist = int(m.distance)
        if dist > 0:
            distances[idx] = dist
    return distances


def build_png_bytes(mapbytes, size, x_mm, y_mm, theta_deg, map_meters):
    img  = Image.frombytes('L', (size, size), bytes(mapbytes)).convert('RGB')
    draw = ImageDraw.Draw(img)

    scale = size / (map_meters * 1000)
    px = int(x_mm * scale)
    py = size - int(y_mm * scale)   # flip Y axis

    r = 5
    draw.ellipse([px - r, py - r, px + r, py + r], fill=(255, 0, 0))

    rad = math.radians(theta_deg)
    ex  = int(px + 15 * math.cos(rad))
    ey  = int(py - 15 * math.sin(rad))
    draw.line([px, py, ex, ey], fill=(255, 0, 0), width=2)

    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


HTML_PAGE = f'''<!DOCTYPE html>
<html>
<head>
  <title>Alex SLAM Map</title>
  <meta http-equiv="refresh" content="{REFRESH_INTERVAL}">
  <style>
    body {{ background:#111; display:flex; flex-direction:column;
            align-items:center; justify-content:center; color:#eee;
            font-family:sans-serif; margin:0; height:100vh; }}
    img  {{ max-width:90vmin; max-height:90vmin; image-rendering:pixelated;
            border:2px solid #555; }}
  </style>
</head>
<body>
  <h2>Alex Robot - Live SLAM Map</h2>
  <img src="/map.png" alt="map">
  <p style="font-size:0.8em;color:#888">Auto-refreshes every {REFRESH_INTERVAL}s</p>
</body>
</html>'''.encode()


class MapHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE)

        elif self.path == '/map.png':
            with png_lock:
                data = latest_png
            if data:
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b'No map yet - waiting for first scan')
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        pass   # silence per-request logs


def http_server_thread(port):
    HTTPServer(('0.0.0.0', port), MapHandler).serve_forever()


def lidar_thread(lidar):
    global latest_scan
    try:
        for scan in lidar.start_scan_express(2):
            if stop_event.is_set():
                break
            distances = measurements_to_distances(scan)
            if sum(1 for d in distances if d > 0) >= MIN_VALID_POINTS:
                with scan_lock:
                    latest_scan = distances
    except Exception as e:
        print(f'[lidar_thread] error: {e}')
    finally:
        stop_event.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',       default=DEFAULT_PORT)
    parser.add_argument('--http-port',  type=int,   default=HTTP_PORT)
    parser.add_argument('--map-pixels', type=int,   default=MAP_SIZE_PIXELS)
    parser.add_argument('--map-meters', type=float, default=MAP_SIZE_METERS)
    args = parser.parse_args()

    print(f'[LiDAR] connecting on {args.port} ...')
    lidar = PyRPlidar()
    lidar.connect(port=args.port, baudrate=BAUDRATE, timeout=3)
    print(f'[LiDAR] info   : {lidar.get_info()}')
    print(f'[LiDAR] health : {lidar.get_health()}')
    lidar.set_motor_pwm(600)
    time.sleep(1)

    slam     = RMHC_SLAM(LaserModel(), args.map_pixels, args.map_meters, random_seed=0xabcd)
    mapbytes = bytearray(args.map_pixels * args.map_pixels)

    threading.Thread(target=http_server_thread, args=(args.http_port,), daemon=True).start()
    threading.Thread(target=lidar_thread,       args=(lidar,),          daemon=True).start()

    try:
        ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        ip = '<pi-ip>'
    print(f'[HTTP] open http://{ip}:{args.http_port} on your laptop')
    print('[SLAM] running - press Ctrl-C to stop.\n')

    try:
        while not stop_event.is_set():
            with scan_lock:
                scan = list(latest_scan)

            if len(scan) == 360:
                slam.update(scan)
                x_mm, y_mm, theta_deg = slam.getpos()
                slam.getmap(mapbytes)

                png = build_png_bytes(
                    mapbytes, args.map_pixels,
                    x_mm, y_mm, theta_deg, args.map_meters)

                with png_lock:
                    global latest_png
                    latest_png = png

                print(f'\r[SLAM] x={x_mm/1000:.2f}m  y={y_mm/1000:.2f}m  '
                      f'theta={theta_deg:.1f} deg   ', end='', flush=True)
            else:
                time.sleep(0.05)

    except KeyboardInterrupt:
        print('\n[SLAM] Stopped by user.')
    finally:
        stop_event.set()
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        print('[SLAM] LiDAR disconnected.')


if __name__ == '__main__':
    main()