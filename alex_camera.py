#!/usr/bin/env python3
"""
alex_camera.py - Camera helper library for Studio 13: Sensor Mini-Project

Wraps picamera2 to capture frames from the Raspberry Pi Camera Module
and render them in the terminal. Read the docstrings below to understand
how to use each function.

Requirements:
    Use Raspberry Pi OS system packages for picamera2 and numpy.
    Do not install picamera2 or numpy with pip inside this project venv.
    If camera import fails with "numpy.dtype size changed", run:
      pip uninstall -y numpy picamera2
"""

# ---------------------------------------------------------------
# Rendering constants
# ---------------------------------------------------------------

# Rendered frame size in terminal characters.
# Two image rows are packed into one terminal row using the Unicode
# LOWER HALF BLOCK character (▄), so the captured image height must
# be even.
RENDER_WIDTH  = 80
RENDER_HEIGHT = 44

RESET = "\033[0m"


# ---------------------------------------------------------------
# Public API
# ---------------------------------------------------------------

def cameraOpen():
    """
    Open and configure the Raspberry Pi Camera Module.

    Returns:
        A Picamera2 instance ready for capture.  Pass this handle to
        captureGreyscaleFrame() and cameraClose().
    """
    from picamera2 import Picamera2
    import time
    cam = Picamera2()
    config = cam.create_still_configuration(
        main={"size": (RENDER_WIDTH, RENDER_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    time.sleep(0.5)   # allow auto-exposure to settle
    return cam


def cameraClose(cam):
    """Stop and close the camera."""
    cam.stop()
    cam.close()


def captureFrame(cam):
    """
    Capture a single still frame in color.

    Args:
        cam: A Picamera2 instance returned by cameraOpen().

    Returns:
        A numpy array of shape (RENDER_HEIGHT, RENDER_WIDTH, 3),
        dtype uint8, RGB channel order.
    """
    import numpy as np
    raw = cam.capture_array("main")
    # picamera2 RGB888 returns bytes in BGR order on most platforms; swap to RGB.
    return raw[:, :, [2, 1, 0]]


def captureGreyscaleFrame(cam):
    """
    Capture a single still frame converted to greyscale.

    Uses the standard luminance formula: Y = 0.299 R + 0.587 G + 0.114 B.

    Args:
        cam: A Picamera2 instance returned by cameraOpen().

    Returns:
        A numpy array of shape (RENDER_HEIGHT, RENDER_WIDTH), dtype uint8.
    """
    import numpy as np
    rgb = captureFrame(cam)
    grey = (0.299 * rgb[:, :, 0] +
            0.587 * rgb[:, :, 1] +
            0.114 * rgb[:, :, 2]).astype(np.uint8)
    return grey


def renderGreyscaleFrame(frame):
    """
    Render a greyscale frame in the terminal using ANSI escape codes.

    Each pair of vertically adjacent pixels is packed into one terminal
    character using the LOWER HALF BLOCK (▄):
      - Top pixel intensity → background grey level.
      - Bottom pixel intensity → foreground grey level.

    Args:
        frame: A numpy array of shape (H, W), dtype uint8, as returned
               by captureGreyscaleFrame().
    """
    h, w = frame.shape
    rows = h // 2

    lines = []
    for term_row in range(rows):
        top = term_row * 2
        bot = top + 1
        line = ""
        for col in range(w):
            vt = int(frame[top, col])
            vb = int(frame[bot, col])
            line += (
                f"\033[48;2;{vt};{vt};{vt}m"
                f"\033[38;2;{vb};{vb};{vb}m"
                "▄"
                f"{RESET}"
            )
        lines.append(line)

    print("\n".join(lines))
