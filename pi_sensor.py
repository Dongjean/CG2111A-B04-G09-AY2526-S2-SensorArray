#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP  = 0
COMMAND_COLOUR = 1
COMMAND_GO = 2
COMMAND_CW = 3
COMMAND_CCW = 4
COMMAND_BACK = 5
COMMAND_STOP = 6

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOUR = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form.

    The 'data' field carries an optional debug string from the Arduino.
    When non-empty, it is printed automatically so you can embed debug
    messages in any outgoing TPacket on the Arduino side (set pkt.data to
    a null-terminated string up to 31 characters before calling sendFrame).
    This works like Serial.print(), but sends output to the Pi terminal.
    """
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
        elif cmd == RESP_COLOUR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"Response: R: {r} Hz, G: {g} Hz, B: {b} Hz")
            # TODO (Activity 2): add an elif branch here to handle your color
            # response.  Display the three channel frequencies in Hz, e.g.:
            #   R: <params[0]> Hz, G: <params[1]> Hz, B: <params[2]> Hz
        else:
            print(f"Response: unknown command {cmd}")
        # Print the optional debug string from the data field.
        # On the Arduino side, fill pkt.data before calling sendFrame() to
        # send debug messages to this terminal (similar to Serial.print()).
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    TODO (Activity 2): request a color reading from the Arduino and display it.

    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """
    # TODO
    if isEstopActive():
        print("cannot use colour sensor as E-Stop has been activated")
    else:
        sendCommand(COMMAND_COLOUR)
    


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

# TODO (Activity 3): import the camera library provided (alex_camera.py).
import alex_camera as alex
_camera = alex.cameraOpen()          # TODO (Activity 3): open the camera (cameraOpen()) before first use.
_frames_remaining = 5   # frames remaining before further captures are refused


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.

    Gate on E-Stop state and the remaining frame count.
    Use captureGreyscaleFrame() and renderGreyscaleFrame() from alex_camera.
    """
    global _frames_remaining
    # TODO
    if isEstopActive():
        print("cannot use camera as E-Stop has been activated")
    elif _frames_remaining <= 0:
        print("ran out of frames, cannot use camera anymore")
    else:
        frame = alex.captureGreyscaleFrame(_camera)
        alex.renderGreyscaleFrame(frame)
        _frames_remaining -= 1
        print("There are " + str(_frames_remaining) + " frames remaining");


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

# TODO (Activity 4): import from lidar.alex_lidar and lidar_example_cli_plot
#   (lidar_example_cli_plot.py is in the same folder; alex_lidar.py is in lidar/).
import lidar.alex_lidar as l
import lidar_example_cli_plot as lidarEX


def handleLidarCommand():
    """
    TODO (Activity 4): perform a single LIDAR scan and render it.

    Gate on E-Stop state, then use the LIDAR library to capture one scan
    and the CLI plot helpers to display it.
    """
    # TODO
    if isEstopActive():
        print("cannot use lidar as E-Stop has been activated")
    else:
        lidarEX.plot_single_scan()



# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# User input -> action mapping:
#   e  send a software E-Stop command to the Arduino (pre-wired)
#   c  request color reading from the Arduino        (Activity 2 - implement yourself)
#   p  capture and display a camera frame            (Activity 3 - implement yourself)
#   l  perform a single LIDAR scan                   (Activity 4 - implement yourself)


def handleUserInput(line):
    """
    Dispatch a single line of user input.

    The 'e' case is pre-wired to send a software E-Stop command.
    TODO (Activities 2, 3 & 4): add 'c' (color), 'p' (camera) and 'l' (LIDAR).
    """
    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    # TODO (Activity 2): add an elif branch for 'c' (color sensor) that calls handleColorCommand().
    elif line == 'c':
        handleColorCommand()
    # TODO (Activities 3 & 4): add elif branches for 'p' (camera) and 'l' (LIDAR).
    elif line == 'p':
        handleCameraCommand()
    elif line == 'l':
        handleLidarCommand()
    elif line.startswith() == 'w':
        duration = line.split()[1]
        if not(duration.isdigit()) or duration < 0:
            print("Unknown duration input")
        else:
            sendCommand(COMMAND_GO, params=duration)
    elif line.startswith() == 'a':
        duration = line.split()[1]
        if not(duration.isdigit()) or duration < 0:
            print("Unknown duration input")
        else:
            sendCommand(COMMAND_CCW, params=duration)
    elif line.startswith() == 'd':
        duration = line.split()[1]
        if not(duration.isdigit()) or duration < 0:
            print("Unknown duration input")
        else:
            sendCommand(COMMAND_CW, params=duration)
    elif line.startswith() == 's':
        duration = line.split()[1]
        if not(duration.isdigit()) or duration < 0:
            print("Unknown duration input")
        else:
            sendCommand(COMMAND_BACK, params=duration)
    elif line.startswith() == 'x':
        duration = line.split()[1]
        if not(duration.isdigit()) or duration < 0:
            print("Unknown duration input")
        else:
            sendCommand(COMMAND_STOP, params=duration)
    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    print("Sensor interface ready. Type e / c / p / l and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        # TODO (Activities 3 & 4): close the camera and disconnect the LIDAR here if you opened them.
        if '_camera' in globals() and _camera:
            alex.cameraClose(_camera)  # Assuming standard close function in your library
            
        if '_lidar' in globals() and _lidar:
            # Stops the motor, sets PWM to 0, and closes the port
            l.lidarDisconnect(_lidar)
        closeSerial()
