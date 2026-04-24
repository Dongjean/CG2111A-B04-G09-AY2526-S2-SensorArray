#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal.

This terminal connects to pi_sensor.py over TCP.  It:
  - Displays every TPacket forwarded from the robot (via pi_sensor.py).
  - Sends a software E-Stop command when you type 'e'.

Architecture
------------
   [Arduino] <--USB serial--> [pi_sensor.py] <--TCP--> [second_terminal.py]
                                (TCP server,               (TCP client,
                                 port 65432)                localhost:65432)

Run pi_sensor.py FIRST (it starts the TCP server), then run this script.
Both scripts run on the same Raspberry Pi.

IMPORTANT: Update the TPacket constants below to match your pi_sensor.py.
---------------------------------------------------------------------------
The packet constants (PACKET_TYPE_*, COMMAND_*, RESP_*, STATE_*, sizes) are
duplicated here from pi_sensor.py.  They MUST stay in sync with your
pi_sensor.py (and with the Arduino sketch).  Update them whenever you change
your protocol.

Tip: consider abstracting all TPacket constants into a shared file (e.g.
packets.py) that both pi_sensor.py and second_terminal.py import, so there
is only one place to update them.  You do not have to do this now, but it
avoids hard-to-find bugs caused by constants getting out of sync.

Commands
--------
  e   Send a software E-Stop to the robot (same as pressing the button).
  q   Quit.

Usage
-----
    source env/bin/activate
    python3 second_terminal/second_terminal.py

Press Ctrl+C to exit.
"""

import select
import signal
import time
import struct
import sys
import time
import serial
import os
import tty
import termios
import ssl

estopState = False

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.append(parent_dir)

# Keys we track for movement commands (lowercase and uppercase)
WASD_KEYS = {b'w', b'a', b's', b'd', b'W', b'A', b'S', b'D'}

# The duration to determine whether a key has timed out or not
KEY_HELD_TIMEOUT = 0.1

# The timestamps of last seen and first seen of each key
# Current Timestamp - _last_seen[key] gives us the time since we last saw the key
# Used for timeout of held keys
_last_seen: dict[bytes, float] = {}
# _first_seen: dict[bytes, float] = {}

# Currently held keys
keys_held: dict[bytes, bool] = {}

from packets import *

# net_utils is imported with an absolute import because this script is designed
# to be run directly (python3 second_terminal/second_terminal.py), which adds
# this file's directory to sys.path automatically.
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
# Both scripts run on the same Pi, so the host is 'localhost'.
# Change PI_HOST to the Pi's IP address if you run this from a different machine.
PI_HOST = 'localhost'
PI_PORT = 65432


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    """Pack a TPacket into a 103-byte framed byte string."""
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 103-byte frame.  Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False


def _printPacket(pkt):
    """Pretty-print a TPacket forwarded from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']
    
    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        else:
            print(f"[robot] Response: unknown command {cmd}")
        # Print any debug string embedded in the data field.
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def on_key_event(key: str, client: TCPClient, held: bool):
    """
    Called whenever a key is being pressed or released.

    When a key is pressed, we send a packet for 5 encoder ticks to move in that direction.
    If something goes wrong in the stop packet sending, at least the movement command will time out and the robot will not crash into something.

    When a key is released, we send a stop packet.
    Allows for small micro movements with taps of keys
    """
    action = "HELD" if held else "released"
    print(f"\r  [{action}]  {key.upper()}        ", end="", flush=True)

    # Dont do anything if E-stop is active
    if (held == True and (_estop_active == False)):
        # Logic for pressed keys
        params = [0]*16
        params[0] = 5
        cmd = key.lower()
        if cmd == 'w':
            frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_GO, params=params)
            sendTPacketFrame(client.sock, frame)
        elif cmd == 'a':
            frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CCW, params=params)
            sendTPacketFrame(client.sock, frame)
        elif cmd == 's':
            frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_BACK, params=params)
            sendTPacketFrame(client.sock, frame)
        elif cmd == 'd':
            frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CW, params=params)
            sendTPacketFrame(client.sock, frame)
    elif (held == False):
        # Logic for released keys
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_STOP)
        sendTPacketFrame(client.sock, frame)

def _refresh_held_states(client: TCPClient):
    """
    Mark keys as released if they haven't been seen recently.
    """
    now = time.monotonic()
    released = []

    # Check every single key that is currently marked as held
    for key, last in _last_seen.items():
        # Check if these "held" keys have been seen recently
        if now - last > KEY_HELD_TIMEOUT:
            # If it has not been seen recently, append it to the list of keys to release
            released.append(key)
    for key in released: # This key has not been seen recently

        # If the key was marked as held, update its state to be released
        if keys_held.get(key):
            keys_held[key] = False

            # Run the key event, but this time for key release event
            on_key_event(key.decode(), client, held=False)
        
        # Reset this key from all our dictionaries handling this
        del _last_seen[key]
        # _first_seen.pop(key, None)

def instainput(client: TCPClient):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
 
    print("WASD hold Movement Controls. Ctrl-C or Q to quit.\n")
    print(" W = forward | A = rotate left | S = backward | D = rotate right\n")
 
    try:
        tty.setraw(fd)
 
        # Since this keyboard listener blocks the main run() function:
        # Add a packet listener here so we can be updated of any E-stop state changes
        while True:
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("\r\n[second_terminal] Connection closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    print("\r", end="")
                    _printPacket(pkt)

            # Here onwards is the keyboard listener

            # Non-blocking check for a byte (1 ms window)
            ready, _, _ = select.select([sys.stdin], [], [], 0.001)
 
            if ready:
                ch = sys.stdin.buffer.read(1)
 
                # Quit on Ctrl-C (0x03), Ctrl-D (0x04), or Q
                if ch in (b'\x03', b'\x04', b'q', b'Q'):
                    break
 
                if ch in WASD_KEYS:
                    now = time.monotonic()
                    _last_seen[ch] = now
 
                    # If the processed key is not already held, update its key held state
                    if not keys_held.get(ch):
                        keys_held[ch] = True
                        # _first_seen[ch] = time.monotonic()
                        on_key_event(ch.decode(), client, held=True)
 
            # Always update release states
            _refresh_held_states(client)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\nExiting — terminal restored.")

def _handleInput(line: str, client: TCPClient):
    """Handle one line of keyboard input."""
    line = line.strip().lower()
    if not line:
        return

    # E-stop Handler
    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")
    
    # Original Movement Commands Handler
    elif line.split()[0] in "wasd":
        if len(line.split()) >= 2 and not(_estop_active):
            cmd = line.split()[0]
            duration = line.split()[1]
            if not(duration.isdigit()) or int(duration) < 0:
                print("Unknown duration input")
            else:
                params = [0]*16
                params[0] = int(duration)
                if cmd == 'w':
                    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_GO, params=params)
                    sendTPacketFrame(client.sock, frame)
                elif cmd == 'a':
                    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CCW, params=params)
                    sendTPacketFrame(client.sock, frame)
                elif cmd == 's':
                    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_BACK, params=params)
                    sendTPacketFrame(client.sock, frame)
                elif cmd == 'd':
                    frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CW, params=params)
                    sendTPacketFrame(client.sock, frame)
    elif line == 'x':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_STOP)
        sendTPacketFrame(client.sock, frame)
    
    # New Movement Commands Handler
    # Starts a blocking function which listens for keypresses
    elif line == 'm':
        instainput(client)
    
    # Movement Speed Editor
    elif line == '+':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_FASTER)
        sendTPacketFrame(client.sock, frame)
    elif line == '-':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_SLOWER)
        sendTPacketFrame(client.sock, frame)

    elif line == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    else:
        print(f"[second_terminal] Unknown: '{line}'.  Valid: e (E-Stop)  q (quit)")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    client = TCPClient(host=PI_HOST, port=PI_PORT, ssl_context = context)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=10.0):
        print("[second_terminal] Could not connect.")
        print("  Make sure pi_sensor.py is running and waiting for a"
              " second terminal connection.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Commands:  e = E-Stop   q = quit")
    print("[second_terminal] Incoming robot packets will be printed below.\n")

    try:
        while True:
            # Check for forwarded TPackets from pi_sensor.py (non-blocking).
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection to pi_sensor.py closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            # Check for keyboard input (non-blocking via select).
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()

                # Handle keyboard input
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()


if __name__ == '__main__':
    run()
