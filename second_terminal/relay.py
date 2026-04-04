#!/usr/bin/env python3
"""
Studio 16: Robot Integration
relay.py  -  Second terminal relay.

This module relays TPacket messages between pi_sensor.py and a second
operator terminal (second_terminal.py) running on the same Raspberry Pi.

Architecture:
    [Arduino] <--USB serial--> [pi_sensor.py] <--TCP:65432--> [second_terminal.py]

The relay works in two directions:
  1. Packets from the Arduino are forwarded to second_terminal.py.
  2. Commands from second_terminal.py are relayed to the Arduino.

Student tasks (Activity 3)
--------------------------
  The relay functions below are already implemented.
  Your main task is to update the TPacket constants in second_terminal.py
  to match your pi_sensor.py and Arduino sketch.

  Add these lines to pi_sensor.py (see the studio handout for context):

    from second_terminal import relay
    ...
    relay.start()              # after openSerial() in main()
    relay.onPacketReceived(packFrame(...))  # in the receive loop, after printPacket(pkt)
    relay.checkSecondTerminal(_ser) # once per main loop iteration
    relay.shutdown()           # in the finally block of main()
"""

from .net_utils import TCPServer, sendTPacketFrame, recvTPacketFrame
import ssl

# ============================================================
# Configuration
# ============================================================

SECOND_TERM_PORT    = 65432   # TCP port second_terminal.py connects to
SECOND_TERM_TIMEOUT = 300      # Seconds to wait for second_terminal.py to connect
TLS_ENABLED = True
TLS_CERT_PATH = 'certs/server.crt'
TLS_KEY_PATH = 'certs/server.key'


# ============================================================
# Module state  (do not modify)
# ============================================================

_st_server = None   # TCPServer waiting for second_terminal.py
_st_conn   = None   # Active client socket from second_terminal.py


# ============================================================
# Second terminal relay
# ============================================================

def _make_server_ssl_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(TLS_CERT_PATH, TLS_KEY_PATH)
    return ctx

def onPacketReceived(raw_frame: bytes):
    """Forward a raw TPacket frame to second_terminal.py.

    Call this after printPacket() in pi_sensor.py's receive loop.

    Args:
        raw_frame: the complete framed packet (MAGIC + TPacket + checksum)
                   just received from the Arduino.
    """
    global _st_conn
    print("hi im relay")
    if _st_conn is not None:
        ok = sendTPacketFrame(_st_conn, raw_frame)
        if not ok:
            print("[relay] Second terminal disconnected (send failed).")
            _st_conn = None


def checkSecondTerminal(serial_port):
    """Receive an incoming command from second_terminal.py and relay it to the Arduino.

    Call this once per iteration of the main loop in pi_sensor.py.

    Args:
        serial_port: the open serial.Serial object for the Arduino.
    """
    global _st_conn

    if not (_st_conn and _st_server and _st_server.hasData()):
        return

    frame = recvTPacketFrame(_st_conn)
    if frame is not None:
        serial_port.write(frame)
    else:
        print("[relay] Second terminal disconnected.")
        _st_conn = None


# ============================================================
# Lifecycle
# ============================================================

def start():
    """Start the TCP server and wait for second_terminal.py to connect.

    Call this once in pi_sensor.py after openSerial(), before the main loop.
    """
    global _st_server, _st_conn

    #_st_server = TCPServer(port=SECOND_TERM_PORT)
    ssl_context = _make_server_ssl_context() if TLS_ENABLED else None
    _st_server = TCPServer(port=SECOND_TERM_PORT, ssl_context=ssl_context)
    if _st_server.start():
        print("[relay] Waiting for second_terminal.py to connect "
              "(open a new terminal: python3 second_terminal/second_terminal.py)...")
        _st_conn = _st_server.accept(timeout=SECOND_TERM_TIMEOUT)
        print("connect")
        if _st_conn is None:
            print(f"[relay] No second terminal connected within {SECOND_TERM_TIMEOUT}s. Continuing without it.")
        else:
            print("konnec")


def shutdown():
    """Close all network connections.

    Call this in the finally block of pi_sensor.py's main function.
    """
    global _st_server

    if _st_server:
        _st_server.close()
        _st_server = None

    print("[relay] Shutdown complete.")
