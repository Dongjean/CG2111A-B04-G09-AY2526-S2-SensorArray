#!/usr/bin/env python3
"""
Studio 16: Robot Integration
net_utils.py  -  Network utilities for the second terminal relay.

Provides a minimal, SSL-upgradeable TCP layer used by:
  - pi_sensor.py        (acts as the server for second_terminal.py)
  - second_terminal.py  (client that connects to pi_sensor.py)

Public API
----------
TCPServer         - bind on a port, accept one client connection
TCPClient         - connect to a TCPServer
sendTPacketFrame  - forward a raw TPacket frame (MAGIC + payload + checksum)
recvTPacketFrame  - receive a raw TPacket frame
"""

import select as _select
import socket
import struct


# ---------------------------------------------------------------------------
# Length-prefix framing
# ---------------------------------------------------------------------------
# Every message is preceded by a 4-byte big-endian unsigned integer that
# gives the payload length.  The receiver reads the header first, then reads
# exactly that many bytes.
#
# Example wire format (5-byte payload "hello"):
#   00 00 00 05  68 65 6c 6c 6f
#   ^-header--^  ^---payload---^
# ---------------------------------------------------------------------------

_LEN_FMT  = '>I'                        # 4-byte big-endian uint32
_LEN_SIZE = struct.calcsize(_LEN_FMT)   # = 4


def _sendFramed(sock, data: bytes) -> bool:
    """Send *data* over *sock* with a 4-byte length header.

    Args:
        sock: a connected socket (plain TCP or TLS).
        data: the raw bytes to transmit.

    Returns:
        True on success; False if a network error occurred (the error is
        printed so the user can see what went wrong).
    """
    try:
        header = struct.pack(_LEN_FMT, len(data))
        sock.sendall(header + data)
        return True
    except (OSError, BrokenPipeError) as err:
        print(f"[net_utils] send error: {err}")
        return False


def _recvFramed(sock):
    """Receive one length-prefixed message from *sock*.

    Blocks until the full message has arrived.

    Returns:
        The payload bytes, or None if the connection was closed or an error
        occurred (the error is printed so the user can see what went wrong).
    """
    header = _recvExact(sock, _LEN_SIZE)
    if header is None:
        return None
    length = struct.unpack(_LEN_FMT, header)[0]
    if length == 0:
        return b''
    return _recvExact(sock, length)


def _recvExact(sock, n: int):
    """Read exactly *n* bytes from *sock*, blocking until all have arrived.

    Returns:
        The bytes read, or None on error or connection close.
    """
    buf = b''
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (OSError, ConnectionResetError) as err:
            print(f"[net_utils] recv error: {err}")
            return None
        if not chunk:
            # Remote end closed the connection cleanly.
            return None
        buf += chunk
    return buf


# ---------------------------------------------------------------------------
# TPacket frame forwarding
# ---------------------------------------------------------------------------
# A TPacket frame is:
#   MAGIC(2) + TPacket(100) + XOR_checksum(1)  =  103 bytes total
#
# sendTPacketFrame / recvTPacketFrame wrap the raw 103-byte frame in the
# length-prefix framing above so the receiver always reads a complete frame.
# ---------------------------------------------------------------------------

def sendTPacketFrame(sock, frame: bytes) -> bool:
    """Send a complete raw TPacket frame over *sock*.

    Args:
        sock:  a connected socket.
        frame: the 103-byte frame (MAGIC + TPacket bytes + XOR checksum).

    Returns:
        True on success; False on error (error is printed).
    """
    return _sendFramed(sock, frame)


def recvTPacketFrame(sock):
    """Receive a complete raw TPacket frame from *sock*.

    Returns:
        The 103-byte frame, or None if the connection was closed or errored.
    """
    return _recvFramed(sock)


# ---------------------------------------------------------------------------
# TCPServer: bind, listen, accept one client
# ---------------------------------------------------------------------------

class TCPServer:
    """A minimal TCP server that accepts a single client connection.

    To upgrade to TLS, pass an ssl.SSLContext via the *ssl_context*
    parameter. The send/recv API is identical for plain and encrypted
    connections.

    Typical usage::

        server = TCPServer(port=65432)
        if server.start():
            conn = server.accept(timeout=30)
            if conn:
                sendTPacketFrame(conn, frame)
    """

    def __init__(self, host: str = '0.0.0.0', port: int = 65432,
                 ssl_context=None):
        """
        Args:
            host:        IP address to bind to ('0.0.0.0' listens on all interfaces).
            port:        TCP port number.
            ssl_context: None for plain TCP. Pass an ssl.SSLContext here to
                         enable encryption with the same API.
        """
        self.host         = host
        self.port         = port
        self.ssl_context  = ssl_context
        self._server_sock = None
        self.conn         = None   # the accepted client socket

    def start(self) -> bool:
        """Bind to the configured host/port and start listening.

        Returns:
            True on success; False if the port is unavailable (error is printed).
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            self._server_sock = s
            print(f"[TCPServer] Listening on {self.host}:{self.port}")
            return True
        except OSError as err:
            print(f"[TCPServer] Could not bind to {self.host}:{self.port}: {err}")
            return False

    def accept(self, timeout: float = 5.0):
        """Wait up to *timeout* seconds for one client to connect.

        The accepted socket is stored in self.conn.

        Returns:
            The connected client socket on success; None on timeout or error.
        """
        if not self._server_sock:
            print("[TCPServer] Server not started. Call start() first.")
            return None
        self._server_sock.settimeout(timeout)
        try:
            conn, addr = self._server_sock.accept()
            if self.ssl_context:
                conn = self.ssl_context.wrap_socket(conn, server_side=True)
            conn.setblocking(False)
            self.conn = conn
            print(f"[TCPServer] Client connected from {addr}")
            return conn
        except socket.timeout:
            return None
        except OSError as err:
            print(f"[TCPServer] Accept error: {err}")
            return None

    def hasData(self) -> bool:
        """Return True (without blocking) if the client socket has data ready."""
        if not self.conn:
            return False
        r, _, _ = _select.select([self.conn], [], [], 0)
        return bool(r)

    def close(self):
        """Close both the client connection and the listening socket."""
        if self.conn:
            try:
                self.conn.close()
            except OSError:
                pass
            self.conn = None
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None


# ---------------------------------------------------------------------------
# TCPClient: connect to a TCPServer
# ---------------------------------------------------------------------------

class TCPClient:
    """A minimal TCP client.

    To upgrade to TLS, pass an ssl.SSLContext via the *ssl_context*
    parameter. The send/recv API is identical for plain and encrypted
    connections.

    Typical usage::

        client = TCPClient(host='localhost', port=65432)
        if client.connect():
            sendTPacketFrame(client.sock, frame)
    """

    def __init__(self, host: str = 'localhost', port: int = 65432,
                 ssl_context=None, server_hostname: str = None):
        """
        Args:
            host:            Server IP address or hostname.
            port:            Server TCP port.
            ssl_context:     None for plain TCP. Pass an ssl.SSLContext to
                             enable encryption.
            server_hostname: Hostname for TLS certificate verification
                             (only needed when ssl_context is set).
        """
        self.host            = host
        self.port            = port
        self.ssl_context     = ssl_context
        self.server_hostname = server_hostname
        self.sock            = None

    def connect(self, timeout: float = 5.0) -> bool:
        """Connect to the server.

        Returns:
            True on success; False if the connection failed (error is printed).
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((self.host, self.port))
            if self.ssl_context:
                s = self.ssl_context.wrap_socket(
                    s,
                    server_hostname=self.server_hostname or self.host,
                )
            s.setblocking(False)
            self.sock = s
            print(f"[TCPClient] Connected to {self.host}:{self.port}")
            return True
        except (OSError, socket.timeout) as err:
            print(f"[TCPClient] Connection to {self.host}:{self.port} failed: {err}")
            return False

    def hasData(self) -> bool:
        """Return True (without blocking) if the socket has data ready."""
        if not self.sock:
            return False
        r, _, _ = _select.select([self.sock], [], [], 0)
        return bool(r)

    def close(self):
        """Close the socket."""
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
