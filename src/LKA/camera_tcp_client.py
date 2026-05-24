"""
camera_tcp_client.py
TCP client for receiving camera frames from Movie NX on port 2210.

Protocol (Movie NX / RSDA framework — matches CameraStream.cpp exactly):
  - Text header: 64-byte block, starts with '*' followed by a capital letter [A-Z]
  - Camera frame header format:
        *CameraRSI <channel> <imgtype> <simtime> <width>x<height> <imglen>
    e.g. *CameraRSI 0 rgb 1.234 640x480 921600
  - Payload: <imglen> raw packed bytes immediately after the 64-byte header block
    * imgtype 'rgb'   -> uint8, width*height*3 bytes, row-major, R first
    * imgtype 'depth' -> float32, width*height*4 bytes
  - Resync: on bad header, scan forward byte-by-byte to the next '*[A-Z]' pair
"""

import socket
import re
import numpy as np
import cv2
import time
import config as cfg

_HEADER_SIZE = 64
_CAMERA_RSI_RE = re.compile(
    r'^\*CameraRSI\s+(\d+)\s+(\S+)\s+([\d.eE+\-]+)\s+(\d+)x(\d+)\s+(\d+)'
)


class CameraClient:
    """
    TCP client for Movie NX GPU camera stream on port 2210.
    Implements the text-header protocol identical to CameraStream.cpp.
    """

    def __init__(self, host="localhost", port=2210, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
        self._last_frame = None
        self._frame_count = 0
        self._recv_buffer = b''

    def connect(self):
        """
        Establish TCP connection to Movie NX camera stream server.

        Returns:
            True if successful, False otherwise.
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self._recv_buffer = b''
            print(f"[CameraClient] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[CameraClient] Connection failed: {e}")
            self.connected = False
            return False

    def reconnect(self):
        """Attempt to reconnect after a lost connection."""
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        time.sleep(cfg.CAMERA_RECONNECT_DELAY)
        return self.connect()

    def close(self):
        """Close the socket and mark as disconnected."""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None

    def get_frame(self):
        """Read one RGB frame from the TCP stream.

        Returns:
            numpy array (BGR, uint8, shape HxWx3) or None on error.
        """
        if not self.connected:
            return None
        return self._read_frame()

    # ── Internal helpers ────────────────────────────────────────────────────

    def _recv_exact(self, n):
        """Pull exactly n bytes from the socket (via internal buffer)."""
        while len(self._recv_buffer) < n:
            try:
                chunk = self.socket.recv(max(4096, n - len(self._recv_buffer)))
            except Exception:
                self.connected = False
                return None
            if not chunk:
                self.connected = False
                return None
            self._recv_buffer += chunk
        data = self._recv_buffer[:n]
        self._recv_buffer = self._recv_buffer[n:]
        return data

    def _read_header(self):
        """
        Read and resync to a valid 64-byte text header.
        Mirrors TCP_RecvHdr() from CameraStream.cpp exactly:
          - Reads 64-byte blocks.
          - A valid header starts with '*' followed by a capital letter [A-Z].
          - On mismatch, scans forward to the next '*' byte and refills.

        Returns:
            Decoded and stripped header string, or None on socket error.
        """
        buf = self._recv_exact(_HEADER_SIZE)
        if buf is None:
            return None

        n_skipped = 0
        while True:
            if len(buf) >= 2 and buf[0:1] == b'*' and b'A' <= buf[1:2] <= b'Z':
                header_str = buf.rstrip(b'\x00 \t\r\n').decode('ascii', errors='replace')
                if n_skipped > 0:
                    print(f"[CameraClient] Header resync: {n_skipped} bytes skipped")
                return header_str

            # Scan forward to find the next '*', starting at index 1
            i = 1
            while i < len(buf) and buf[i:i + 1] != b'*':
                i += 1
            n_skipped += i
            buf = buf[i:]

            # Refill buf back to 64 bytes
            needed = _HEADER_SIZE - len(buf)
            if needed > 0:
                more = self._recv_exact(needed)
                if more is None:
                    return None
                buf = buf + more

    def _read_frame(self):
        """
        Read one complete frame (text header + payload) from the TCP stream.

        Returns:
            numpy array (BGR, uint8) on success, None on parse or socket error.
        """
        # Step 1: receive text header
        header_str = self._read_header()
        if header_str is None:
            return None

        # Step 2: parse *CameraRSI fields
        m = _CAMERA_RSI_RE.match(header_str)
        if not m:
            # Other RSDA message type (e.g. *Hello); not an error, just no frame.
            print(f"[CameraClient] Non-CameraRSI header skipped: {header_str!r}")
            return None

        width    = int(m.group(4))
        height   = int(m.group(5))
        img_len  = int(m.group(6))
        img_type = m.group(2)

        # Step 3: sanity-check dimensions and payload size
        if width <= 0 or height <= 0 or width > 4000 or height > 4000:
            print(f"[CameraClient] Invalid dimensions in header: {width}x{height}")
            return None
        if img_len <= 0 or img_len > 100 * 1024 * 1024:
            print(f"[CameraClient] Invalid imglen in header: {img_len}")
            return None

        # Step 4: read payload
        payload = self._recv_exact(img_len)
        if payload is None:
            print("[CameraClient] Socket closed while reading payload")
            return None

        # Step 5: reconstruct image
        if img_type == 'rgb':
            expected = width * height * 3
            if img_len != expected:
                print(f"[CameraClient] RGB size mismatch: header={img_len}, expected={expected}")
                return None
            try:
                rgb = np.frombuffer(payload, dtype=np.uint8).reshape((height, width, 3))
            except ValueError as e:
                print(f"[CameraClient] RGB reshape failed: {e}")
                return None
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        elif img_type == 'depth':
            expected = width * height * 4
            if img_len != expected:
                print(f"[CameraClient] Depth size mismatch: header={img_len}, expected={expected}")
                return None
            try:
                depth = np.frombuffer(payload, dtype=np.float32).reshape((height, width))
            except ValueError as e:
                print(f"[CameraClient] Depth reshape failed: {e}")
                return None
            # Normalize depth to 8-bit BGR for downstream OpenCV processing
            norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            bgr = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        else:
            print(f"[CameraClient] Unknown image type: {img_type!r}")
            return None

        # Warn once per minute if resolution differs from config
        if (width, height) != (cfg.IMG_W, cfg.IMG_H) and self._frame_count % 60 == 0:
            print(
                f"[CameraClient] Stream resolution {width}x{height} differs from "
                f"config {cfg.IMG_W}x{cfg.IMG_H}"
            )

        self._last_frame = bgr
        self._frame_count += 1
        return bgr