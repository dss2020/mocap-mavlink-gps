import socket
import struct
import time
import logging

logger = logging.getLogger("VRPNClient")

class VRPNClient:
    """
    A pure-Python VRPN protocol client that connects over TCP, 
    performs the handshake, dynamically maps message types and senders, 
    and decodes tracker reports.
    """
    def __init__(self, host="127.0.0.1", port=3883, tracker_name="Rigidbody"):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self._socket = None
        self._connected = False
        
        # Mappings of ID -> Name
        self.senders = {}
        self.types = {}
        
        # System type constants
        self.VRPN_CONNECTION_SENDER_DESCRIPTION = -1
        self.VRPN_CONNECTION_TYPE_DESCRIPTION = -2
        self.VRPN_CONNECTION_UDP_DESCRIPTION = -3
        self.VRPN_CONNECTION_DISCONNECT_MESSAGE = -5

    def connect(self):
        """Establishes TCP connection and performs VRPN version handshake."""
        self._connected = False
        self.senders.clear()
        self.types.clear()
        
        logger.info(f"Connecting to VRPN server at {self.host}:{self.port}...")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(10.0)
        self._socket.connect((self.host, self.port))
        
        # VRPN Version Handshake
        # 1. Read server cookie (32 bytes)
        server_cookie = self._read_exact(32)
        if not server_cookie:
            raise ConnectionError("Server disconnected immediately without cookie.")
        
        cookie_str = server_cookie.split(b"\x00")[0].decode("utf-8", errors="replace")
        logger.info(f"Received VRPN server version: '{cookie_str}'")
        
        # 2. Write version cookie back to server
        # We write back exactly the same version to ensure compatibility
        self._socket.sendall(server_cookie)
        self._connected = True
        logger.info("VRPN Handshake successful.")

    def _read_exact(self, n):
        """Helper to read exactly n bytes from the TCP socket."""
        data = b""
        logger.debug(f"_read_exact attempting to read {n} bytes...")
        while len(data) < n:
            try:
                logger.debug(f"calling recv({n - len(data)})...")
                chunk = self._socket.recv(n - len(data))
                logger.debug(f"recv returned {len(chunk) if chunk else 0} bytes")
            except socket.timeout:
                logger.warning("Socket read timeout")
                return None
            except Exception as e:
                logger.error(f"Socket read error: {e}")
                return None
                
            if not chunk:
                logger.debug("recv returned EOF")
                return None  # Socket EOF
            data += chunk
        logger.debug(f"_read_exact successfully read {len(data)} bytes: {data.hex()}")
        return data

    def stream_reports(self):
        """
        Generator that continuously reads, parses VRPN packets, 
        and yields tracker updates as dictionary reports:
        {
            "time": float,
            "sensor": int,
            "position": (x, y, z),
            "orientation": (qx, qy, qz, qw)
        }
        """
        while True:
            if not self._connected:
                try:
                    self.connect()
                except Exception as e:
                    logger.error(f"Connection failed: {e}. Retrying in 2 seconds...")
                    time.sleep(2)
                    continue

            # Read VRPN 24-byte message header
            logger.debug("Reading VRPN 24-byte header...")
            header = self._read_exact(24)
            if not header:
                logger.warning("VRPN Connection closed by server.")
                self._connected = False
                continue

            try:
                # Wire format:
                #   payload_len (uint32) - 4 bytes
                #   tv_sec (uint32) - 4 bytes
                #   tv_usec (uint32) - 4 bytes
                #   sender_id (int32) - 4 bytes
                #   type_id (int32) - 4 bytes
                #   sequence_number (uint32) - 4 bytes
                payload_len, tv_sec, tv_usec, sender_id, type_id, seq = struct.unpack(">IIIiiI", header)
                logger.debug(f"Unpacked header: payload_len={payload_len}, tv_sec={tv_sec}, tv_usec={tv_usec}, sender_id={sender_id}, type_id={type_id}, seq={seq}")
            except Exception as e:
                logger.error(f"Failed to unpack message header: {e}")
                self._connected = False
                continue

            # Read payload
            logger.debug(f"Reading payload of length {payload_len}...")
            payload = self._read_exact(payload_len)
            if payload is None:
                logger.warning("VRPN Connection closed during payload read.")
                self._connected = False
                continue

            timestamp = tv_sec + (tv_usec / 1e6)

            # Handle System Messages
            if type_id == self.VRPN_CONNECTION_SENDER_DESCRIPTION:
                try:
                    desc_sender_id, = struct.unpack(">i", payload[:4])
                    name = payload[4:].split(b"\x00")[0].decode("utf-8", errors="replace")
                    self.senders[desc_sender_id] = name
                    logger.info(f"Registered Sender ID {desc_sender_id} -> '{name}'")
                except Exception as e:
                    logger.error(f"Error parsing sender description: {e}")

            elif type_id == self.VRPN_CONNECTION_TYPE_DESCRIPTION:
                try:
                    desc_type_id, = struct.unpack(">i", payload[:4])
                    name = payload[4:].split(b"\x00")[0].decode("utf-8", errors="replace")
                    self.types[desc_type_id] = name
                    logger.info(f"Registered Type ID {desc_type_id} -> '{name}'")
                except Exception as e:
                    logger.error(f"Error parsing type description: {e}")

            # Handle Tracker Messages
            elif self.types.get(type_id) == "vrpn_Tracker":
                sender_name = self.senders.get(sender_id)
                if sender_name == self.tracker_name:
                    if len(payload) >= 64:
                        try:
                            # Tracker position payload layout:
                            #   sensor (int32) - 4 bytes
                            #   padding - 4 bytes (unused, aligns doubles to 8 bytes)
                            #   pos (3 doubles: x, y, z) - 24 bytes
                            #   quat (4 doubles: qx, qy, qz, qw) - 32 bytes
                            sensor, = struct.unpack(">i", payload[:4])
                            pos_x, pos_y, pos_z, qx, qy, qz, qw = struct.unpack(">ddddddd", payload[8:64])
                            
                            yield {
                                "time": timestamp,
                                "sensor": sensor,
                                "position": (pos_x, pos_y, pos_z),
                                "orientation": (qx, qy, qz, qw)
                            }
                        except Exception as e:
                            logger.error(f"Error unpacking tracker payload: {e}")
                    else:
                        logger.warning(f"Tracker payload length too short: {len(payload)} bytes")
