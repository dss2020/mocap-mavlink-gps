import socket
import struct
import time
import math
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] MockServer: %(message)s")
logger = logging.getLogger("MockServer")

VRPN_CONNECTION_SENDER_DESCRIPTION = -1
VRPN_CONNECTION_TYPE_DESCRIPTION = -2

def pad_payload(payload):
    """Pads payload to satisfy VRPN 8-byte alignment rules."""
    pad_len = (8 - (len(payload) % 8)) % 8
    return payload + (b"\x00" * pad_len)

def pack_msg(type_id, sender_id, payload):
    """Packs message payload with a 24-byte VRPN TCP header."""
    payload_len = len(payload)
    t = time.time()
    tv_sec = int(t)
    tv_usec = int((t - tv_sec) * 1e6)
    seq = 0
    header = struct.pack(">IIIiiI", payload_len, tv_sec, tv_usec, sender_id, type_id, seq)
    return header + payload

def create_description_payload(desc_id, name):
    """Creates VRPN description payload containing description ID and name."""
    name_bytes = name.encode("utf-8") + b"\x00"
    payload = struct.pack(">i", desc_id) + name_bytes
    return pad_payload(payload)

def handle_client(client_sock):
    """Handles VRPN client session."""
    try:
        client_sock.settimeout(5.0)
        # 1. Send Server version cookie (32 bytes)
        cookie = b"vrpn-connection-cookie v07.36"
        cookie = cookie + (b"\x00" * (32 - len(cookie)))
        client_sock.sendall(cookie)
        
        # 2. Read Client version cookie (32 bytes)
        client_cookie = client_sock.recv(32)
        if len(client_cookie) < 32:
            logger.warning("Client disconnected during handshake.")
            return
            
        client_version = client_cookie.split(b"\x00")[0].decode("utf-8", errors="replace")
        logger.info(f"Client connected. Handshook with client version: '{client_version}'")
        
        # 3. Send system registration messages
        # Register Type ID 0 -> "vrpn_Tracker"
        type_payload = create_description_payload(0, "vrpn_Tracker")
        client_sock.sendall(pack_msg(VRPN_CONNECTION_TYPE_DESCRIPTION, 0, type_payload))
        
        # Register Sender ID 0 -> "Rigidbody"
        sender_payload = create_description_payload(0, "Rigidbody")
        client_sock.sendall(pack_msg(VRPN_CONNECTION_SENDER_DESCRIPTION, 0, sender_payload))
        
        logger.info("Sent VRPN registration descriptors. Starting tracking stream...")
        
        # 4. Stream circular movement reports at 20Hz
        angle = 0.0
        radius = 3.0       # 3 meters radius
        omega = 0.25       # Angular velocity
        loop_interval = 0.05 # 20 Hz
        
        while True:
            t = time.time()
            
            # Simulated circular trajectory
            # Motive convention: X=East, Y=Up, Z=South.
            # circular motion in X-Z horizontal plane, with Y oscillation.
            pos_x = radius * math.cos(angle)
            pos_z = radius * math.sin(angle)
            pos_y = 1.5 + 0.5 * math.sin(t) # Hovering between 1.0m and 2.0m height
            
            # Simulated orientation (no rotation, identity quaternion)
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            
            # Tracker positions message payload (64 bytes):
            #   sensor (int32) - 4 bytes
            #   padding - 4 bytes
            #   position (3 doubles: x, y, z) - 24 bytes
            #   orientation (4 doubles: qx, qy, qz, qw) - 32 bytes
            sensor = 0
            tracker_payload = struct.pack(">i4xddddddd", sensor, pos_x, pos_y, pos_z, qx, qy, qz, qw)
            
            # Send message with type=0, sender=0 (matches descriptions)
            client_sock.sendall(pack_msg(0, 0, tracker_payload))
            
            # Advance trajectory
            angle += omega * loop_interval
            if angle > 2 * math.pi:
                angle -= 2 * math.pi
                
            time.sleep(loop_interval)
            
    except Exception as e:
        logger.error(f"Client handler exception: {e}")
    finally:
        client_sock.close()
        logger.info("Client connection closed.")

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to localhost (3883 is standard VRPN port)
    server.bind(("127.0.0.1", 3883))
    server.listen(1)
    logger.info("Mock VRPN server listening on 127.0.0.1:3883...")
    
    try:
        while True:
            client_sock, client_addr = server.accept()
            logger.info(f"Accepted connection from {client_addr}")
            handle_client(client_sock)
    except KeyboardInterrupt:
        logger.info("Shutting down mock VRPN server.")
    finally:
        server.close()

if __name__ == "__main__":
    main()
