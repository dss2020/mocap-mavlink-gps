import socket
import struct
import time
import math
import logging
import json
import sys

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] MockServer: %(message)s")
logger = logging.getLogger("MockServer")

VRPN_CONNECTION_SENDER_DESCRIPTION = -1
VRPN_CONNECTION_TYPE_DESCRIPTION = -2

def load_config(config_path="config.json"):
    """Loads configuration parameters from JSON file."""
    try:
        with open(config_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.warning(f"Could not load config file {config_path}: {e}. Using default coordinate mapping.")
        return {}

def map_local_to_vrpn(east, north, up, mapping):
    """
    Translates local coordinate frames (East, North, Up) back into VRPN (x, y, z) 
    coordinates based on the convention mapping specified in config.json.
    """
    pos = {"x": 0.0, "y": 0.0, "z": 0.0}
    
    def assign_axis(mapping_key, val):
        clean_key = mapping_key.strip().lower()
        sign = 1.0
        if clean_key.startswith("-"):
            sign = -1.0
            clean_key = clean_key[1:]
        if clean_key in pos:
            pos[clean_key] = sign * val

    assign_axis(mapping.get("east", "x"), east)
    assign_axis(mapping.get("north", "-z"), north)
    assign_axis(mapping.get("up", "y"), up)
    
    return pos["x"], pos["y"], pos["z"]


def pad_payload(payload):
    """Pads payload to satisfy VRPN 8-byte alignment rules."""
    pad_len = (8 - (len(payload) % 8)) % 8
    return payload + (b"\x00" * pad_len)

def pack_msg(type_id, sender_id, payload):
    """Packs message payload with a 24-byte VRPN TCP header."""
    payload_len = len(payload)
    total_len = payload_len + 24
    t = time.time()
    tv_sec = int(t)
    tv_usec = int((t - tv_sec) * 1e6)
    seq = 0
    header = struct.pack(">IIIiiI", total_len, tv_sec, tv_usec, sender_id, type_id, seq)
    return header + payload

def create_description_payload(name):
    """Creates VRPN description payload containing name length and name."""
    name_bytes = name.encode("utf-8") + b"\x00"
    payload = struct.pack(">i", len(name_bytes)) + name_bytes
    return pad_payload(payload)

def handle_client(client_sock):
    """Handles VRPN client session."""
    try:
        client_sock.settimeout(5.0)
        # 1. Send Server version cookie (24 bytes)
        cookie = b"vrpn: ver. 07.36  0"
        cookie = cookie + (b"\x00" * (24 - len(cookie)))
        client_sock.sendall(cookie)
        
        # 2. Read Client version cookie (24 bytes)
        client_cookie = client_sock.recv(24)
        if len(client_cookie) < 24:
            logger.warning("Client disconnected during handshake.")
            return
            
        client_version = client_cookie.split(b"\x00")[0].decode("utf-8", errors="replace")
        logger.info(f"Client connected. Handshook with client version: '{client_version}'")
        
        # 3. Send system registration messages
        # Register Type ID 0 -> "vrpn_Tracker"
        type_payload = create_description_payload("vrpn_Tracker")
        client_sock.sendall(pack_msg(VRPN_CONNECTION_TYPE_DESCRIPTION, 0, type_payload))
        
        # Register Sender ID 0 -> "Rigidbody"
        sender_payload = create_description_payload("Rigidbody")
        client_sock.sendall(pack_msg(VRPN_CONNECTION_SENDER_DESCRIPTION, 0, sender_payload))
        
        logger.info("Sent VRPN registration descriptors. Starting tracking stream...")
        
        # 4. Stream circular movement reports at 20Hz
        config = load_config()
        mapping = config.get("coordinate_mapping", {})
        logger.info(f"Loaded coordinate mapping for simulation: {mapping}")

        angle = 0.0
        radius = 10.0       # 10 meters radius
        omega = 0.5       # Angular velocity
        loop_interval = 0.05 # 20 Hz
        
        while True:
            t = time.time()
            
            # Generate circular trajectory in local frame (East/North plane, with Up hovering height)
            east = radius * math.cos(angle)
            north = radius * math.sin(angle)
            up = 1.5 + 0.5 * math.sin(t) # Hovering between 1.0m and 2.0m height
            
            # Map East, North, Up back to VRPN (x, y, z) using config.json conventions
            pos_x, pos_y, pos_z = map_local_to_vrpn(east, north, up, mapping)
            
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
