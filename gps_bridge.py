import json
import math
import sys
import time
import logging
from pymavlink import mavutil
import serial.tools.list_ports
from vrpn_client import VRPNClient

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("GPSBridge")

def load_config(config_path="config.json"):
    """Loads configuration parameters from JSON file."""
    try:
        with open(config_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.critical(f"Failed to load config file: {e}")
        sys.exit(1)

def get_serial_port():
    """Scans and prompts user for serial port selection."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        logger.error("No serial ports discovered on the system.")
        return None
        
    logger.info("Discovered serial ports:")
    for idx, p in enumerate(ports, 1):
        print(f" [{idx}] {p.device} ({p.description})")
        
    while True:
        try:
            choice = input("Select a serial port number (or press Enter to cancel): ").strip()
            if not choice:
                return None
            idx = int(choice)
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device
            else:
                print(f"Please enter a number between 1 and {len(ports)}.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")

def map_axis(mapping_val, pos_x, pos_y, pos_z):
    """
    Evaluates mapping string (e.g. 'x', '-z', 'y') to return the value 
    using the raw mocap positions.
    """
    clean_val = mapping_val.strip().lower()
    sign = 1.0
    if clean_val.startswith("-"):
        sign = -1.0
        clean_val = clean_val[1:]
    
    if clean_val == "x":
        return sign * pos_x
    elif clean_val == "y":
        return sign * pos_y
    elif clean_val == "z":
        return sign * pos_z
    else:
        raise ValueError(f"Invalid coordinate mapping axis: {mapping_val}")

def main():
    config = load_config()
    
    # 1. Parse configuration
    vrpn_conf = config.get("vrpn", {})
    mav_conf = config.get("mavlink", {})
    gps_conf = config.get("gps", {})
    axis_conf = config.get("coordinate_mapping", {})
    
    # 2. Setup VRPN Client
    client = VRPNClient(
        host=vrpn_conf.get("host", "127.0.0.1"),
        port=vrpn_conf.get("port", 3883),
        tracker_name=vrpn_conf.get("tracker_name", "Rigidbody")
    )
    
    # 3. Setup MAVLink Output Connection
    connection_type = mav_conf.get("connection_type", "udp").lower()
    port_config = mav_conf.get("port", "127.0.0.1:14550")
    baudrate = mav_conf.get("baudrate", 115200)
    system_id = mav_conf.get("system_id", 1)
    component_id = mav_conf.get("component_id", 220)
    
    connection_str = None
    
    if connection_type == "serial":
        if port_config in ["prompt", "auto"]:
            selected_port = get_serial_port()
            if not selected_port:
                logger.critical("No serial port selected. Exiting.")
                sys.exit(1)
            connection_str = selected_port
        else:
            connection_str = port_config
            
        logger.info(f"Establishing MAVLink connection on serial port: {connection_str} (baudrate: {baudrate})")
        # pymavlink uses standard serial port connection without protocol prefix if serial is selected
    else:
        # UDP Mode
        if port_config.startswith("udp:") or port_config.startswith("udpin:"):
            connection_str = port_config
        else:
            # Fallback to udpout if only address:port is provided
            connection_str = f"udpout:{port_config}"
        logger.info(f"Establishing MAVLink connection on UDP: {connection_str}")
        
    try:
        # Connect to flight controller / network port
        master = mavutil.mavlink_connection(
            connection_str, 
            baud=baudrate, 
            source_system=system_id, 
            source_component=component_id
        )
    except Exception as e:
        logger.critical(f"Failed to create MAVLink connection: {e}")
        sys.exit(1)
        
    # 4. GPS Origin Config
    origin_lat = gps_conf.get("origin_lat", 1.342859)
    origin_lon = gps_conf.get("origin_lon", 103.966484)
    origin_alt = gps_conf.get("origin_alt", 10.0)
    update_rate_hz = gps_conf.get("update_rate_hz", 10)
    send_interval = 1.0 / update_rate_hz
    
    logger.info(f"GPS Origin: Lat {origin_lat}, Lon {origin_lon}, Alt {origin_alt} m")
    logger.info(f"Target Update Rate: {update_rate_hz} Hz")
    
    # 5. Tracking Variables for Velocity Derivatives
    last_pos_time = None
    last_north = 0.0
    last_east = 0.0
    last_down = 0.0
    last_sent_time = 0.0
    
    logger.info("Bridge initialized. Waiting for VRPN updates...")
    
    # Stream from VRPN Client
    for report in client.stream_reports():
        try:
            current_time = report["time"]
            pos_x, pos_y, pos_z = report["position"]
            
            # Map raw coordinate axes to East, North, Up
            east = map_axis(axis_conf.get("east", "x"), pos_x, pos_y, pos_z)
            north = map_axis(axis_conf.get("north", "-z"), pos_x, pos_y, pos_z)
            up = map_axis(axis_conf.get("up", "y"), pos_x, pos_y, pos_z)
            down = -up
            
            # Compute Geodetic coordinates (Flat-Earth WGS84 projection)
            # Latitude: 1 degree approx 111,111 meters
            lat_deg = origin_lat + (north / 111111.0)
            
            # Longitude: depends on latitude
            rad_lat = math.radians(origin_lat)
            lon_deg = origin_lon + (east / (111111.0 * math.cos(rad_lat)))
            
            alt_m = origin_alt + up
            
            # Compute velocities (North, East, Down) from numerical derivative
            vn, ve, vd = 0.0, 0.0, 0.0
            if last_pos_time is not None:
                dt = current_time - last_pos_time
                if dt > 0.001:  # Avoid division by zero/extreme values
                    vn = (north - last_north) / dt
                    ve = (east - last_east) / dt
                    vd = (down - last_down) / dt
            
            # Save tracking states
            last_pos_time = current_time
            last_north = north
            last_east = east
            last_down = down
            
            # Rate-limit transmission to the configured update rate
            now = time.time()
            if now - last_sent_time >= send_interval:
                # Pack and send GPS_INPUT (ID 232)
                # lat/lon must be degrees * 1e7
                # alt is meters
                # velocities are meters/sec
                # hdop/vdop are 1.0 (perfect dilution of precision)
                # fix_type is 3 (3D fix)
                # satellites_visible is 15 (stable lock simulation)
                master.mav.gps_input_send(
                    int(current_time * 1e6), # time_usec (microseconds)
                    0,                       # gps_id (GPS instance ID)
                    0,                       # ignore_flags (0 = use all fields)
                    0,                       # time_week_ms (0 if unknown)
                    0,                       # time_week (0 if unknown)
                    3,                       # fix_type (3D fix)
                    int(lat_deg * 1e7),      # lat (degrees * 1e7)
                    int(lon_deg * 1e7),      # lon (degrees * 1e7)
                    alt_m,                   # alt (meters)
                    1.0,                     # hdop
                    1.0,                     # vdop
                    vn,                      # vn (velocity North, m/s)
                    ve,                      # ve (velocity East, m/s)
                    vd,                      # vd (velocity Down, m/s)
                    0.1,                     # speed_accuracy (m/s)
                    0.1,                     # horiz_accuracy (m)
                    0.1,                     # vert_accuracy (m)
                    15                       # satellites_visible
                )
                
                last_sent_time = now
                logger.info(
                    f"Sent GPS_INPUT: Lat={lat_deg:.7f}, Lon={lon_deg:.7f}, Alt={alt_m:.2f}m | "
                    f"VelNED=({vn:.2f}, {ve:.2f}, {vd:.2f}) m/s"
                )
                
        except Exception as e:
            logger.error(f"Error processing tracker report: {e}")

if __name__ == "__main__":
    main()
