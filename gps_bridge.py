import json
import math
import sys
import time
import logging
from pymavlink import mavutil
import serial.tools.list_ports
from vrpn_client import VRPNClient
from velocity_filter import VelocityFilter
from gps_noise import GPSNoiseGenerator

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

def utc_to_gps_time(utc_timestamp):
    """
    Converts a UTC Unix timestamp (seconds since 1970-01-01 00:00:00 UTC) 
    to GPS week number and GPS time of week in milliseconds (TOW ms).
    
    GPS epoch started on 1980-01-06 00:00:00 UTC.
    GPS time is continuous (no leap seconds), currently 18 seconds ahead of UTC.
    """
    GPS_EPOCH_OFFSET = 315964800  # Seconds between 1970-01-01 and 1980-01-06 UTC
    LEAP_SECONDS = 18             # GPS time - UTC time offset in seconds
    SEC_PER_WEEK = 604800

    gps_sec = utc_timestamp - GPS_EPOCH_OFFSET + LEAP_SECONDS
    time_week = int(gps_sec // SEC_PER_WEEK)
    time_week_ms = int((gps_sec % SEC_PER_WEEK) * 1000)
    return time_week, time_week_ms

def main():
    config = load_config()
    
    # 1. Parse configuration
    vrpn_conf = config.get("vrpn", {})
    mav_conf = config.get("mavlink", {})
    gps_conf = config.get("gps", {})
    mocap_conf = config.get("mocap", {})
    axis_conf = config.get("coordinate_mapping", {})
    filter_conf = config.get("velocity_filter", {})
    noise_conf = config.get("gps_noise", {})
    
    # Message type selection: "ATT_POS_MOCAP", "GPS_INPUT", or "BOTH"
    message_type = mav_conf.get("message_type", "BOTH").upper()
    
    # Setup Velocity Filter
    vel_filter = VelocityFilter(
        window_duration_s=filter_conf.get("window_duration_s", 0.2),
        ema_alpha=filter_conf.get("ema_alpha", 0.3),
        max_dt_s=filter_conf.get("max_dt_s", 0.5),
        max_velocity_ms=filter_conf.get("max_velocity_ms", 15.0),
        enabled=filter_conf.get("enabled", True)
    )

    # Setup GPS Noise Generator
    gps_noise_gen = GPSNoiseGenerator.from_dict(noise_conf)
    
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
        
    # 4. GPS & MOCAP Config
    origin_lat = gps_conf.get("origin_lat", 1.342859)
    origin_lon = gps_conf.get("origin_lon", 103.966484)
    origin_alt = gps_conf.get("origin_alt", 10.0)
    
    gps_rate = gps_conf.get("update_rate_hz", 10)
    mocap_rate = mocap_conf.get("update_rate_hz", 20)
    
    gps_interval = 1.0 / gps_rate if gps_rate > 0 else 0.1
    mocap_interval = 1.0 / mocap_rate if mocap_rate > 0 else 0.05
    
    logger.info(f"MAVLink Message Type: {message_type}")
    logger.info(f"GPS Origin: Lat {origin_lat}, Lon {origin_lon}, Alt {origin_alt} m")
    logger.info(f"Target Update Rates -> GPS: {gps_rate} Hz, MOCAP: {mocap_rate} Hz")
    
    # 5. Tracking Variables for Velocity Derivatives and Transmission Timing
    last_pos_time = None
    last_north = 0.0
    last_east = 0.0
    last_down = 0.0
    last_gps_sent_time = 0.0
    last_mocap_sent_time = 0.0
    
    logger.info("Bridge initialized. Waiting for VRPN updates...")
    
    # Stream from VRPN Client
    for report in client.stream_reports():
        try:
            current_time = report["time"]
            pos_x, pos_y, pos_z = report["position"]
            orient = report.get("orientation", (0.0, 0.0, 0.0, 1.0))
            
            # Map raw coordinate axes to East, North, Up
            east = map_axis(axis_conf.get("east", "x"), pos_x, pos_y, pos_z)
            north = map_axis(axis_conf.get("north", "-z"), pos_x, pos_y, pos_z)
            up = map_axis(axis_conf.get("up", "y"), pos_x, pos_y, pos_z)
            down = -up
            
            # Compute Geodetic coordinates (Flat-Earth WGS84 projection)
            lat_deg = origin_lat + (north / 111111.0)
            rad_lat = math.radians(origin_lat)
            lon_deg = origin_lon + (east / (111111.0 * math.cos(rad_lat)))
            alt_m = origin_alt + up
            
            # Compute velocities (North, East, Down) from numerical derivative
            vn, ve, vd = 0.0, 0.0, 0.0
            if last_pos_time is not None:
                dt = current_time - last_pos_time
                if dt > 0.001:  # Avoid division by zero/extreme values
                    raw_vn = (north - last_north) / dt
                    raw_ve = (east - last_east) / dt
                    raw_vd = (down - last_down) / dt
                    vn, ve, vd = vel_filter.update(current_time, raw_vn, raw_ve, raw_vd)
            
            # Save tracking states
            last_pos_time = current_time
            last_north = north
            last_east = east
            last_down = down
            
            now = time.time()
            # Use VRPN report timestamp if valid Unix UTC timestamp (>= 1e9), otherwise fallback to host system UTC time
            utc_time = current_time if current_time >= 1e9 else now
            time_usec = int(utc_time * 1e6)
            
            # 1. ATT_POS_MOCAP Transmission (Raw unfiltered mocap data)
            if message_type in ["ATT_POS_MOCAP", "MOCAP", "BOTH"]:
                if now - last_mocap_sent_time >= mocap_interval:
                    # Convert VRPN quaternion (qx, qy, qz, qw) to MAVLink standard [qw, qx, qy, qz]
                    qx, qy, qz, qw = orient
                    q_mav = [qw, qx, qy, qz]
                    
                    master.mav.att_pos_mocap_send(
                        time_usec,  # time_usec
                        q_mav,      # q [qw, qx, qy, qz]
                        north,      # x position (meters, North)
                        east,       # y position (meters, East)
                        down        # z position (meters, Down)
                    )
                    last_mocap_sent_time = now
                    logger.info(
                        f"Sent ATT_POS_MOCAP: PosNED=({north:.2f}, {east:.2f}, {down:.2f}) m | "
                        f"Quat=[{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]"
                    )
            
            # 2. GPS_INPUT Transmission
            if message_type in ["GPS_INPUT", "GPS", "BOTH"]:
                if now - last_gps_sent_time >= gps_interval:
                    # Apply GPS noise to position and velocity
                    noisy_n, noisy_e, noisy_d = gps_noise_gen.apply_position_noise(current_time, north, east, down)
                    noisy_up = -noisy_d

                    lat_deg = origin_lat + (noisy_n / 111111.0)
                    rad_lat = math.radians(origin_lat)
                    lon_deg = origin_lon + (noisy_e / (111111.0 * math.cos(rad_lat)))
                    alt_m = origin_alt + noisy_up

                    noisy_vn, noisy_ve, noisy_vd = gps_noise_gen.apply_velocity_noise(current_time, vn, ve, vd)

                    time_week, time_week_ms = utc_to_gps_time(utc_time)
                    master.mav.gps_input_send(
                        time_usec,               # time_usec
                        0,                       # gps_id
                        0,                       # ignore_flags
                        time_week_ms,            # time_week_ms
                        time_week,               # time_week
                        3,                       # fix_type
                        int(lat_deg * 1e7),      # lat
                        int(lon_deg * 1e7),      # lon
                        alt_m,                   # alt
                        1.0,                     # hdop
                        1.0,                     # vdop
                        noisy_vn,                # vn
                        noisy_ve,                # ve
                        noisy_vd,                # vd
                        0.1,                     # speed_accuracy
                        0.1,                     # horiz_accuracy
                        0.1,                     # vert_accuracy
                        15                       # satellites_visible
                    )
                    last_gps_sent_time = now
                    logger.info(
                        f"Sent GPS_INPUT: Lat={lat_deg:.7f}, Lon={lon_deg:.7f}, Alt={alt_m:.2f}m | "
                        f"VelNED=({noisy_vn:.2f}, {noisy_ve:.2f}, {noisy_vd:.2f}) m/s"
                    )
                
        except Exception as e:
            logger.error(f"Error processing tracker report: {e}")

if __name__ == "__main__":
    main()
