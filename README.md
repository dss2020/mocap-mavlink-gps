# Mocap VRPN to MAVLink GPS_INPUT Bridge

This Python application acts as a geodetic bridge, translating motion capture (mocap) tracking data from a VRPN server into MAVLink `GPS_INPUT` (ID 232) messages to simulate GPS lock and telemetry updates on ArduPilot/PX4 flight controllers.

---

## 🛠 Features

- **Pure-Python VRPN TCP Client**: Low latency, lightweight, zero binary dependencies.
- **Dynamic System Registration**: Automatically negotiates VRPN version handshake and maps sender/type descriptors.
- **Dynamic Axis Mapping**: Map mocap cartesian coordinates (e.g. OptiTrack Motive `X=East, Y=Up, Z=South`) to NED via `config.json`.
- **Geodetic Translation**: Translates local coordinates (in meters) to global WGS-84 coordinates based on a configurable home origin (Singapore by default).
- **Numerical Velocity Derivatives**: Automatically calculates 3D velocities ($v_n, v_e, v_d$) from sequential positions and timestamps.
- **Rate-Limiting**: Limits MAVLink broadcasts to 10 Hz (configurable) to prevent overloading the flight controller.
- **Interactive Serial Fallback**: Scans system serial ports and prompts for selection if serial connection is selected.

---

## 📁 Project Structure

- `gps_bridge.py`: Main entry point. Handles setup, flat-Earth geodetic projection, velocity estimation, rate-limiting, and serial prompt logic.
- `vrpn_client.py`: Socket client implementing the VRPN TCP protocol parsing (24-byte version cookie exchange and dynamic descriptor mappings).
- `config.json`: Master configuration file.
- `simulate_vrpn_server.py`: Utility that mocks a VRPN server streaming 3D circular coordinates at 20Hz.
- `test_bridge.py`: Unit tests validating geodetic translations, velocity calculations, and axis mappings.

---

## ⚙️ Configuration (`config.json`)

Configure your VRPN host, MAVLink connections, and origin coordinates in `config.json`:

```json
{
  "vrpn": {
    "host": "192.168.1.100",
    "port": 3883,
    "tracker_name": "Rigidbody"
  },
  "mavlink": {
    "connection_type": "udp",
    "port": "127.0.0.1:14550",
    "baudrate": 115200,
    "system_id": 1,
    "component_id": 220
  },
  "gps": {
    "origin_lat": 1.342859,
    "origin_lon": 103.966484,
    "origin_alt": 10.0,
    "update_rate_hz": 10
  },
  "coordinate_mapping": {
    "east": "x",
    "north": "-z",
    "up": "y"
  }
}
```

### Configuration Options:
- **VRPN**:
  - `host`: The IP address of the VRPN server (e.g. `192.168.1.100`).
  - `port`: Port number (`3883` is the standard VRPN port).
  - `tracker_name`: The name of the tracked rigid body in your mocap software.
- **MAVLink**:
  - `connection_type`: `"udp"` or `"serial"`.
  - `port`: UDP endpoint (e.g. `127.0.0.1:14550`) or serial port path. Use `"prompt"` or `"auto"` to trigger serial port discovery and user prompts.
  - `baudrate`: Connection baudrate (default `115200` for serial).
  - `system_id`: MAVLink System ID of the bridge (default `1`).
  - `component_id`: MAVLink Component ID of the bridge (default `220` / `MAV_COMP_ID_GPS`).
- **GPS**:
  - `origin_lat`: Origin Latitude (default `1.342859`).
  - `origin_lon`: Origin Longitude (default `103.966484`).
  - `origin_alt`: Origin Altitude in meters (default `10.0`).
  - `update_rate_hz`: Frequency at which GPS_INPUT messages are sent (default `10`).
- **Coordinate Mapping**:
  - Defines which local coordinates represent the East, North, and Up axes. Prepend a minus sign (`-`) to invert the direction of an axis.

---

## 🚀 How to Run

Initialize the virtual environment and install dependencies:
```bash
# Install and run unit tests
uv run python -m unittest test_bridge.py

# Run the mock VRPN server (for testing)
uv run simulate_vrpn_server.py

# Start the GPS Bridge
uv run gps_bridge.py
```

### 1. Verification Example (Circular Path Simulation)

To verify the bridge:
1. Edit `config.json` and change `"vrpn.host"` to `"127.0.0.1"`.
2. Start the mock server:
   ```bash
   uv run simulate_vrpn_server.py
   ```
3. Start the bridge:
   ```bash
   uv run gps_bridge.py
   ```
4. Listen for MAVLink telemetry on UDP port 14550 (e.g., using QGroundControl, Mission Planner, or pymavlink).
