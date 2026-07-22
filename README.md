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

Configure your VRPN host, MAVLink connections, origin coordinates, and message transmission parameters in `config.json`:

```json
{
  "vrpn": {
    "host": "192.168.1.100",
    "port": 3883,
    "tracker_name": "ndpmonoco"
  },
  "mavlink": {
    "connection_type": "serial",
    "port": "auto",
    "baudrate": 115200,
    "system_id": 1,
    "component_id": 220,
    "message_type": "BOTH"
  },
  "gps": {
    "origin_lat": 1.341575,
    "origin_lon": 103.964866,
    "origin_alt": 10.0,
    "update_rate_hz": 10
  },
  "mocap": {
    "update_rate_hz": 20
  },
  "coordinate_mapping": {
    "east": "-x",
    "north": "-y",
    "up": "z"
  },
  "velocity_filter": {
    "enabled": true,
    "window_duration_s": 0.5,
    "ema_alpha": 0.5,
    "max_dt_s": 0.5,
    "max_velocity_ms": 15.0
  }
}
```

### 📋 Field Descriptions & Available Options

| Section | Parameter | Type | Description | Available Options / Example Values |
| :--- | :--- | :--- | :--- | :--- |
| **`vrpn`** | `host` | `string` | IP address or hostname of the VRPN server. | e.g. `"192.168.1.100"`, `"127.0.0.1"` |
| | `port` | `integer` | TCP port of the VRPN server. | Default `3883` |
| | `tracker_name` | `string` | Rigid body name defined in MoCap software (OptiTrack/Vicon/PhaseSpace). | e.g. `"Rigidbody"`, `"ndpmonoco"` |
| **`mavlink`** | `connection_type` | `string` | Transport protocol for MAVLink telemetry. | `"udp"`, `"serial"` |
| | `port` | `string` | UDP endpoint or serial port device path. | UDP: `"127.0.0.1:14550"`, `"udpin:0.0.0.0:14550"`<br>Serial: `"/dev/ttyUSB0"`, `"COM3"`, or `"auto"` / `"prompt"` for interactive detection |
| | `baudrate` | `integer` | Baud rate for serial connections. | e.g. `57600`, `115200`, `921600` |
| | `system_id` | `integer` | MAVLink System ID of the bridge node. | `1` to `255` (default `1`) |
| | `component_id` | `integer` | MAVLink Component ID of the bridge node. | `1` to `255` (default `220` for `MAV_COMP_ID_GPS`) |
| | `message_type` | `string` | MAVLink message output mode. | `"ATT_POS_MOCAP"` (raw mocap pose), `"GPS_INPUT"` (simulated GPS), `"BOTH"` (send both) |
| **`gps`** | `origin_lat` | `float` | WGS-84 reference origin latitude in degrees. | `-90.0` to `90.0` (default `1.341575`) |
| | `origin_lon` | `float` | WGS-84 reference origin longitude in degrees. | `-180.0` to `180.0` (default `103.964866`) |
| | `origin_alt` | `float` | Reference origin altitude in meters above MSL. | e.g. `10.0` |
| | `update_rate_hz` | `number` | Broadcast frequency for `GPS_INPUT` messages (Hz). | Positive number (default `10`) |
| **`mocap`** | `update_rate_hz` | `number` | Broadcast frequency for `ATT_POS_MOCAP` messages (Hz). | Positive number (default `20`) |
| **`coordinate_mapping`** | `east` | `string` | VRPN axis mapping to local East direction. | `"x"`, `"-x"`, `"y"`, `"-y"`, `"z"`, `"-z"` |
| | `north` | `string` | VRPN axis mapping to local North direction. | `"x"`, `"-x"`, `"y"`, `"-y"`, `"z"`, `"-z"` |
| | `up` | `string` | VRPN axis mapping to local Up direction. | `"x"`, `"-x"`, `"y"`, `"-y"`, `"z"`, `"-z"` |
| **`velocity_filter`** | `enabled` | `boolean` | Enable numerical velocity filtering & spike rejection. | `true`, `false` |
| | `window_duration_s` | `float` | Moving median filter window duration (seconds). | e.g. `0.1` to `1.0` (default `0.5`) |
| | `ema_alpha` | `float` | Exponential Moving Average (EMA) smoothing coefficient. | `0.0` to `1.0` (default `0.5`) |
| | `max_dt_s` | `float` | Maximum delta time gap before filter state reset. | Positive float (default `0.5`) |
| | `max_velocity_ms` | `float` | Velocity magnitude threshold for outlier clamping (m/s). | Positive float (default `15.0`) |

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
