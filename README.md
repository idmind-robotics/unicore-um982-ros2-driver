# Unicore UM982 ROS 2 Driver

A comprehensive ROS 2 driver for the Unicore UM982 dual-antenna RTK GPS receiver, providing high-frequency position and heading data with RTK correction support.

This driver provides high-frequency, high-accuracy position and heading data to the ROS 2 ecosystem by parsing the proprietary Unicore PVTSLN message format and publishing standard ROS 2 navigation messages.

## Features

- **High-frequency GPS data**: 20Hz position and heading updates
- **RTK correction support**: NTRIP corrections relayed via str2str
- **Single-owner serial**: the driver node is the only process that opens the GPS serial port; str2str relays NTRIP to a local TCP port instead of writing to the receiver directly
- **Automatic hardware configuration**: 18-command UM982 initialization sequence
- **Standard ROS 2 messages**: sensor_msgs/NavSatFix and sensor_msgs/Imu
- **Comprehensive diagnostics**: Real-time GPS health monitoring on `/gps/diagnostics`
- **Flexible parameterization**: YAML configuration with launch argument override
- **Professional launch system**: Single-command deployment with error handling

## Supported Fix Types

- **SINGLE**: Standard GPS positioning (~3-5m accuracy)
- **DGPS**: Differential GPS positioning (~1-3m accuracy)  
- **RTK_FLOAT**: RTK float solution (~10-50cm accuracy)
- **RTK_FIXED**: RTK fixed solution (~1-5cm accuracy)

## Requirements

- **ROS 2**: Jazzy or later (tested on Jazzy)
- **Operating System**: Linux (Ubuntu 22.04+ recommended)
- **Hardware**: Unicore UM982 dual-antenna RTK GPS receiver
- **Dependencies**: 
  - rclcpp
  - serial_driver
  - sensor_msgs
  - geometry_msgs
  - diagnostic_updater
  - RTKLIB (str2str utility for NTRIP corrections)

## Prerequisites

### Installing RTKLIB (str2str utility)

The driver requires the `str2str` utility from RTKLIB for NTRIP RTK corrections. Follow these steps to install it:

1. **Install dependencies:**
   ```bash
   sudo apt update && sudo apt install build-essential
   ```

1. **Clone the RTKLIB repository:**
   ```bash
   cd ~
   git clone https://github.com/idmind-robotics/unicore-um982-ros2-driver.git
   cd RTKLIB
   ```

2. **Build the CUI applications:**
   ```bash
   cd app/str2str/gcc
   make
   ```

3. **Install str2str to system PATH:**
   ```bash
   sudo cp str2str /usr/local/bin/
   ```

4. **Verify installation:**
   ```bash
   str2str -h
   ```
   You should see the str2str help message.


## Installation

1. **Create a ROS 2 workspace (if you don't have one):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/tyleralford/unicore-um982-ros2-driver.git um982-driver
   ```

3. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the package:**
   ```bash
   colcon build --packages-select unicore_um982_driver
   ```

5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Configuration

The driver uses a YAML configuration file located at `config/unicore_driver_params.yaml`. Key parameters include:

### GPS Connection Parameters
- `port`: Serial port device (default: `/dev/ttyUSB0`)
- `baudrate`: Serial communication rate (default: `115200`)

### NTRIP Client Parameters
- `ntrip_server`: NTRIP caster hostname (default: `rtk2go.com`)
- `ntrip_port`: NTRIP caster port (default: `2101`)
- `ntrip_user`: NTRIP username (default: `user`)
- `ntrip_pass`: NTRIP password (default: `password`)
- `ntrip_mountpoint`: NTRIP mountpoint (default: `FIXED`)

### Local Corrections Bridge
- `ntrip_local_port`: Local TCP port that str2str relays RTCM corrections to (default: `40001`); the driver node connects to this port and forwards the bytes into the serial port
- `enable_ntrip`: Set `false` to run without RTK corrections (default: `true`)

The GPS serial port is owned exclusively by the driver node. str2str never touches
`/dev/ttyUSB*`; instead it acts as an NTRIP → local TCP relay:

```
NTRIP caster ──> str2str ──> 127.0.0.1:40001 (tcpsvr) ──> driver node ──> /dev/ttyUSB0
```

### UM982 Configuration Commands
The driver automatically configures the UM982 with a command sequence optimizing it for RTK positioning, including:
- Multi-constellation support (GPS, GLONASS, Galileo, BeiDou, QZSS)
- RTK settings with 300-second timeout
- Smoothing filters for improved accuracy
- High-frequency PVTSLN message output

## Usage

### Basic Usage (with NTRIP RTK corrections)

```bash
ros2 launch unicore_um982_driver unicore.launch.py
```

### Usage without NTRIP corrections

```bash
ros2 launch unicore_um982_driver unicore.launch.py enable_ntrip:=false
```

### Custom NTRIP configuration

```bash
ros2 launch unicore_um982_driver unicore.launch.py \
  ntrip_server:=your.ntrip.server \
  ntrip_port:=2101 \
  ntrip_user:=your_username \
  ntrip_pass:=your_password \
  ntrip_mountpoint:=YOUR_MOUNTPOINT
```

Launch arguments take precedence over the values in `config/unicore_driver_params.yaml`.
Any argument you do not set falls back to the corresponding YAML value (and then to
the default shown in the table below).

### Custom GPS port

```bash
ros2 launch unicore_um982_driver unicore.launch.py \
  gps_port:=/dev/ttyUSB1 \
  gps_baudrate:=115200
```

## Published Topics

- `/gps/fix` (sensor_msgs/NavSatFix): GPS position data with covariance
- `/gps/imu` (sensor_msgs/Imu): Heading and orientation data
- `/gps/diagnostics` (diagnostic_msgs/DiagnosticArray): GPS health status

## Diagnostic Status Levels

The driver publishes comprehensive diagnostic information:

- **OK (Green)**: RTK_FIXED solution achieved
- **WARN (Yellow)**: SINGLE, DGPS, or RTK_FLOAT positioning
- **ERROR (Red)**: No GPS data received or unknown status

Diagnostic data includes:
- Position fix status
- Satellite count
- Data freshness (age in seconds)
- NTRIP bridge status (CONNECTED / DISCONNECTED / DISABLED)
- Hardware identification

## Launch File Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `config_file` | string | `unicore_driver_params.yaml` | Path to parameter file |
| `enable_ntrip` | bool | `true` | Enable NTRIP RTK corrections |
| `ntrip_server` | string | `rtk2go.com` | NTRIP caster hostname |
| `ntrip_port` | int | `2101` | NTRIP caster port |
| `ntrip_user` | string | `user` | NTRIP username |
| `ntrip_pass` | string | `password` | NTRIP password |
| `ntrip_mountpoint` | string | `FIXED` | NTRIP mountpoint |
| `ntrip_local_port` | int | `40001` | Local TCP port str2str relays corrections to |
| `gps_port` | string | `/dev/ttyUSB0` | GPS serial port |
| `gps_baudrate` | int | `115200` | GPS serial baudrate (one of 9600, 115200, 230400) |

All of these arguments override the YAML config file when set on the command line;
omitted arguments use the YAML values (`config/unicore_driver_params.yaml`) as defaults.

`gps_baudrate` and `ntrip_local_port` are validated at launch: a non-integer value
(e.g. `gps_baudrate:=abc`) aborts the launch with a clear error. `gps_baudrate` also
overrides the receiver's own `CONFIG COM3 <baud>` command, so host and receiver always
agree on the line rate.

## Troubleshooting

### GPS not detected
- Check that the UM982 is connected via USB or serial
- Verify the correct port: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- Ensure proper baudrate (UM982 default is usually 115200 or 230400)

### NTRIP connection issues
- Verify str2str is installed: `which str2str`
- Check NTRIP credentials and mountpoint availability
- Test the NTRIP → TCP bridge manually:
  ```bash
  str2str -in ntrip://user:pass@server:port/mountpoint -out tcpsvr://127.0.0.1:40001 -r 1000
  ```
  Then confirm the node reports `NTRIP bridge: CONNECTED` on `/gps/diagnostics`.

### No RTK fix
- Ensure clear sky view for GPS antennas
- Verify NTRIP corrections are streaming
- Check diagnostic messages: `ros2 topic echo /gps/diagnostics`
- Monitor GPS status: `ros2 topic echo /gps/fix`

### Permission denied on serial port
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

## Monitoring and Debugging

### Monitor GPS fix status
```bash
ros2 topic echo /gps/fix
```

### Monitor diagnostic status
```bash
ros2 topic echo /gps/diagnostics
```

### Monitor all GPS topics
```bash
ros2 topic list | grep gps
ros2 topic hz /gps/fix  # Check data rate
```

### Launch with debug logging
```bash
ros2 launch unicore_um982_driver unicore.launch.py --ros-args --log-level DEBUG
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
