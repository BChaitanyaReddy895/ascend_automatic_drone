# ASCEND — GPS-Denied Autonomous Hover & Auto-Land System

> Raspberry Pi 4B + Pixhawk 2.4.8 + Intel RealSense D455
> All RPi interaction via **WiFi + SSH**

## System Architecture

```
┌─────────────┐    USB 3.0    ┌──────────────┐   UART/Telem2   ┌──────────────┐
│  RealSense  │──────────────▶│  Raspberry   │────────────────▶│   Pixhawk    │
│    D455     │               │   Pi 4B      │   921600 baud   │    2.4.8     │
│             │               │              │                 │              │
│ • Depth     │               │ • VisualOdom │                 │ • EKF3       │
│ • IR/RGB    │               │ • EKF Fusion │   MAVLink       │ • PID Loops  │
│ • IMU       │               │ • Flight Ctrl│◀────────────────│ • ESC/Motors │
└─────────────┘               └──────┬───────┘                 └──────────────┘
                                     │ WiFi
                               ┌─────┴─────┐
                               │  Laptop   │
                               │  (SSH)    │
                               │           │
                               │ • Deploy  │
                               │ • Monitor │
                               │ • GCS     │
                               └───────────┘
```

## Data Pipeline

```
D455 Camera → Optical Flow (VO) → EKF Fusion (VO + IMU) → Position (x,y,z)
    → VISION_POSITION_ESTIMATE → Pixhawk EKF3 → PID → Motors
```

## Quick Start

### 1. Setup RPi (first time only)
```bash
# SSH into RPi
ssh pi@192.168.1.100

# Run setup script
chmod +x scripts/setup_rpi.sh
./scripts/setup_rpi.sh
sudo reboot
```

### 2. Configure ArduPilot
Load `config/arducopter_params.txt` in Mission Planner:
- Config → Full Parameter List → Load from file → Write Params

### 3. Deploy & Run
```bash
# From your laptop:
./scripts/ssh_deploy.sh 192.168.1.100 pi      # Deploy code
./scripts/ssh_start_mission.sh 192.168.1.100 pi # Start mission
```

### 4. Flight Procedure
1. **Arm** the drone and take off in **STABILIZE** mode
2. **Climb** to 3–4 meters altitude
3. **Switch** transmitter to **LOITER** mode
4. System automatically hovers for **60 seconds**
5. System automatically switches to **LAND** mode
6. Drone descends and lands with camera stabilization

### 5. Monitor / Stop
```bash
# Watch live output
ssh pi@192.168.1.100 'tmux attach -t ascend_mission'

# Emergency stop
./scripts/ssh_stop_mission.sh 192.168.1.100 pi

# Force kill
./scripts/ssh_stop_mission.sh 192.168.1.100 pi --force
```

## Project Structure

```
├── src/
│   ├── config.py              # All tunable parameters
│   ├── mavlink_manager.py     # Pixhawk MAVLink communication
│   ├── realsense_manager.py   # D455 camera pipeline
│   ├── visual_odometry.py     # Optical flow position tracking
│   ├── ekf_fusion.py          # Sensor fusion (VO + IMU)
│   ├── flight_controller.py   # State machine (hover → land)
│   ├── logger.py              # Timestamped logging + CSV telemetry
│   └── main.py                # Entry point (3 threads)
├── scripts/
│   ├── setup_rpi.sh           # RPi first-time setup
│   ├── ssh_deploy.sh          # Deploy code via SCP
│   ├── ssh_start_mission.sh   # Start mission via SSH
│   ├── ssh_stop_mission.sh    # Stop mission via SSH
│   └── mavproxy_bridge.sh     # Forward MAVLink to GCS over WiFi
├── config/
│   ├── arducopter_params.txt  # Pixhawk parameters for Mission Planner
│   └── wifi_config.md         # WiFi setup guide
├── tests/
│   ├── test_config.py         # Config validation
│   ├── test_ekf_fusion.py     # EKF unit tests
│   └── test_flight_logic.py   # State machine tests
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## WiFi + SSH Setup

See [config/wifi_config.md](config/wifi_config.md) for detailed WiFi configuration (client mode or hotspot mode).

## ArduPilot Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| SERIAL2_PROTOCOL | 2 | MAVLink 2 on Telem2 |
| SERIAL2_BAUD | 921 | 921600 baud rate |
| EK3_SRC1_POSXY | 6 | External Nav for X,Y |
| EK3_SRC1_POSZ | 1 | Barometer for Z |
| EK3_SRC1_YAW | 6 | External Nav for yaw |
| VISO_TYPE | 1 | MAVLink visual odometry |
| COMPASS_USE | 0 | Disable compass (indoor) |

## Running Tests (no hardware needed)

```bash
python tests/test_config.py
python tests/test_ekf_fusion.py
python tests/test_flight_logic.py
```
