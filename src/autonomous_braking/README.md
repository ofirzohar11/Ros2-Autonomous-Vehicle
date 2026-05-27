# Autonomous Emergency Braking (AEB) System

## ROS2 Humble | University Final Project - ADAS

### Features
- Realistic 3D perspective visualization with animated environment
- AEB with automatic braking and lane change avoidance
- Forward-facing LIDAR sensor simulation (±60° FOV, 120 beams)
- Professional HUD dashboard with full telemetry data
- Dynamic background traffic (3–6 vehicles)
- Multiple obstacle types: cars, pedestrians, children, motorcycles, bicycles, animals, fallen trees
- Scenario system: front vehicle braking, fallen tree, pedestrian crossing

---

### Project Structure

```
ros2_AB/
├── src/
│   └── autonomous_braking/
│       ├── autonomous_braking/
│       │   ├── config.py        # All parameters and enums
│       │   ├── simulation.py    # Physics, AEB logic, scenarios, ROS2 node
│       │   └── visualizer.py    # Pygame rendering and dashboard
│       ├── launch/
│       │   └── aeb_system.launch.py
│       ├── config/
│       │   └── aeb_params.yaml
│       ├── sprites/             # Pre-generated PNG assets
│       ├── package.xml
│       └── setup.py
└── .gitignore
```

---

### Installation (first time only)

**1. Copy the project into your ROS2 workspace:**
```bash
mkdir -p ~/ros2_AB/src
cp -r /mnt/c/Users/<your-user>/Downloads/Autonomous\ Vehicle/ros2_AB/src/autonomous_braking ~/ros2_AB/src/
```

**2. Install Python dependencies:**
```bash
pip install pygame numpy
```

**3. Build the package:**
```bash
cd ~/ros2_AB
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomous_braking
```

---

### Running the Simulation

Open a **fresh terminal** and run each line separately:

```bash
cd ~/ros2_AB
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomous_braking aeb_system.launch.py
```

**After modifying source files, rebuild first:**
```bash
cd ~/ros2_AB
source /opt/ros/humble/setup.bash
export DISPLAY=:0
colcon build --packages-select autonomous_braking
source install/setup.bash
ros2 launch autonomous_braking aeb_system.launch.py
```

> **Important:** Always use a **fresh terminal** — old terminals may have stale environment variables from previous runs that cause conflicts.

---

### Controls

| Key | Action |
|-----|--------|
| Q / ESC | Quit visualizer |
| Ctrl+C | Stop all nodes |

---

### AEB Decision Logic

| Distance  | Status    | Action                            |
|-----------|-----------|-----------------------------------|
| > 30 m    | SAFE      | Normal cruise                     |
| 20–30 m   | CAUTION   | Slight deceleration               |
| 15–20 m   | WARNING   | Braking + lane change attempt     |
| 10–15 m   | DANGER    | Strong braking + emergency LC     |
| < 10 m    | EMERGENCY | Full emergency brake              |

TTC (Time-To-Collision) override: emergency stop if TTC < 0.8 s regardless of distance.

---

### Scenarios

| Scenario | Description |
|----------|-------------|
| Front Vehicle Braking | Car ahead brakes hard — ego follows and brakes |
| Fallen Tree (Right) | Tree blocks right lane — ego changes left |
| Fallen Tree (Left) | Tree blocks left lane — ego changes right |
| Pedestrian Crossing | Child crosses all lanes — ego performs emergency stop |

Scenarios trigger automatically every 6–11 seconds during the simulation.
