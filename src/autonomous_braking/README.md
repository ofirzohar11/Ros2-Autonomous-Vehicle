# Autonomous Emergency Braking (AEB) System

## ROS2 Humble | University Final Project - ADAS

### Features
- Realistic 3D perspective visualization with animated environment
- AEB with automatic braking and lane change avoidance
- Two PID controllers for smooth longitudinal control: speed (throttle) + braking (safe gap)
- Live PID telemetry panel (target vs current, error, output, P/I/D terms)
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
│       │   ├── config.py          # All parameters, PID gains, and enums
│       │   ├── pid_controller.py   # Generic PID controller (speed + braking)
│       │   ├── simulation.py       # Physics, AEB logic, PID control, scenarios, ROS2 node
│       │   └── visualizer.py       # Pygame rendering and dashboard
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

The FSM sets the **policy** (target speed + lane decision) per zone; two PID
controllers produce the actual throttle and brake (see *Longitudinal Control* below).

| Distance  | Status    | Policy (target speed / lane)              |
|-----------|-----------|-------------------------------------------|
| > 30 m    | SAFE      | Cruise at target speed                    |
| 20–30 m   | CAUTION   | Reduce target speed / prepare lane change |
| 15–20 m   | WARNING   | Lower target speed + lane change attempt  |
| 10–15 m   | DANGER    | Strongly reduce target speed + emergency LC |
| < 10 m    | EMERGENCY | Target speed = 0 → full emergency brake   |

TTC (Time-To-Collision) override: emergency full-brake if TTC < 0.8 s regardless of distance.

---

### Longitudinal Control — Two PID Controllers

All gas/brake is produced by PID controllers in `pid_controller.py`, turning an
error (`target − current`) into a command: `Kp·error + Ki·∫error·dt + Kd·d(error)/dt`.

| Controller | Tracks            | Error                          | Output               |
|-----------|-------------------|--------------------------------|----------------------|
| Speed PID  | target speed      | `target_speed − current_speed` | throttle / ease-off  |
| Brake PID  | safe following gap| `desired_gap − actual_distance`| brake force (0–100%) |

- Safe gap grows with speed: `desired_gap = min_gap + time_headway · speed`.
- The closer/faster you close the gap, the harder the Brake PID brakes.
- While braking, the Speed PID is reset so the two controllers don't fight.
- **Emergency override:** in the EMERGENCY range / on a critical TTC the Brake PID
  is bypassed and full braking is commanded directly.
- Gains are tunable in `config.py` — `PIDConfig` (speed) and `BrakePIDConfig` (braking).

---

### Scenarios

| Scenario | Description |
|----------|-------------|
| Front Vehicle Braking | Car ahead brakes hard — ego follows and brakes |
| Fallen Tree (Right) | Tree blocks right lane — ego changes left |
| Fallen Tree (Left) | Tree blocks left lane — ego changes right |
| Pedestrian Crossing | Child crosses all lanes — ego performs emergency stop |

Scenarios trigger automatically every 6–11 seconds during the simulation.
