# Autonomous Emergency Braking (AEB) System
### ROS2 Humble | University Final Project — ADAS

---

## What is an Autonomous Emergency Braking (AEB) System?

An AEB system is a safety feature that **automatically detects obstacles** in the vehicle's path and applies emergency braking without driver input. It is a core component of modern ADAS (Advanced Driver Assistance Systems).

This project simulates a full AEB pipeline using **ROS2**, a **LIDAR sensor model**, a **5-level Finite State Machine (FSM)**, two **PID controllers** for smooth longitudinal control (speed + braking), intelligent **lane-change logic**, and a real-time **Pygame visualizer**.

---

## How the FSM Works in This Project

The AEB system uses a **Finite State Machine** to decide *what the policy should be* (target speed, whether to change lane), based on the distance and time-to-collision (TTC) with the nearest obstacle. The actual **how-much-gas / how-much-brake** is then produced by two **PID controllers** (see next section).

```
LIDAR Sensor → Obstacle Detection → FSM Safety State (policy)
                                          │
                                          ├─► target speed ──► Speed PID  ──► throttle
                                          └─► safe gap      ──► Brake PID  ──► brake force
                                                                    │
                                                              Lane-change logic → Visualizer
```

👉 The 5 FSM safety states set the policy:

| State | Distance | Policy (target speed / lane) |
|-------|----------|------------------------------|
| ✅ SAFE | > 30 m | Cruise at target speed |
| ⚠️ CAUTION | 20–30 m | Reduce target speed / prepare lane change |
| 🟡 WARNING | 15–20 m | Lower target speed + lane change attempt |
| 🔴 DANGER | 10–15 m | Strongly reduce target speed + emergency lane change |
| 🚨 EMERGENCY | < 10 m | Target speed = 0 → full emergency brake |

> The braking force in every state is computed by the **Brake PID** based on how far the vehicle is *inside* its safe following gap — **except** in the EMERGENCY range / on a critical TTC, where a hard override forces **100% braking** as a deterministic safety reflex.

> **TTC Override:** If Time-To-Collision < 0.8 seconds *and* the obstacle is within the warning range (< 15 m), the emergency full-brake is triggered regardless of the current FSM zone.

---

## 🔥 Why FSM + PID Together

✅ **Predictable** → The FSM gives clear, defined safety states (policy)  
✅ **Smooth** → The PIDs turn that policy into smooth, proportional gas/brake instead of fixed steps  
✅ **Safe** → A deterministic emergency override always beats the PID when it matters  
✅ **Scalable** → Easy to add states, and the controller gains are tuned in one place (`config.py`)  

---

## 🎚️ Longitudinal Control — Two PID Controllers

All longitudinal motion (gas + brake) is handled by two PID controllers in `pid_controller.py`. A PID turns an **error** (the gap between what we want and what we have) into a control command:

```
error  = target − current
output = Kp·error + Ki·∫error·dt + Kd·d(error)/dt
```

| Controller | Tracks | Error | Output |
|-----------|--------|-------|--------|
| **Speed PID** | target speed | `target_speed − current_speed` | throttle / ease-off (m/s²) |
| **Brake PID** | safe following gap | `desired_gap − actual_distance` | brake force (0–100%) |

- **Speed PID** — accelerates toward the target speed and holds it (cruise / adaptive-cruise following).
- **Brake PID** — the closer we are inside the desired safe gap (and the faster we close it), the harder it brakes. The safe gap grows with speed:  
  `desired_gap = min_gap + time_headway · speed`.
- The two cascade cleanly: while the brakes are active the Speed PID is reset so they don't fight each other.
- **Emergency override:** in the EMERGENCY range or on a critical TTC, the Brake PID is bypassed and full braking is commanded directly.

All gains live in `config.py` (`PIDConfig` for speed, `BrakePIDConfig` for braking) and are easy to tune.

---

## ROS2 Architecture

```
simulation_node  ──► /vehicle_state   ──► visualizer_node
                 ──► /obstacle_data   ──► visualizer_node
                 ──► /aeb_status      ──► visualizer_node
                 ──► /ego_speed       ──► visualizer_node
```

- **simulation_node** — Physics engine, LIDAR, FSM logic, scenario manager
- **visualizer_node** — Real-time Pygame rendering and HUD dashboard

---

## Project Structure

```
ros2_AB/
├── src/
│   └── autonomous_braking/
│       ├── autonomous_braking/
│       │   ├── config.py          # All parameters, PID gains, and enums
│       │   ├── pid_controller.py   # Generic PID controller (speed + braking)
│       │   ├── simulation.py       # Physics, AEB logic, PID control, scenarios, ROS2 node
│       │   └── visualizer.py       # Pygame rendering and HUD dashboard
│       ├── launch/
│       │   └── aeb_system.launch.py
│       ├── config/
│       │   └── aeb_params.yaml
│       ├── sprites/             # PNG assets
│       ├── package.xml
│       └── setup.py
└── .gitignore
```

---

## ⚙️ Installation (First Time Only)

**1. Install Python dependencies:**
```bash
pip install pygame numpy
```

**2. Navigate to the project directory:**
```bash
cd Ros2_Autonomous-Vehicle-main
```

**3. Build the package:**
```bash
colcon build --packages-select autonomous_braking
```

**4. Source the workspace:**
```bash
source install/setup.bash
```

**5. Launch the simulation:**
```bash
ros2 launch autonomous_braking aeb_system.launch.py
```

---

## 🚀 Running the Simulation

Open a **fresh terminal** and run:

```bash
cd Ros2_Autonomous-Vehicle-main
source install/setup.bash
ros2 launch autonomous_braking aeb_system.launch.py
```

**After modifying source files, rebuild first:**
```bash
cd Ros2_Autonomous-Vehicle-main
colcon build --packages-select autonomous_braking
source install/setup.bash
ros2 launch autonomous_braking aeb_system.launch.py
```

> ⚠️ **Important:** Always use a **fresh terminal** — old terminals may have stale environment variables that cause conflicts.

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| `Q` / `ESC` | Quit visualizer |
| `Ctrl+C` | Stop all nodes |

---

## 🚗 Scenarios

Scenarios trigger **automatically** every 6–11 seconds during the simulation.

| Scenario | Description |
|----------|-------------|
| Front Vehicle Braking | Car ahead brakes hard — ego follows and brakes |
| Fallen Tree (Right) | Tree blocks right lane — ego changes left |
| Fallen Tree (Left) | Tree blocks left lane — ego changes right |
| Pedestrian Crossing | Child crosses all lanes — ego performs emergency stop |

---

## 🔭 LIDAR Sensor Model

- **FOV:** ±60° (forward-facing)
- **Beams:** 120 rays
- **Range:** up to 150 m
- Detects all obstacle types: cars, pedestrians, children, motorcycles, bicycles, animals, fallen trees

---

## 🧠 Obstacle Types

| Type | Behavior |
|------|----------|
| 🚗 Cars | Same direction, oncoming, or braking |
| 🚶 Pedestrians / Children | Cross lanes unpredictably |
| 🏍️ Motorcycles / Bicycles | Same direction |
| 🐕 Animals | Random crossing |
| 🌲 Fallen Trees | Static obstacle |

---

## Features

- Real-time **Pygame visualizer** with 3D perspective and animated environment
- **5-level FSM** safety state machine (policy layer)
- **Two PID controllers** for smooth longitudinal control — speed (throttle) and braking (safe gap)
- Live **PID telemetry panel** showing target vs current, error, output, and P/I/D terms
- **Forward-facing LIDAR** sensor simulation (±60° FOV, 120 beams)
- Professional **HUD dashboard** with full telemetry data
- **Dynamic background traffic** (3–6 vehicles)
- **Intelligent lane-change logic** to avoid obstacles
- Multiple randomized **scenario events**
