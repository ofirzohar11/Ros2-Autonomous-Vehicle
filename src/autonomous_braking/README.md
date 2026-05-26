# Autonomous Emergency Braking (AEB) System

## ROS2 Humble | University Final Project - ADAS

### Features
- Realistic 3D visualization with perspective projection
- AEB with automatic braking and lane change avoidance
- 360-degree LIDAR sensor simulation with large radar display
- Professional HUD with organized telemetry data
- Reduced, natural traffic (1-3 vehicles)
- Multiple obstacle types: cars, pedestrians, children, motorcycles, etc.

### Installation
```bash
source /opt/ros/humble/setup.bash
pip3 install pygame numpy --break-system-packages
cp -r autonomous_braking ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select autonomous_braking
source install/setup.bash
```

### Run
```bash
ros2 launch autonomous_braking aeb_system.launch.py
```

### Controls
- Q / ESC: Quit
- Ctrl+C: Stop simulation

### AEB Decision Logic
| Distance    | Status    | Action                          |
|-------------|-----------|----------------------------------|
| > 50m       | SAFE      | Normal cruise                    |
| 35-50m      | CAUTION   | Slight deceleration              |
| 20-35m      | WARNING   | Braking + lane change attempt    |
| 12-20m      | DANGER    | Strong braking + emergency LC    |
| < 12m       | EMERGENCY | Full emergency brake             |
