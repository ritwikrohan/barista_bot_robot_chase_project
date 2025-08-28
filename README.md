# Rick Chasing Morty
Multi-robot coffee serving simulation with differential drive robots, cup holder mechanisms, and coordinated fleet operations in ROS2.

## Overview

This project implements autonomous barista robots capable of navigating café environments and serving beverages. The system features modular XACRO-based robot design with configurable components, multi-robot coordination capabilities (Rick & Morty themed demo), and full simulation support in Gazebo. Perfect for learning robot modeling, multi-robot systems, and service robotics applications.

## Demo

### Multi-Robot Coordination (Rick & Morty - Catch me if you can)
<p align="center">
  <img src="https://i.imgflip.com/a4fs7m.gif" alt="Multi-Robot Demo" width="600">
</p>

*Two robots (Rick in blue, Morty in red) operating simultaneously in shared workspace with Rick chasing Morty*

## Key Features

- **Modular XACRO Design**: Reusable components (wheels, standoffs, cup holder) for easy customization
- **Multi-Robot Support**: Namespace-based separation for fleet operations
- **Differential Drive Control**: Two-wheel drive with caster wheels for stability
- **Cup Holder Platform**: Elevated tray design for safe beverage transport
- **Laser-Based Perception**: 360° LiDAR scanning for navigation and obstacle avoidance
- **Dynamic Robot Spawning**: Configurable robot colors and positions
- **RViz Integration**: Real-time visualization of robot state and sensor data
- **Gazebo Simulation**: Physics-based testing environment

## Robot Specifications

| Component | Specification | Details |
|-----------|--------------|---------|
| Base Platform | Cylindrical body | Diameter: 0.356m, Height: 0.155m |
| Drive System | Differential drive | 2 powered wheels + 2 caster wheels |
| Cup Holder | Elevated platform | Diameter: 0.32m, Height from base: 0.342m |
| Sensors | 2D LiDAR | Range: 0.15-5.0m, 200 samples, 360° FOV |
| Wheel Configuration | Active + Passive | Active: 0.0704m diameter, Passive: Spherical casters |
| Support Structure | 4 standoff rods | Height: 0.22m, connecting base to tray |
| Max Payload | Cup holder capacity | ~2kg (typical for 4-6 beverages) |
| Simulation Rate | Update frequency | Joint states: 30Hz, LiDAR: 100Hz |

## Technical Stack

- **Framework**: ROS2 Humble
- **Simulation**: Gazebo Classic
- **Robot Description**: URDF/XACRO
- **Visualization**: RViz2
- **Control**: gazebo_ros_diff_drive plugin
- **Sensors**: gazebo_ros_ray_sensor (LiDAR)
- **State Publishing**: robot_state_publisher
- **Transform Tree**: tf2_ros

## Installation

### Prerequisites
```bash
# ROS2 Humble
sudo apt update && sudo apt install ros-humble-desktop-full

# Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs

# XACRO processor
sudo apt install ros-humble-xacro

# Additional tools
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

### Build from Source
```bash
# Create workspace
mkdir -p ~/barista_ws/src
cd ~/barista_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/barista-robot.git
cd ..

# Build
colcon build --packages-select barista_robot_description
source install/setup.bash
```

## Usage

### Launch Single Robot
```bash
# Method 1: Using URDF directly
ros2 launch barista_robot_description barista_urdf.launch.py

# Method 2: Using XACRO (recommended)
ros2 launch barista_robot_description barista_xacro.launch.py
```

### Launch Multi-Robot System (Rick & Morty)
```bash
# Spawn two robots with different namespaces
ros2 launch barista_robot_description barista_two_robots.launch.py
```

### Control Robot Movement
```bash
# Single robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Multi-robot (Rick)
ros2 topic pub /rick/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Multi-robot (Morty)
ros2 topic pub /morty/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: -0.3}}"
```

### Visualization Commands
```bash
# View laser scan data
ros2 topic echo /scan

# Monitor joint states
ros2 topic echo /joint_states

# Check transform tree
ros2 run tf2_tools view_frames
```

## Repository Structure

```
barista_robot_description/
├── launch/                          # Launch files
│   ├── barista_urdf.launch.py      # Single robot with URDF
│   ├── barista_xacro.launch.py     # Single robot with XACRO
│   └── barista_two_robots.launch.py # Multi-robot setup
├── urdf/                            # Robot descriptions
│   └── barista_robot_model.urdf    # Complete URDF model
├── xacro/                           # Modular components
│   ├── barista_robot_model.urdf.xacro           # Main robot
│   ├── barista_robot_model_rick_morty.urdf.xacro # Multi-robot variant
│   ├── wheel.xacro                 # Wheel macro
│   ├── standoff.xacro              # Support rod macro
│   └── cup_holder_tray.xacro       # Cup platform macro
├── meshes/                          # 3D models
│   └── sick_s300.stl               # LiDAR sensor model
├── worlds/                          # Gazebo worlds
│   └── empty.world                 # Basic environment
├── rviz/                           # Visualization configs
│   ├── config.rviz                # Single robot view
│   └── config_cp8.rviz            # Multi-robot view
├── CMakeLists.txt                  # Build configuration
└── package.xml                     # Package manifest
```

## XACRO Components

### Wheel Macro (`wheel.xacro`)
```xml
<xacro:wheel wheel_name="left_wheel"/>
```
- Configurable width and radius
- Integrated inertial calculations
- High-friction Gazebo properties

### Standoff Macro (`standoff.xacro`)
```xml
<xacro:standoff standoff_name="rod_front" standoff_width="0.01" standoff_length="0.22"/>
```
- Parameterized dimensions
- Support structure between base and tray

### Cup Holder Macro (`cup_holder_tray.xacro`)
```xml
<xacro:cup_holder_tray cup_holder_tray_name="cup_holder"/>
```
- Platform for beverage transport
- Color-coded for multi-robot systems

## Multi-Robot Configuration

The system supports multiple robots through namespacing:

```python
# In launch file
robot_name_1 = "rick"  # Blue robot
robot_name_2 = "morty" # Red robot
```

Each robot gets:
- Unique namespace (`/rick/*`, `/morty/*`)
- Separate TF tree with prefix
- Independent cmd_vel topics
- Isolated sensor topics

## Customization

### Change Robot Colors
Edit `xacro/barista_robot_model_rick_morty.urdf.xacro`:
```xml
<xacro:if value="${robot_name == 'rick'}">
    <material name="blue"/>
</xacro:if>
```

### Adjust Robot Dimensions
Modify properties in XACRO files:
```xml
<xacro:property name="wheel_radius" value="0.0352"/>
<xacro:property name="cup_holder_tray_radius" value="0.160"/>
```

### Add New Sensors
Include additional sensors in main XACRO:
```xml
<xacro:laser_sensors include_laser="true"/>
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot not moving | Check `/cmd_vel` topic is being published |
| No laser data | Verify sensor plugin is loaded: `ros2 topic list \| grep scan` |
| Transform errors | Ensure robot_state_publisher is running |
| Gazebo crash | Reduce physics update rate or simplify collision meshes |
| Namespace conflicts | Use unique robot names in multi-robot launch |

## Future Enhancements

- [ ] Add RGB-D camera for cup detection
- [ ] Implement autonomous navigation stack (Nav2)
- [ ] Create café world with tables and obstacles
- [ ] Add gripper mechanism for cup handling
- [ ] Develop order management system
- [ ] Integrate voice interaction for orders
- [ ] Add battery simulation and charging stations
- [ ] Create web-based fleet management interface


## Contact

**Ritwik Rohan**  
Robotics Engineer | Johns Hopkins MSE '25  
Email: ritwikrohan7@gmail.com  
LinkedIn: [linkedin.com/in/ritwik-rohan](https://linkedin.com/in/ritwik-rohan)  
GitHub: [github.com/ritwikrohan](https://github.com/ritwikrohan)
