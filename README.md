# Robotic Arm Path Planning Demo

This project demonstrates robotic arm path planning using **ROS 2**, **MoveIt 2**, and **RViz2** with the Panda robot model. The demo showcases automatic collision-free motion planning in a simulated environment.

## Features

- Uses the Franka Emika Panda robotic arm model.
- Sets initial and target poses for the robot in RViz2.
- Adds static obstacles (cubes) between the initial and target positions.
- Utilizes MoveIt 2 to plan and execute collision-free paths for two poses.
- Visualizes the robot, obstacles, and planned trajectories in RViz2.
- Interactive planning and execution via RViz Visual Tools GUI.

## Project Structure

```
ws_richtech/
├── build/                # Build artifacts (colcon)
├── install/              # Installed packages and setup scripts
├── log/                  # Build and run logs
├── src/
│   └── richtech_panda/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/       # MoveIt and robot config files (YAML)
│       ├── include/  
│       ├── launch/       # Launch files for simulation and Rviz config file
│       ├── moveit_resources/ # Additional robot/config resources
│       └── src/
│           └── richtech_panda.cpp # Main demo source code
└── README.md
```

## Requirements

- **Operating System:** Linux (tested on Ubuntu 22.04+)
- **ROS 2:** Humble
- **MoveIt 2:** Installed for ROS 2 Humble
- **RViz2:** Installed with ROS 2 Humble
- **Colcon:** For building ROS 2 workspaces
- **C++ Compiler:** GCC 9+

## Installation Guide

### 1. Install ROS 2
Follow the official ROS 2 installation guide for your OS:  
https://docs.ros.org/en/humble/Installation.html

### 2. Install MoveIt 2
```bash
sudo apt update
sudo apt install ros-humble-moveit
```

### 3. Install Additional Dependencies
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-rviz2
```

### 4. Clone the Repository
```bash
git clone git@github.com:SwarajMundruppadyRao/Richtech-Motion-Planning-Around-Objects.git ~/ws_richtech
cd ~/ws_richtech
```

### 5. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 6. Source the Workspace
```bash
source install/setup.bash
```

## How to Run the Simulation

1. **Launch the MoveIt + RViz2 Demo:**
   ```bash
   ros2 launch richtech_panda panda.launch.py
   ```
   This will start:
   - MoveIt 2 move_group node
   - RViz2 with robot and planning scene
   - ros2_control and controllers
   - Static transforms and robot state publisher

2. **Run the Path Planning Demo Node:**
   In a new terminal (after sourcing ROS 2 and workspace):
   ```bash
   ros2 run richtech_panda richtech_panda
   ```
   This will:
   - Set initial and target poses
   - Add static obstacles (cubes) in the scene
   - Use MoveIt 2 to plan a collision-free path for two poses
   - Visualize the planned path and obstacles in RViz2

## Notes
- You can modify poses and obstacles in `src/richtech_panda.cpp`.
- RViz2 will prompt for planning/execution steps via the GUI.
- All configuration files are in `src/richtech_panda/config/`.

## Troubleshooting
- Ensure all dependencies are installed and sourced.
- If controllers fail to start, check `ros2_control` and config files.
- For custom robots, update config and launch files accordingly.

---
For further help, see the official MoveIt 2 and ROS 2 documentation.

## Author

Swaraj
