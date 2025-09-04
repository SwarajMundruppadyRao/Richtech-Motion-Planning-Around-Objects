# Robotic Arm Path Planning Demo (ROS 2 + MoveIt 2 + RViz2)

This project demonstrates robotic arm path planning using ROS 2, MoveIt 2, and RViz2. It features:
- Panda robotic arm simulation
- Initial and target pose setting
- Static obstacle insertion
- Automatic collision-free path planning
- Visualization in RViz2


## Project Structure

```
ws_richtech/
├── src/
│   └── richtech_panda/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/       # MoveIt config files (YAML)
│       ├── include/
│       ├── launch/       # Launch files for simulation and Rviz Config File
│       ├── moveit_resources/ # Used https://github.com/moveit/moveit_resources for Panda Robot
│       └── src/
│           └── richtech_panda.cpp # Main demo source code
└── README.md             
```


## Requirements

- **Operating System:** Linux (tested on Ubuntu 22.04+)
- **ROS 2:** Humble (or newer)
- **MoveIt 2:** For ROS 2 Humble
- **RViz2:** For ROS 2
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
(Replace `humble` with your ROS 2 distro if different)

### 3. Install Additional Dependencies
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-rviz2
```

### 4. Clone the Repository
```bash
git clone <your-repo-url> ~/ws_richtech
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

### 1. Launch MoveIt 2 and RViz2
```bash
ros2 launch richtech_panda panda.launch.py
```
This will start:
- MoveIt 2 move_group node
- RViz2 with robot and planning scene
- ros2_control and controllers
- Static transforms and robot state publisher

### 2. Run the Path Planning Demo Node
Open a new terminal, source ROS 2 and the workspace, then run:
```bash
ros2 run richtech_panda richtech_panda
```
This will:
- Set initial and target poses
- Add static obstacles (cubes) in the scene
- Use MoveIt 2 to plan a collision-free path
- Visualize the planned path and obstacles in RViz2

## Customization & Notes
- Modify poses and obstacles in `src/richtech_panda/src/richtech_panda.cpp`.
- RViz2 will prompt for planning/execution steps via the GUI.
- All configuration files are in `src/richtech_panda/config/`.

## Troubleshooting
- Ensure all dependencies are installed and sourced.
- If controllers fail to start, check `ros2_control` and config files.
- For custom robots, update config and launch files accordingly.

---
For further help, see the official MoveIt 2 and ROS 2 documentation:
https://moveit.ros.org/
https://docs.ros.org/en/humble/index.html


## Author

Swaraj Mundruppady Rao  
swarajmrao@gmail.com
