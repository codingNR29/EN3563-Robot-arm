# Yaskawa Motoman GP7 Robot Arm - ROS 2 Project

Welcome to the **Yaskawa Motoman GP7 Robot Arm** ROS 2 repository! This project provides complete ROS 2 integration for the 6-axis Yaskawa Motoman GP7 industrial robotic arm equipped with an adaptive 2-finger gripper. It includes URDF/Xacro kinematic models, mesh representations, RViz2 visualization pipelines, and Gazebo Sim integration powered by `ros2_control`.

---

## 📌 Table of Contents

- [Overview](#-overview)
- [Directory & Folder Structure](#-directory--folder-structure)
- [Prerequisites](#-prerequisites)
- [Step-by-Step Installation & Setup](#-step-by-step-installation--setup)
- [How to Run the Project](#-how-to-run-the-project)
  - [1. Visualizing in RViz2](#1-visualizing-in-rviz2)
  - [2. Simulating in Gazebo Sim](#2-simulating-in-gazebo-sim)
- [Robot Kinematics & Controllers](#-robot-kinematics--controllers)
- [Troubleshooting & Common Issues](#-troubleshooting--common-issues)
- [License](#-license)

---

## 🤖 Overview

The **Motoman GP7** is a high-speed 6-axis industrial robot arm designed for assembly, handling, and pick-and-place operations. This repository packages the robot model into ROS 2 format for testing, trajectory planning, visualization, and simulation.

Key features included in this workspace:
- 🎨 **Detailed URDF & 3D Meshes**: High-resolution visual (`.dae`, `.stl`) and collision geometries for both the GP7 arm and its adaptive gripper.
- 🎛️ **RViz2 Integration**: Joint GUI control for testing kinematics and inspecting joint limits interactively.
- 🌐 **Gazebo Simulation**: Ignition / Gazebo Sim integration with `gz_ros2_control` for physics-based simulation.
- 🕹️ **ROS 2 Control & Controllers**: Configured joint trajectory controller (`JointTrajectoryController`) for arm movements and position controller (`JointGroupPositionController`) for the gripper.

---

## 📂 Directory & Folder Structure

Here is a breakdown of what each folder in this repository contains and its purpose:

```text
EN3563-Robot-arm/
└── gp7_ros2/                      # Meta-folder containing all ROS 2 packages
    ├── gp7_description/           # Robot Model & Visualization Package
    │   ├── launch/                # Launch files (e.g., display.launch.py)
    │   ├── meshes/                # 3D Mesh files (STL/DAE for arm and gripper)
    │   ├── rviz/                  # RViz configuration profiles (display.rviz)
    │   ├── urdf/                  # Unified Robot Description Format (gp7.urdf)
    │   └── CMakeLists.txt / package.xml
    │
    ├── gp7_gazebo/                # Physics Simulation Package
    │   ├── config/                # ros2_control YAML configuration (ros2_controllers.yaml)
    │   ├── launch/                # Simulation launch files (gazebo.launch.py)
    │   └── CMakeLists.txt / package.xml
    │
    ├── gp7_ros2_pkg/              # Core Meta/Wrapper Package
    │   └── CMakeLists.txt / package.xml
    │
    ├── gp7_bringup/               # Workspace directory for system bringup launch files
    ├── gp7_moveit_config/         # Reserved package for MoveIt 2 motion planning setup
    └── gp7_system_tests/          # Reserved package for automated system & integration tests
```

### Detailed Package Descriptions:

1. **`gp7_description`**:
   Contains everything related to the physical and visual representation of the Motoman GP7 robot arm and its adaptive 2-finger gripper.
   - `urdf/gp7.urdf`: Defines link dimensions, inertial properties, joint axes, limits, and visual/collision mesh references.
   - `meshes/`: Stores visual `.stl` and `.dae` mesh files for the GP7 links and adaptive gripper components.
   - `launch/display.launch.py`: Launches `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

2. **`gp7_gazebo`**:
   Contains configuration and launch scripts required to spawn and simulate the robot inside Gazebo Sim.
   - `config/ros2_controllers.yaml`: Defines `controller_manager` update rates and controller configurations (`arm_controller`, `gripper_controller`, and `joint_state_broadcaster`).
   - `launch/gazebo.launch.py`: Launches Gazebo Sim (`ros_gz_sim`), sets environment paths, spawns the robot model, and auto-spawns ROS 2 controllers.

3. **`gp7_ros2_pkg`**:
   Acts as the primary package interface uniting robot dependencies across the repository.

4. **`gp7_bringup`**, **`gp7_moveit_config`**, **`gp7_system_tests`**:
   Modular ROS 2 package spaces dedicated for multi-node bringup scripts, MoveIt 2 kinematic solver configs, and test suites.

---

## 🛠️ Prerequisites

Before getting started, make sure your computer meets the following system requirements:

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 24.04 LTS
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (or ROS 2 Jazzy / Rolling)
- **Gazebo**: Gazebo Sim (formerly Ignition Gazebo, Fortress or Harmonic)
- **Build Tools**: `colcon`, `rosdep`, `git`, `python3-pip`

---

## ⚙️ Step-by-Step Installation & Setup

If you are setting up this repository on a fresh PC, follow these instructions step-by-step in your terminal:

### Step 1: Install ROS 2 & Dependencies

Ensure ROS 2 Desktop is installed on your system. If not, follow the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).

Next, install the required ROS 2 control, Gazebo, and GUI tools:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep
```

*(Note: Replace `humble` with `jazzy` if using ROS 2 Jazzy).*

### Step 2: Create a ROS 2 Workspace

Create a new directory structure for your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 3: Clone this Repository

Clone the project repository into the `src/` directory of your ROS 2 workspace:

```bash
git clone https://github.com/codingNR29/EN3563-Robot-arm.git .
```

### Step 4: Install ROS Dependencies via `rosdep`

Initialize and update `rosdep` to automatically install any missing dependencies:

```bash
cd ~/ros2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Build the Workspace

Build the packages using `colcon`:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 6: Source the Environment

Before running any ROS 2 nodes, source your workspace setup file:

```bash
source ~/ros2_ws/install/setup.bash
```

> 💡 **Pro-Tip**: You can add this line to your `~/.bashrc` so it sources automatically whenever you open a new terminal:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

---

## 🚀 How to Run the Project

### 1. Visualizing in RViz2

To test the robot URDF model and interactively move its joints using a slider GUI:

```bash
ros2 launch gp7_description display.launch.py
```

**What this command does:**
- Starts `robot_state_publisher` to publish the robot's transforms (`tf`).
- Opens a **Joint State Publisher GUI** window allowing you to adjust joint angles for Joints 1–6 and the gripper.
- Opens **RViz2** displaying the Motoman GP7 3D model.

---

### 2. Simulating in Gazebo Sim

To spawn the robot in a Gazebo Sim physics environment with ROS 2 hardware controllers enabled:

```bash
ros2 launch gp7_gazebo gazebo.launch.py
```

**What this command does:**
1. Launches Gazebo Sim with an empty world.
2. Spawns the `motoman_gp7` robot model into the world.
3. Automatically sets the `GZ_SIM_RESOURCE_PATH` environment variable so Gazebo can locate 3D mesh files.
4. Starts `controller_manager` and spawns the following controllers:
   - `joint_state_broadcaster`
   - `arm_controller` (Joint Trajectory Controller)
   - `gripper_controller` (Position Controller)

---

## 🦾 Robot Kinematics & Controllers

### Robot Arm Joints (6-DOF)
| Joint Name | Type | Description |
| :--- | :--- | :--- |
| `joint_1_s` | Revolute | Base Rotation (S-axis) |
| `joint_2_l` | Revolute | Lower Arm Pitch (L-axis) |
| `joint_3_u` | Revolute | Upper Arm Pitch (U-axis) |
| `joint_4_r` | Revolute | Wrist Roll (R-axis) |
| `joint_5_b` | Revolute | Wrist Pitch (B-axis) |
| `joint_6_t` | Revolute | Tool Flange Roll (T-axis) |

### Adaptive Gripper Joints
- `gripper_controller`
- `gripper_base_to_gripper_left2`
- `gripper_left3_to_gripper_left1`
- `gripper_base_to_gripper_right3`
- `gripper_base_to_gripper_right2`
- `gripper_right3_to_gripper_right1`

### Active ROS 2 Controllers
- **`arm_controller`**: Interface type `joint_trajectory_controller/JointTrajectoryController` accepting trajectory goals on command interfaces.
- **`gripper_controller`**: Interface type `position_controllers/JointGroupPositionController` controlling gripper positioning.
- **`joint_state_broadcaster`**: Publishes state of all joints to `/joint_states`.

---

## ❓ Troubleshooting & Common Issues

#### Issue 1: Meshes are invisible or missing in Gazebo / RViz
- **Cause**: Environment variable `GZ_SIM_RESOURCE_PATH` or package paths not sourced.
- **Solution**: Ensure you ran `source ~/ros2_ws/install/setup.bash` in the terminal before launching.

#### Issue 2: `Package 'gp7_description' not found`
- **Cause**: Workspace was not built or setup script was not sourced.
- **Solution**: Run `cd ~/ros2_ws && colcon build --symlink-install` and source `install/setup.bash`.

#### Issue 3: Missing `ros_gz_sim` or `gz_ros2_control` executable
- **Cause**: ROS 2 Gazebo bridge packages are missing.
- **Solution**: Install them via `sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control`.

---

## 📜 License

This project is licensed under the BSD-3-Clause License. See individual package `package.xml` files for detailed licensing metadata.
