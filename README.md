> **Assignment pick-and-place:** See [run instructions, implementation and validation](gp7_ros2/PICK_PLACE.md).

# Yaskawa Motoman GP7 Robot Arm — ROS 2 Project

Welcome to the **Yaskawa Motoman GP7 Robot Arm** ROS 2 repository! This project provides complete ROS 2 integration for the 6-axis Yaskawa Motoman GP7 industrial robotic arm equipped with an adaptive 2-finger gripper. It includes URDF kinematic models, mesh representations, RViz2 visualization, Gazebo Sim physics simulation powered by `gz_ros2_control`, and Python controller nodes for commanding the robot via code.

---

## 📌 Table of Contents

- [Overview](#-overview)
- [Directory & Folder Structure](#-directory--folder-structure)
- [Prerequisites](#-prerequisites)
- [Step-by-Step Installation & Setup](#-step-by-step-installation--setup)
- [How to Run the Project](#-how-to-run-the-project)
  - [1. Visualizing in RViz2](#1-visualizing-in-rviz2)
  - [2. Simulating in Gazebo Sim](#2-simulating-in-gazebo-sim)
  - [3. Controlling the Robot via Code](#3-controlling-the-robot-via-code)
- [Adding Your Own Movements](#-adding-your-own-movements)
- [Robot Kinematics & Controllers](#-robot-kinematics--controllers)
- [Troubleshooting & Common Issues](#-troubleshooting--common-issues)
- [License](#-license)

---

## 🤖 Overview

The **Motoman GP7** is a high-speed 6-axis industrial robot arm designed for assembly, handling, and pick-and-place operations. This repository packages the robot model into ROS 2 format for trajectory planning, visualization, and simulation.

Key features:
- 🎨 **Detailed URDF & 3D Meshes** — High-resolution visual (`.stl`) and collision geometries for the GP7 arm and adaptive gripper.
- 🎛️ **RViz2 Integration** — Joint GUI slider control for testing kinematics interactively.
- 🌐 **Gazebo Sim** — Physics-based simulation with `gz_ros2_control`; robot is ground-anchored via a fixed world joint.
- 🕹️ **Python Controller Nodes** — `arm_controller_node`, `gripper_controller_node`, and `demo_sequence_node` for commanding the robot programmatically.
- ✅ **Working Controller Stack** — `joint_state_broadcaster` → `arm_controller` (JointTrajectoryController) → `gripper_controller` (JointGroupPositionController), all verified active.

---

## 📂 Directory & Folder Structure

```text
ros2_ws/src/
└── gp7_ros2/
    ├── gp7_description/           # Robot model & visualization
    │   ├── launch/                # display.launch.py (RViz2)
    │   ├── meshes/                # STL mesh files (arm + gripper)
    │   ├── rviz/                  # RViz2 config (display.rviz)
    │   └── urdf/gp7.urdf          # Full robot URDF (with ros2_control + world joint)
    │
    ├── gp7_gazebo/                # Gazebo simulation package
    │   ├── config/
    │   │   └── ros2_controllers.yaml   # Controller types and joint mappings
    │   └── launch/
    │       └── gazebo.launch.py        # Main sim launch (spawns robot + controllers)
    │
    └── gp7_ros2_pkg/              # Python controller nodes package
        ├── scripts/
        │   ├── arm_controller_node.py       # Trajectory action client
        │   ├── gripper_controller_node.py   # Gripper position forwarder
        │   └── demo_sequence_node.py        # Automated demo sequence
        └── launch/
            └── gp7_sim_control.launch.py    # All-in-one launch (Gazebo + nodes)
```

---

## 🛠️ Prerequisites

| Requirement | Version |
| :--- | :--- |
| Operating System | Ubuntu 24.04 LTS |
| ROS 2 Distribution | **ROS 2 Jazzy** |
| Gazebo | Gazebo Sim 8 (Harmonic) via `ros-jazzy-ros-gz` |
| Build Tools | `colcon`, `rosdep`, `git` |

---

## ⚙️ Step-by-Step Installation & Setup

### Step 1: Install ROS 2 Jazzy & Required Packages

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html), then install the required packages:

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-publisher-gui \
  python3-colcon-common-extensions \
  python3-rosdep
```

### Step 2: Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 3: Clone the Repository

```bash
git clone https://github.com/codingNR29/EN3563-Robot-arm.git .
```

### Step 4: Install Dependencies

```bash
cd ~/ros2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 5: Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 6: Source the Environment

```bash
source ~/ros2_ws/install/setup.bash
```

> 💡 **Pro-tip**: Add this to your `~/.bashrc` so it runs automatically in every new terminal:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

---

## 🚀 How to Run the Project

### 1. Visualizing in RViz2

Interactively drag joint sliders to explore the robot's range of motion:

```bash
ros2 launch gp7_description display.launch.py
```

Opens `robot_state_publisher` + Joint State Publisher GUI + RViz2.

---

### 2. Simulating in Gazebo Sim

Launch the full physics simulation with controllers:

```bash
ros2 launch gp7_gazebo gazebo.launch.py
```

**What happens:**
1. Gazebo Sim opens with an empty world.
2. The GP7 robot spawns — **fixed to the ground** via a world anchor joint.
3. After ~20 seconds, three controllers activate in sequence:
   - ✅ `joint_state_broadcaster` → publishes `/joint_states`
   - ✅ `arm_controller` → accepts trajectory goals
   - ✅ `gripper_controller` → accepts gripper position commands

> ⚠️ **Important**: Wait until you see all three `Activated ...` messages in the terminal before running controller nodes. If activation fails, see [Troubleshooting](#-troubleshooting--common-issues).

---

### 3. Controlling the Robot via Code

#### Option A — Gazebo + Controller nodes separately (recommended for development)

**Terminal 1** — Launch Gazebo, wait for all 3 controllers to activate:
```bash
ros2 launch gp7_gazebo gazebo.launch.py
```

**Terminal 2** — Run the arm controller node, wait for the ready message:
```bash
ros2 run gp7_ros2_pkg arm_controller_node
# Wait for: ✅ /joint_states received — arm controller is ready for goals.
```

**Terminal 3** — Send movement commands:
```bash
# Move to preset position (0=home, 1=reach_out, 2=compact, 3=wave)
ros2 topic pub /gp7/goal_index std_msgs/msg/Int32 "{data: 1}" --once
```

**Terminal 4** — Control the gripper:
```bash
ros2 run gp7_ros2_pkg gripper_controller_node
# Then in another terminal:
ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: -0.5}" --once
```

#### Option B — Automated demo (no manual input)

```bash
# Terminal 1: Gazebo
ros2 launch gp7_gazebo gazebo.launch.py

# Terminal 2: Auto-demo (home → reach → gripper → compact → home)
ros2 run gp7_ros2_pkg demo_sequence_node
```

### 4. Running the Pick and Place Task

To launch the complete pick-and-place sequence:

```bash
cd /home/banula/ros2_ws/src
unset AMENT_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
export GZ_PARTITION=gp7_pick_place
ros2 launch gp7_ros2_pkg pick_place.launch.py
```

---

## 🧩 Adding Your Own Movements

All arm presets live in `gp7_ros2_pkg/scripts/arm_controller_node.py`. You can add new motions in **two steps**.

### Step 1 — Know your joint order and limits

The arm has **6 joints**, always listed in this order:

| # | Joint Name | Axis | Approx. Limit |
| :-: | :--- | :--- | :--- |
| 1 | `joint_1_s` | Base rotation | ±170° (~±2.97 rad) |
| 2 | `joint_2_l` | Lower arm pitch | −65° to +155° |
| 3 | `joint_3_u` | Upper arm pitch | −170° to +190° |
| 4 | `joint_4_r` | Wrist roll | ±190° (~±3.32 rad) |
| 5 | `joint_5_b` | Wrist pitch | ±135° (~±2.36 rad) |
| 6 | `joint_6_t` | Tool roll | ±360° (~±6.28 rad) |

All values are in **radians**.

### Step 2 — Add a preset to `arm_controller_node.py`

Open [`gp7_ros2_pkg/scripts/arm_controller_node.py`](file:///home/banula/ros2_ws/src/gp7_ros2/gp7_ros2_pkg/scripts/arm_controller_node.py) and find the `PRESETS` dictionary. Add a new entry:

```python
PRESETS = {
    0: { 'name': 'home', 'waypoints': [([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)] },
    1: { 'name': 'reach_out', ... },
    2: { 'name': 'compact',   ... },
    3: { 'name': 'wave',      ... },

    # ✅ ADD YOUR OWN PRESET HERE:
    4: {
        'name': 'my_pose',
        'waypoints': [
            # ( [j1_s, j2_l, j3_u, j4_r, j5_b, j6_t], time_seconds )
            ([0.0, 0.3, 0.5, 0.0, 0.0, 0.0], 2.0),   # intermediate
            ([0.5, 0.8, 1.0, 0.3, 0.5, 0.0], 5.0),   # final pose
        ],
    },
}
```

**Waypoints explained:**
- Each waypoint is a `(positions, time)` tuple.
- `positions` = list of 6 joint angles **in radians**, matching the joint order above.
- `time` = seconds from trajectory start when this waypoint should be reached.
- The controller computes a **smooth spline** between waypoints — no need to specify velocities.
- Add as many intermediate waypoints as you need for a smooth, complex path.

Then trigger it:
```bash
ros2 topic pub /gp7/goal_index std_msgs/msg/Int32 "{data: 4}" --once
```

> 💡 No rebuild needed — the node uses `--symlink-install` so edits to the script take effect immediately on next run.

### Gripper Commands

The gripper is controlled separately via a float value:

```bash
# Open gripper
ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: 0.15}" --once

# Close gripper
ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: -0.5}" --once

# Neutral
ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}" --once
```

### Sending raw trajectories directly (bypassing the node)

You can also send a trajectory goal directly via the ROS 2 action CLI — useful for one-off testing without editing code:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t],
      points: [
        {positions: [0.0, 0.5, 0.8, 0.0, 0.3, 0.0],
         time_from_start: {sec: 3, nanosec: 0}}
      ]
  }}"
```

---

## 🦾 Robot Kinematics & Controllers

### 6-DOF Arm Joints

| Joint Name | Type | Description |
| :--- | :--- | :--- |
| `joint_1_s` | Revolute | Base Rotation (S-axis) |
| `joint_2_l` | Revolute | Lower Arm Pitch (L-axis) |
| `joint_3_u` | Revolute | Upper Arm Pitch (U-axis) |
| `joint_4_r` | Revolute | Wrist Roll (R-axis) |
| `joint_5_b` | Revolute | Wrist Pitch (B-axis) |
| `joint_6_t` | Revolute | Tool Flange Roll (T-axis) |

### Adaptive Gripper

The gripper has **one actuated joint** (`gripper_controller`) and **5 mimic joints** that follow it automatically via Gazebo's mimic constraint.

### Active ROS 2 Controllers

| Controller | Type | Topic / Action |
| :--- | :--- | :--- |
| `joint_state_broadcaster` | JointStateBroadcaster | `/joint_states` |
| `arm_controller` | JointTrajectoryController | `/arm_controller/follow_joint_trajectory` |
| `gripper_controller` | JointGroupPositionController | `/gripper_controller/commands` |

---

## ❓ Troubleshooting & Common Issues

#### Controllers fail to activate (`Failed to activate controller`)

- **Cause A — YAML path not found**: The `gz_ros2_control` plugin couldn't load the controllers YAML. The URDF must contain an **absolute path** — `$(find ...)` syntax is not supported by Gazebo Sim.
- **Cause B — Mimic joints in ros2_control**: Mimic joints have no command interfaces in Gazebo Sim. Only the primary actuated joint should appear in the `<ros2_control>` section.
- **Fix**: Both issues are already resolved in this repo. If you re-clone and get this error, ensure you ran `colcon build --symlink-install` and the YAML path in `gp7.urdf` matches your install path.

#### Goal rejected by controller

- **Cause**: The arm controller received a goal before joint states were published (hardware state = NaN).
- **Fix**: The `arm_controller_node` waits for `/joint_states` before accepting any goal. Look for the `✅ /joint_states received` log line before sending goals.

#### Robot falls through the ground or floats

- **Cause**: No world→base fixed joint in the URDF.
- **Fix**: The URDF now includes `<link name="world"/>` and `<joint name="world_to_base" type="fixed">` anchoring `base_link` to the world frame.

#### Meshes invisible in Gazebo / RViz2

- **Cause**: `GZ_SIM_RESOURCE_PATH` not set or workspace not sourced.
- **Fix**: Always run `source ~/ros2_ws/install/setup.bash` before launching. The launch file also sets `GZ_SIM_RESOURCE_PATH` automatically.

#### `Package not found` errors

```bash
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 📜 License

This project is licensed under the BSD-3-Clause License. See individual package `package.xml` files for details.
