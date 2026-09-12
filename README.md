<div align="center">

# 🤖 Robotic Arm Pick-and-Place System

### EN3563 Robotics Mini Project

**Design, modelling, simulation and implementation of a multi-DoF robotic manipulator for an orientation-preserving pick-and-place task**

<br>

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?style=for-the-badge\&logo=ros\&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113?style=for-the-badge)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?style=for-the-badge\&logo=ubuntu\&logoColor=white)
![CAD](https://img.shields.io/badge/CAD-Robot%20Design-4A90E2?style=for-the-badge)
![URDF](https://img.shields.io/badge/Robot-URDF-0A66C2?style=for-the-badge)

<br>

**Robot Modelling → Kinematics → Motion Planning → Control → Pick & Place**

</div>

---

## 📌 Project Overview

This project focuses on the development of a **multi-degree-of-freedom robotic arm** capable of performing an automated **pick-and-place task**.

The project explores the complete robotic-manipulator development process, including:

* mechanical modelling
* coordinate-frame assignment
* forward and inverse kinematics
* trajectory planning
* joint velocity control
* end-effector orientation control
* simulation
* robot control
* potential physical implementation

The robotic arm must pick up an object from a predefined location and place it at a target location while maintaining the object's **upright orientation throughout the motion**.

The project is being developed in stages, beginning with simulation and extending toward a possible **3D-printed physical robotic arm implementation**.

---

## 🎯 Main Objective

The goal is to develop a robotic manipulator that can:

> **Detect or receive a target pick position, move toward the object, grasp it, transport it smoothly, and place it at the required destination while maintaining the desired orientation.**

The project combines:

```text
Mechanical Design
       ↓
Robot Modelling
       ↓
Kinematics
       ↓
Trajectory Planning
       ↓
Joint Control
       ↓
Pick-and-Place Execution
```

---

## 🦾 Project Scope

The robotic arm will contain at least **4 Degrees of Freedom (DoF)**.

Depending on the final implementation, the system may include:

* Base rotation
* Shoulder rotation
* Elbow rotation
* Wrist rotation
* Additional wrist joints
* Gripper mechanism

The overall architecture is designed so that both **simulation and physical hardware** can use the same underlying kinematic concepts.

---

## 🧠 System Architecture

```text
                 Robotic Arm
                     │
                     ▼
              Mechanical Model
                     │
                     ▼
            Coordinate Frames
                     │
                     ▼
             D-H Parameters
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
 Forward Kinematics      Inverse Kinematics
          │                     │
          └──────────┬──────────┘
                     ▼
             Trajectory Planner
                     │
                     ▼
             Velocity Profile
                     │
                     ▼
              Joint Controller
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
      Simulation          Physical Robot
          │                     │
          └──────────┬──────────┘
                     ▼
              Pick & Place Task
```

---

## 🧩 Technologies

<table>
<tr>
<td align="center"><b>ROS 2</b><br>Robot Middleware</td>
<td align="center"><b>Gazebo</b><br>Simulation</td>
<td align="center"><b>RViz</b><br>Visualization</td>
<td align="center"><b>Ubuntu</b><br>Development Environment</td>
</tr>

<tr>
<td align="center"><b>URDF</b><br>Robot Description</td>
<td align="center"><b>CAD</b><br>Mechanical Design</td>
<td align="center"><b>Python / C++</b><br>Control Algorithms</td>
<td align="center"><b>ROS 2 Control</b><br>Joint Control</td>
</tr>
</table>

Possible physical implementation technologies may include:

```text
3D Printed Links
Servo Motors / Other Actuators
Microcontroller
Power Electronics
Sensors
Custom Gripper
```

---

## 🔄 Development Workflow

```text
CAD / Mechanical Model
        │
        ▼
Define Robot Links
        │
        ▼
Define Robot Joints
        │
        ▼
Robot Description
        │
        ▼
RViz Verification
        │
        ▼
Gazebo Simulation
        │
        ▼
Joint Control
        │
        ▼
Forward Kinematics
        │
        ▼
Inverse Kinematics
        │
        ▼
Trajectory Planning
        │
        ▼
Pick & Place
        │
        ▼
Physical Prototype
```

---

## 📐 Kinematics

### Forward Kinematics

Forward kinematics determines the position and orientation of the end effector from the robot joint variables.

For an \(n\)-joint robotic manipulator:


$\{}^{0}T_{n}$
=
$\{}^{0}T_{1}$
$\{}^{1}T_{2}$
$\{}^{2}T_{3}$
$\\cdots$
$\{}^{n-1}T_{n}$


The joint vector can be written as:

$$
q=
\begin{bmatrix}
q_1 &
q_2 &
\cdots &
q_n
\end{bmatrix}^{T}
$$

The transformation matrix provides the final:

* position
* orientation

of the end effector.

---

## 🔄 Inverse Kinematics

Inverse kinematics solves the opposite problem.

```text
Desired End-Effector Pose

[x, y, z, orientation]

          ↓

Inverse Kinematics

          ↓

Joint Configuration

[q1, q2, ..., qn]
```

The inverse-kinematics implementation will determine suitable joint angles while respecting:

* mechanical joint limits
* reachable workspace
* desired end-effector orientation
* collision constraints

---

## 🧭 Coordinate Frames

Coordinate frames are assigned to each robot link and joint.

The robot model follows a serial-link structure such as:

```text
base_link
    │
 joint_1
    │
  link_1
    │
 joint_2
    │
  link_2
    │
 joint_3
    │
  link_3
    │
   ...
    │
 end_effector
```

The coordinate frames will be used to derive the **Denavit-Hartenberg parameters**.

---

## 📊 D-H Parameters

Each robot joint is represented using:

| Parameter    | Meaning     |
| ------------ | ----------- |
| $\(a_i\)$      | Link length |
| $\(\alpha_i\)$ | Link twist  |
| $\(d_i\)$      | Link offset |
| $\(\theta_i\)$ | Joint angle |

The final D-H table will be determined from the geometry of the selected robotic arm configuration.

---

## 🛣️ Pick-and-Place Motion

The planned motion sequence is:

```text
HOME
  │
  ▼
APPROACH OBJECT
  │
  ▼
MOVE ABOVE OBJECT
  │
  ▼
DESCEND
  │
  ▼
GRASP
  │
  ▼
LIFT
  │
  ▼
MOVE TO DESTINATION
  │
  ▼
DESCEND
  │
  ▼
RELEASE
  │
  ▼
RETREAT
```

Intermediate waypoints are used to avoid unnecessary collisions and make the robot motion easier to control.

---

## 📦 Orientation Control

A major requirement of the project is maintaining the object in an **upright orientation**.

The end-effector position changes throughout the trajectory:

$$
p(t)=
\begin{bmatrix}
x(t)\\
y(t)\\
z(t)
\end{bmatrix}
$$

while the desired orientation remains fixed:

$$
R(t)=R_{\text{desired}}
$$

The robot wrist joints compensate for changes in the shoulder and elbow configuration.

---

## 📈 Trajectory Planning

Robot joints should not move instantaneously between two configurations.

The trajectory planner will generate smooth motion using methods such as:

### Trapezoidal Velocity Profile

```text
Velocity

          ┌───────────────┐
         /                 \
        /                   \
───────                       ───────
      Accel      Constant      Decel

────────────────────────────────────→ Time
```

The motion contains:

1. Acceleration
2. Constant velocity
3. Deceleration

Possible future improvement:

```text
S-Curve Velocity Profile
```

which provides smoother acceleration and reduced jerk.

---

## 🖥️ Simulation

The simulation environment is being developed using:

```text
ROS 2
Gazebo
RViz
URDF
ros2_control
```

The simulation allows the team to verify:

* robot geometry
* joint behaviour
* kinematics
* workspace
* trajectory planning
* object manipulation

before implementing the physical system.

---

## 🔧 Physical Robot Development

A physical version of the robotic arm may also be developed.

The hardware implementation may include:

```text
3D Printed Robot Structure
        │
        ▼
Servo / Joint Actuators
        │
        ▼
Motor Controller
        │
        ▼
Microcontroller
        │
        ▼
Robot Control Software
        │
        ▼
Custom Gripper
```

Design priorities include:

* low manufacturing cost
* lightweight links
* sufficient joint torque
* internal cable routing
* modular components
* easy maintenance
* 3D-printable construction

---

## 🖨️ 3D Printed Design Goals

If the hardware implementation is developed, the mechanical design will aim for:

* Fully or mostly 3D-printable parts
* Hidden or internally routed wiring
* Compact industrial-style appearance
* Replaceable joints
* Modular link sections
* Lightweight structure
* Stable robot base
* Accessible electronics
* Simple gripper mounting

---

## 🎮 Robot Control

The control system will provide commands for each robot joint.

Example structure:

```text
Trajectory Planner
       │
       ▼
Desired Joint Positions
       │
       ▼
Joint Controller
       │
       ▼
Joint Actuators
       │
       ▼
Robot Motion
```

For simulation, joint control can be implemented using:

```text
ros2_control
        │
        ▼
joint_state_broadcaster
        │
        ▼
joint_trajectory_controller
```

---

## 📂 Repository Structure

```text
robotic-arm-project/
│
├── robot_description/
│   │
│   ├── urdf/
│   ├── meshes/
│   ├── launch/
│   ├── config/
│   └── worlds/
│
├── kinematics/
│   ├── forward_kinematics/
│   └── inverse_kinematics/
│
├── trajectory/
│   ├── trajectory_planner/
│   └── velocity_profile/
│
├── control/
│
├── simulation/
│
├── hardware/
│   ├── cad/
│   ├── electronics/
│   └── firmware/
│
├── docs/
│
├── media/
│   ├── images/
│   └── videos/
│
└── README.md
```

---

## 🚀 Current Development Status

| Development Stage          |   Status   |
| -------------------------- | :--------: |
| Project planning           |      ✅     |
| CAD model preparation      |      ✅     |
| Robot link identification  |      ✅     |
| Joint identification       |      ✅     |
| URDF generation            |      ✅     |
| URDF cleanup               |      ✅     |
| Robot visualization        |     🚧     |
| Gazebo simulation          |     🚧     |
| Joint control              |      ⏳     |
| Gripper design             |      ⏳     |
| D-H parameter derivation   |      ⏳     |
| Forward kinematics         |      ⏳     |
| Inverse kinematics         |      ⏳     |
| Trajectory planning        |      ⏳     |
| Velocity control           |      ⏳     |
| Pick-and-place execution   |      ⏳     |
| Physical robot development | 🔬 Planned |

---

## 🗺️ Project Roadmap

```text
✅ Define Project Requirements
        │
        ▼
✅ Prepare Robot CAD
        │
        ▼
✅ Define Links & Joints
        │
        ▼
✅ Generate URDF
        │
        ▼
🚧 Verify Robot Model
        │
        ▼
⬜ Gazebo Simulation
        │
        ▼
⬜ Joint Control
        │
        ▼
⬜ Coordinate Frames
        │
        ▼
⬜ D-H Parameters
        │
        ▼
⬜ Forward Kinematics
        │
        ▼
⬜ Inverse Kinematics
        │
        ▼
⬜ Trajectory Planning
        │
        ▼
⬜ Velocity Control
        │
        ▼
⬜ Orientation Control
        │
        ▼
⬜ Pick & Place
        │
        ▼
⬜ Physical Prototype
```

---

## 📸 Project Media

### Robot Model

<p align="center">
  <i>Robot model screenshot will be added here.</i>
</p>

<!--
<p align="center">
  <img src="media/images/robot_model.png" width="750">
</p>
-->

---

### Simulation

<p align="center">
  <i>Simulation screenshot will be added here.</i>
</p>

<!--
<p align="center">
  <img src="media/images/simulation.png" width="750">
</p>
-->

---

### Physical Prototype

<p align="center">
  <i>Physical robot images will be added here if hardware implementation is completed.</i>
</p>

---

### Pick-and-Place Demonstration

<p align="center">
  <i>Final GIF / video will be added here.</i>
</p>

---

## 🎥 Demonstration

The final demonstration will show:

* robot initialization
* movement toward the object
* object grasping
* smooth transportation
* upright orientation preservation
* placement at the destination
* return or retreat motion

---

## 🧪 Future Improvements

Potential extensions include:

* Computer vision
* Automatic object detection
* Camera-based object localisation
* Collision avoidance
* Workspace optimisation
* Improved gripper design
* S-curve trajectory generation
* Feedback control
* Physical joint encoders
* Wireless robot control
* Mobile or web-based control interface

---

## 📚 Academic Scope

This project is developed for:

**EN3563 – Robotics**
**Mini Project: Implementation of a Robot Arm**

Key concepts covered include:

* Serial-link manipulators
* Degrees of freedom
* Coordinate systems
* Denavit-Hartenberg modelling
* Forward kinematics
* Inverse kinematics
* Robot workspace
* Trajectory generation
* Velocity control
* Orientation control
* Robot simulation
* Robotic manipulation

---

## 👥 Team

| Member   | Index Number |
| -------- | ------------ |
| Member 1 | `XX_ENG_XXX` |
| Member 2 | `XX_ENG_XXX` |
| Member 3 | `XX_ENG_XXX` |

---

## 📖 References

* ROS 2 Documentation
* Gazebo Documentation
* Robotics course material
* CAD and mechanical-design references
* Robot actuator and controller documentation

---

<div align="center">

## ⭐ Project Goal

**Develop a complete robotic-arm system capable of smooth, controlled and orientation-preserving pick-and-place manipulation.**

<br>

`Mechanical Design` • `Kinematics` • `ROS 2` • `Simulation` • `Control` • `3D Printing`

<br>

### 🤖 EN3563 Robotics

</div>
