# GP7 pick-and-place simulation

This repository contains the original GP7 design assets and a ROS 2 / Gazebo
simulation for the EN3563 robotics assignment. The simulation uses a six-axis
GP7 model with an adaptive two-finger gripper.

## Assignment and status

The [task sheet](Task_sheet.pdf) asks for a robot with at least four degrees of
freedom, labeled coordinate frames, a DH table, forward and inverse kinematics,
and a pick-and-place task with unchanged object orientation and smooth motion.
It prints Box A at (1, 1, 0), outside this robot's reachable workspace from the
origin. After the sheet was issued, the lecturer verbally clarified that a
reachable pickup location may be used. Under that subsequent verbal
clarification, the fixed Box A at (0.55, 0.35, 0) used here is acceptable. This
clarification is not written in the assignment sheet; no written permission is
claimed. Seeded near-mode positions are randomized Box B test pickup positions,
not Box A placements. Submission also requires a report and demonstration
video.

The current implementation uses the alternate reachable scene below.
Positions are in metres, in the world/base frame.

| Item | Setting |
| --- | --- |
| Robot base | (0, 0, 0) |
| Box A dimensions | 0.30 × 0.30 × 0.20 |
| Fixed Box A ground position | (0.55, 0.35, 0); model center z = 0.10 |
| Box B dimensions | 0.05 × 0.05 × 0.05 |
| Default far-mode Box B pickup center | Approximately (-0.068740764, -0.928078308, 0.025) |
| Placement target, Box B center | (0.55, 0.35, 0.225) |

`pickup_mode:=far` optimizes the horizontal ground-grasp radius while keeping
the tool pointing down. The calculated boundary is about 0.935621 m; the box
starts 5 mm inside it at radius 0.930621 m. This is a model-specific orientation
constraint, not the reach of every possible pose. The far pickup is not random.

`pickup_mode:=near seed:=7` randomizes the **Box B pickup** x in [0.50, 0.60]
and y in [-0.23, -0.15]. The seed makes this pickup repeatable and does not
randomize the fixed Box A position. A successful Gazebo run in near mode is
recorded in [validation](docs/validation.md).

A GUI run of the far-pickup task succeeded on 11 October 2026. Robot spawning,
all three active controllers, ROS communication, and Gazebo Python transport
communication were also checked. The DH model agrees with URDF-based FK to
numerical roundoff. The report draft and labeled frame drawing are in
[docs/report.md](docs/report.md) and [docs/coordinate_frames.svg](docs/coordinate_frames.svg).
A video recording, physical grasp reliability, and equivalence to the original
CAD have not been verified. See [results and remaining work](docs/validation.md).

## Files

| Location | Purpose |
| --- | --- |
| [Original GP7 assets](<GP7 (Solidworks 2020)/>) | SolidWorks, STEP, STL, and exported URDF files |
| [Temporary design parts](<Temporary 3D model parts/>) | Original STEP design assets; retained as project inputs |
| [gp7_description](gp7_ros2/gp7_description/) | [Robot URDF](gp7_ros2/gp7_description/urdf/gp7.urdf), meshes, [RViz display launch](gp7_ros2/gp7_description/launch/display.launch.py), and RViz configuration |
| [gp7_gazebo](gp7_ros2/gp7_gazebo/) | [Task world](gp7_ros2/gp7_gazebo/worlds/pick_place.sdf), [controllers](gp7_ros2/gp7_gazebo/config/ros2_controllers.yaml), and [empty-world launch](gp7_ros2/gp7_gazebo/launch/gazebo.launch.py) |
| [gp7_ros2_pkg](gp7_ros2/gp7_ros2_pkg/) | Python motion and control package |
| [kinematics.py](gp7_ros2/gp7_ros2_pkg/gp7_motion/kinematics.py) | URDF FK, bounded numerical IK, and trajectory sampling |
| [reach.py](gp7_ros2/gp7_ros2_pkg/gp7_motion/reach.py) | Far-pickup optimization and carry waypoints |
| [model.py](gp7_ros2/gp7_ros2_pkg/gp7_motion/model.py) | Mesh/controller path resolution and grasp plugin |
| [pick_place_node.py](gp7_ros2/gp7_ros2_pkg/scripts/pick_place_node.py) | Task sequencing, feedback, cancellation, and result checks |
| [pick_place.launch.py](gp7_ros2/gp7_ros2_pkg/launch/pick_place.launch.py) | Task scene, robot, controllers, and optional motion node |
| [Arm presets](gp7_ros2/gp7_ros2_pkg/scripts/arm_controller_node.py), [gripper commands](gp7_ros2/gp7_ros2_pkg/scripts/gripper_controller_node.py), [demo sequence](gp7_ros2/gp7_ros2_pkg/scripts/demo_sequence_node.py) | Separate manual/preset controls; the demo is not the assignment task |
| [gp7_sim_control.launch.py](gp7_ros2/gp7_ros2_pkg/launch/gp7_sim_control.launch.py) | Empty-world simulation with manual controller nodes |
| [test_kinematics.py](gp7_ros2/gp7_ros2_pkg/test/test_kinematics.py) | Six offline regression tests |

## Setup and build

Use Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (Sim 8), and system Python 3.12.
Start in the repository root. Full dependency checks, installation commands,
Fast-CDR troubleshooting, and the Transport 13.5.0 bindings build are in
[setup instructions](docs/setup.md).

If Conda is active, run `conda deactivate` until it is inactive. Then:

```bash
unset AMENT_PREFIX_PATH PYTHONPATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/jazzy/setup.bash
command -v python3  # should be /usr/bin/python3
colcon build --base-paths gp7_ros2 --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash

PYTHONPATH="$PWD/gp7_ros2/gp7_ros2_pkg${PYTHONPATH:+:$PYTHONPATH}" \
  /usr/bin/python3 -m unittest discover -s gp7_ros2/gp7_ros2_pkg/test -v
```

The tests check dimensions, home FK, near/far Cartesian routes and controller
interpolation, gripper links, joint bounds, sampled velocity limits, and
unreachable-target rejection. They do not run Gazebo.

## Inspect and run

Use the same terminal after building. On the verified vendor installation, add
the locally built transport bindings and existing message bindings:

```bash
export PYTHONPATH="$HOME/.local/gp7-gz-transport13/lib/python:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python${PYTHONPATH:+:$PYTHONPATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=87
export GZ_PARTITION=gp7_pick_place
/usr/bin/python3 -c 'from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V'

ros2 launch gp7_ros2_pkg pick_place.launch.py run_task:=false headless:=false
```

This starts the GUI and controllers but disables task motion. The grasp plugin
starts attached; the task node normally detaches it during initialization, so
Box B can be constrained in the inspection scene. Wait for activation of
`joint_state_broadcaster`, `arm_controller`, and `gripper_controller`.

In another terminal with the same sourced ROS environment and domain, check:

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

All three states should be `active`. If the optional `ros2controlcli` package is
installed, `ros2 control list_controllers` provides the same check.

Close inspection with Ctrl+C before starting a fresh task scene. Run one of:

```bash
# Complete task with the optimized far pickup
ros2 launch gp7_ros2_pkg pick_place.launch.py run_task:=true headless:=false

# Complete task with seeded random nearby pickup
ros2 launch gp7_ros2_pkg pick_place.launch.py \
  run_task:=true headless:=false pickup_mode:=near seed:=7
```

The task runs once. Look for `PICK AND PLACE VERIFIED`. Restart the scene to
repeat it. Stop the launch with Ctrl+C. For motion cancellation from another
terminal in the same domain:

```bash
ros2 topic pub --once /gp7/stop std_msgs/msg/Empty '{}'
```

Cancellation retains an attached box; restart the scene before another task.
This is a simulation cancellation mechanism, not a physical emergency stop.
If a failed launch leaves Gazebo running, inspect `pgrep -af 'gz sim'` and verify
process identities before stopping them. Avoid starting another scene in the
same partition. [Troubleshooting](docs/setup.md#middleware-version-mismatch)
explains the middleware failure encountered during setup.

The current [helper](gp7_ros2/run_pick_place.sh) clears `PYTHONPATH`, which removes
the explicit binding paths above. Use these direct launch commands; the helper
has not been changed to support the local bindings installation.

## Kinematics and motion

[The DH derivation](docs/kinematics.md) uses standard DH:
`A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`. Its only joint-angle offset is
`theta_2 = q_2 - pi/2`. The fixed base transform is identity, and a separate
fixed tool transform includes the flange, tool0 rotation, and grasp-frame offset.
The table was compared with executable URDF FK at four stated configurations
and 100 configurations sampled within joint limits.

The executable uses URDF transforms for FK. IK uses bounded SciPy least squares
to minimize position error and rotation-vector error while respecting joint
limits. It can reject unreachable targets or fail to converge; it is not an
analytic IK solver or a collision checker.

The planner preflights a scene-specific Cartesian route with fixed downward
rotation `diag(1, -1, -1)`. The initial approach is joint-space motion. At the far
pickup, it first lifts to z = 0.10 m, moves inward while rising, then transfers
above Box A at z = 0.40 m. It lowers to z = 0.226 m, releases, and retreats.

Quintic time scaling `10u^3 - 15u^4 + 6u^5` gives smooth position interpolation.
Position/velocity samples start at 50 ms spacing, and time is stretched when
needed to keep sampled joint speeds below 0.5 rad/s or the smaller URDF limit.
The controller uses position command interfaces. No measured acceleration or
jerk limits are claimed.

The gripper has one commanded driver and five mimic joints. Its seven CAD
meshes and joint spacing are scaled to 1.5 times their native size to fit the
50 mm box; the arm remains at unit scale. Finger motion uses quintic scaling
and position feedback. After closing near Box B, a detachable fixed joint
retains the object. Release removes that constraint and the box settles under
gravity. This demonstrates idealized retention, not a friction/contact-force
grasp. Arm mass and inertia entries are estimates, and the route is not a
general collision planner.
