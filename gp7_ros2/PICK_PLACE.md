# Real-size GP7 with adaptive gripper

The task uses the GP7 at **1:1 scale**, the supplied seven adaptive-gripper meshes,
and correctly sized boxes. The gripper opens, closes around Box B, carries it
upright, releases it on Box A and retracts. The default pickup is now near the
maximum reachable ground-grasp distance.

## Dimensions and placement

| Item | Dimensions / setting |
| --- | --- |
| GP7 arm meshes | Scale `1 1 1`; STL vertices are already in metres |
| Shoulder height | 330 mm |
| Shoulder horizontal offset | 40 mm |
| Upper arm | 445 mm |
| Forearm | 440 mm, with 40 mm elbow offset |
| Wrist to flange | 80 mm |
| Manufacturer's rated horizontal reach | 927 mm |
| Box A | 300 × 300 × 200 mm |
| Box B | 50 × 50 × 50 mm |
| Robot base | (0, 0, 0) m |
| Box A ground position | (0.55, 0.35, 0) m; model centre z=0.10 m |
| Box B starting centre (default far mode) | Approximately (−0.06874, −0.92808, 0.025) m |
| Final Box B centre | (0.55, 0.35, 0.225) m |

The arm lengths, offsets, joint ranges and speed limits were checked against the
[Yaskawa GP7 dimension drawing and specifications](https://www.yaskawa.eu.com/_downloads/download_d2303).
The previous 2× arm mesh scale and doubled translations have been removed.
The manufacturer's reach describes the robot flange envelope, not an arbitrary
end-effector's TCP envelope.

**Approved change to the assignment scene:** Box A was moved from (1,1,0) to
(0.55,0.35,0), because the original location is 1.414 m horizontally from the base
and is outside the real GP7's flange reach with this downward gripper. Both boxes
retain the assignment's specified dimensions. Mention the changed position in
your report.

## Maximum-distance pickup

`pickup_mode:=far` is the default. The launch computes the maximum horizontal
radius for the box centre at z=0.025 m, subject to the actual URDF joint limits
and the fixed downward grasp orientation. For this model the calculated boundary
is **0.935621 m**. Box B is placed 5 mm inside it, at **0.930621 m** radius.
The optimizer's result is checked against an independent two-link geometric
upper bound in the tests. This is the maximum for this ground-grasp orientation,
not a claim that all poses have the same maximum reach.

The exact maximum corresponds to a straight-elbow singularity. The small margin
allows continuous IK and feedback tracking. The robot first lifts the box centre
to z=0.10 m, then moves inward to (0.45,−0.45,0.40) m before transferring to Box A.
Trying to lift vertically to 0.40 m at maximum radius would exceed the workspace.
All Cartesian segments preserve the box orientation and are preflighted before
execution. The destination, arm dimensions, gripper and box sizes are unchanged.

To restore the earlier nearby pickup:

```bash
ros2 launch gp7_ros2_pkg pick_place.launch.py pickup_mode:=near seed:=7
```

## Adaptive gripper

The task now retains `gripper_base` and all six finger links from
`gp7_description/meshes/adaptive_gripper/visual`. It no longer replaces them with
a suction cylinder. `gripper_controller` commands `gripper_driver`; the remaining
five joints follow their URDF mimic relationships.

The DAE files declare millimetres (`unit meter="0.001"`). Their native linkage
with the supplied joint layout cannot open far enough for the 50 mm cube. As
approved, **only the gripper is sized to 1.5× native CAD dimensions** (previously
3×); its mesh offsets and pivot spacing use the same factor. There is no supplied
gripper manufacturer/model specification, so this is a task-sized custom gripper,
not a claim of manufacturer-certified dimensions. The GP7 remains 1:1.

A `grasp_frame` explicitly defines the centre between the fingers, rather than a
suction tip: relative to tool0 it is at (−0.0009111, 0.0123, 0.14598) m.
Open angle is +0.15 rad and the task gripping angle is −0.162 rad. Finger commands
use smooth time scaling and measured joint feedback.

The simulation uses an **idealized fixed-joint grasp constraint** after the
fingers close and the grasp frame is near Box B. It releases that constraint at
placement. The object is dynamic and settles under gravity; its pose is not
teleported during transfer. This is not a friction/contact-force grasp validation.
Existing arm mass and inertia entries remain estimates, not verified CAD mass
properties. The scene-specific route is not a general collision planner.

## Build and run

Close the old launch with Ctrl+C first: a running simulator keeps its previously
spawned robot and does not reload an edited URDF.

```bash
cd /home/banula/ros2_ws/src
unset AMENT_PREFIX_PATH PYTHONPATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
colcon build --base-paths gp7_ros2 --symlink-install
source install/local_setup.bash
export ROS_DOMAIN_ID=87
export GZ_PARTITION=gp7_pick_place
ros2 launch gp7_ros2_pkg pick_place.launch.py
```

For subsequent launches, use the helper that isolates Python and ROS package
paths from older installations:

```bash
cd /home/banula/ros2_ws/src
./gp7_ros2/run_pick_place.sh
# Optional: ./gp7_ros2/run_pick_place.sh pickup_mode:=near
```

If you see `No module named gp7_motion.reach`, an older `gp7_motion` package is
usually ahead of this installation on `PYTHONPATH`. Clearing only
`AMENT_PREFIX_PATH` does not fix Python's import ordering. The helper resets both
inside its own process, leaving your terminal's environment unchanged.

The launch waits for the arm and gripper controllers and performs the task once.
Look for `PICK AND PLACE VERIFIED`. Restart the scene to repeat it. Optional
arguments are `headless:=true`, `pickup_mode:=near`, and `run_task:=false` for inspection.
`seed:=12` changes the random position only in near mode.
If running the motion executable manually, supply pickup coordinates matching the
spawned box; it rejects mismatches. Use a different ROS domain and Gazebo partition
for concurrent simulations.

To cancel from another terminal with the same ROS environment:

```bash
ros2 topic pub --once /gp7/stop std_msgs/msg/Empty '{}'
```

Cancellation retains an attached box. Restart the scene before another attempt.
This is a simulation stop, not a physical robot emergency stop.

## Implementation and checks

- `gp7_description/urdf/gp7.urdf`: real-size arm, adaptive gripper, grasp frame.
- `gp7_ros2_pkg/gp7_motion/model.py`: retains the complete URDF and adds grasp retention.
- `gp7_ros2_pkg/gp7_motion/kinematics.py`: URDF FK and bounded numerical IK.
- `gp7_ros2_pkg/gp7_motion/reach.py`: maximum ground reach and inward-lift waypoints.
- `gp7_ros2_pkg/scripts/pick_place_node.py`: finger and arm sequencing with feedback.
- `gp7_ros2_pkg/launch/pick_place.launch.py`: scene, robot and all three controllers.
- `gp7_gazebo/worlds/pick_place.sdf`: true-size boxes and revised reachable destination.

Cartesian segments keep rotation fixed at diag(1,−1,−1), use quintic time scaling
10s³−15s⁴+6s⁵, and send position/velocity samples at 50 ms intervals. Joint speeds
are limited to 0.5 rad/s or the smaller URDF limit. IK preflights the route before
motion. The runtime checks measured joint arrival, finger position, attachment,
box position and orientation. Acceptance requires less than 8 mm placement error
and less than 1 degree orientation change.

```bash
PYTHONPATH=gp7_ros2/gp7_ros2_pkg python3 -m unittest discover \
  -s gp7_ros2/gp7_ros2_pkg/test -v
```

Tests cover arm/box dimensions, preservation of all seven gripper links, home FK,
upright trajectory interpolation, unreachable-target rejection, and the maximum
reach bound plus the full far-pickup route.

A full headless ROS 2 Jazzy / Gazebo Harmonic run with the real-size arm and
adaptive gripper passed on 2026-10-07:

- Final Box B centre: (0.549999963, 0.350000011, 0.224999598) m.
- Position error: 4.04e−7 m.
- Maximum measured orientation change while carried: 2.19e−6 rad.

These are idealized simulation measurements, not physical robot accuracy claims.
The group's DH derivation, frame diagrams and video still belong in the report;
the executable uses URDF transforms and numerical IK.

The far-pickup run passed on 2026-10-08: pickup radius 0.930621 m,
final centre (0.550000133, 0.350000015, 0.224999603) m, placement error
4.19e−7 m, and maximum measured carried orientation change 3.73e−7 rad.
All six regression tests passed. These results use the same idealized grasp
retention described above.
