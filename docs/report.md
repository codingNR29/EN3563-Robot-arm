# EN3563 Robotics — GP7 pick-and-place technical report

> **Submission draft.** Fill in every group member field and attach the recorded
> Gazebo demonstration before submission. This report documents the working
> ROS 2 simulation and its measured checks; it does not claim a physical robot
> test.

## Group information

| Field | Complete before submission |
| --- | --- |
| Module / assignment | EN3563 Robotics — GP7 pick-and-place |
| Group number | `[add group number]` |
| Member 1 name and index number | `[add name and index number]` |
| Member 2 name and index number | `[add name and index number]` |
| Member 3 name and index number | `[add name and index number]` |
| Member 4 name and index number | `[add name and index number]` |
| Demonstration video filename / link | `[add after recording]` |
| Submission date | `[add date]` |

## Abstract

This project implements a six-revolute-joint Yaskawa GP7 model in ROS 2 Jazzy
and Gazebo Harmonic. A feedback-checked task node moves a 50 mm cube (Box B)
from a reachable ground pickup pose to the top center of a fixed 300 × 300 ×
200 mm box (Box A), holding the tool downward and checking that the cube's
orientation is maintained. The runtime obtains forward kinematics (FK) from
the URDF and solves inverse kinematics (IK) with bounded numerical least
squares. A standard Denavit–Hartenberg (DH) model was independently compared
with URDF FK at 104 joint configurations. Recorded headless runs completed the
far and seeded near pickup routes. Retention during those runs used a Gazebo
fixed-joint grasp plugin; it does not establish a frictional grasp or physical
robot performance.

## Assignment interpretation and task scene

The supplied task sheet prints Box A at `(1, 1, 0)`. At a base frame located
at the robot origin, the horizontal distance is

```text
r = sqrt(1^2 + 1^2) = sqrt(2) = 1.4142 m.
```

That location is beyond the implemented model's conservative horizontal reach
bound of 1.03 m, even before joint limits. **After the assignment sheet was
issued, the lecturer verbally clarified that a reachable pickup location may
be used.** The simulation therefore keeps Box A fixed at ground position
`(0.55, 0.35, 0) m`, acceptable under that subsequent verbal clarification.
The clarification is not written in the original assignment sheet, and this
report does not claim written permission. The final submission should preserve
this distinction.

Box A is the large, stationary destination. Its dimensions are 0.30 × 0.30 ×
0.20 m, so its center is at z = 0.10 m. Box B is a 0.05 m cube. The target for
its center is `(0.55, 0.35, 0.225) m`, at the center of Box A's top surface.
The fixed Box A position does not change between runs.

Two Box B pickup modes are implemented:

- **Far mode (default):** model optimization finds the maximum horizontal
  ground-grasp radius for a downward tool orientation, approximately 0.935621
  m. The actual pickup is placed 5 mm inside that boundary, at approximately
  `(-0.068741, -0.928078, 0.025) m`. This pose is optimized, not randomized.
- **Near mode:** the launch file samples Box B's x coordinate uniformly from
  `[0.50, 0.60] m` and y from `[-0.23, -0.15] m`. An integer seed makes that
  randomized pickup repeatable. With seed 7 the pickup was
  `(0.532383, -0.217932, 0.025) m`. This randomization applies only to Box B,
  never to the fixed Box A.

## Robot model and coordinate frames

The simulated arm has six revolute joints in URDF order `joint_1_s` through
`joint_6_t`. The world-to-base transform is identity, and lengths are in
metres. The arm meshes are scaled 1:1. The adaptive gripper CAD meshes are
scaled 1.5× to fit the 50 mm cube; this is a project adaptation, not a
manufacturer-certified gripper configuration.

The frame drawing is [coordinate_frames.svg](coordinate_frames.svg). It shows
the standard-DH origins at the zero joint configuration and annotates the
joint-axis directions, including axes that point out of the page. The axes
shown are DH axes, not a claim that every frame origin is a URDF link origin.
Frame 3 is at the intersection of the extended elbow and wrist-roll axes.
Frames 4, 5, and 6 share the wrist-center origin. The separate fixed tool
transform is included when computing the grasp frame.

## Standard DH derivation

The convention is **standard DH**, with joint i rotating about `z_(i-1)`:

```text
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)

       [ cosθ  -sinθ cosα   sinθ sinα   a cosθ ]
A_i =  [ sinθ   cosθ cosα  -cosθ sinα   a sinθ ]
       [  0        sinα        cosα        d   ]
       [  0         0            0         1   ]
```

Reading the six URDF joint origins and local axes gives these signed DH
parameters. The only joint offset is `theta_2 = q_2 - pi/2`; no joint sign is
reversed.

| i | Joint | `a_i` (m) | `alpha_i` | `d_i` (m) | `theta_i` |
| --- | --- | ---: | ---: | ---: | --- |
| 1 | S | 0.040 | `-pi/2` | 0.330 | `q_1` |
| 2 | L | 0.445 | `pi` | 0 | `q_2 - pi/2` |
| 3 | U | 0.040 | `-pi/2` | 0 | `q_3` |
| 4 | R | 0 | `pi/2` | -0.440 | `q_4` |
| 5 | B | 0 | `-pi/2` | 0 | `q_5` |
| 6 | T | 0 | 0 | 0 | `q_6` |

The negative `d_4` is a signed frame displacement: frame 3's z axis points in
the negative x direction, while the wrist center is 0.440 m in positive x.
It is not a negative physical link length. The wrist axes intersect, so rows 5
and 6 add no translation.

For an explicit matrix example, substituting row 1 gives:

```text
c1 = cos(q1), s1 = sin(q1)

A1 = [ c1   0  -s1   0.040 c1 ]
     [ s1   0   c1   0.040 s1 ]
     [  0  -1    0       0.330 ]
     [  0   0    0           1 ]
```

The corresponding substitutions for rows 2–6, using `v2 = q2 - pi/2`, are:

```text
A2 = [ cos(v2)  sin(v2)   0   0.445 cos(v2) ]
     [ sin(v2) -cos(v2)   0   0.445 sin(v2) ]
     [    0        0     -1          0       ]
     [    0        0      0          1       ]

A3 = [ c3   0  -s3   0.040 c3 ]
     [ s3   0   c3   0.040 s3 ]
     [  0  -1    0          0 ]
     [  0   0    0          1 ]

A4 = [ c4   0   s4       0    ]
     [ s4   0  -c4       0    ]
     [  0   1    0    -0.440  ]
     [  0   0    0       1    ]

A5 = [ c5   0  -s5   0 ]
     [ s5   0   c5   0 ]
     [  0  -1    0   0 ]
     [  0   0    0   1 ]

A6 = [ c6  -s6   0   0 ]
     [ s6   c6   0   0 ]
     [  0    0   1   0 ]
     [  0    0   0   1 ]
```

Here `ci = cos(q_i)` and `si = sin(q_i)`. Multiplying the six matrices gives
the DH frame-6 pose. The flange and gripper grasp point are not frame 6, so a
fixed transform is required:

```text
T_base_grasp(q) = A1 A2 A3 A4 A5 A6 T6_grasp

T6_grasp ≈ [ 1   0   0  -0.0009111 ]
            [ 0  -1   0  -0.0123    ]
            [ 0   0  -1  -0.22598   ]
            [ 0   0   0   1          ]
```

This fixed transform combines the URDF link6-to-flange translation of
`(0.080, 0, 0) m`, flange-to-tool0 rotation, and tool0-to-grasp translation
`(-0.0009111, 0.0123, 0.14598) m`, expressed in the chosen DH frame. The
verification retains the URDF's decimal RPY values.

## Forward kinematics

For joint vector `q = [q1, ..., q6]`, the end-effector transform is the product
above. Its rotation block `R` gives tool orientation and translation column
`p = [x, y, z]^T` gives the grasp-frame position. As a numerical check, at
`q = [0, 0, 0, 0, 0, 0]`, the grasp point is approximately
`(0.705980, -0.012300, 0.814089) m`.

The independent script [verify_dh.py](verify_dh.py) evaluates the DH table and
URDF transforms separately. At four stated configurations and 100
deterministic configurations inside the URDF joint limits, the maximum recorded
position difference was `5.50e-16 m` and maximum rotation difference was
`4.93e-16 rad` (latest revalidation). This establishes numerical agreement
between the derivation and this URDF. It does not validate the CAD export
against the manufacturer's robot.

## Inverse kinematics

The executable does not use a closed-form GP7 inverse solution. Given desired
position `p_d` and rotation `R_d`, it minimizes the six-component pose residual
over joint angles:

```text
r(q) = [ p_FK(q) - p_d ]
       [ log(R_d^T R_FK(q)) ]
```

The rotation term is the rotation vector of the relative rotation. SciPy's
bounded `least_squares` solver uses limits parsed directly from the URDF. A
previous waypoint solution seeds the next solve; optional deterministic
restarts try 24 additional seeds. A solution is accepted when the combined
residual norm is less than `1e-6`. That norm mixes metres and radians, and
therefore is a numerical stopping criterion rather than a universal physical
tolerance.

Excerpt from `gp7_motion/kinematics.py`:

```python
def residual(q):
    pose = self.fk(q)
    return np.r_[pose[:3, 3] - position,
                 Rotation.from_matrix(rotation.T @ pose[:3, :3]).as_rotvec()]

result = least_squares(residual, seed,
                       bounds=(self.lower, self.upper), max_nfev=150,
                       ftol=1e-10, xtol=1e-10, gtol=1e-10)
if np.linalg.norm(residual(result.x)) < 1e-6:
    return result.x
```

IK can fail to converge for an unreachable or poorly conditioned pose. The
planner preflights its route and aborts on a failed solve or discontinuous
joint branch. This solver does not include collision constraints.

## Trajectory planning and control

The task fixes the grasp orientation to a downward rotation
`diag(1, -1, -1)`. The initial approach is a joint-space segment. Cartesian
segments then descend to Box B, lift, transfer above Box A, lower the cube to
1 mm above its target center, release, and retreat. For the far pickup, the
route first lifts to 0.10 m, raises while moving inward via `(0.45, -0.45,
0.40) m`, and then transfers at z = 0.40 m to avoid cutting through the base.
Near pickup lifts to z = 0.40 m before transfer.

Each segment samples the quintic blend

```text
s(u) = 10u^3 - 15u^4 + 6u^5,  u = t/T,  0 <= u <= 1.
```

This has zero endpoint velocity and acceleration. Cartesian position is
interpolated along a straight line while holding orientation, and IK is solved
at 50 ms intervals. Sampled joint speeds are time-stretched to remain below
the smaller of each URDF speed limit and 0.5 rad/s. The implementation sets
endpoint velocities to zero. It does not report measured acceleration or jerk
limits, and waypoint interpolation is not a general obstacle-avoidance
planner.

ROS 2 control uses a `JointTrajectoryController` position interface for the
six arm joints. The task node sends `FollowJointTrajectory` actions, checks
action results and joint feedback, and cancels an active goal on stop or
failure. The gripper driver is commanded through a position controller and
checked with joint feedback.

## ROS 2 and Gazebo architecture

- `gp7_description` supplies the URDF, visual and collision meshes, and RViz
  display resources.
- `gp7_gazebo` supplies the task world, Gazebo launch integration, and
  controller configuration.
- `gp7_ros2_pkg` contains URDF FK/IK, reach planning, launch descriptions,
  manual control nodes, and the feedback-checked pick-and-place task node.
- `robot_state_publisher` publishes transforms from the URDF and joint states.
  Gazebo spawns the robot, runs the simulation, and hosts the ROS 2 control
  plugin. The launch file starts the joint-state broadcaster, arm trajectory
  controller, and gripper controller.
- `pick_place_node` consumes joint state, clock, box pose, and grasp state;
  sends arm trajectory actions, gripper commands, and attach/detach requests.
  It verifies pickup alignment, attachment state, carried orientation,
  placement tolerance, final orientation, and retreat completion.

Build and runtime setup are documented in [setup.md](setup.md), including the
tested Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / system Python stack and
Gazebo Transport Python binding requirements.

## Gripper and grasp model

The adaptive two-finger gripper includes a commanded driver and five mimic
joints. Its meshes and spacing are scaled by 1.5× for this 50 mm cube. The
driver motion uses quintic interpolation, and the task checks driver joint
feedback. Gazebo reported that its selected physics engine does not support
mimic constraints. **Object retention in the validated task is idealized:** a
Gazebo fixed-joint grasp plugin attaches Box B after closure and detaches it at
release. This demonstrates task sequencing, pose tracking, and simulated
attachment state. It is not evidence of finger coupling in the selected
physics engine, frictional/contact-force grasping, or physical grasp
reliability.

## Build, verification, and results

Revalidation on 14 November 2026 recorded:

| Check | Result |
| --- | --- |
| Build | `gp7_description`, `gp7_gazebo`, and `gp7_ros2_pkg` all built successfully |
| Offline regression | Six tests passed |
| DH vs URDF FK | 104 configurations; max `5.50e-16 m`, `4.93e-16 rad` |
| Far-mode Gazebo task | Headless run completed and emitted `PICK AND PLACE VERIFIED` |
| Far-mode placement | Position error `8.88e-7 m`; final and sampled carried orientation error below `3.80e-7 rad` |
| Seeded near mode, seed 7 | Pickup `(0.532383276, -0.217932066, 0.025) m`; task completed in headless Gazebo |
| Near-mode placement | Position error `8.84e-7 m`; final orientation error `2.19e-6 rad` |
| Task phases | Approach, descent, close/attach, lift, transfer, placement, release, retreat |

The reported placement tolerance is 8 mm and orientation tolerance is 1°.
The micrometre-scale simulated errors arise while an idealized fixed joint
retains the cube and from simulator arithmetic; they are not physical accuracy
claims. The headless revalidation runs did not produce a screen recording.
See [validation notes](validation.md) for the complete test context and
limitations.

## Limitations

1. Box A's `(1, 1, 0)` assignment-sheet coordinate is outside the implemented
   robot's reachable region. The fixed reachable location is accepted under a
   subsequent verbal clarification from the lecturer; that clarification is
   not in the supplied sheet and is not written permission.
2. The Gazebo grasp plugin provides idealized fixed-joint attachment. No
   contact-force or frictional grasp result is established.
3. The physics engine does not support the configured mimic constraints, so
   coupled finger behavior was not demonstrated by the validated run.
4. The solver is numerical, can fail, and does not enforce collision
   avoidance. Routes are scene-specific.
5. URDF inertial values are estimates. DH agreement proves consistency with
   the simulation URDF, not equivalence to original CAD or a production GP7.
6. No physical robot test, measured acceleration/jerk analysis, or submitted
   video is included in the current evidence.

## Gazebo demonstration video checklist

### Before recording

- [ ] Fill the group information table above.
- [ ] Install a screen recorder (for example, OBS Studio) and select the Gazebo
  window or full desktop. Use a readable resolution and hide unrelated desktop
  notifications. Keep the terminal visible if it is useful for showing the
  task launch and final success message.
- [ ] Follow the environment and build steps in [setup.md](setup.md). Start
  from a clean sourced ROS 2 Jazzy environment and the repository root.
- [ ] Set the required Python paths and Gazebo/ROS environment exactly as in
  [README.md](../README.md#inspect-and-run). Confirm the Gazebo Transport
  bindings import successfully.
- [ ] Close any old inspection or task simulation. Start a fresh GUI task
  scene. Use near mode with a recorded seed to demonstrate a seeded random
  reachable pickup; optionally make a second clip with default far mode.

### Recording commands

After sourcing ROS and the workspace, exporting the documented binding paths,
and starting the screen recording, run this in a terminal at the repository
root:

```bash
ros2 launch gp7_ros2_pkg pick_place.launch.py \
  run_task:=true headless:=false pickup_mode:=near seed:=7
```

For an optional far-mode run, restart with a fresh scene and run:

```bash
ros2 launch gp7_ros2_pkg pick_place.launch.py \
  run_task:=true headless:=false pickup_mode:=far
```

### What the video should show

- [ ] Gazebo GUI with the GP7, adaptive gripper, fixed Box A, and Box B before
  motion.
- [ ] The selected mode and seed in the terminal, so the pickup pose can be
  distinguished from the fixed Box A destination.
- [ ] Downward approach, descent, gripper close, lift, upright transfer,
  placement, release, and retreat in one continuous, unedited run where
  possible.
- [ ] Box B resting on the center of Box A's top surface after release.
- [ ] Terminal result `PICK AND PLACE VERIFIED` and the final position and
  orientation errors, if the node's output is visible.
- [ ] No cuts that imply unshown task phases. If the recording is split, label
  each clip and state which phases it shows.

Stop recording after the final result. Save the original recording, review the
file to confirm that audio/frames are intact, and include the filename and
mode/seed in the report. Do not describe simulator fixed-joint attachment as a
physical grasp.

## Final assignment requirements checklist

- [x] Robot has at least four degrees of freedom (six revolute joints).
- [x] Labeled coordinate frames and a DH table are included in this report and
  the linked diagram/derivation.
- [x] Forward kinematics is derived and compared with URDF FK.
- [x] Inverse kinematics method, residual, limits, and implementation excerpt
  are documented.
- [x] Pick-and-place uses a fixed downward tool orientation and smooth
  quintic-scaled trajectories.
- [x] Reachable Box A choice is documented as a later verbal clarification,
  distinct from the original sheet's coordinate.
- [x] Fixed Box A placement is distinguished from seeded/random Box B pickup
  test positions.
- [x] Build, regression, FK, and Gazebo simulation results are documented.
- [x] Gripper integration and idealized grasp retention are distinguished
  from unverified physical grasping.
- [ ] Enter all group member names and index numbers.
- [ ] Record and attach the Gazebo demonstration video.
- [ ] Review final report wording and any lecturer-specific submission format
  before upload.
