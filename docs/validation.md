# Results and remaining submission work

## Revalidation on 14 November 2026

The current checkout was rebuilt with the documented system Python setting.
All three packages (`gp7_description`, `gp7_gazebo`, and `gp7_ros2_pkg`)
finished successfully. The six existing offline regression tests passed. The
DH verifier also passed at 104 configurations; maximum position and rotation
errors were `5.50e-16 m` and `4.93e-16 rad`.

A fresh headless Gazebo run activated the joint-state broadcaster, arm
trajectory controller, and gripper controller. It completed approach,
descent, close and attach, lift, inward raise, transfer, lowering, release,
and retreat. The task node emitted `PICK AND PLACE VERIFIED` and exited
successfully. Measured placement error was `8.88e-7 m`; final orientation
error and maximum sampled carried orientation error were both below
`3.80e-7 rad`. As described below, attachment uses the simulator's idealized
fixed-joint grasp plugin, so these measurements do not demonstrate a
force-based grasp. The fresh run was headless; no new screen recording was
created.

Gazebo also reported that its selected physics engine does not support mimic
constraints. The simulated grasp plugin still confirmed attach and detach, and
the task completed, but the run should not be used as evidence of realistic
finger coupling or contact-force performance.

The first attempt in this session failed before motion because the active
Conda Python was first on `PATH` and did not provide NumPy. Prepending
`/usr/bin` selected the tested system Python and the complete run then passed.

## Box A coordinate in the assignment

The supplied `Task_sheet.pdf` specifies Box A at `(1, 1, 0)` and asks for Box B
to be placed at its top-surface center. From a base at the origin, that point
has a horizontal radius of `sqrt(2) = 1.414 m`. The implemented arm geometry
has a conservative horizontal reach bound below `1.03 m` even before applying
joint limits, so this specified target is unreachable. The current simulation
uses Box A at `(0.55, 0.35, 0)`, as listed in the scene table in `README.md`.
Existing project notes report that the lecturer permitted any reachable Box A
location, but that permission is not stated in the supplied task sheet and
could not be independently confirmed from repository materials in this
session. Confirm that permission for the final submission, or obtain an
approved change to the robot placement/assignment coordinates; do not describe
the current scene as satisfying the printed `(1, 1, 0)` coordinate.

## Observed run

On 11 October 2026, a GUI run of the default far-pickup task completed. The task
node emitted `PICK AND PLACE VERIFIED` and exited successfully. Its local log
was `/tmp/gp7_first_pick_place.log`; that temporary log is not committed here.
The summary below transcribes the measured result, not a new accuracy claim.

| Measurement | Value |
| --- | --- |
| Pickup center (m) | (-0.06874076423535015, -0.9280783075872183, 0.025) |
| Pickup radius (m) | 0.9306205659029976 |
| Target Box B center (m) | (0.55, 0.35, 0.225) |
| Final Box B center (m) | (0.5500003051515733, 0.35000014544318847, 0.22499911519319332) |
| Position error (m) | 9.471822891495712e-7, approximately 0.947 micrometres |
| Final orientation change (rad) | 5.965005437765707e-7, approximately 0.0000342 degrees |
| Maximum sampled carried orientation change (rad) | 5.964987133265245e-7 |

The log confirmed successful approach, descent, lift, inward raise, transfer,
lowering, and retreat. Acceptance uses an 8 mm placement tolerance and a
1 degree orientation tolerance. The box is released 1 mm above its target
center height, then allowed to settle before final verification.

## What was checked

- System Python 3.12 build of all three ROS packages.
- Six offline motion regression tests.
- Robot spawning and active joint-state, arm, and gripper controllers.
- ROS node creation and publisher/subscriber communication using Fast RTPS
  after the targeted middleware update.
- Local Transport 13.5.0 Python binding imports and linkage to the existing
  vendor library. Read-only clock and Box B pose subscriptions succeeded.
- Standard DH FK agreement with URDF FK at four stated and 100 sampled joint
  configurations; see [derivation and verification](kinematics.md).
- Default far-pickup GUI task execution and the result above.
- Fresh headless far-pickup Gazebo task execution on 14 November 2026; see
  [revalidation results](#revalidation-on-14-november-2026).

The near-mode route is covered by offline tests. A near-mode Gazebo task run,
a completed submission report, a screen recording, and CAD equivalence have
not been verified. No physical robot results are claimed.

## Limitations

The gripper closes visually and its position is checked, but object retention
uses an idealized fixed joint. The very small errors above reflect that
simulation constraint and simulator arithmetic, not physical grasp accuracy.
Gazebo's physics engine also reports that it does not support mimic constraints,
so realistic coupled finger motion has not been demonstrated.
Orientation is sampled while the carrying flag is set, and checked again after
placement; the log is not continuous proof at every instant of the entire task.

Mass/inertia values are estimates. Waypoints are specific to this scene and do
not constitute general collision planning. The runtime uses bounded numerical
IK, not an analytic solution. Velocity limits are checked at samples; measured
velocity, acceleration, and jerk plots are not available in the recorded log.
The DH comparison proves agreement with the implemented URDF, not its original
SolidWorks geometry or manufacturer specifications.

## Remaining report requirements

The lecturer permits any reachable Box A position. State that permission and
the selected (0.55, 0.35, 0) ground position in the report. The original task
sheet asks for a random reachable pickup: far mode is optimized, whereas near
mode is seeded random. A separate near-mode recording can address that part.

The group's submission still needs:

1. Member index numbers and a concise explanation of the implementation.
2. Labeled frame diagrams consistent with the documented DH convention.
3. The DH table, individual FK matrices, base/tool transforms, and calculation
   examples. The linked derivation provides a starting point.
4. Numerical IK equations, method explanation, and code snippets for IK,
   trajectory planning, and velocity control.
5. Scene dimensions, pickup/placement coordinates, orientation constraint,
   results, and limitations.
6. A screen-recorded demonstration video and its relationship to the reported
   run. A working GUI session alone is not a submitted recording.

The repository may be linked in the report. Retain the original task sheet and
CAD assets as inputs; do not represent the two URDF exports in the CAD folder
as interchangeable with the working simulation model.
