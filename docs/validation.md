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

A second headless run exercised seeded near mode (`seed:=7`) at pickup center
`(0.532383276, -0.217932066, 0.025) m`. It completed the same sequence and
emitted `PICK AND PLACE VERIFIED`, with `8.84e-7 m` placement error and
`2.19e-6 rad` final orientation error. This verifies the implemented seeded
reachable-pickup path in Gazebo. The destination uses the reachable Box A
position covered by the lecturer's subsequent verbal clarification; it is not
the coordinate printed in the original sheet.

Gazebo also reported that its selected physics engine does not support mimic
constraints. The simulated grasp plugin still confirmed attach and detach, and
the task completed, but the run should not be used as evidence of realistic
finger coupling or contact-force performance.

The first attempt in this session failed before motion because the active
Conda Python was first on `PATH` and did not provide NumPy. Prepending
`/usr/bin` selected the tested system Python and the complete run then passed.

## Box A coordinate and lecturer clarification

The supplied `Task_sheet.pdf` specifies Box A at `(1, 1, 0)` and asks for Box B
to be placed at its top-surface center. From a base at the origin, that point
has a horizontal radius of `sqrt(2) = 1.414 m`. The implemented arm geometry
has a conservative horizontal reach bound below `1.03 m` even before applying
joint limits, so this specified target is unreachable. **After the assignment
sheet was issued, the lecturer verbally clarified that a reachable pickup
location may be used.** Under that subsequent verbal clarification, the fixed
Box A ground position `(0.55, 0.35, 0)` used in this simulation is acceptable.
The clarification is not written in the supplied assignment sheet; no written
permission is claimed. State both facts in the report and identify the
clarification as verbal. Random or seeded near-mode coordinates refer to Box
B's pickup pose and do not change Box A. Default far-mode pickup is a separate
model-optimized Box B test pose.

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

The seeded near-mode route has now been checked both offline and in Gazebo.
A report draft, labeled coordinate frame diagram, and video checklist are now
included in [the technical report](report.md). A screen recording, CAD
equivalence, and physical robot results have not been verified or claimed.

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

## Report and submission status

The task sheet prints Box A at `(1, 1, 0)`; the lecturer's subsequent
clarification allowing a reachable pickup is verbal. Report the fixed Box A
position `(0.55, 0.35, 0)` separately from the randomized/seeded Box B near-mode
test pickups. Record the GUI demonstration using the checklist in
[report.md](report.md#gazebo-demonstration-video-checklist).

The [technical report draft](report.md) now contains the DH derivation, frame
diagram, forward/inverse kinematics, code excerpts, simulation results,
limitations, a video recording checklist, and a final assignment checklist.
Fill in group member information and record the GUI demonstration before
submission. A working GUI session alone is not a submitted recording.

The repository may be linked in the report. Retain the original task sheet and
CAD assets as inputs; do not represent the two URDF exports in the CAD folder
as interchangeable with the working simulation model.
