# Results and remaining submission work

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

The near-mode route is covered by offline tests. A near-mode Gazebo task run,
a completed submission report, a screen recording, and CAD equivalence have
not been verified. No physical robot results are claimed.

## Limitations

The gripper closes visually and its position is checked, but object retention
uses an idealized fixed joint. The very small errors above reflect that
simulation constraint and simulator arithmetic, not physical grasp accuracy.
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
