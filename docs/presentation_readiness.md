# Presentation readiness checklist

Use this checklist before the EN3563 group presentation and final submission.

## Completed and documented

- [x] Six-axis GP7 URDF, adaptive gripper, ROS 2 controllers, and Gazebo task launch are implemented.
- [x] URDF-based FK and bounded numerical IK are implemented; the standard-DH model is independently checked against URDF FK at 104 configurations.
- [x] Pick-and-place planning, action execution, simulated attach/release, and final pose checks are implemented.
- [x] Six offline regression tests and prior Gazebo far/seeded-near runs are recorded in [validation.md](validation.md). The video recording is still outstanding.
- [x] Assignment coordinate discrepancy is documented: the sheet specifies Box A at `(1, 1, 0)`; the lecturer later verbally allowed a reachable location. The fixed Box A location is `(0.55, 0.35, 0)`. Seeded/randomized pickup locations refer to Box B. No written permission is claimed.
- [x] Setup, build, launch, environment variables, troubleshooting, report, coordinate diagrams, and the comprehensive LaTeX study guide are included in this repository.
- [x] The report distinguishes the Gazebo fixed-joint attachment from force-based grasping; no physical grasp has been validated.

## Complete manually before submission

- [ ] Add all group member names and index numbers to `docs/report.md` and the study-guide cover page.
- [ ] Review the report and assignment checklist with the group; confirm any lecturer-specific formatting and submission requirements.
- [ ] Record a GUI Gazebo run using the documented command and environment. Show the full sequence through placement, release, retreat, and `PICK AND PLACE VERIFIED`.
- [ ] Save and review the video; add its filename/link, selected pickup mode, and seed (if near mode) to the report.
- [ ] Upload/compile the editable LaTeX source in Overleaf and review the resulting PDF before submission.

## Demonstration setup

From the repository root, follow [setup.md](setup.md) to deactivate Conda,
source ROS 2 Jazzy, source the workspace, set the host's Gazebo Transport
Python paths, and export the ROS/Gazebo environment. On the previously tested
host, the task environment included:

```bash
export PYTHONPATH="$HOME/.local/gp7-gz-transport13/lib/python:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python${PYTHONPATH:+:$PYTHONPATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=87
export GZ_PARTITION=gp7_pick_place
```

Run a fresh GUI demonstration with a repeatable near-mode Box B pickup:

```bash
ros2 launch gp7_ros2_pkg pick_place.launch.py \
  run_task:=true headless:=false pickup_mode:=near seed:=7
```

For the optimized far-mode Box B pickup, restart the scene and use:

```bash
ros2 launch gp7_ros2_pkg pick_place.launch.py \
  run_task:=true headless:=false pickup_mode:=far
```

The pickup mode changes Box B's pickup location. Box A remains fixed. Allow a
complete run to finish, capture the terminal success output, then stop the
launch with Ctrl+C. Detailed recording steps and visual checklist are in
[report.md](report.md#gazebo-demonstration-video-checklist).

## Evidence boundary

The regression tests and DH verifier are offline checks. Gazebo success is
recorded from earlier runs; it is not evidence of physical robot performance.
The comprehensive guide source is in
[`technical_study_guide/`](technical_study_guide/). A separately reviewed
source ZIP was not found in `~/Downloads` during finalization; place it at
`~/Downloads/GP7_Technical_Study_Guide_Overleaf_Reviewed.zip` to make it
available for comparison and incorporation. The compiled PDF found there
does not provide editable LaTeX sources.
