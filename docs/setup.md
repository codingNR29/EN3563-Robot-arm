# Environment and setup

Commands below use Bash and start in the repository root, wherever you cloned
it. Builds produce `build/`, `install/`, and `log/` there. These are ignored.

## System dependencies

The tested stack is Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic / Sim 8, and
`/usr/bin/python3` 3.12. Follow the official [Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
first. Install Gazebo through the Jazzy ROS integration packages to use the
ROS vendor libraries. Do not add another Gazebo installation just to obtain
Python bindings.

Deactivate Conda before building or running. `conda deactivate` may need to be
repeated if environments are nested. Confirm `command -v python3` points to
`/usr/bin/python3`. Clear stale overlays before sourcing:

```bash
unset AMENT_PREFIX_PATH PYTHONPATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 --version
```

Check package candidates before installing:

```bash
apt-cache policy ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-control-msgs ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-position-controllers python3-numpy python3-scipy
```

On a new system with Jazzy repositories configured, the core dependency command
is below. Use `apt-get --simulate install` with the same list to review changes
first. Installation requires administrator access.

```bash
sudo apt-get update
sudo apt-get install \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-control-msgs ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-position-controllers ros-jazzy-robot-state-publisher \
  ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python \
  ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
  python3-colcon-common-extensions python3-numpy python3-scipy
```

Optional RViz sliders and controller-list CLI:

```bash
sudo apt-get install ros-jazzy-rviz2 ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-urdf-tutorial ros-jazzy-ros2controlcli
```

`urdf_tutorial` is declared by the description package. The task launch does
not directly use it. `ros2controlcli` is optional: the ListControllers service
can be called without it. The manual arm/gripper demo uses the core ROS control
stack and does not require Gazebo's Python transport binding.

Check imports and ROS package discovery after sourcing Jazzy:

```bash
/usr/bin/python3 -c 'import rclpy, numpy, scipy; from control_msgs.action import FollowJointTrajectory'
ros2 pkg prefix gz_ros2_control
ros2 pkg prefix controller_manager
```

The task's package manifest names `python3-gz-transport13` and
`python3-gz-msgs10`. They were not available in the configured ROS/Ubuntu apt
indexes on the tested host. Do not assume a blanket `rosdep install` can resolve
these two names; use the vendor-compatible setup below.

## Transport 13.5.0 Python bindings

The official [Transport documentation](https://gazebosim.org/api/transport/13/python.html)
names `python3-gz-transport13`. On 11 October 2026, the official OSRF Noble
[index](https://packages.osrfoundation.org/gazebo/ubuntu-stable/dists/noble/main/binary-amd64/Packages.gz)
provided only version 13.6.0, requiring standalone Transport 13.6.0 and message
packages. The tested Jazzy vendor install contains Transport 13.5.0 and Messages
10.3.2. We instead built only the matching Python extension, preserving those
libraries. Check your installed library version before using these commands:

```bash
readlink -f /opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so.13
```

These commands apply to **Transport 13.5.0**, not arbitrary vendor versions.
Official [bindings-only build instructions](https://gazebosim.org/api/transport/13/installation.html)
permit configuring the `python` directory against an existing library.

Needed tools are CMake >= 3.22.1, a C++ compiler, Python 3.12 development headers,
pybind11, and the vendor library's development dependencies. All were present on
the tested host; configuration required no extra packages. Check first:

```bash
cmake --version
dpkg-query -W build-essential python3-dev pybind11-dev libprotobuf-dev libzmq3-dev
```

If missing, inspect candidates and simulate installation of the missing names
before installing them. Stop if CMake cannot find a vendor dependency; do not
silently substitute another Transport version.

Download the exact official tag into a separate temporary directory. The
recorded archive was 626,532 bytes, SHA-256
`eb07437111c84ccaa45df85c18705af81dfce61cc1dec5e023335fbd6eb22fde`.

```bash
GP7_BINDING_WORK=$(mktemp -d /tmp/gp7-transport13.XXXXXX)
export GP7_BINDING_WORK
/usr/bin/python3 - <<'PY'
import hashlib, io, os, pathlib, tarfile, urllib.request
url = 'https://codeload.github.com/gazebosim/gz-transport/tar.gz/refs/tags/gz-transport13_13.5.0'
data = urllib.request.urlopen(url, timeout=60).read()
expected = 'eb07437111c84ccaa45df85c18705af81dfce61cc1dec5e023335fbd6eb22fde'
if hashlib.sha256(data).hexdigest() != expected:
    raise SystemExit('Archive hash differs; inspect the source before proceeding')
root = pathlib.Path(os.environ['GP7_BINDING_WORK'])
(root / 'source.tar.gz').write_bytes(data)
with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
    archive.extractall(root, filter='data')
PY

cmake -S "$GP7_BINDING_WORK/gz-transport-gz-transport13_13.5.0/python" \
  -B "$GP7_BINDING_WORK/build" \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/gp7-gz-transport13" \
  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -Dgz-transport13_DIR=/opt/ros/jazzy/opt/gz_transport_vendor/lib/cmake/gz-transport13
cmake --build "$GP7_BINDING_WORK/build" --parallel 2
cmake --install "$GP7_BINDING_WORK/build"
```

Source and build files stay outside the project. The install contains
`lib/python/gz/transport13/__init__.py` and a CPython 3.12 extension; it does not
install a new Transport C++ library. Binding upstream tests are disabled in this
build; imports, linkage, and read-only scene subscriptions were checked instead.

After sourcing Jazzy and the project's `install/local_setup.bash`, set:

```bash
export PYTHONPATH="$HOME/.local/gp7-gz-transport13/lib/python:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -c 'from gz.transport13 import Node; from gz.msgs10.empty_pb2 import Empty; from gz.msgs10.pose_v_pb2 import Pose_V; from gz.msgs10.stringmsg_pb2 import StringMsg'
ldd "$HOME/.local/gp7-gz-transport13/lib/python/gz/transport13/"_transport*.so
```

The Transport library should resolve under
`/opt/ros/jazzy/opt/gz_transport_vendor/lib/`. On this install, message bindings
exist under the vendor prefix but its environment hooks do not add their Python
directory. That is why both paths are explicit. A standalone `ldd -r` may show
Python C API symbols; these are supplied when the extension is loaded by Python.
Non-Python unresolved symbols or missing libraries require investigation.

The repository helper clears `PYTHONPATH`. Use the README's direct launch method
so these binding paths reach the task executable.

## Middleware version mismatch

The first inspection launch failed while loading controller messages:

```text
libcontroller_manager_msgs__rosidl_typesupport_fastrtps_cpp.so:
undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj
```

`ldd -r` resolved Fast-CDR to the Jazzy library, not Conda. New controller-message
binaries were mixed with older middleware packages. Updating Fast-CDR alone
removed the unresolved symbol but caused `BadParamException: This member is not
been selected` during ROS node initialization. Updating the associated Fast
RTPS, RMW, and typesupport packages resolved the observed failure.

The verified set was Fast-CDR 2.2.8, Fast RTPS 2.14.7, Fast RTPS typesupport C/C++
3.6.4, and RMW Fast RTPS C++/shared C++ 8.4.4. These are recorded versions, not a
requirement to downgrade newer compatible Jazzy packages.

Check current versions and simulate a targeted transaction first:

```bash
apt-cache policy ros-jazzy-fastcdr ros-jazzy-fastrtps \
  ros-jazzy-rosidl-typesupport-fastrtps-c ros-jazzy-rosidl-typesupport-fastrtps-cpp \
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rmw-fastrtps-shared-cpp
apt-get --simulate install --only-upgrade \
  ros-jazzy-fastcdr ros-jazzy-fastrtps \
  ros-jazzy-rosidl-typesupport-fastrtps-c ros-jazzy-rosidl-typesupport-fastrtps-cpp \
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rmw-fastrtps-shared-cpp
```

If that transaction is suitable, run the same install command with `sudo` and
without `--simulate`. This is not a full system upgrade. Before launching Gazebo,
check library resolution and basic ROS node creation in a clean environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ldd -r /opt/ros/jazzy/lib/libcontroller_manager_msgs__rosidl_typesupport_fastrtps_cpp.so
/usr/bin/python3 - <<'PY'
import rclpy
rclpy.init()
node = rclpy.create_node('gp7_middleware_check')
print('Node creation passed')
node.destroy_node()
rclpy.shutdown()
PY
```

The tested host also passed publisher/subscriber communication before the
successful inspection launch. Graphics warnings about NVIDIA EGL were present,
but Intel OpenGL initialized; the fatal issue was middleware, not proof of a
required driver change.
