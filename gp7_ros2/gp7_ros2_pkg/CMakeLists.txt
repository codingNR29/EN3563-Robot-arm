cmake_minimum_required(VERSION 3.8)
project(gp7_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ── Install Python scripts as ROS 2 executables ──────────────────────────────
install(PROGRAMS
  scripts/arm_controller_node.py
  scripts/gripper_controller_node.py
  scripts/demo_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Rename executables (drop the .py extension) ──────────────────────────────
# The install(PROGRAMS ...) above keeps the .py extension.
# We use a CMake symlink trick: install renamed copies via a custom command.
install(
  PROGRAMS
    scripts/arm_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_controller_node
)
install(
  PROGRAMS
    scripts/gripper_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_controller_node
)
install(
  PROGRAMS
    scripts/demo_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_sequence_node
)

# ── Install launch files ──────────────────────────────────────────────────────
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
