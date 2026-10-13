cmake_minimum_required(VERSION 3.16)
project(litearm_vr_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# One frame from the headset, forwarded verbatim by the Python transport node.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VrControllerFrame.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# ---------------------------------------------------------------------------
# Layer 1: pure logic, no ROS headers at all.
#
# The teleop increment mapping, the joint filter and the point to point profile
# live here, so they can be unit tested without a robot, a controller manager or
# a headset. On real hardware these are the layers where a wrong sign is
# hardest to see and easiest to catch in a test, which is exactly why the split
# is drawn at this line.
# ---------------------------------------------------------------------------
add_library(litearm_vr_teleop_math SHARED
  src/teleop_math.cpp
  src/joint_move.cpp
  src/forward_command.cpp
)
target_include_directories(litearm_vr_teleop_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Plain (not keyword) form throughout this file: ament_target_dependencies uses the
# plain signature, and CMake refuses to mix the two on one target. The plain
# signature is PUBLIC by default, so include directories still propagate.
target_link_libraries(litearm_vr_teleop_math Eigen3::Eigen)
set_target_properties(litearm_vr_teleop_math PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Layer 2: planning. Forward kinematics, the Jacobian and the two IK backends.
#
# Deliberately does not depend on controller_interface: planning is not a
# control concern and is exercised by the node, not by the controller.
# ---------------------------------------------------------------------------
add_library(litearm_vr_teleop_planning SHARED
  src/teleop_planner.cpp
)
target_include_directories(litearm_vr_teleop_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(litearm_vr_teleop_planning
  litearm_vr_teleop_math
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_kinematics_base
  srdfdom::srdfdom
  urdf::urdf
)
ament_target_dependencies(litearm_vr_teleop_planning
  rclcpp
  geometry_msgs
  moveit_msgs
  srdfdom
  pluginlib
)
set_target_properties(litearm_vr_teleop_planning PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Layer 3: the ros2_control controller plugin.
#
# Derives from forward_command_controller::ForwardControllersBase, which is what
# makes this a forward controller rather than a from scratch servo loop: the
# base class owns the ~/commands subscription and the realtime buffer, and the
# derived class only has to decide WHAT gets written to the interfaces. It has
# to decide, because writing position alone freezes this arm's firmware (see
# vr_forward_controller.cpp).
# ---------------------------------------------------------------------------
add_library(litearm_vr_teleop_controller SHARED
  src/vr_forward_controller.cpp
)
target_include_directories(litearm_vr_teleop_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(litearm_vr_teleop_controller litearm_vr_teleop_math)
ament_target_dependencies(litearm_vr_teleop_controller
  controller_interface
  forward_command_controller
  hardware_interface
  pluginlib
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
pluginlib_export_plugin_description_file(controller_interface litearm_vr_teleop.xml)

# ---------------------------------------------------------------------------
# The teleop node: headset frames in, controller commands out.
# ---------------------------------------------------------------------------
add_executable(vr_teleop_node src/vr_teleop_node.cpp)
target_link_libraries(vr_teleop_node
  litearm_vr_teleop_math
  litearm_vr_teleop_planning
)
ament_target_dependencies(vr_teleop_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  control_msgs
)

# The node consumes the generated message, so it needs the generated C++ typesupport.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vr_teleop_node "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# The target viewer: headset frames in, a pose for RViz out.
#
# Not a controller and not on the command path: it reads the same frames the
# teleop node reads, maps them with the same library, and publishes a tf frame, a
# Marker and a PoseStamped. It links the same math and planning targets as the
# teleop node precisely so that what it draws is computed by the code that
# commands the arm rather than by a second implementation of it.
# ---------------------------------------------------------------------------
add_executable(vr_target_viz src/vr_target_viz.cpp)
target_link_libraries(vr_target_viz
  litearm_vr_teleop_math
  litearm_vr_teleop_planning
)
ament_target_dependencies(vr_target_viz
  rclcpp
  geometry_msgs
  tf2_ros
  visualization_msgs
)

# Same generated typesupport as the teleop node above.
target_link_libraries(vr_target_viz "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Python transport node.
#
# One file, installed as a program, so launch can start it like any other node.
# It is deliberately not a Python package: rosidl already installs a module named
# `litearm_vr_teleop` for the generated messages, and a second one of the same name
# collides on its install targets.
#
# The script re-executes itself into the interpreter that has `websockets` (see
# its module docstring) and does so in place, so launch's process handling and
# SIGINT delivery are unaffected.
# ---------------------------------------------------------------------------
install(PROGRAMS python/headset_link.py
  DESTINATION lib/${PROJECT_NAME}
)

# The description check the launch runs: it needs no ROS libraries, only the
# AMENT prefix path, so it installs as a plain program like the link does.
install(PROGRAMS tools/check_description_meshes.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  litearm_vr_teleop_math
  litearm_vr_teleop_planning
  litearm_vr_teleop_controller
  vr_teleop_node
  vr_target_viz
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # Executables go under lib/<package>, which is where ros2 launch and ros2 run
  # look for them. Installing them to bin produces a node that exists, runs from
  # a terminal when given its full path, and is invisible to every launch file.
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(litearm_vr_teleop_math litearm_vr_teleop_planning)
ament_export_targets(export_${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Tests
#
# Everything under test here is pure logic: the increment mapping, the joint
# filter and the point to point profile. No ROS, no hardware. run_tests is not
# what proves the stack works; it is what proves a sign flip gets caught before
# it reaches a motor.
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_teleop_math test/test_teleop_math.cpp)
  if(TARGET test_teleop_math)
    target_link_libraries(test_teleop_math litearm_vr_teleop_math)
  endif()

  ament_add_gtest(test_joint_move test/test_joint_move.cpp)
  if(TARGET test_joint_move)
    target_link_libraries(test_joint_move litearm_vr_teleop_math)
  endif()

  ament_add_gtest(test_forward_command test/test_forward_command.cpp)
  # Header only: the test needs the include directory, not the math library.
  ament_add_gtest(test_conditioning_guard test/test_conditioning_guard.cpp)
  if(TARGET test_conditioning_guard)
    target_include_directories(test_conditioning_guard PRIVATE include)
  endif()
  if(TARGET test_forward_command)
    target_link_libraries(test_forward_command litearm_vr_teleop_math)
  endif()

  # The values repeated from litearm_vr_moveit_config must not drift apart from
  # the originals. PYTEST_DISABLE_PLUGIN_AUTOLOAD keeps third party pytest
  # plugins from being imported, which on this machine means one that needs a
  # newer pytest than the system provides; this test only needs pytest itself.
  ament_add_pytest_test(test_kinematics_consistency test/test_kinematics_consistency.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )

  # The headset frame parsing and the hand-off to the callback. Needs no socket,
  # no headset and no rclpy: the module is loaded by path and frames are fed in
  # directly.
  ament_add_pytest_test(test_headset_link test/test_headset_link.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )

  # The description-mesh check the launch runs: it must name the package a
  # description refers to and cannot reach, and must stay quiet when it can.
  ament_add_pytest_test(test_description_meshes test/test_description_meshes.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )

  # The two rates the command path is built on, and the joint order the flat
  # command array means. Both are repeated between this package's own two
  # configuration files, which is what makes them checkable here at all.
  ament_add_pytest_test(test_rate_consistency test/test_rate_consistency.py
    APPEND_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
