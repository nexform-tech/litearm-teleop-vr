cmake_minimum_required(VERSION 3.16)
project(litearm_vr_moveit_config)

find_package(ament_cmake REQUIRED)

# Configuration only: robot description, semantic description and MoveIt
# configuration. The package builds no binaries of its own; the executables
# that consume it live in the sibling VR teleoperation package.

# -----------------------------------------------------------------------------
# Which package provides the bare arm description?
#
# This package needs the arm's links and joints WITHOUT any ros2_control block,
# because its mock hardware mode replaces the hardware and must not inherit the
# real one. The real hardware mode gets the same description the other way, by
# including litearm_ros2_control's entry point, so only the mock mode has to name
# it - and the name is a property of the WORKSPACE, not of this package.
#
# There is exactly ONE arm description this stack is written against:
# `litearm_description`, whose URDF names its joints joint1..joint7, its links
# joint1..joint7 and its tool flange ee_link. litearm_ros2_control's hardware
# block, litearm_moveit_config's SRDF, the mock hardware here and every config
# file follow that naming.
#
# A superseded CAD export of this arm is still around: it names its joints
# Joint1..Joint7, its links Link1..Link7 and its tool flange ee_frame_link. It is NOT
# a naming variant of the same arm: the two URDFs share 0 of 7 joint origins and
# their total mass differs (3.5305 kg vs 2.7911 kg). Pointing this stack at it
# produces an URDF that urdfdom refuses:
#   parent link [ee_link] of joint [gripper_mount_joint] not found
# so it is deliberately not supported - this package builds against
# litearm_description alone.
#
# The description file is <package>.urdf, so one word builds the path.
# -----------------------------------------------------------------------------
# The description package is found through the AMENT INDEX rather than through
# find_package, because the index is what $(find <pkg>) will use at run time and is
# therefore the predicate that actually matters here. Two weaker tests were tried
# and both were wrong against a hand copied install prefix:
#
#   find_package(<pkg> QUIET)     can SUCCEED on a partial copy whose CMake config
#                                 survived but whose export file did not, and the
#                                 build then dies with "imported targets are
#                                 referenced, but are missing".
#   find_package(<pkg> REQUIRED)  can FAIL on a package that is perfectly usable at
#                                 run time, for the same reason. Requiring a CMake
#                                 config in order to learn a package's NAME would be
#                                 a strange dependency to take.
#
# The candidate is still checked for the names this package actually uses, rather
# than accepted for having the right package name: a stale install of the old
# generation under a lookalike name would otherwise pass here and fail inside
# xacro. An earlier attempt read the package name out of litearm_ros2_control's own
# include list, which looked authoritative and was not: that file mentions its own
# name too, and the first `.urdf` in it is part of `litearm.urdf.xacro`.
string(REPLACE ":" ";" _ament_prefixes "$ENV{AMENT_PREFIX_PATH}")

set(ARM_DESCRIPTION_PACKAGE "")
set(ARM_DESCRIPTION_PREFIX "")
set(_rejected "")
foreach(_candidate litearm_description)
  if(NOT ARM_DESCRIPTION_PACKAGE)
    foreach(_prefix ${_ament_prefixes})
      if(NOT ARM_DESCRIPTION_PACKAGE AND EXISTS
         "${_prefix}/share/ament_index/resource_index/packages/${_candidate}")
        set(_urdf "${_prefix}/share/${_candidate}/urdf/${_candidate}.urdf")
        if(NOT EXISTS "${_urdf}")
          string(CONCAT _why
            "${_candidate} (registered, but has no urdf/${_candidate}.urdf)")
          list(APPEND _rejected "${_why}")
        else()
          file(READ "${_urdf}" _urdf_text)
          if(_urdf_text MATCHES "name=\"joint1\"" AND _urdf_text MATCHES "name=\"ee_link\"")
            set(ARM_DESCRIPTION_PACKAGE "${_candidate}")
            set(ARM_DESCRIPTION_PREFIX "${_prefix}")
          else()
            # string(CONCAT) and not two adjacent literals: in CMake those become a
            # semicolon separated LIST, and the message grows a stray ";" where the
            # line break was.
            string(CONCAT _why
              "${_candidate} (registered, but its URDF does not declare joint1 / ee_link, "
              "so it is not the generation this stack is written against)")
            list(APPEND _rejected "${_why}")
          endif()
        endif()
      endif()
    endforeach()
  endif()
endforeach()

if(NOT ARM_DESCRIPTION_PACKAGE)
  message(FATAL_ERROR
    "this package needs the arm description `litearm_description`, whose URDF declares "
    "joint1..joint7 and ee_link (the generation litearm_ros2_control's hardware "
    "block, litearm_moveit_config's SRDF and this package's configs are written "
    "against), and none was found in the AMENT_PREFIX_PATH of this build. Build or "
    "source the workspace that provides it before building this one. Note that copying "
    "a share/ directory into an install tree does not register a package; it has to be "
    "built. Candidate descriptions found: ${_rejected}")
endif()
message(STATUS "litearm_vr_moveit_config: arm description package = ${ARM_DESCRIPTION_PACKAGE} (${ARM_DESCRIPTION_PREFIX})")

# One line, read by the launch files with $(command 'cat ...'). A recorded file
# rather than a launch argument default because the answer belongs to the build
# environment: whoever built the package already had the description on their
# prefix path, and the launch file runs later, possibly from a cleaner shell.
#
# No trailing newline, deliberately. The consumer is a launch substitution, and a
# substitution is not a shell: it does not strip whitespace, so a file ending in a
# newline hands the launch a package name with a newline in it and the launch
# refuses it as "not a valid package name".
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/arm_description_package.txt"
     "${ARM_DESCRIPTION_PACKAGE}")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/arm_description_package.txt"
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf srdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
