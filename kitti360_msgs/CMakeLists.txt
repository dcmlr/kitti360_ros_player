cmake_minimum_required(VERSION 3.8)
project(kitti360_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Kitti360BoundingBox.msg"
  "msg/Kitti360Confidence.msg"
  "msg/Kitti360InstanceID.msg"
  "msg/Kitti360SemanticID.msg"
  "msg/Kitti360SemanticRGB.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
  # DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rosidl_default_runtime)
ament_package()