cmake_minimum_required(VERSION 3.8)
project(dronetrack_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# New ground-station boundary messages only.
# Detection/DetectionArray/TargetError and the Pi-internal types are reused
# unchanged from drone_interfaces (copied in by scripts/setup_*.sh) so that the
# laptop and an existing Pi remain wire-compatible on the same message types.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GroundStationHeartbeat.msg"
  "msg/LinkStatus.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
