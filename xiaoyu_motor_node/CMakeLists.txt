cmake_minimum_required(VERSION 3.8)
project(xiaoyu_motor_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Findament_cmake and rosidl_default_generators
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/COData.msg"
  "srv/COWriteID.srv"
  "srv/COWrite.srv"
  "srv/COTargetDouble.srv"
  "srv/COReadID.srv"
  "srv/CORead.srv"
  "srv/CONode.srv"
  "srv/CONmtID.srv"
  "srv/COHeartbeatID.srv"
  "srv/FirmwareUpdate.srv"
)

# Python package specific setup
find_package(ament_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Install marker file for ament_index_python
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
