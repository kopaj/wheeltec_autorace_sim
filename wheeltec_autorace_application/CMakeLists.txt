cmake_minimum_required(VERSION 3.8)

project(wheeltec_autorace_application)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(
  OpenCV REQUIRED
  COMPONENTS
    core
    imgproc
)


# ============================================================
# Lane perception
# ============================================================

add_executable(
  lane_perception
  src/lane_perception.cpp
)


target_include_directories(
  lane_perception
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(
  lane_perception

  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)


target_link_libraries(
  lane_perception

  ${OpenCV_LIBS}
)


# ============================================================
# PID lane controller
# ============================================================

add_executable(
  pid_lane_controller
  src/pid_lane_controller.cpp
)


target_include_directories(
  pid_lane_controller
  PRIVATE
    include
)


ament_target_dependencies(
  pid_lane_controller

  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)


# ============================================================
# Pure Pursuit controller
# ============================================================


add_executable(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(
  pure_pursuit_controller

  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)

target_include_directories(pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# ============================================================
# Installation
# ============================================================

install(
  TARGETS
    lane_perception
    pid_lane_controller
    pure_pursuit_controller

  DESTINATION
    lib/${PROJECT_NAME}
)


install(
  DIRECTORY
    include/

  DESTINATION
    include
)


install(
  DIRECTORY
    config

  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch

  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()