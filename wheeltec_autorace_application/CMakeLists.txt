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

find_package(cv_bridge REQUIRED)

find_package(
  OpenCV REQUIRED
  COMPONENTS
    core
    imgproc
)


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


install(
  TARGETS
    lane_perception

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


ament_package()