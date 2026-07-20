cmake_minimum_required(VERSION 3.8)
project(autonav_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(include)

add_executable(autonav_mode src/main.cpp)
ament_target_dependencies(autonav_mode rclcpp geometry_msgs)
target_link_libraries(autonav_mode px4_ros2_cpp::px4_ros2_cpp)

install(TARGETS autonav_mode DESTINATION lib/${PROJECT_NAME})

ament_package()
