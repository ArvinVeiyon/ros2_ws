cmake_minimum_required(VERSION 3.5)
project(rov_collision_stop_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)   # <--- add this

include_directories(include)

add_executable(rov_collision_stop
  src/main.cpp
)

ament_target_dependencies(rov_collision_stop
  rclcpp
  px4_msgs
  std_msgs                 # <--- add this
)

install(TARGETS
  rov_collision_stop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
