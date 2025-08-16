cmake_minimum_required(VERSION 3.5)
project(rov_manual_cpp)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(rov_manual
    src/main.cpp
)

ament_target_dependencies(rov_manual
    rclcpp
    px4_ros2_cpp
    px4_msgs
)

install(TARGETS
    rov_manual
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
