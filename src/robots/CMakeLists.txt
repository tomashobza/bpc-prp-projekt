cmake_minimum_required(VERSION 3.8)
project(robots)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(test)

# Add include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Create executable
add_executable(my_program
  src/main.cpp
)

# Link dependencies
ament_target_dependencies(my_program
  rclcpp
  std_msgs
)

# Install targets
install(TARGETS
  my_program
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize ament package
ament_package()