cmake_minimum_required(VERSION 3.5)
project(fun4)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES #################
find_package(ament_cmake REQUIRED)         # cpp (optional)
find_package(rclcpp REQUIRED)              # cpp (optional)
# find_package(ament_cmake_python REQUIRED)  # python (optional)
find_package(rclpy REQUIRED)               # python (optional)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#####################################################

# Install Python Module
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/jointstate_script.py
  scripts/randomizer.py
  scripts/pose_analyzer.py
  scripts/controller.py
  scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  config
  launch
  meshes
  robot
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # Added OPTIONAL to avoid errors if directories don't exist
)

# Generate ROSIDL interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModeSelector.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
