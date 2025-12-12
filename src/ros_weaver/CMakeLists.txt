cmake_minimum_required(VERSION 3.8)
project(ros_weaver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Qt5 setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  Network
)

# Include directories
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/canvas/weaver_canvas.cpp
  src/canvas/package_block.cpp
  src/canvas/connection_line.cpp
  src/canvas/node_group.cpp
  src/core/project.cpp
  src/core/ros_package_index.cpp
  src/core/code_generator.cpp
  src/core/external_editor.cpp
  src/widgets/param_dashboard.cpp
)

# Header files (for MOC)
set(HEADERS
  include/ros_weaver/main_window.hpp
  include/ros_weaver/canvas/weaver_canvas.hpp
  include/ros_weaver/canvas/package_block.hpp
  include/ros_weaver/canvas/connection_line.hpp
  include/ros_weaver/canvas/node_group.hpp
  include/ros_weaver/core/project.hpp
  include/ros_weaver/core/ros_package_index.hpp
  include/ros_weaver/core/code_generator.hpp
  include/ros_weaver/core/external_editor.hpp
  include/ros_weaver/widgets/param_dashboard.hpp
)

# Create executable
add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rcl_interfaces
  ament_index_cpp
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install example files
install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Add tests here as they are created
endif()

ament_package()
