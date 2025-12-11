# ROS Weaver

A visual tool for assembling, extending, and iterating on ROS2 packages and nodes.

## Quickstart

```bash
# Install dependencies (Ubuntu)
sudo apt install -y qtbase5-dev qtbase5-dev-tools libqt5widgets5 libyaml-cpp-dev

# Source ROS2 and build
source /opt/ros/jazzy/setup.bash  # or your ROS2 distro
colcon build --packages-select ros_weaver
source install/setup.bash

# Run
ros2 run ros_weaver ros_weaver
```

## Overview

ROS Weaver provides a graphical interface for ROS2 development, allowing users to:
- Visually compose ROS2 packages and nodes using a drag-and-drop canvas
- Wire nodes together to define topic connections, services, and actions
- Generate code and configuration files automatically
- Preview and iterate on ROS2 systems without manual code editing

Inspired by visual programming tools like Unreal Engine's Blueprint and Houdini's node-based workflow.

## Requirements

- ROS2 (Humble or later recommended)
- Qt5 (Widgets, Gui, Core)
- CMake 3.8+
- C++17 compatible compiler
- yaml-cpp

### Installing Dependencies (Ubuntu)

```bash
sudo apt update
sudo apt install -y \
  qtbase5-dev \
  libqt5widgets5 \
  libyaml-cpp-dev \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-rcl-interfaces
```

## Building

```bash
# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Navigate to workspace root (parent of src)
cd /path/to/ROS2Weaver

# Build with colcon
colcon build --packages-select ros_weaver

# Source the workspace
source install/setup.bash
```

## Running

```bash
ros2 run ros_weaver ros_weaver
```

## Usage

### Canvas Navigation
- **Middle Mouse Button**: Pan the canvas
- **Ctrl + Scroll**: Zoom in/out
- **Right Click on Canvas**: Add new nodes (Publisher, Subscriber, Service, etc.)
- **Right Click on Block**: Edit or delete package blocks
- **Delete/Backspace**: Remove selected items
- **Double Click on Block**: Expand/collapse node details

### Creating Connections
- Drag from an output pin to an input pin to create a topic connection
- Connections are automatically styled based on data type (Topic, Service, Action)

### Package Browser
- Drag packages from the left panel onto the canvas
- Browse workspace packages, system packages, and templates

## Project Structure

```
src/ros_weaver/
├── CMakeLists.txt
├── package.xml
├── include/ros_weaver/
│   ├── main_window.hpp
│   └── canvas/
│       ├── weaver_canvas.hpp
│       ├── package_block.hpp
│       └── connection_line.hpp
└── src/
    ├── main.cpp
    ├── main_window.cpp
    └── canvas/
        ├── weaver_canvas.cpp
        ├── package_block.cpp
        └── connection_line.cpp
```

## License

Apache-2.0
