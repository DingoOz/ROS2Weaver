# ROS Weaver

A visual tool for assembling, extending, and iterating on ROS2 packages and nodes.

![ROS Weaver Screenshot](Screenshot.png)

## Quickstart

```bash
# Install dependencies (Ubuntu)
sudo apt install -y qtbase5-dev qtbase5-dev-tools libqt5widgets5 libyaml-cpp-dev \
  ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-msgs

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
- Monitor live ROS2 systems with real-time topic visualization
- Generate code and configuration files automatically
- Debug TF trees and frame relationships
- Preview and iterate on ROS2 systems without manual code editing

Inspired by visual programming tools like Unreal Engine's Blueprint and Houdini's node-based workflow.

## Features

### Visual Canvas Editor
- **Drag-and-drop** package blocks onto the canvas
- **Connection wiring** between nodes with type-safe pins
- **Node grouping** with Unreal Engine-style comment boxes
- **Animated connections** showing live data flow rates
- **Zoom and pan** navigation with middle mouse button

### ROS2 System Status
- **Live status indicator** in title bar showing ROS2 connectivity
- **Node discovery** scanning running ROS2 nodes
- **Topic monitoring** with publish rates and message counts

### Real-Time Topic Viewer
- **Topic browser** with filtering and search
- **Live message monitoring** with rate display
- **Auto-monitor canvas topics** button for quick setup
- **Echo to output panel** for debugging

### TF Tree Viewer & Frame Linker
- **Live TF tree** visualization with parent-child hierarchy
- **Frame status** indicators (healthy, stale, static, orphan)
- **Transform details** showing translation and rotation (quaternion + RPY)
- **Frame linking** to canvas blocks, topics, and YAML parameters
- **Update rate tracking** for dynamic transforms

### Live System Discovery
- **System scan** to discover running nodes, topics, and services
- **Canvas mapping** to match discovered nodes with canvas blocks
- **Visual feedback** showing matched/unmatched nodes
- **Auto-refresh** option for continuous monitoring

### Code Generation
- **Package generation wizard** with customizable templates
- **Launch file generation** for composed systems
- **Parameter file export** in YAML format
- **VS Code integration** to open generated packages

### Node Grouping
- **Comment boxes** to organize related nodes
- **Group movement** moves all contained nodes together
- **Resizable groups** with drag handles
- **Color customization** with preset colors

### Example Projects
- **Turtlesim Teleop** - Basic publisher/subscriber example
- **TurtleBot3 Navigation** - SLAM and Nav2 with Gazebo simulation

## Requirements

- ROS2 (Humble, Iron, or Jazzy)
- Qt5 (Widgets, Gui, Core)
- CMake 3.8+
- C++17 compatible compiler
- yaml-cpp
- TF2 libraries

### Installing Dependencies (Ubuntu)

```bash
sudo apt update
sudo apt install -y \
  qtbase5-dev \
  libqt5widgets5 \
  libyaml-cpp-dev \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-rcl-interfaces \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-msgs \
  ros-${ROS_DISTRO}-geometry-msgs
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
| Action | Control |
|--------|---------|
| Pan canvas | Middle Mouse Button drag |
| Zoom | Ctrl + Scroll |
| Add nodes | Right Click on canvas |
| Edit/delete block | Right Click on block |
| Delete selected | Delete / Backspace |
| Expand/collapse | Double Click on block |

### Creating Connections
- Drag from an output pin to an input pin to create a topic connection
- Connections are automatically styled based on data type (Topic, Service, Action)
- Live connections animate based on message rate when monitoring is enabled

### Keyboard Shortcuts
| Shortcut | Action |
|----------|--------|
| Ctrl+N | New Project |
| Ctrl+O | Open Project |
| Ctrl+S | Save Project |
| Ctrl+Shift+S | Save As |
| Ctrl+G | Generate Code |
| Ctrl+T | Show TF Tree |
| Ctrl+Shift+T | Show Topic Viewer |
| Ctrl+M | Toggle Live Monitoring |
| F5 | Scan System |

### Package Browser
- Drag packages from the left panel onto the canvas
- Search for ROS2 packages by name
- Browse workspace packages, system packages, and templates

### Live Monitoring
1. Click the **Live** button in the Topic Viewer or press Ctrl+M
2. Click the **Canvas** button to auto-subscribe to topics matching your canvas
3. Connection lines will animate based on data flow rate

### TF Tree Viewer
1. Open via ROS2 > Show TF Tree (Ctrl+T)
2. Click the **Live** button to start listening to /tf and /tf_static
3. Select frames to see details and links to canvas blocks

## Project Structure

```
src/ros_weaver/
├── CMakeLists.txt
├── package.xml
├── examples/                    # Example projects
│   ├── turtlesim_teleop/
│   └── turtlebot3_navigation/
├── include/ros_weaver/
│   ├── main_window.hpp
│   ├── canvas/
│   │   ├── weaver_canvas.hpp
│   │   ├── package_block.hpp
│   │   ├── connection_line.hpp
│   │   └── node_group.hpp
│   ├── core/
│   │   ├── code_generator.hpp
│   │   ├── project.hpp
│   │   ├── system_discovery.hpp
│   │   └── topic_monitor.hpp
│   └── widgets/
│       ├── output_panel.hpp
│       ├── param_dashboard.hpp
│       ├── tf_tree_panel.hpp
│       └── topic_viewer_panel.hpp
└── src/
    ├── main.cpp
    ├── main_window.cpp
    ├── canvas/
    ├── core/
    └── widgets/
```

## License

Apache-2.0
