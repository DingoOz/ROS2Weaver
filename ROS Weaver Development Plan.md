# ROS Weaver Development Plan

## Project Overview

ROS Weaver is a visual tool for assembling, extending, and iterating on ROS2 packages and nodes. It aims to streamline the current text-heavy workflow by providing a graphical interface for package discovery, node composition, parameter tuning, and simulation integration. Inspired by tools like Unreal Engine's node editor and Houdini's procedural workflows, it will allow users to drag-and-drop packages, wire nodes, and generate code/configs automatically.

Given that the source code analysis of BranchForge indicates it is a ROS2 Behavior Tree management tool built with Qt for its GUI (a common choice for ROS2 visual tools like RViz and rqt, involving C++ for core components and PyQt for Python bindings), ROS Weaver will adopt the same Qt-based technology stack. This ensures compatibility with ROS2 ecosystems, leverages existing libraries for graphical canvases, and supports cross-platform development.

If BranchForge uses additional elements (e.g., specific Qt modules or integrations), they can be mirrored, but based on typical setups for similar tools, Qt provides the foundation for drag-and-drop interfaces, custom widgets, and real-time visualizations.

**GUI Note**: Qt will be used as the primary GUI framework to align with ROS2 tools like RViz2 and rqt, maintaining a similar visual style (e.g., consistent widgets, themes, and integration patterns for embedded views like RViz panels). However, the development will be kept modular—e.g., abstracting GUI components via interfaces or factories—to allow swapping out Qt for ImGui (via Dear ImGui) during testing or prototyping phases. This modularity will facilitate quick comparisons or lightweight alternatives without major refactoring.

## Tech Stack

- **Core Language**: C++ for performance-critical parts (e.g., ROS2 node integrations, graph rendering), with Python for scripting and rapid prototyping where possible.
- **GUI Framework**: Qt (via Qt Widgets or Qt Quick/QML for modern UIs). Use QGraphicsView/Scene for the canvas-based node editor, similar to how Behavior Tree editors like Groot handle hierarchical graphs.
- **ROS2 Integrations**:
  - rclcpp (C++) and rclpy (Python) for node communication.
  - colcon for building workspaces.
  - rosdep for dependency resolution.
  - rviz2 and gazebo_ros for embedded simulation previews.
- **Dependencies** (mirroring BranchForge's likely setup):
  - BehaviorTree.CPP (if extending BT concepts, but generalized for ROS nodes).
  - XML/YAML parsers (e.g., tinyxml2 or yaml-cpp) for config generation.
  - Graph libraries: Qt's built-in graphics or external like Graphviz for layout algorithms.
  - Build System: CMake with CMakeLists.txt for compilation, package.xml for ROS2 metadata.
- **Other Libraries**:
  - Boost for utilities.
  - Python packages: PyQt5/6 for GUI in Python scripts, pyyaml for param handling.
- **Environment**: Cross-platform (Linux primary for ROS2, with Windows/Mac support via Qt).

## Architecture

ROS Weaver will be structured as a modular ROS2 package itself, installable via colcon. Key components:

1. **Main Application**:
   - Qt-based executable launching the GUI.
   - Central window with tabs/panels: Canvas for package assembly, Node Editor for drilling into packages, Param Dashboard, and Simulator View.
2. **Visual Canvas Module** (QGraphicsScene-based):
   - Represents packages as draggable QGraphicsItems.
   - Connections as QGraphicsLines with custom logic for topic remapping.
   - Event handlers for drag-drop, right-click extensions (auto-generate new package scaffolds).
3. **Node Editor Module**:
   - Hierarchical graph view for nodes within packages.
   - Custom Qt widgets for ROS primitives (e.g., Publisher/Subscriber pins).
   - Code blending: Embed QTextEdit for snippets, with auto-generation of full node code.
4. **Config and Build Backend**:
   - Non-Qt C++ core for generating YAML, CMakeLists.txt, package.xml.
   - Integration with colcon for incremental builds.
   - Live validation engine to check topic types, dependencies.
5. **Simulation and Debug Integration**:
   - Embed RViz widgets via rviz_default_plugins.
   - Launch ros2 processes in background threads (using QProcess).
6. **Plugin System**:
   - Qt plugins for extensibility (e.g., add new node types or package templates).

The architecture emphasizes non-destructive workflows: Changes to graphs trigger partial regenerations, not full rebuilds, for fast iteration.

## Development Phases

### Phase 1: Setup and Prototyping (2-4 weeks)

- Fork or reference BranchForge's structure: Copy CMake setup, Qt project files (.pro if using qmake, or CMake integration).
- Create base ROS2 package: ros2 pkg create --build-type ament_cmake ros_weaver.
- Integrate Qt: Add find_package(Qt5 COMPONENTS Widgets Gui) in CMakeLists.txt.
- Build MVP GUI: Simple QMainWindow with QGraphicsView canvas.
- Test basic drag-drop of mock package blocks.

### Phase 2: Core Features Implementation (4-6 weeks)

- Implement Package Assembly: Search/integration with ROS Index API (use QtNetwork for HTTP).
- Node Editor: Custom QGraphicsItems for nodes, with pin connections.
- Param Dashboard: QTreeView or custom forms linked to YAML.
- Code Generation: Templates in C++ to output files.
- Node Grouping: Unreal Engine-style comment boxes for organizing nodes (see Feature: Node Grouping below).

### Phase 3: Iteration and Integration (4 weeks)

- Add hot-reload: Monitor file changes, relaunch ros2 nodes via QProcess.
- Embed Simulator: Integrate rviz_rendering for previews.
- AI Suggestions: Optional Python module using networkx for graph analysis.

### Phase 4: Testing and Polish (2-4 weeks)

- Unit tests with gtest/QtTest.
- User testing with ROS2 examples (e.g., TurtleBot).
- Documentation: README with setup, similar to BranchForge.
- Release: Open-source on GitHub, package for ROS distribution.

## Risks and Mitigations

- **Qt Learning Curve**: If team unfamiliar, start with PyQt prototypes.
- **Performance for Large Graphs**: Use Qt's scene optimizations; profile with Qt Creator.
- **ROS2 Version Compatibility**: Target Humble/Iron; use conditionals in CMake.
- **Dependencies Conflicts**: Mirror BranchForge's resolutions.

## Feature: Node Grouping

### Overview

Node Grouping provides Unreal Engine-style "Comment Box" functionality for organizing and managing nodes in the visual canvas. Groups are semi-transparent colored rectangles that can contain multiple nodes and move them together as a unit.

### Features

1. **Visual Grouping**: Groups render as semi-transparent colored rectangles behind nodes (z-order: -100). Each group has:
   - Customizable title displayed in a header bar
   - Configurable colors (Blue, Green, Red, Orange, Purple presets, or custom)
   - Rounded corners with consistent visual styling

2. **Group Creation**: Groups can be created by:
   - Selecting one or more nodes with rubber-band selection (or clicking a single node), then right-click → "Create Group from Selection"
   - A dialog prompts for the group title
   - Single-node groups are supported with larger default padding

3. **Group Movement**: When a group is dragged:
   - All contained nodes move together with the group
   - Connections between nodes are automatically updated

4. **Dynamic Node Membership**: Nodes can be added or removed from groups by dragging:
   - Drag a node into a group's bounds to add it to the group
   - Drag a node out of a group's bounds to remove it from the group
   - Group membership is automatically detected when a node drag operation completes

5. **Resizable**: Groups have 8 resize handles (corners and edges):
   - Hover shows appropriate resize cursor
   - Minimum size enforced (150x100 pixels)

6. **Title Editing**: Double-click the group header to edit the title via input dialog

7. **Color Options**: Right-click menu provides:
   - "Group Color" submenu with preset colors
   - "Fit to Contents" to auto-resize around contained nodes

8. **Deletion**: Groups can be deleted via:
   - Select + Delete/Backspace key
   - Right-click → "Delete Group"
   - Deleting a group does NOT delete contained nodes

### Technical Implementation

- **NodeGroup class** (`canvas/node_group.hpp/cpp`): QGraphicsObject subclass with:
  - Title, color, size properties
  - List of contained PackageBlock pointers
  - Resize handle hit-testing and cursor management
  - Movement propagation via itemChange() override

- **Project Serialization**: NodeGroupData struct stores:
  - ID, title, position, size, color
  - List of contained node IDs for reconstruction on load

- **WeaverCanvas Integration**:
  - `createNodeGroup()` and `createGroupFromSelection()` methods
  - `removeNodeGroup()` for deletion
  - Export/import via Project class

### Usage

1. Add several nodes to the canvas
2. Select the nodes you want to group (rubber-band selection, or just click one node)
3. Right-click on a selected node → "Create Group from Selection"
4. Enter a title (e.g., "Sensor Processing", "Motor Control")
5. The group box appears around your nodes
6. Drag the group header to move all nodes together
7. Use resize handles to adjust group size as needed
8. Drag additional nodes into the group to add them
9. Drag nodes out of the group bounds to remove them from the group

