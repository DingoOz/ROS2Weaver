# ROS Weaver Architecture

## Project Overview

ROS Weaver is a visual tool for assembling, extending, and iterating on ROS2 packages and nodes. It aims to streamline the current text-heavy workflow by providing a graphical interface for package discovery, node composition, parameter tuning, and simulation integration. Inspired by tools like Unreal Engine's node editor and Houdini's procedural workflows, it allows users to drag-and-drop packages, wire nodes, and generate code/configs automatically.

Given that the source code analysis of BranchForge indicates it is a ROS2 Behavior Tree management tool built with Qt for its GUI (a common choice for ROS2 visual tools like RViz and rqt, involving C++ for core components and PyQt for Python bindings), ROS Weaver adopts the same Qt-based technology stack. This ensures compatibility with ROS2 ecosystems, leverages existing libraries for graphical canvases, and supports cross-platform development.

**GUI Note**: Qt is used as the primary GUI framework to align with ROS2 tools like RViz2 and rqt, maintaining a similar visual style (e.g., consistent widgets, themes, and integration patterns for embedded views like RViz panels). The development is kept modular—e.g., abstracting GUI components via interfaces or factories—to allow swapping out Qt for ImGui (via Dear ImGui) during testing or prototyping phases.

## Tech Stack

- **Core Language**: C++ for performance-critical parts (e.g., ROS2 node integrations, graph rendering), with Python for scripting and rapid prototyping where possible.
- **GUI Framework**: Qt (via Qt Widgets or Qt Quick/QML for modern UIs). Uses QGraphicsView/Scene for the canvas-based node editor, similar to how Behavior Tree editors like Groot handle hierarchical graphs.
- **ROS2 Integrations**:
  - rclcpp (C++) and rclpy (Python) for node communication.
  - colcon for building workspaces.
  - rosdep for dependency resolution.
  - rviz2 and gazebo_ros for embedded simulation previews.
- **Dependencies**:
  - BehaviorTree.CPP (if extending BT concepts, but generalized for ROS nodes).
  - XML/YAML parsers (e.g., tinyxml2 or yaml-cpp) for config generation.
  - Graph libraries: Qt's built-in graphics or external like Graphviz for layout algorithms.
  - Build System: CMake with CMakeLists.txt for compilation, package.xml for ROS2 metadata.
- **Other Libraries**:
  - Boost for utilities.
  - Python packages: PyQt5/6 for GUI in Python scripts, pyyaml for param handling.
- **Environment**: Cross-platform (Linux primary for ROS2, with Windows/Mac support via Qt).

## Architecture

ROS Weaver is structured as a modular ROS2 package itself, installable via colcon. Key components:

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

---

## Implemented Features

### ROS2 System Status Display

Discrete, always-visible status indicators showing the current ROS2 system state and ROS_DOMAIN_ID. This gives users constant awareness of their ROS2 environment without cluttering the interface.

**Features:**
1. **Status Bar Indicators**: Compact status display in the main window status bar:
   - ROS2 connection status (Connected/Disconnected/Error)
   - Current ROS_DOMAIN_ID value
   - Visual indicator (colored dot or icon) for at-a-glance status

2. **Title Bar Integration** (Optional):
   - Display ROS_DOMAIN_ID in window title
   - Configurable: show in title bar, status bar, or both

3. **Status States**:
   - **Connected** (green): ROS2 context is active and healthy
   - **Disconnected** (gray): ROS2 not initialized or shutdown
   - **Error** (red): ROS2 initialization failed or daemon unreachable
   - **Warning** (yellow): Partial connectivity or unusual state

4. **ROS_DOMAIN_ID Display**:
   - Show current domain ID prominently
   - Highlight if non-default (ID ≠ 0) to draw attention
   - Tooltip showing environment variable source

**Technical Implementation:**
- Status Detection via `rclcpp::ok()` for ROS2 context status
- Read `ROS_DOMAIN_ID` environment variable
- Periodic health check (every 1-2 seconds)
- Custom `RosStatusWidget` for status bar
- Settings stored in QSettings

**Source Files:** `src/ros_weaver/src/widgets/ros_status_widget.cpp`

---

### ROS Logger Integration

ROS2 logging capabilities integrated into ROS Weaver to capture and display logs from running nodes, providing real-time feedback during development and debugging.

**Features:**
1. **Log Viewer Panel**: A dockable panel that displays ROS2 logs with:
   - Real-time log streaming from `/rosout` topic
   - Severity filtering (DEBUG, INFO, WARN, ERROR, FATAL)
   - Node-based filtering to focus on specific nodes
   - Search/filter by message content
   - Color-coded severity levels

2. **Log Levels**: Support all ROS2 log levels:
   - DEBUG (gray/dim)
   - INFO (default/white)
   - WARN (yellow)
   - ERROR (red)
   - FATAL (red/bold)

3. **Log Export**: Ability to export logs to file for debugging

4. **Integration with Output Panel**: Logs directed to the Output dock

**Technical Implementation:**
- Subscribe to `/rosout` topic (rcl_interfaces/msg/Log)
- Uses rclcpp for C++ log subscription
- QPlainTextEdit for log display
- Circular buffer to limit memory usage for long-running sessions

**Source Files:** `src/ros_weaver/src/widgets/output_panel.cpp`

---

### Node Grouping

Unreal Engine-style "Comment Box" functionality for organizing and managing nodes in the visual canvas. Groups are semi-transparent colored rectangles that can contain multiple nodes and move them together as a unit.

**Features:**
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

**Technical Implementation:**
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

**Source Files:** `src/ros_weaver/src/canvas/node_group.cpp`

---

### VS Code Integration

Seamless integration with Visual Studio Code for editing generated code files, allowing users to quickly open source files directly from ROS Weaver's context menus and toolbars.

**Features:**
1. **Open in VS Code from Context Menu**: Right-click options on various elements:
   - Right-click on a PackageBlock → "Open Node Source in VS Code"
   - Right-click on a generated package → "Open Package in VS Code"
   - Right-click on a YAML file in param dashboard → "Open in VS Code"

2. **Menu Bar Integration**: File menu options:
   - "Open Generated Package in VS Code" (after code generation)
   - "Open Project Folder in VS Code"

3. **Toolbar Button**: Quick-access button to open the current project/workspace in VS Code

4. **Post-Generation Action**: After generating a ROS2 package, offer to open it in VS Code

5. **Line-Specific Navigation**: When possible, open files at specific lines

**Technical Implementation:**
- **VS Code Command**: Uses `code` CLI command via QProcess:
  - `code <folder>` - Open folder in VS Code
  - `code <file>` - Open specific file
  - `code -g <file>:<line>` - Open file at specific line
  - `code -r <folder>` - Reuse existing window

- **Settings Support**:
  - User preference for default editor (VS Code, other editors)
  - Option to auto-open after generation
  - Configurable editor command (for non-standard installations)

- **Cross-Platform Considerations**:
  - Linux: `code` command (from PATH)
  - macOS: `code` or `/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code`
  - Windows: `code.cmd` or full path to VS Code

- **Error Handling**:
  - Check if `code` command is available
  - Show helpful message if VS Code is not installed
  - Fallback to system default editor via `QDesktopServices::openUrl()`

**Source Files:** `src/ros_weaver/src/core/external_editor.cpp`

---

### ROS2 Package Generation Wizard

A step-by-step wizard that guides users through ROS2 package generation, allowing fine-grained control over what gets generated and how.

**Features:**
1. **Wizard Launch**: Accessible via:
   - Menu: File → "Generate ROS2 Package (Wizard)..."
   - Keyboard shortcut: Ctrl+Shift+G

2. **Package Information Step**:
   - Package name (with validation: lowercase, alphanumeric, underscores)
   - Package version (default: 0.1.0)
   - Description (pre-filled from project metadata)
   - Maintainer name and email
   - License selection (dropdown: Apache-2.0, MIT, BSD-3-Clause, GPL-3.0, custom)
   - ROS distribution target (dropdown: humble, iron, jazzy, rolling)

3. **Output Configuration Step**:
   - Output directory selection with browse button
   - Subdirectory creation option
   - Overwrite behavior (ask, overwrite, skip existing)
   - Preview of output directory structure

4. **Node Selection Step**:
   - Checklist of all nodes in the project
   - Select/deselect individual nodes to include
   - Preview of selected nodes with their publishers/subscribers

5. **Generated Files Step**:
   - Checkboxes for optional files:
     - CMakeLists.txt (required for C++)
     - package.xml (required)
     - Launch file (optional)
     - Parameters YAML (optional)
     - README.md for generated package (optional)
     - Unit test stubs (optional)

6. **Review & Generate Step**:
   - Summary of all selections
   - Preview of files to be generated (tree view)
   - Progress bar during generation

**Technical Implementation:**
- **Wizard Framework**: Uses `QWizard` and `QWizardPage` classes for step-by-step UI
- **Validation**: Each page validates before allowing "Next"
- **State Management**: Stores wizard state in a `GeneratorWizardOptions` struct
- **Preview Generation**: Generates file previews without writing to disk

**Source Files:**
- `src/ros_weaver/src/wizards/package_wizard.cpp`
- `src/ros_weaver/src/core/code_generator.cpp`

---

### Real-Time Topic Viewer and Monitor

A comprehensive dockable panel for monitoring, filtering, and searching ROS2 topics in real-time. Provides introspection capabilities similar to `ros2 topic` CLI tools but with a rich graphical interface.

**Design Philosophy:**
1. **Lazy by Default**: Only subscribe when topic is expanded/monitored
2. **Non-Blocking UI**: All ROS2 operations happen on background threads
3. **Progressive Disclosure**: Show summary info first, drill down for details on demand
4. **Canvas Integration**: Seamlessly connect topic exploration with the visual node graph

**Features:**

#### Topic Discovery and Listing
- **Auto-Discovery**: Periodically refresh topic list (configurable: 1-10 seconds, or manual only)
- **Hierarchical View**: Option to view topics as flat list or grouped by namespace tree
- **Topic Metadata**: For each topic, show:
  - Full topic name
  - Message type (with tooltip showing full type path)
  - Current publish rate (Hz) - sampled, not subscribed
  - Publisher count
  - Subscriber count
  - QoS profile summary (reliability, durability)

#### Filtering System
**Quick Filters (Toolbar Dropdown):**
- All Topics
- Active Only (Hz > 0)
- With Publishers
- With Subscribers
- Standard Messages (std_msgs/*)
- Sensor Messages (sensor_msgs/*)
- Navigation Messages (nav_msgs/*, geometry_msgs/*)
- Custom filter...

#### Search Capabilities
- Instant search-as-you-type filtering on topic names
- Fuzzy matching option
- Search history with recent queries
- Type search with wildcard support

#### Real-Time Monitoring
**Subscription Modes:**
- **Off**: No subscription, only metadata shown
- **Sample**: Subscribe briefly to get one message, then unsubscribe
- **Monitor**: Continuous subscription with configurable throttle
- **Record**: Monitor + save to circular buffer

**Message Display:**
- **Tree View**: Expandable JSON-like structure for nested messages
- **Raw View**: Pretty-printed text representation
- **Delta Highlighting**: Changed fields flash/highlight on update

#### Canvas Integration
**Topic → Canvas:**
- Click topic → Highlight all nodes that publish/subscribe to it
- Double-click topic → Center canvas on connected nodes
- Drag topic to canvas → Create connection from available matching pin

**Canvas → Topics:**
- Select node on canvas → Filter topic list to that node's topics
- Right-click node → "Show Topics" opens filtered topic viewer
- Connection hover → Show topic stats tooltip

**Technical Implementation:**

#### Threading Model
- **Qt UI Thread**: All widget updates, user interaction
- **ROS2 Executor Thread**: Dedicated `SingleThreadedExecutor` for subscriptions
- **Worker Thread Pool**: Rate calculation, message analysis, search operations

#### Performance Optimizations
1. Lazy Subscription: Only subscribe when topic is expanded/monitored
2. Message Sampling: For rate calculation, sample 10 messages then extrapolate
3. Virtual Scrolling: QTreeView with lazy model loading for 1000+ topics
4. Throttled Updates: Batch UI updates at 30-60 Hz regardless of message rate
5. Subscription Pooling: Reuse subscription objects when toggling monitor on/off
6. LRU Cache: Cache parsed message structures for repeated message types

**Source Files:**
- `src/ros_weaver/src/widgets/topic_viewer_panel.cpp`
- `src/ros_weaver/src/widgets/topic_list_model.cpp`
- `src/ros_weaver/src/core/topic_monitor.cpp`
- `src/ros_weaver/src/widgets/topic_inspector.cpp`

---

### Live System Discovery and Canvas Mapping

Scan the running ROS2 system to discover active topics and nodes, then map them to the visual representation on the canvas. This bridges the gap between the design-time canvas (what you've drawn) and the runtime system (what's actually running).

**Features:**

#### System Scan Trigger
**Manual Scan:**
- Toolbar button: "Scan ROS2 System" (radar/refresh icon)
- Menu: Tools → "Scan Running System"
- Keyboard shortcut: Ctrl+Shift+R
- Right-click canvas → "Scan for Active Topics"

**Auto-Scan Options:**
- Scan on project open (optional)
- Periodic background scan (configurable: 5-60 seconds, or disabled)
- Scan on canvas focus (when switching back to ROS Weaver)

#### Discovery Operations
- Topic Discovery: Equivalent to `ros2 topic list -t`
- Node Discovery: Equivalent to `ros2 node list` plus `ros2 node info <node>`
- Full System Graph: Build complete pub/sub graph

#### Canvas Mapping Algorithm
**Matching Strategy (Priority Order):**
1. **Exact Node Name Match**: Canvas block name matches ROS2 node name exactly
2. **Fuzzy Node Name Match**: Ignore namespace prefix, suffixes, case-insensitive
3. **Topic-Based Inference**: If canvas block publishes topic X and ROS2 node publishes topic X → likely match
4. **Package Name Match**: Canvas block name matches ROS2 package name

**Mapping Confidence Levels:**
- **High** (green): Exact name match + matching topics
- **Medium** (yellow): Fuzzy name match or topic-only match
- **Low** (orange): Partial topic overlap, uncertain
- **None** (gray): No match found in running system

#### Visual Feedback on Canvas
**Status Indicators:**
- **Node badge**: Green dot (running), Gray dot (not found), Yellow dot (partial match)
- **Pin indicators**: Filled circle (topic active), Hollow circle (topic not found)
- **Connection styling**: Solid line (both ends active), Dashed line (one/both ends inactive)
- **Glow effect**: Optional pulsing glow on nodes with active data flow

#### Mapping Results Panel
- Dockable results view showing last scan timestamp and match counts
- Click row to highlight corresponding canvas node
- Expand row to see topic-level matching details

**Source Files:**
- `src/ros_weaver/src/core/system_discovery.cpp`
- `src/ros_weaver/src/core/canvas_mapper.cpp`
- `src/ros_weaver/src/widgets/system_mapping_panel.cpp`

---

### TF Tree Viewer and Frame Linker

A dedicated panel for visualizing the ROS2 TF (Transform) tree hierarchy in real-time, with intelligent linking back to canvas packages, topics, and YAML parameters.

**Why TF Visualization Matters:**
TF2 is fundamental to ROS2 robotics - it defines how coordinate frames relate to each other. Common pain points addressed:
- "Why can't my planner find a transform to the goal?"
- "Which node is supposed to publish the odom→base_link transform?"
- "Is my URDF frame name matching what's in my YAML config?"

**View Modes:**

#### Tree View (Default)
Traditional hierarchical tree similar to a file explorer, showing parent-child relationships.

#### Graph View
Visual node-and-edge graph layout (similar to rqt_tf_tree).

**Layout Options:**
- **Top-Down**: Root at top, children below (default)
- **Left-Right**: Root at left, better for wide/shallow trees
- **Radial**: Root at center, frames radiate outward
- **Force-Directed**: Auto-arranged, good for complex graphs

#### Table View
Flat list with sortable columns for frame, parent, type, rate, status.

**Visual Encoding:**

#### Frame Status Colors
- Green: Healthy - Recent updates, connected to tree
- Yellow: Warning - Stale (no update in 1-5 seconds), or orphan with publisher
- Red: Error - Very stale (>5s), or critical orphan
- Blue: Static - Transform published once on /tf_static
- Gray: Unlinked - Frame exists but no canvas/YAML links found
- Purple: Selected/Highlighted

**Link Detection and Display:**
| Link Type | Source | Detection Method |
|-----------|--------|------------------|
| Frame → Canvas Block | Parameter values | String matching in block parameters |
| Frame → Topic | Message headers | frame_id field inspection |
| Frame → Package | Package manifest | Frame names in package.xml, URDF |
| Frame → Node | ROS2 graph | Publisher/subscriber introspection |
| Frame → YAML Config | Config files | Parameter file scanning |
| Frame → URDF | Robot description | URDF link/joint parsing |

**Transform Lookup Tool:**
Inline tool for looking up transforms between any two frames with live update option.

**Problem Detection:**
- Orphan frames (no parent connection)
- Stale transforms (no recent updates)
- Frame mismatches (typos between config and TF tree)

**Canvas Integration:**
- Click frame → highlight all canvas blocks that reference this frame
- Double-click → center canvas on most relevant block
- Sync selection between Canvas and TF Panel

**Source Files:** `src/ros_weaver/src/widgets/tf_tree_panel.cpp`

---

### Param Dashboard

Widget for editing and managing ROS2 parameters with:
- QTreeView or custom forms linked to YAML
- Parameter value editing
- Export to YAML files

**Source Files:** `src/ros_weaver/src/widgets/param_dashboard.cpp`

---

### Robot Configuration Wizard

Multi-step wizard for creating robots with ros2_control support. Accessible via File > Robot Configuration Wizard (Ctrl+R).

**Implemented Robot Types** (based on ros2_control_demos):
- Differential Drive Robot (TurtleBot, DiffBot style)
- RRBot (2-DOF Revolute-Revolute Manipulator)
- 6-DOF Industrial Robot Arm
- Car-like Robot (Ackermann Steering)
- Tricycle Robot
- Parallel Gripper
- Custom Robot (user-defined)

**Wizard Steps**:
1. **Robot Type Selection**: Choose type with detailed descriptions, set robot name
2. **Robot Configuration**: Configure joints, wheels, mass, base frame
3. **Hardware Interfaces**: Set command/state interfaces (position/velocity/effort)
4. **Controllers Selection**: Select ros2_control controllers with descriptions
5. **Sensors Configuration**: Add LiDAR, camera, IMU, GPS with parameters
6. **Teleop Setup**: Configure keyboard, joystick, SpaceMouse teleoperation
7. **Visualization Setup**: Configure RViz2 and Gazebo world options
8. **Review & Generate**: Generate complete robot description package

**Generated Package Contents**:
- URDF/Xacro robot description with ros2_control tags
- Controller configuration YAML
- Launch files for display and Gazebo
- package.xml and CMakeLists.txt

**Source Files:** `src/ros_weaver/src/wizards/robot_wizard.cpp`

---

### Plot Panel

Real-time plotting of time-series data for tuning and spotting anomalies. Visualize sensor values, velocities, errors, battery levels, and any numeric ROS2 topic data over time.

**Features:**
1. **Topic Subscription**:
   - Subscribe to any numeric topic (Float32, Float64, Int32, etc.)
   - Extract fields from complex messages (e.g., `geometry_msgs/Twist.linear.x`)
   - Support for array indexing (e.g., `sensor_msgs/JointState.position[0]`)
   - Multiple topics on same plot with different colors
   - Auto-detection of plottable fields in message types

2. **Plot Types**:
   - Line Plot: Standard time-series line graph (default)
   - Scatter Plot: Discrete data points
   - Step Plot: For discrete state changes

3. **Time Window Options**:
   - Sliding window (last N seconds): 10s, 30s, 60s, 5min
   - Pause/resume to freeze display

4. **Visual Customization**:
   - Line colors and styles
   - Legend position and visibility
   - Dark/light theme support
   - Axis labels and titles

5. **Analysis Tools**:
   - Statistics Display: Min, max, mean, current value
   - Zoom & Pan: Mouse wheel zoom, drag to pan

6. **Data Export**:
   - Export to CSV
   - Screenshot plot as PNG

**Technical Implementation:**
- Uses QtCharts for rendering
- Ring buffer for efficient memory usage
- Background thread for data collection, UI thread for rendering
- Message introspection for field extraction

**Source Files:** `src/ros_weaver/src/widgets/plot_panel.cpp`

---

### Local AI Assistant (Ollama Integration)

Integrated AI assistant using local LLMs via Ollama to help users debug, understand, and optimize their robot systems. Fully offline-capable, free, and private.

**Features:**
1. **Chat Panel**:
   - Dedicated dockable panel for AI conversation
   - Markdown rendering for formatted responses
   - Code syntax highlighting in responses
   - Conversation history

2. **Ollama Integration**:
   - Auto-detect running Ollama instance
   - Model selection from available models
   - Hardware detection and model recommendation
   - Pull models from within the application

3. **Settings Panel** (tabbed interface):
   - Connection settings (endpoint, model selection)
   - System prompt configuration
   - Model management (pull/delete models)

4. **Status Indicator**:
   - Shows Ollama connection status in status bar
   - Model loading state indication

**Technical Implementation:**
- Uses Ollama REST API at `http://localhost:11434`
- Background thread for API calls
- Streaming response support
- QSettings for configuration persistence

**Source Files:**
- `src/ros_weaver/src/core/ollama_manager.cpp`
- `src/ros_weaver/src/widgets/llm_chat_widget.cpp`
- `src/ros_weaver/src/widgets/ollama_settings_widget.cpp`
- `src/ros_weaver/src/widgets/local_ai_status_widget.cpp`
- `src/ros_weaver/src/widgets/system_prompt_dialog.cpp`

---

### In-Program Help Documentation

Comprehensive in-program help documentation accessible via menus and F1 key.

**Features:**
1. **Help Menu Integration**:
   - Help > Getting Started
   - Help > Keyboard Shortcuts
   - Help > About ROS Weaver

2. **Context-Sensitive Help**:
   - F1 key shows help for currently focused widget/panel
   - Widget-specific help topics

3. **Help Browser Window**:
   - Searchable documentation viewer
   - Table of contents navigation
   - Markdown rendering

4. **Keyboard Shortcuts Dialog**:
   - Complete shortcut reference
   - Searchable list

**Source Files:**
- `src/ros_weaver/src/widgets/help_browser.cpp`
- `src/ros_weaver/src/core/context_help.cpp`
- `src/ros_weaver/src/widgets/keyboard_shortcuts_dialog.cpp`

---

### Guided Tour (Tutorial Mode)

Interactive tutorial that guides new users through ROS Weaver's features with pulsing visual highlights.

**Features:**
- Step-by-step walkthrough of main features
- Visual highlights on UI elements
- Progress tracking
- Skip/resume functionality

**Source Files:** `src/ros_weaver/src/widgets/guided_tour.cpp`

---

### Data Lineage

Right-click context menu feature that shows the origin of any displayed data.

**Features:**
- Shows where data originates from in source files
- Option to open files in VS Code for inspection/editing
- Complete data lineage visualization for generated or indirectly related data

**Source Files:**
- `src/ros_weaver/src/core/data_lineage.cpp`
- `src/ros_weaver/src/core/lineage_provider.cpp`
- `src/ros_weaver/src/widgets/lineage_dialog.cpp`

---

### Theme Manager

Application theming support with dark/light mode.

**Features:**
- Dark and light theme support
- Consistent styling across all widgets
- Theme persistence in settings

**Source Files:** `src/ros_weaver/src/core/theme_manager.cpp`

---

### GitHub Actions CI/CD

Basic continuous integration pipeline using GitHub Actions.

**Current Implementation:**
- Build workflow triggered on push/PR to main branch
- ROS2 Jazzy on Ubuntu latest
- Qt5 Charts dependency installation
- Build and test with `colcon test`

**Workflow File:** `.github/workflows/ci.yml`

---

### Example Projects

#### Turtlesim Teleop (Basic)

A simple introductory example using the ROS2 turtlesim package:
- `turtlesim_node` for the turtle simulation
- `turtle_teleop_key` for keyboard control
- `mimic` node demonstrating topic remapping
- Demonstrates basic node connections and topic flow

**Location:** `examples/turtlesim_teleop/`

---

## Source File Overview

### Core
- `src/ros_weaver/src/main.cpp` - Application entry point
- `src/ros_weaver/src/main_window.cpp` - Main application window
- `src/ros_weaver/src/core/project.cpp` - Project save/load management
- `src/ros_weaver/src/core/code_generator.cpp` - ROS2 package code generation
- `src/ros_weaver/src/core/ros_package_index.cpp` - ROS package discovery and indexing
- `src/ros_weaver/src/core/external_editor.cpp` - VS Code and external editor integration
- `src/ros_weaver/src/core/system_discovery.cpp` - Live ROS2 system introspection
- `src/ros_weaver/src/core/canvas_mapper.cpp` - Canvas to ROS2 system mapping
- `src/ros_weaver/src/core/topic_monitor.cpp` - Topic subscription and monitoring
- `src/ros_weaver/src/core/ollama_manager.cpp` - Ollama LLM API integration
- `src/ros_weaver/src/core/theme_manager.cpp` - Application theming
- `src/ros_weaver/src/core/data_lineage.cpp` - Data origin tracking
- `src/ros_weaver/src/core/lineage_provider.cpp` - Lineage information provider
- `src/ros_weaver/src/core/context_help.cpp` - Context-sensitive help system

### Canvas
- `src/ros_weaver/src/canvas/weaver_canvas.cpp` - Main QGraphicsView canvas
- `src/ros_weaver/src/canvas/package_block.cpp` - Draggable package block items
- `src/ros_weaver/src/canvas/connection_line.cpp` - Topic connection lines
- `src/ros_weaver/src/canvas/node_group.cpp` - Node grouping (comment boxes)

### Widgets
- `src/ros_weaver/src/widgets/ros_status_widget.cpp` - ROS2 connection status indicator
- `src/ros_weaver/src/widgets/output_panel.cpp` - Log output and ROS logger
- `src/ros_weaver/src/widgets/topic_viewer_panel.cpp` - Topic monitoring panel
- `src/ros_weaver/src/widgets/topic_inspector.cpp` - Topic details inspector
- `src/ros_weaver/src/widgets/topic_list_model.cpp` - Topic list data model
- `src/ros_weaver/src/widgets/topic_echo_dialog.cpp` - Topic echo modal dialog
- `src/ros_weaver/src/widgets/tf_tree_panel.cpp` - TF tree visualization
- `src/ros_weaver/src/widgets/system_mapping_panel.cpp` - System mapping results
- `src/ros_weaver/src/widgets/param_dashboard.cpp` - Parameter editing dashboard
- `src/ros_weaver/src/widgets/plot_panel.cpp` - Real-time data plotting
- `src/ros_weaver/src/widgets/llm_chat_widget.cpp` - AI chat interface
- `src/ros_weaver/src/widgets/ollama_settings_widget.cpp` - Ollama configuration panel
- `src/ros_weaver/src/widgets/local_ai_status_widget.cpp` - AI status indicator
- `src/ros_weaver/src/widgets/system_prompt_dialog.cpp` - System prompt editor
- `src/ros_weaver/src/widgets/help_browser.cpp` - Help documentation browser
- `src/ros_weaver/src/widgets/keyboard_shortcuts_dialog.cpp` - Shortcut reference dialog
- `src/ros_weaver/src/widgets/guided_tour.cpp` - Interactive tutorial
- `src/ros_weaver/src/widgets/lineage_dialog.cpp` - Data lineage display

### Wizards
- `src/ros_weaver/src/wizards/package_wizard.cpp` - Package generation wizard
- `src/ros_weaver/src/wizards/robot_wizard.cpp` - Robot configuration wizard
