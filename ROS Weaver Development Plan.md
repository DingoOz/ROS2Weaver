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

## Feature: ROS2 System Status Display

### Overview

Provide discrete, always-visible status indicators showing the current ROS2 system state and ROS_DOMAIN_ID. This gives users constant awareness of their ROS2 environment without cluttering the interface, helping prevent common issues like connecting to the wrong ROS_DOMAIN_ID or operating while ROS2 is not properly initialized.

### Features

1. **Status Bar Indicators**: Compact status display in the main window status bar:
   - ROS2 connection status (Connected/Disconnected/Error)
   - Current ROS_DOMAIN_ID value
   - Visual indicator (colored dot or icon) for at-a-glance status

2. **Title Bar Integration** (Optional):
   - Display ROS_DOMAIN_ID in window title: "ROS Weaver - Project Name [Domain: 0]"
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

5. **User Settings**:
   - Toggle to show/hide ROS2 status display (default: ON)
   - Toggle to show/hide ROS_DOMAIN_ID display (default: ON)
   - Option to show in title bar vs status bar vs both
   - Accessible via Settings/Preferences dialog

### Technical Implementation

- **Status Detection**:
  - Check `rclcpp::ok()` for ROS2 context status
  - Read `ROS_DOMAIN_ID` environment variable
  - Periodic health check (every 1-2 seconds)

- **UI Components**:
  - Custom `RosStatusWidget` for status bar
  - QLabel or custom widget with icon + text
  - Signal-slot connection for status updates

- **Settings Storage**:
  - Store preferences in QSettings
  - Keys: `display/show_ros_status`, `display/show_domain_id`, `display/status_location`

### UI/UX

**Status Bar Layout**:
```
┌────────────────────────────────────────────────────────────────────────┐
│ Ready | ● ROS2: Connected | Domain ID: 0 | Nodes: 5/7 matched         │
└────────────────────────────────────────────────────────────────────────┘
```

**Title Bar Format**:
```
ROS Weaver - MyProject.weaver [ROS2 ● | Domain: 42]
```

**Settings Panel**:
```
┌─ ROS2 Status Display ────────────────────────────────┐
│                                                       │
│  ☑ Show ROS2 connection status                       │
│  ☑ Show ROS_DOMAIN_ID                                │
│                                                       │
│  Display location:                                    │
│    ○ Status bar only (default)                       │
│    ○ Title bar only                                  │
│    ○ Both status bar and title bar                   │
│                                                       │
│  ☑ Highlight non-default Domain ID                   │
│                                                       │
└───────────────────────────────────────────────────────┘
```

### Default Behavior

- Both status indicators are **ON by default**
- Display in status bar by default
- Non-default Domain IDs are highlighted with a different color

## Feature: ROS Logger Integration

### Overview

Integrate ROS2 logging capabilities into ROS Weaver to capture and display logs from running nodes, providing real-time feedback during development and debugging.

### Features

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

4. **Integration with Output Panel**: Logs can be directed to the existing Output dock or a dedicated Log dock

### Technical Implementation

- Subscribe to `/rosout` topic (rcl_interfaces/msg/Log)
- Use rclcpp for C++ log subscription
- QPlainTextEdit or QTableView for log display
- Circular buffer to limit memory usage for long-running sessions

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

## Feature: Live Topic Inspection and Data Flow Visualization

### Overview

Provide real-time visibility into ROS2 topic activity by allowing users to select a topic and see which nodes are actively publishing or subscribing to it. Additionally, visualize active data flow on connections with animated edges that indicate when messages are being transmitted.

### Features

1. **Topic Selection Panel**: A searchable list/tree of all topics in the running ROS2 system:
   - Filter by topic name or message type
   - Show topic type (e.g., `sensor_msgs/msg/LaserScan`)
   - Display current publish rate (Hz) when available
   - Group topics by namespace

2. **Node Highlighting on Topic Selection**: When a topic is selected:
   - Highlight all nodes that publish to the topic (e.g., green glow/border)
   - Highlight all nodes that subscribe to the topic (e.g., blue glow/border)
   - Dim or fade non-participating nodes for visual clarity
   - Show pub/sub icons or badges on highlighted nodes

3. **Active Data Flow Animation**: Connections (edges) between nodes animate when data is flowing:
   - Animated dashed line or "marching ants" effect for active topics
   - Animation speed correlates with message frequency
   - Static/solid line for inactive or idle connections
   - Optional: Color intensity based on bandwidth usage

4. **Connection Details on Hover**: Hovering over an animated connection shows:
   - Topic name
   - Message type
   - Current publish rate
   - Publisher and subscriber node names
   - Message count (since monitoring started)

5. **Activity Indicators**: Visual badges or indicators on nodes showing:
   - Number of active publishers
   - Number of active subscribers
   - Overall activity state (active/idle)

### Technical Implementation

- **Topic Discovery**: Use `ros2 topic list` via rclcpp or QProcess to enumerate topics
- **Activity Monitoring**: Subscribe to topics with a lightweight callback to detect activity, or use `ros2 topic hz` for rate estimation
- **Node-Topic Mapping**: Use `ros2 topic info <topic>` to get publisher/subscriber lists
- **Animation System**:
  - QPropertyAnimation or custom QGraphicsItem paint logic for edge animations
  - Timer-based updates (e.g., 10-30 Hz refresh) for smooth animation
  - QPen dash patterns with offset animation for "marching ants" effect
- **Performance Considerations**:
  - Limit monitoring to user-selected topics to reduce overhead
  - Use separate thread for ROS2 introspection to avoid UI blocking
  - Cache topic info with periodic refresh (e.g., every 1-2 seconds)

### UI/UX

1. **Topic Inspector Dock**: Dockable panel listing all topics with:
   - Search/filter bar at top
   - Tree view grouped by namespace
   - Click to select and highlight nodes
   - Double-click to focus canvas on connected nodes

2. **Toolbar Toggle**: Button to enable/disable live data flow animation globally (for performance)

3. **Context Menu Integration**: Right-click on a connection to:
   - "Monitor this topic"
   - "Show topic details"
   - "Echo topic messages" (opens output panel with live messages)

### Usage

1. Launch your ROS2 nodes (or connect to a running system)
2. Open the Topic Inspector panel
3. Select a topic (e.g., `/scan`) from the list
4. Observe: publisher nodes glow green, subscriber nodes glow blue
5. If data is flowing, the connection line animates with a moving dash pattern
6. Hover over the connection to see message rate and details
7. Use the toolbar toggle to disable animations if performance is impacted

## Feature: VS Code Integration

### Overview

Provide seamless integration with Visual Studio Code for editing generated code files, allowing users to quickly open source files directly from ROS Weaver's context menus and toolbars.

### Features

1. **Open in VS Code from Context Menu**: Right-click options on various elements:
   - Right-click on a PackageBlock → "Open Node Source in VS Code"
   - Right-click on a generated package → "Open Package in VS Code"
   - Right-click on a YAML file in param dashboard → "Open in VS Code"

2. **Menu Bar Integration**: File menu options:
   - "Open Generated Package in VS Code" (after code generation)
   - "Open Project Folder in VS Code"

3. **Toolbar Button**: Quick-access button to open the current project/workspace in VS Code

4. **Post-Generation Action**: After generating a ROS2 package, offer to open it in VS Code:
   - Dialog: "Package generated successfully. Open in VS Code?"
   - Or auto-open if user preference is set

5. **Line-Specific Navigation**: When possible, open files at specific lines:
   - Click on a parameter in the dashboard → Opens params.yaml at that parameter
   - Click on a node's publisher/subscriber → Opens node source at the relevant callback

### Technical Implementation

- **VS Code Command**: Use `code` CLI command via QProcess:
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

### UI/UX

1. **Context Menu Items**:
   - Icon: VS Code logo or generic "external editor" icon
   - Keyboard shortcut: Ctrl+Shift+E for "Edit in VS Code"

2. **Generation Complete Dialog**:
   ```
   [✓] Package generated successfully!

   Generated files:
   - CMakeLists.txt
   - package.xml
   - src/node_name_node.cpp
   - launch/package_launch.py
   - config/params.yaml

   [Open in VS Code]  [Open Folder]  [Close]
   ```

3. **Settings Panel**: Preferences → External Editor:
   - Editor command: [code] (default)
   - Auto-open after generation: [checkbox]
   - Reuse existing window: [checkbox]

## Feature: ROS2 Package Generation Wizard

### Overview

An optional step-by-step wizard that guides users through ROS2 package generation, allowing fine-grained control over what gets generated and how. This provides an alternative to the quick "Generate ROS2 Package" action for users who want more customization.

### Features

1. **Wizard Launch**: Accessible via:
   - Menu: File → "Generate ROS2 Package (Wizard)..."
   - Keyboard shortcut: Ctrl+Shift+G
   - Button in generation complete dialog: "Generate Another (Wizard)"

2. **Step 1 - Package Information**:
   - Package name (with validation: lowercase, alphanumeric, underscores)
   - Package version (default: 0.1.0)
   - Description (pre-filled from project metadata)
   - Maintainer name and email
   - License selection (dropdown: Apache-2.0, MIT, BSD-3-Clause, GPL-3.0, custom)
   - ROS distribution target (dropdown: humble, iron, jazzy, rolling)

3. **Step 2 - Output Configuration**:
   - Output directory selection with browse button
   - Subdirectory creation option
   - Overwrite behavior (ask, overwrite, skip existing)
   - Preview of output directory structure

4. **Step 3 - Node Selection**:
   - Checklist of all nodes in the project
   - Select/deselect individual nodes to include
   - "Select All" / "Deselect All" buttons
   - Preview of selected nodes with their publishers/subscribers

5. **Step 4 - Language & Style**:
   - Language selection: C++ or Python (per-node or global)
   - Code style options:
     - Namespace naming convention
     - Include/import organization
     - Comment style (minimal, standard, verbose)
   - ROS2 coding style compliance checkbox

6. **Step 5 - Generated Files**:
   - Checkboxes for optional files:
     - [ ] CMakeLists.txt (required for C++)
     - [ ] package.xml (required)
     - [ ] Launch file (optional)
     - [ ] Parameters YAML (optional)
     - [ ] README.md for generated package (optional)
     - [ ] Unit test stubs (optional)
   - Launch file format: Python (default) or XML

7. **Step 6 - Parameters Configuration**:
   - Review all parameters that will be generated
   - Edit default values before generation
   - Group parameters by node
   - Option to export only modified parameters vs all parameters

8. **Step 7 - Review & Generate**:
   - Summary of all selections
   - Preview of files to be generated (tree view)
   - Estimated file count and size
   - "Back" button to modify any step
   - "Generate" button to execute
   - Progress bar during generation

### Technical Implementation

- **Wizard Framework**: Use `QWizard` and `QWizardPage` classes for step-by-step UI
- **Validation**: Each page validates before allowing "Next"
- **State Management**: Store wizard state in a `GeneratorWizardOptions` struct
- **Preview Generation**: Generate file previews without writing to disk
- **Template System**: Consider moving to a template-based system for easier customization:
  - Templates stored in `share/ros_weaver/templates/`
  - User-customizable templates in `~/.config/ros_weaver/templates/`

### UI/UX

1. **Wizard Style**: Modern flat design matching ROS Weaver theme
2. **Progress Indicator**: Step indicator at top showing current progress (1/7, 2/7, etc.)
3. **Help Text**: Contextual help tooltips and descriptions for each option
4. **Presets**: Save/load wizard configurations as presets for repeated use
5. **Quick Mode**: Option to skip to summary if user accepts all defaults

### Usage

1. Open your project in ROS Weaver
2. Select File → "Generate ROS2 Package (Wizard)..."
3. Step through each page, customizing options as needed
4. Review the summary on the final page
5. Click "Generate" to create the package
6. Optionally open in VS Code or file browser

### Future Enhancements

- Template editor for custom code generation templates
- Integration with `ros2 pkg create` for additional scaffolding
- Support for generating multiple packages (e.g., interfaces package separate from nodes)
- Export wizard configuration for team sharing

## Feature: Real-Time Topic Viewer and Monitor

### Overview

A comprehensive dockable panel for monitoring, filtering, and searching ROS2 topics in real-time. This provides introspection capabilities similar to `ros2 topic` CLI tools but with a rich graphical interface optimized for rapid exploration and debugging.

### Design Philosophy

The topic viewer should follow these principles:

1. **Lazy by Default**: Don't subscribe to topics until explicitly requested - subscribing to all topics simultaneously would overwhelm the system
2. **Non-Blocking UI**: All ROS2 operations happen on background threads; the UI remains responsive
3. **Progressive Disclosure**: Show summary info first, drill down for details on demand
4. **Canvas Integration**: Seamlessly connect topic exploration with the visual node graph

### Core Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Topic Viewer Panel                          │
├─────────────────────────────────────────────────────────────────┤
│ [🔍 Search...        ] [Filter ▼] [⟳ Refresh] [▶ Auto] [⚙]     │
├─────────────────────────────────────────────────────────────────┤
│ Topic Name           │ Type              │ Hz    │ Pubs │ Subs  │
├─────────────────────────────────────────────────────────────────┤
│ ▶ /scan              │ sensor_msgs/...   │ 10.0  │  1   │  2    │
│ ▼ /odom              │ nav_msgs/Odome... │ 30.0  │  1   │  3    │
│   └─ [Message Preview Panel]                                    │
│      pose.position.x: 1.234                                     │
│      pose.position.y: 5.678                                     │
│      twist.linear.x: 0.22                                       │
│ ▶ /cmd_vel           │ geometry_msgs/... │ 10.0  │  1   │  1    │
│ ▶ /map               │ nav_msgs/Occup... │  0.2  │  1   │  2    │
├─────────────────────────────────────────────────────────────────┤
│ Status: 47 topics │ 12 active │ Monitoring: /odom              │
└─────────────────────────────────────────────────────────────────┘
```

### Features

#### 1. Topic Discovery and Listing

- **Auto-Discovery**: Periodically refresh topic list (configurable: 1-10 seconds, or manual only)
- **Hierarchical View**: Option to view topics as flat list or grouped by namespace tree
  ```
  / (root)
  ├── robot1/
  │   ├── scan
  │   ├── odom
  │   └── cmd_vel
  └── sensors/
      ├── camera/image_raw
      └── imu/data
  ```
- **Topic Metadata**: For each topic, show:
  - Full topic name
  - Message type (with tooltip showing full type path)
  - Current publish rate (Hz) - sampled, not subscribed
  - Publisher count
  - Subscriber count
  - QoS profile summary (reliability, durability)

#### 2. Filtering System

**Quick Filters (Toolbar Dropdown):**
- All Topics
- Active Only (Hz > 0)
- With Publishers
- With Subscribers
- Standard Messages (std_msgs/*)
- Sensor Messages (sensor_msgs/*)
- Navigation Messages (nav_msgs/*, geometry_msgs/*)
- Custom filter...

**Advanced Filter Dialog:**
```
┌─ Advanced Filter ─────────────────────────────────┐
│                                                   │
│ Topic Name:  [          ] ☑ Regex  ☑ Case-sens  │
│ Message Type:[          ] ☑ Regex               │
│                                                   │
│ Namespace:   [/robot1/*        ▼]                │
│                                                   │
│ Activity:    ○ Any  ● Active  ○ Idle            │
│                                                   │
│ Publishers:  Min [0  ] Max [999]                 │
│ Subscribers: Min [0  ] Max [999]                 │
│ Rate (Hz):   Min [0.0] Max [1000]                │
│                                                   │
│ [Save as Preset...] [Load Preset ▼]              │
│                                                   │
│              [Cancel]  [Apply]                   │
└───────────────────────────────────────────────────┘
```

**Filter Presets:**
- Save named filter configurations
- Quick-access to recent/favorite filters
- Share presets via export/import

#### 3. Search Capabilities

**Topic Search (Fast):**
- Instant search-as-you-type filtering on topic names
- Fuzzy matching option (e.g., "cmd" matches "/cmd_vel", "/robot/command")
- Search history with recent queries

**Message Content Search (On-Demand):**
- Search within message field values across monitored topics
- Requires explicit "Search in Messages" action (expensive operation)
- Results show topic + timestamp + matching field
- Regex support for complex patterns

**Type Search:**
- Find all topics of a specific message type
- Wildcard support: `sensor_msgs/*`, `*Image*`

#### 4. Real-Time Monitoring

**Subscription Modes:**
- **Off**: No subscription, only metadata shown
- **Sample**: Subscribe briefly to get one message, then unsubscribe
- **Monitor**: Continuous subscription with configurable throttle
- **Record**: Monitor + save to circular buffer

**Message Display:**
- **Tree View**: Expandable JSON-like structure for nested messages
  ```
  ▼ header
      seq: 12345
      stamp: 1704067200.123456
      frame_id: "base_link"
  ▼ pose
    ▼ position
        x: 1.234
        y: 5.678
        z: 0.0
    ▼ orientation
        x: 0.0
        y: 0.0
        z: 0.707
        w: 0.707
  ```
- **Raw View**: Pretty-printed text representation
- **Hex View**: For binary/encoded data inspection
- **Delta Highlighting**: Changed fields flash/highlight on update

**Throttling & Performance:**
- Configurable max update rate per topic (default: 30 Hz display)
- Automatic throttling for high-frequency topics
- Bandwidth limiting option
- Pause/Resume all monitoring

#### 5. Message History and Playback

- Circular buffer per monitored topic (configurable size: 10-10000 messages)
- Timeline scrubber to review past messages
- Pause live updates to examine specific message
- Export history to:
  - JSON file
  - CSV (for numeric data)
  - ROS2 bag file (single topic)

#### 6. Visualization Widgets

**Numeric Fields:**
- Inline sparkline for trending
- Click to open full plot window
- Multi-field overlay plotting

**Common Message Types:**
- `geometry_msgs/Pose`: 3D pose preview widget
- `sensor_msgs/Image`: Thumbnail preview (click to enlarge)
- `sensor_msgs/LaserScan`: Polar plot preview
- `nav_msgs/OccupancyGrid`: Minimap preview
- `sensor_msgs/PointCloud2`: Point count + bounds summary

#### 7. Canvas Integration

**Topic → Canvas:**
- Click topic → Highlight all nodes that publish/subscribe to it
- Double-click topic → Center canvas on connected nodes
- Drag topic to canvas → Create connection from available matching pin

**Canvas → Topics:**
- Select node on canvas → Filter topic list to that node's topics
- Right-click node → "Show Topics" opens filtered topic viewer
- Connection hover → Show topic stats tooltip

**Synchronized Selection:**
- Option to link selection: selecting topic selects corresponding connections on canvas
- Visual indicators on canvas edges showing monitored topics

### Technical Implementation

#### Threading Model

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Qt UI      │     │  ROS2 Thread │     │ Worker Pool  │
│   Thread     │◄───►│  (Executor)  │◄───►│ (Analysis)   │
└──────────────┘     └──────────────┘     └──────────────┘
       │                    │                    │
       │ Signals/Slots      │ Subscriptions      │ Heavy compute
       │ (thread-safe)      │ Callbacks          │ (hz calc, etc)
```

- **Qt UI Thread**: All widget updates, user interaction
- **ROS2 Executor Thread**: Dedicated `SingleThreadedExecutor` for subscriptions
- **Worker Thread Pool**: Rate calculation, message analysis, search operations

#### Key Classes

```cpp
class TopicViewerPanel : public QDockWidget {
  // Main UI container
};

class TopicListModel : public QAbstractItemModel {
  // Efficient model for topic list with lazy loading
  // Supports both flat and hierarchical views
};

class TopicMonitor : public QObject {
  // Manages subscriptions and message buffering
  // Runs callbacks on ROS2 thread, emits signals to UI
};

class MessageIntrospector {
  // Dynamic message parsing using rosidl_typesupport_introspection
  // Converts any message to tree structure for display
};

class TopicFilterEngine {
  // Compiled filter evaluation for performance
  // Supports complex boolean expressions
};
```

#### Message Introspection

Use `rosidl_typesupport_introspection_cpp` to dynamically parse any message type:

```cpp
// Get type support for any message
auto type_support = get_message_type_support(message_type);
auto members = get_members(type_support);

// Iterate fields dynamically
for (auto& member : members) {
  QString name = member.name_;
  QString type = get_type_name(member);
  QVariant value = extract_value(msg_data, member);
  // Build tree node...
}
```

#### Performance Optimizations

1. **Lazy Subscription**: Only subscribe when topic is expanded/monitored
2. **Message Sampling**: For rate calculation, sample 10 messages then extrapolate
3. **Virtual Scrolling**: `QTreeView` with lazy model loading for 1000+ topics
4. **Throttled Updates**: Batch UI updates at 30-60 Hz regardless of message rate
5. **Subscription Pooling**: Reuse subscription objects when toggling monitor on/off
6. **LRU Cache**: Cache parsed message structures for repeated message types

#### QoS Handling

- Auto-detect publisher QoS and match for reliable subscription
- Fallback chain: try RELIABLE → BEST_EFFORT
- Show QoS mismatch warnings in UI
- Allow manual QoS override for debugging

### UI/UX Details

#### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+F | Focus search box |
| Ctrl+R | Refresh topic list |
| Space | Toggle monitor on selected topic |
| Enter | Expand/collapse selected topic |
| Ctrl+E | Echo selected topic (open detail view) |
| Ctrl+C | Copy selected message to clipboard |
| Escape | Clear search / close dialogs |

#### Context Menu (Right-Click on Topic)

- Monitor Topic
- Echo to Output Panel
- Copy Topic Name
- Copy Message Type
- Show on Canvas
- Plot Numeric Fields...
- Export Messages...
- Topic Info (detailed dialog)

#### Status Bar

- Total topic count
- Active/monitored count
- Network bandwidth usage (if monitoring)
- Last refresh timestamp

### Configuration Options

```yaml
topic_viewer:
  auto_refresh: true
  refresh_interval_sec: 2.0
  default_throttle_hz: 30.0
  message_buffer_size: 100
  show_hidden_topics: false  # Topics starting with _
  default_view: flat  # or 'tree'
  highlight_active: true
  qos_auto_match: true
  max_concurrent_subscriptions: 10
```

### Future Enhancements

- Message diffing: Compare two messages side-by-side
- Latency analysis: Measure time between correlated topics
- Topic statistics dashboard: Aggregate view of system health
- Record to bag: Multi-topic recording directly from viewer
- Remote monitoring: Connect to topics on remote ROS2 systems via DDS discovery
- Message injection: Publish test messages from viewer for debugging
- Custom message renderers: Plugin system for specialized visualizations

## Feature: Live System Discovery and Canvas Mapping

### Overview

Scan the running ROS2 system to discover active topics and nodes, then map them to the visual representation on the canvas. This bridges the gap between the design-time canvas (what you've drawn) and the runtime system (what's actually running), providing immediate visual feedback on system state.

### Core Concept

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│  Canvas View    │ ◄─────► │  Mapping Engine │ ◄─────► │  ROS2 System    │
│  (Design-time)  │         │                 │         │  (Runtime)      │
│                 │         │  • Match names  │         │                 │
│  PackageBlocks  │         │  • Match topics │         │  ros2 topic list│
│  Connections    │         │  • Match types  │         │  ros2 node list │
└─────────────────┘         └─────────────────┘         └─────────────────┘
```

### Features

#### 1. System Scan Trigger

**Manual Scan:**
- Toolbar button: "Scan ROS2 System" (radar/refresh icon)
- Menu: Tools → "Scan Running System"
- Keyboard shortcut: Ctrl+Shift+R
- Right-click canvas → "Scan for Active Topics"

**Auto-Scan Options:**
- Scan on project open (optional)
- Periodic background scan (configurable: 5-60 seconds, or disabled)
- Scan on canvas focus (when switching back to ROS Weaver)

#### 2. Discovery Operations

**Topic Discovery:**
```cpp
// Equivalent to: ros2 topic list -t
// Returns: topic name, message type, publisher count, subscriber count
```

**Node Discovery:**
```cpp
// Equivalent to: ros2 node list
// Plus: ros2 node info <node> for each node
// Returns: node name, namespace, publishers, subscribers, services
```

**Full System Graph:**
```cpp
// Build complete pub/sub graph:
// - Which nodes publish to which topics
// - Which nodes subscribe to which topics
// - Message types for each topic
```

#### 3. Canvas Mapping Algorithm

**Matching Strategy (Priority Order):**

1. **Exact Node Name Match**
   - Canvas block name matches ROS2 node name exactly
   - E.g., Canvas "slam_toolbox" ↔ ROS2 node "/slam_toolbox"

2. **Fuzzy Node Name Match**
   - Ignore namespace prefix: "/robot1/slam_toolbox" matches "slam_toolbox"
   - Ignore suffixes: "slam_toolbox_node" matches "slam_toolbox"
   - Case-insensitive comparison

3. **Topic-Based Inference**
   - If canvas block publishes topic X and ROS2 node publishes topic X → likely match
   - Weight by number of matching topics
   - Consider message types for stronger matching

4. **Package Name Match**
   - Canvas block name matches ROS2 package name
   - Useful when node names differ from package names

**Mapping Confidence Levels:**
- **High** (green): Exact name match + matching topics
- **Medium** (yellow): Fuzzy name match or topic-only match
- **Low** (orange): Partial topic overlap, uncertain
- **None** (gray): No match found in running system

#### 4. Visual Feedback on Canvas

**Node Status Indicators:**
```
┌─────────────────────────────┐
│ ● slam_toolbox         [▶] │  ← Green dot = running, [▶] = live indicator
├─────────────────────────────┤
│ ○ scan      ────────►       │  ← Filled pin = active topic
│ ○ odom      ────────►       │
│       ◄──────── map ●       │  ← Hollow pin = not seen in system
└─────────────────────────────┘
```

**Status Indicators:**
- **Node badge**: Green dot (running), Gray dot (not found), Yellow dot (partial match)
- **Pin indicators**: Filled circle (topic active), Hollow circle (topic not found)
- **Connection styling**: Solid line (both ends active), Dashed line (one/both ends inactive)
- **Glow effect**: Optional pulsing glow on nodes with active data flow

**Overlay Mode:**
- Toggle button to show/hide runtime status overlay
- When enabled, dims unmatched nodes and highlights matched ones
- Status legend in corner of canvas

#### 5. Mapping Results Panel

**Dockable Results View:**
```
┌─ System Mapping ──────────────────────────────────────┐
│ Last scan: 12:34:56 | 7/9 nodes matched | ⟳ Rescan   │
├───────────────────────────────────────────────────────┤
│ Canvas Block      │ ROS2 Node          │ Status      │
├───────────────────────────────────────────────────────┤
│ ● slam_toolbox    │ /slam_toolbox      │ ✓ Matched   │
│ ● nav2_controller │ /controller_server │ ~ Fuzzy     │
│ ● lidar_sensor    │ (not found)        │ ✗ Missing   │
├───────────────────────────────────────────────────────┤
│ Topics: 12/15 matched | 3 extra in system            │
└───────────────────────────────────────────────────────┘
```

**Details on Selection:**
- Click row to highlight corresponding canvas node
- Expand row to see topic-level matching details
- Show which topics matched, which are missing

#### 6. Unmatched Discovery

**Extra Nodes in System:**
- List ROS2 nodes not represented on canvas
- Option to "Add to Canvas" - creates new block with discovered topics

**Extra Topics in System:**
- List topics published/subscribed but not on canvas connections
- Option to "Add Connection" - creates pin and connection

**Missing from System:**
- List canvas elements not found in running system
- Helpful for debugging: "Why isn't my node running?"

#### 7. Namespace Handling

**Namespace Resolution:**
```
Canvas: "slam_toolbox"

Could match any of:
  /slam_toolbox
  /robot1/slam_toolbox
  /ns1/ns2/slam_toolbox

User preference:
  ○ Match any namespace (default)
  ○ Require exact match
  ○ Specify expected namespace: [/robot1/    ]
```

**Multi-Robot Support:**
- Detect namespace patterns (e.g., /robot1/*, /robot2/*)
- Option to filter by namespace prefix
- Canvas "instance" concept for multi-robot scenarios

### Technical Implementation

#### ROS2 Graph API

```cpp
class SystemDiscovery : public rclcpp::Node {
public:
  struct TopicInfo {
    std::string name;
    std::string type;
    std::vector<std::string> publishers;   // Node names
    std::vector<std::string> subscribers;  // Node names
  };

  struct NodeInfo {
    std::string name;
    std::string namespace_;
    std::vector<std::string> publishers;   // Topic names
    std::vector<std::string> subscribers;  // Topic names
    std::vector<std::string> services;
  };

  // Discovery methods
  std::vector<TopicInfo> discoverTopics();
  std::vector<NodeInfo> discoverNodes();
  SystemGraph buildGraph();  // Full pub/sub graph

private:
  // Use rclcpp::Node::get_topic_names_and_types()
  // Use rclcpp::Node::get_node_names()
  // Use rcl_get_publisher_names_and_types_by_node() etc.
};
```

#### Mapping Engine

```cpp
class CanvasMapper {
public:
  struct MappingResult {
    QUuid canvasBlockId;
    QString canvasName;
    QString ros2NodeName;      // Empty if not matched
    MatchConfidence confidence;
    QList<TopicMapping> topicMappings;
  };

  struct TopicMapping {
    QString canvasPinName;
    QString ros2TopicName;
    bool typeMatches;
    bool isActive;  // Has recent messages
  };

  QList<MappingResult> mapCanvasToSystem(
    const Project& project,
    const SystemGraph& systemGraph
  );

private:
  MatchConfidence calculateConfidence(
    const BlockData& block,
    const NodeInfo& node
  );
};
```

#### Update Flow

```
User clicks "Scan"
       │
       ▼
┌─────────────────────┐
│ SystemDiscovery     │ (ROS2 thread)
│ • get_topic_names() │
│ • get_node_names()  │
│ • build graph       │
└─────────┬───────────┘
          │ Signal: discoveryComplete(SystemGraph)
          ▼
┌─────────────────────┐
│ CanvasMapper        │ (Worker thread)
│ • Match blocks      │
│ • Score confidence  │
│ • Map topics        │
└─────────┬───────────┘
          │ Signal: mappingComplete(Results)
          ▼
┌─────────────────────┐
│ Canvas UI Update    │ (Qt thread)
│ • Update badges     │
│ • Style connections │
│ • Update panel      │
└─────────────────────┘
```

### UI/UX Details

#### Toolbar Integration

```
[New] [Open] [Save] | [Undo] [Redo] | [🔍 Scan System ▼] [📡 Live] | ...
                                           │
                                           ▼
                                    ┌──────────────┐
                                    │ Scan Now     │
                                    │ Auto-Scan: ✓ │
                                    │ ──────────── │
                                    │ Scan Options │
                                    │ Clear Status │
                                    └──────────────┘
```

#### Status Bar Integration

```
┌────────────────────────────────────────────────────────────────────┐
│ Ready | Nodes: 7/9 matched | Topics: 12/15 active | Last: 12:34:56│
└────────────────────────────────────────────────────────────────────┘
```

#### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+Shift+R | Scan ROS2 system |
| Ctrl+Shift+L | Toggle live mode (auto-scan) |
| Ctrl+Shift+M | Open mapping results panel |

### Configuration Options

```yaml
system_discovery:
  auto_scan_on_open: false
  auto_scan_interval_sec: 0  # 0 = disabled
  match_ignore_namespace: true
  match_case_sensitive: false
  match_fuzzy_threshold: 0.7  # For fuzzy string matching
  show_status_overlay: true
  highlight_active_connections: true
  dim_unmatched_nodes: false
```

### Integration with Other Features

**Topic Viewer Integration:**
- Scan results feed into topic viewer's topic list
- Clicking topic in viewer highlights mapped connections on canvas

**Data Flow Visualization:**
- Scan provides the "active" state for animated connections
- Only animate connections where both topics are discovered as active

**Code Generation:**
- Warn if generating code for nodes not found in system
- Option to only generate for matched/running nodes

## Feature: TF Tree Viewer and Frame Linker

### Overview

A dedicated panel for visualizing the ROS2 TF (Transform) tree hierarchy in real-time, with intelligent linking back to canvas packages, topics, and YAML parameters. This provides roboticists with immediate insight into coordinate frame relationships and helps debug transform-related issues by connecting the visual TF tree to the design elements in ROS Weaver.

### Why TF Visualization Matters

TF2 is fundamental to ROS2 robotics - it defines how coordinate frames relate to each other (e.g., how the robot's base relates to its sensors, how odometry relates to the map). Common pain points:
- "Why can't my planner find a transform to the goal?"
- "Which node is supposed to publish the odom→base_link transform?"
- "Is my URDF frame name matching what's in my YAML config?"

This feature addresses these by making the TF tree visible and connected to the rest of the project.

### Core Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           TF Tree Panel                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│ [🔍 Search frames...] [View: Tree ▼] [⟳ Refresh] [▶ Live] [⚙ Settings]     │
├───────────────────────────────────┬─────────────────────────────────────────┤
│                                   │                                         │
│         Tree/Graph View           │           Details Panel                 │
│                                   │                                         │
│    map                            │  Frame: base_link                       │
│    └── odom                       │  ─────────────────────────────────      │
│        └── base_footprint         │  Parent: base_footprint                 │
│            └── base_link ●        │  Type: Dynamic (10.2 Hz)                │
│                ├── laser_frame    │  Last update: 0.02s ago                 │
│                ├── camera_link    │  ─────────────────────────────────      │
│                │   └── camera_rgb │  Transform:                             │
│                └── imu_link       │    Translation: (0.0, 0.0, 0.1)         │
│                                   │    Rotation: (0, 0, 0, 1)               │
│    [orphan] tool_frame ⚠         │  ─────────────────────────────────      │
│                                   │  🔗 Links:                              │
│                                   │    📦 slam_toolbox (canvas)             │
│                                   │    📄 nav2_params.yaml:line 42          │
│                                   │    📡 /odom topic                        │
│                                   │                                         │
├───────────────────────────────────┴─────────────────────────────────────────┤
│ Frames: 12 | Static: 8 | Dynamic: 4 | ⚠ 1 orphan | Updated: 12:34:56       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### View Modes

#### 1. Tree View (Default)
Traditional hierarchical tree similar to a file explorer:
```
▼ map
  └─▼ odom
      └─▼ base_footprint
          └─▼ base_link
              ├── laser_frame
              ├─▼ camera_link
              │   ├── camera_rgb_frame
              │   ├── camera_depth_frame
              │   └── camera_ir_frame
              └── imu_link
```

**Advantages:** Familiar, compact, easy to navigate deep hierarchies
**Best for:** Understanding parent-child relationships, finding specific frames

#### 2. Graph View
Visual node-and-edge graph layout (similar to rqt_tf_tree):
```
        ┌─────┐
        │ map │
        └──┬──┘
           │
        ┌──▼──┐
        │odom │
        └──┬──┘
           │
    ┌──────▼───────┐
    │base_footprint│
    └──────┬───────┘
           │
      ┌────▼────┐
      │base_link│
      └─┬──┬──┬─┘
        │  │  │
   ┌────┘  │  └────┐
   ▼       ▼       ▼
┌─────┐ ┌─────┐ ┌─────┐
│laser│ │cam  │ │imu  │
└─────┘ └──┬──┘ └─────┘
           │
        ┌──▼──┐
        │depth│
        └─────┘
```

**Layout Options:**
- **Top-Down**: Root at top, children below (default)
- **Left-Right**: Root at left, better for wide/shallow trees
- **Radial**: Root at center, frames radiate outward
- **Force-Directed**: Auto-arranged, good for complex graphs

**Advantages:** Visual relationships, see whole tree at once
**Best for:** Understanding overall structure, presentations, debugging disconnections

#### 3. Table View
Flat list with sortable columns:
```
| Frame           | Parent          | Type    | Rate   | Publisher      | Status |
|-----------------|-----------------|---------|--------|----------------|--------|
| base_link       | base_footprint  | Dynamic | 10.2Hz | robot_state_pub| ✓ OK   |
| laser_frame     | base_link       | Static  | -      | robot_state_pub| ✓ OK   |
| camera_link     | base_link       | Static  | -      | urdf_publisher | ✓ OK   |
| tool_frame      | (none)          | Dynamic | 1.0Hz  | tool_driver    | ⚠ Orphan|
```

**Advantages:** Dense information, sortable, filterable
**Best for:** Finding specific issues, bulk analysis, export

### Visual Encoding

#### Frame Status Colors
```
🟢 Green  - Healthy: Recent updates, connected to tree
🟡 Yellow - Warning: Stale (no update in 1-5 seconds), or orphan with publisher
🔴 Red    - Error: Very stale (>5s), or critical orphan
🔵 Blue   - Static: Transform published once on /tf_static
⚪ Gray   - Unlinked: Frame exists but no canvas/YAML links found
🟣 Purple - Selected/Highlighted: Currently selected frame
```

#### Connection Lines (Graph View)
```
───────  Solid thick:   High-rate dynamic transform (>10 Hz)
─ ─ ─ ─  Dashed:        Low-rate dynamic transform (<1 Hz)
═══════  Double:        Static transform
· · · ·  Dotted:        Stale/problematic connection
```

#### Frame Icons
```
🔲 Standard frame
📍 Root frame (no parent)
🎯 Base link (robot body frame)
📷 Camera/sensor frame
🔗 Frame with canvas links
⚠️ Problem frame (orphan, stale)
📌 Pinned/bookmarked frame
```

### Link Detection and Display

#### 1. Frame → Canvas Block Links

**Detection Methods:**
- Parameter matching: Block has parameter containing frame name
  - E.g., `base_frame: "base_link"` in slam_toolbox block
- Pin frame_id: Connection pins with matching frame references
- Block name inference: Block named "laser" → likely publishes laser_frame

**Display:**
```
🔗 Canvas Links (2):
  📦 slam_toolbox
     └─ Parameter: base_frame = "base_link"
  📦 nav2_controller
     └─ Parameter: robot_base_frame = "base_link"
```

**Interactions:**
- Click link → highlight block on canvas
- Double-click → center canvas on block
- Right-click → "Show block parameters"

#### 2. Frame → Topic Links

**Detection Methods:**
- Topic message contains frame_id field matching this frame
- Topic is /tf or /tf_static and contains this transform
- Publisher node also publishes transforms for this frame

**Display:**
```
📡 Topic Links (3):
  /scan
     └─ Header.frame_id = "laser_frame" @ 10 Hz
  /odom
     └─ Header.frame_id = "odom" @ 30 Hz
     └─ child_frame_id = "base_footprint"
  /tf
     └─ Publishes: odom → base_footprint
```

**Interactions:**
- Click → open in Topic Viewer
- Right-click → "Echo topic", "Show publishers"

#### 3. Frame → YAML Parameter Links

**Detection Methods:**
- Scan loaded YAML files for string values matching frame name
- Common parameter patterns: `*_frame`, `*_frame_id`, `frame_id`
- Exact and fuzzy matching (e.g., "base_link" matches "base_link_frame")

**Display:**
```
📄 YAML Links (2):
  nav2_params.yaml
     └─ Line 42: global_frame: "map"
     └─ Line 43: robot_base_frame: "base_link"
  slam_params.yaml
     └─ Line 15: odom_frame: "odom"
     └─ Line 16: base_frame: "base_link"
```

**Interactions:**
- Click → open YAML file in param dashboard (scrolled to line)
- Right-click → "Open in VS Code at line"
- Hover → show surrounding context

#### 4. Frame → Node/Publisher Links

**Detection Methods:**
- Query /tf topic for publishers of this specific transform
- Use ros2 node info to find which nodes publish TF

**Display:**
```
🖥️ Publishers:
  /robot_state_publisher
     └─ Publishes: base_link → laser_frame (static)
     └─ Publishes: base_link → camera_link (static)
  /ekf_localization
     └─ Publishes: odom → base_footprint (dynamic, 50 Hz)
```

### Details Panel

When a frame is selected, show comprehensive details:

```
┌─ Frame Details ─────────────────────────────────────────┐
│                                                         │
│  Frame: base_link                                       │
│  ════════════════════════════════════════════════════   │
│                                                         │
│  Hierarchy                                              │
│  ──────────                                             │
│  Parent: base_footprint                                 │
│  Children: laser_frame, camera_link, imu_link          │
│  Depth: 4 (from root "map")                            │
│                                                         │
│  Transform (from parent)                                │
│  ────────────────────────                               │
│  Translation: x=0.0, y=0.0, z=0.05                     │
│  Rotation (quaternion): x=0, y=0, z=0, w=1             │
│  Rotation (RPY): roll=0°, pitch=0°, yaw=0°             │
│                                                         │
│  Status                                                 │
│  ──────                                                 │
│  Type: Dynamic                                          │
│  Update Rate: 50.2 Hz                                   │
│  Last Update: 0.019 seconds ago                        │
│  Publisher: /robot_state_publisher                      │
│  Status: ✓ Healthy                                      │
│                                                         │
│  Links                                  [Show All...]   │
│  ─────                                                  │
│  📦 slam_toolbox (base_frame)                          │
│  📦 nav2_controller (robot_base_frame)                 │
│  📄 nav2_params.yaml:42                                │
│  📡 /odom (child_frame_id)                             │
│                                                         │
│  Actions                                                │
│  ───────                                                │
│  [Lookup Transform...] [Echo TF] [Copy Frame Name]     │
│  [Show on Canvas] [Find in YAML] [Pin Frame]           │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Transform Lookup Tool

Inline tool for looking up transforms between any two frames:

```
┌─ Transform Lookup ──────────────────────────────────────┐
│                                                         │
│  From: [map          ▼]  To: [base_link      ▼]        │
│                                                         │
│  [Lookup]  [Swap]  [Copy]                              │
│                                                         │
│  Result:                                                │
│  ────────                                               │
│  Translation: x=1.234, y=5.678, z=0.0                  │
│  Rotation (quat): x=0, y=0, z=0.707, w=0.707           │
│  Rotation (RPY): roll=0°, pitch=0°, yaw=90°            │
│                                                         │
│  Path: map → odom → base_footprint → base_link         │
│  Total hops: 3                                          │
│                                                         │
│  ☑ Live update (currently: 10 Hz)                      │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Problem Detection and Alerts

#### Issue Types

```
┌─ TF Issues (3) ─────────────────────────────────────────┐
│                                                         │
│ ⚠️ ORPHAN FRAME                                         │
│    tool_frame has no parent connection                  │
│    Publisher: /tool_driver                              │
│    Suggestion: Add transform from base_link → tool_frame│
│    [Show Frame] [Ignore]                               │
│                                                         │
│ ⚠️ STALE TRANSFORM                                      │
│    camera_link → camera_depth hasn't updated in 5.2s   │
│    Expected rate: 30 Hz                                 │
│    Last publisher: /camera_driver (may have crashed)   │
│    [Show Frame] [Check Node]                           │
│                                                         │
│ ⚠️ FRAME MISMATCH                                       │
│    YAML config uses "base_footprint"                   │
│    But TF tree has "base_foot_print" (typo?)           │
│    Location: slam_params.yaml:16                       │
│    [Show in YAML] [Show in TF]                         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

#### Problem Indicators in Tree
```
map
└── odom
    └── base_footprint
        └── base_link
            ├── laser_frame
            ├── camera_link
            │   └── camera_depth ⚠️ [stale: 5.2s]
            └── imu_link
[orphan] tool_frame ⚠️
```

### Filtering and Search

#### Search Box
```
[🔍 Search: base                                    ]
     ↓
Matching frames:
  • base_footprint
  • base_link
  • base_scan

Search in:
  ☑ Frame names
  ☑ Parent names
  ☐ Publisher names
  ☐ Linked parameters
```

#### Filter Options
```
Filter ▼
┌────────────────────────┐
│ ☑ Show static frames   │
│ ☑ Show dynamic frames  │
│ ☐ Show orphans only    │
│ ☐ Show problems only   │
│ ───────────────────    │
│ ☑ Show linked frames   │
│ ☐ Show unlinked only   │
│ ───────────────────    │
│ Publisher: [Any     ▼] │
│ Min rate:  [0    ] Hz  │
└────────────────────────┘
```

### Context Menu Actions

Right-click on a frame:
```
┌────────────────────────────────┐
│ 📋 Copy Frame Name             │
│ 📋 Copy Full Path (map/odom/…) │
│ ─────────────────────────────  │
│ 🔍 Lookup Transform To...      │
│ 🔍 Lookup Transform From...    │
│ ─────────────────────────────  │
│ 📺 Echo Transform              │
│ 📊 Plot Transform Over Time    │
│ ─────────────────────────────  │
│ 📦 Show on Canvas              │
│ 📄 Find in YAML Files          │
│ 📡 Show Related Topics         │
│ ─────────────────────────────  │
│ 📌 Pin Frame                   │
│ 👁️ Hide Frame                  │
│ ─────────────────────────────  │
│ 🎯 Set as Reference Frame      │
│ ⚙️ Frame Properties...         │
└────────────────────────────────┘
```

### Canvas Integration

#### Bidirectional Linking

**TF Panel → Canvas:**
- Click frame → highlight all canvas blocks that reference this frame
- Double-click → center canvas on most relevant block
- Drag frame to canvas → add frame reference to selected block's parameters

**Canvas → TF Panel:**
- Select block on canvas → highlight all frames it references in TF tree
- Right-click block → "Show TF Frames" opens TF panel filtered to relevant frames
- Connection hover → show frame info if connection involves TF

#### Visual Sync Mode
```
☑ Sync selection between Canvas and TF Panel
```
When enabled:
- Selecting a block on canvas auto-selects its frames in TF panel
- Selecting a frame in TF panel auto-selects blocks that use it

### Timeline / History View

For debugging transform issues over time:

```
┌─ TF Timeline ───────────────────────────────────────────┐
│                                                         │
│ Frame: odom → base_footprint                           │
│                                                         │
│ Time ──────────────────────────────────────────►       │
│ 12:34:50    :51    :52    :53    :54    :55   NOW     │
│    │         │      │      │      │      │      │      │
│    ●─────────●──────●──────●──────●──────●──────●      │
│    50Hz     50Hz   50Hz   ⚠️2Hz  50Hz   50Hz   50Hz    │
│                            │                            │
│                     [Rate drop detected]               │
│                                                         │
│ [◀◀] [◀] [▶] [▶▶]  [⏸ Pause]  [📊 Plot]              │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+T | Open/focus TF Panel |
| Ctrl+F | Focus search box (when panel focused) |
| F5 | Refresh TF tree |
| Space | Toggle live updates |
| Enter | Show details for selected frame |
| Ctrl+L | Open transform lookup dialog |
| Ctrl+G | Toggle graph/tree view |
| Arrow keys | Navigate tree |
| Ctrl+C | Copy selected frame name |

### Technical Implementation

#### TF Listener Integration

```cpp
class TFTreeModel : public QAbstractItemModel {
public:
  // Listens to /tf and /tf_static
  void startListening();
  void stopListening();

  // Build tree from tf2_ros::Buffer
  void rebuildTree();

  // Query frame info
  FrameInfo getFrameInfo(const QString& frame);
  QStringList getFrameChildren(const QString& frame);
  QString getFrameParent(const QString& frame);

signals:
  void frameUpdated(const QString& frame);
  void treeStructureChanged();
  void issueDetected(const TFIssue& issue);

private:
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  QMap<QString, FrameInfo> frameCache_;
};
```

#### Link Discovery Engine

```cpp
class TFLinkDiscovery {
public:
  struct FrameLinks {
    QList<CanvasBlockLink> canvasLinks;
    QList<TopicLink> topicLinks;
    QList<YAMLLink> yamlLinks;
    QList<NodeLink> publisherLinks;
  };

  // Discover all links for a frame
  FrameLinks discoverLinks(
    const QString& frameName,
    const Project& project,
    const SystemGraph& systemGraph
  );

private:
  // Search strategies
  QList<CanvasBlockLink> findCanvasLinks(const QString& frame, const Project& project);
  QList<YAMLLink> findYAMLLinks(const QString& frame, const QList<YamlFileInfo>& yamlFiles);
  QList<TopicLink> findTopicLinks(const QString& frame, const SystemGraph& graph);
};
```

#### Update Strategy

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   TF Listener   │────►│   Frame Cache   │────►│    Qt Model     │
│   (ROS thread)  │     │  (shared data)  │     │   (UI thread)   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                      │                       │
         │ /tf callbacks        │ Periodic sync         │ dataChanged()
         │ ~100 Hz              │ 10-30 Hz              │ signals
         ▼                      ▼                       ▼
    Raw transforms      Aggregated state          UI updates
```

### Configuration Options

```yaml
tf_viewer:
  default_view: tree  # tree, graph, table
  live_update: true
  update_rate_hz: 10
  stale_threshold_sec: 1.0
  critical_stale_sec: 5.0
  show_static_frames: true
  show_orphan_frames: true
  highlight_problems: true
  sync_canvas_selection: true
  graph_layout: top_down  # top_down, left_right, radial, force
  show_transform_values: true
  rotation_format: rpy  # rpy, quaternion, axis_angle
```

### Future Enhancements

- URDF import: Load URDF and compare expected vs actual TF tree
- Transform recording: Record transforms to bag for replay
- Visual transform editor: Adjust transforms visually and publish corrections
- Multi-robot TF: Handle namespaced TF trees for multi-robot setups
- AR overlay: Show TF frames overlaid on camera images
- Performance profiler: Identify TF-related bottlenecks

## Feature: RViz2 Integration

### Overview

Integrate RViz2 visualization capabilities into ROS Weaver, either as an embedded panel within the application or as an external process that can be launched with pre-configured settings. This allows users to visualize their robot, sensor data, and navigation while designing node graphs.

### Integration Options

#### Option 1: Embedded RViz Panel

Embed RViz2 as a Qt widget directly within ROS Weaver:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ ROS Weaver                                                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                           │                                  │
│        Canvas (Node Graph)                │      RViz2 Panel                 │
│                                           │                                  │
│   ┌──────┐    ┌──────┐    ┌──────┐       │   ┌─────────────────────────┐   │
│   │sensor├────►│slam  ├────►│nav2 │       │   │                         │   │
│   └──────┘    └──────┘    └──────┘       │   │   3D Robot View         │   │
│                                           │   │   + LaserScan           │   │
│                                           │   │   + Map                 │   │
│                                           │   │   + Path                │   │
│                                           │   │                         │   │
│                                           │   └─────────────────────────┘   │
│                                           │                                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Pros:**
- Seamless integration, no window switching
- Direct interaction between canvas and visualization
- Shared ROS2 context

**Cons:**
- Complex implementation (requires rviz_common library)
- Potential performance impact
- Tighter coupling with RViz2 version

**Technical Approach:**
- Use `rviz_common::RenderPanel` and `rviz_common::VisualizationManager`
- Embed as QWidget in a dockable panel
- Share the same rclcpp::Node for subscriptions

#### Option 2: External RViz2 Process

Launch RViz2 as a separate process with managed configuration:

```
┌─────────────────────────────┐     ┌─────────────────────────────┐
│ ROS Weaver                   │     │ RViz2 (External)            │
│                              │     │                             │
│  [Launch RViz2]──────────────┼────►│  Loaded with config from    │
│                              │     │  ROS Weaver project         │
│  Canvas shows which topics   │     │                             │
│  are being visualized        │◄────┤  Updates config on changes  │
│                              │     │                             │
└─────────────────────────────┘     └─────────────────────────────┘
```

**Pros:**
- Simpler implementation
- RViz2 runs independently (can survive ROS Weaver crashes)
- Familiar RViz2 interface
- No version coupling

**Cons:**
- Separate window management
- Config synchronization complexity
- Less tight integration

**Technical Approach:**
- Use QProcess to launch `rviz2`
- Generate .rviz config files from project settings
- Use `-d <config.rviz>` flag to load configuration
- Monitor process state

### Features

#### 1. Quick Launch Buttons

**Toolbar/Menu:**
- "Launch RViz2" - Opens RViz2 with project-specific config
- "Launch RViz2 (Default)" - Opens with default config
- "Launch RViz2 (Empty)" - Opens with no displays

**Context Menu on Canvas:**
- Right-click topic connection → "Visualize in RViz2"
- Right-click node → "Show outputs in RViz2"

#### 2. Auto-Generated RViz Configuration

Generate .rviz config based on project contents:

```yaml
# Auto-generated by ROS Weaver
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: Robot
      Enabled: true
    - Class: rviz_default_plugins/LaserScan
      Name: Scan
      Topic: /scan
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Map
      Topic: /map
      Enabled: true
    - Class: rviz_default_plugins/Path
      Name: Path
      Topic: /path
      Enabled: true
```

**Auto-Detection:**
- Scan project topics for visualizable types
- sensor_msgs/LaserScan → LaserScan display
- sensor_msgs/Image → Image display
- nav_msgs/OccupancyGrid → Map display
- nav_msgs/Path → Path display
- geometry_msgs/PoseStamped → Pose display
- sensor_msgs/PointCloud2 → PointCloud2 display
- visualization_msgs/MarkerArray → MarkerArray display

#### 3. RViz Config Editor

Simple UI for configuring RViz displays without opening RViz:

```
┌─ RViz Configuration ────────────────────────────────────┐
│                                                         │
│ Display Configuration                                   │
│ ─────────────────────                                   │
│                                                         │
│ ☑ Robot Model                                          │
│   Description Topic: [/robot_description     ]         │
│                                                         │
│ ☑ LaserScan                                            │
│   Topic: [/scan                    ▼]                  │
│   Style: [Points ▼] Size: [0.05  ]                    │
│                                                         │
│ ☑ Map                                                  │
│   Topic: [/map                     ▼]                  │
│                                                         │
│ ☐ Camera Image                                          │
│   Topic: [/camera/image_raw        ▼]                  │
│                                                         │
│ [Add Display...] [Remove] [Move Up] [Move Down]        │
│                                                         │
│ ──────────────────────────────────────────────────────  │
│ Fixed Frame: [map          ▼]                          │
│ Background Color: [■ 48,48,48]                         │
│                                                         │
│              [Save Config] [Launch RViz2]              │
└─────────────────────────────────────────────────────────┘
```

#### 4. Canvas Integration

**Visual Indicators:**
- Topics being visualized in RViz2 get a special indicator on canvas
- Connections with visualizable types show "RViz" badge

**Sync Selection:**
- Select topic on canvas → Highlight in RViz2 (if supported)
- Select display in RViz config → Highlight source node on canvas

#### 5. Preset Configurations

**Built-in Presets:**
- Navigation: Map, Path, Pose, LaserScan, Robot
- SLAM: Map, LaserScan, TF, Robot
- Manipulation: Robot, PointCloud, Markers, TF
- Sensor Debug: All sensor topics visible
- Custom: User-defined presets

**Project Presets:**
- Save RViz configs per project
- Load/switch between saved configs

### Technical Implementation

#### External Process Management

```cpp
class RViz2Launcher : public QObject {
  Q_OBJECT

public:
  // Launch RViz2 with optional config
  bool launch(const QString& configPath = QString());

  // Generate config from project
  QString generateConfig(const Project& project);

  // Check if RViz2 is running
  bool isRunning() const;

  // Bring RViz2 to front
  void bringToFront();

  // Terminate RViz2
  void terminate();

signals:
  void rvizStarted();
  void rvizClosed(int exitCode);
  void rvizError(const QString& error);

private:
  QProcess* process_;
  QString configPath_;
};
```

#### RViz Config Generator

```cpp
class RVizConfigGenerator {
public:
  struct DisplayConfig {
    QString className;
    QString name;
    QString topic;
    QVariantMap properties;
  };

  // Generate full config
  QString generate(
    const QList<DisplayConfig>& displays,
    const QString& fixedFrame = "map"
  );

  // Auto-detect displays from project
  QList<DisplayConfig> autoDetectDisplays(const Project& project);

  // Get available display types for a message type
  QString getDisplayType(const QString& messageType);
};
```

#### Embedded RViz (Advanced)

```cpp
// Requires linking against rviz_common, rviz_rendering, etc.
class EmbeddedRVizPanel : public QWidget {
  Q_OBJECT

public:
  EmbeddedRVizPanel(QWidget* parent = nullptr);

  void initialize(rclcpp::Node::SharedPtr node);
  void loadConfig(const QString& configPath);
  void addDisplay(const QString& type, const QString& topic);
  void setFixedFrame(const QString& frame);

private:
  rviz_common::RenderPanel* renderPanel_;
  rviz_common::VisualizationManager* vizManager_;
  rviz_common::DisplayGroup* displayGroup_;
};
```

### UI/UX Details

#### Menu Structure

```
Tools
├── Launch RViz2
│   ├── With Project Config
│   ├── With Default Config
│   ├── Empty
│   └── From File...
├── RViz Configuration...
├── ────────────────────
└── Stop RViz2
```

#### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+Shift+V | Launch RViz2 with project config |
| Ctrl+Alt+V | Open RViz config editor |

#### Status Bar

```
RViz2: ● Running (PID 12345) | Config: project_viz.rviz
```

### Configuration Options

```yaml
rviz_integration:
  mode: external  # external or embedded (future)
  auto_launch: false  # Launch RViz2 when project opens
  auto_generate_config: true  # Generate config from project topics
  rviz_command: "rviz2"  # Command to launch RViz2
  default_fixed_frame: "map"
  default_background_color: [48, 48, 48]
  save_config_with_project: true
  config_directory: "~/.ros_weaver/rviz_configs"
```

### Future Enhancements

- Embedded RViz panel (requires significant development)
- Two-way config sync (changes in RViz2 reflect in ROS Weaver)
- Record/playback visualization sessions
- Screenshot/video capture integration
- Multi-RViz support (multiple viewports)
- Custom display plugins for ROS Weaver-specific visualizations

