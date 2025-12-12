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

