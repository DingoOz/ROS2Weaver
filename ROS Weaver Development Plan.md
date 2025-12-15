# ROS Weaver Development Plan

This document contains planned features and future development items for ROS Weaver. For implemented features and current architecture, see [Architecture.md](Architecture.md).

## Development Phases

### Phase 3: Iteration and Integration (In Progress)

- Add hot-reload: Monitor file changes, relaunch ros2 nodes via QProcess
- Embed Simulator: Integrate rviz_rendering for previews
- AI Suggestions: Optional Python module using networkx for graph analysis

### Phase 4: Testing and Polish (Planned)

- Unit tests with gtest/QtTest
- User testing with ROS2 examples (e.g., TurtleBot)
- Documentation: README with setup, similar to BranchForge
- Release: Open-source on GitHub, package for ROS distribution

## Risks and Mitigations

- **Qt Learning Curve**: If team unfamiliar, start with PyQt prototypes.
- **Performance for Large Graphs**: Use Qt's scene optimizations; profile with Qt Creator.
- **ROS2 Version Compatibility**: Target Humble/Iron; use conditionals in CMake.
- **Dependencies Conflicts**: Mirror BranchForge's resolutions.

## Feature: GitHub Actions CI/CD Enhancements (Planned)

### Overview

Expand the basic CI pipeline to include comprehensive testing and release automation.

### Planned Enhancements

1. **Multi-Distribution Matrix Build**:
   - Build on Ubuntu 22.04 with ROS2 Humble
   - Build on Ubuntu 24.04 with ROS2 Jazzy
   - Cache colcon workspace and dependencies for faster builds

2. **Automated UI/Integration Testing**:
   - **Qt Test Framework**: Use QTest for widget and signal/slot testing
   - **GUI Automation**: Headless testing with `xvfb` (X Virtual Framebuffer)
   - **Test Categories**:
     - Unit Tests: Individual class/function testing
     - Widget Tests: UI component behavior
     - Integration Tests: End-to-end workflows
     - ROS2 Integration Tests: Mock ROS2 environment

3. **Static Analysis**:
   - `cppcheck` for C++ analysis
   - `clang-tidy` for additional checks

### CD Pipeline (Continuous Deployment)

1. **Release Workflow**: Triggered on version tags (e.g., v1.0.0)
   - Build release binaries for supported platforms
   - Generate Debian packages (.deb)
   - Create GitHub Release with changelog
   - Upload packages as release assets

2. **Documentation Deployment** (optional)
   - Build and deploy documentation to GitHub Pages
   - Auto-generate API docs with Doxygen

## Feature: Live Topic Inspection Enhancements (Planned)

### Overview

Enhance the existing Topic Viewer with visual data flow animation and detailed hover information.

### Active Data Flow Animation

Connections (edges) between nodes animate when data is flowing:
- Animated dashed line or "marching ants" effect for active topics
- Animation speed correlates with message frequency
- Static/solid line for inactive or idle connections
- Optional: Color intensity based on bandwidth usage

### Connection Details on Hover

Hovering over an animated connection shows:
- Topic name
- Message type
- Current publish rate
- Publisher and subscriber node names
- Message count (since monitoring started)

### Activity Indicators

Visual badges or indicators on nodes showing:
- Number of active publishers
- Number of active subscribers
- Overall activity state (active/idle)

### Technical Implementation

- **Animation System**:
  - QPropertyAnimation or custom QGraphicsItem paint logic for edge animations
  - Timer-based updates (e.g., 10-30 Hz refresh) for smooth animation
  - QPen dash patterns with offset animation for "marching ants" effect

## Feature: RViz2 Integration (Planned)

### Overview

Integrate RViz2 visualization capabilities into ROS Weaver, either as an embedded panel within the application or as an external process that can be launched with pre-configured settings.

### Integration Options

#### Option 1: Embedded RViz Panel

Embed RViz2 as a Qt widget directly within ROS Weaver.

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

Launch RViz2 as a separate process with managed configuration.

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

Generate .rviz config based on project contents.

**Auto-Detection:**
- sensor_msgs/LaserScan → LaserScan display
- sensor_msgs/Image → Image display
- nav_msgs/OccupancyGrid → Map display
- nav_msgs/Path → Path display
- geometry_msgs/PoseStamped → Pose display
- sensor_msgs/PointCloud2 → PointCloud2 display
- visualization_msgs/MarkerArray → MarkerArray display

#### 3. Preset Configurations

**Built-in Presets:**
- Navigation: Map, Path, Pose, LaserScan, Robot
- SLAM: Map, LaserScan, TF, Robot
- Manipulation: Robot, PointCloud, Markers, TF
- Sensor Debug: All sensor topics visible

## Example Projects (Planned)

### Example 2: TurtleBot3 Simulation with Teleop (Intermediate)

A more comprehensive example demonstrating TurtleBot3 simulation with multiple input methods:

**Components:**
- TurtleBot3 Gazebo simulation (`turtlebot3_gazebo`)
- Keyboard teleop (`teleop_twist_keyboard`)
- Xbox/Game controller teleop (`joy` + `teleop_twist_joy`)
- Robot state publisher and TF tree
- LaserScan visualization

**Features Demonstrated:**
- Multiple input sources publishing to same topic (`/cmd_vel`)
- Sensor data flow (LaserScan, Odometry, IMU)
- TF frame hierarchy visualization
- Integration with Gazebo simulation
- RViz2 visualization configuration

**Location:** `examples/turtlebot3_teleop/`

### Example 3: Navigation Stack (Advanced)

Full Nav2 navigation stack example with SLAM and autonomous navigation:
- TurtleBot3 with Nav2
- SLAM Toolbox for mapping
- Path planning and obstacle avoidance
- Goal setting interface

**Location:** `examples/turtlebot3_navigation/`

### Future Examples

- **Multi-Robot Coordination**: Multiple TurtleBot3 robots with namespacing
- **Manipulation**: Robot arm with MoveIt2 integration
- **Custom Nodes**: Example showing code generation and custom node development
- **Sensor Fusion**: Combining multiple sensor inputs with message filters

## Feature: ROS2 Controller Lifecycle Status Panel (Planned)

### Overview

Add a dedicated panel for monitoring and managing ros2_control controller lifecycle states. This provides visibility into hardware interfaces and controller status for robot systems using ros2_control.

### Features

1. **Controller Discovery**:
   - Auto-detect running controller_manager nodes
   - List all loaded controllers with their types
   - Show controller chain dependencies
   - Support multiple controller managers (namespaced)

2. **Lifecycle State Display**:
   - Visual state indicators for each controller:
     - Unconfigured (red)
     - Inactive (yellow)
     - Active (green)
     - Finalized (blue)
   - State transition history/timeline
   - Time since last state change

3. **Controller Information**:
   - Controller name and type
   - Claimed interfaces (command/state)
   - Update rate
   - Associated hardware interfaces
   - Parameter values

4. **Lifecycle Management**:
   - Configure/Cleanup buttons
   - Activate/Deactivate buttons
   - Switch controllers (activate one, deactivate another atomically)
   - Emergency stop all controllers

5. **Hardware Interface Status**:
   - List all hardware interfaces
   - Show claimed/available state interfaces
   - Show claimed/available command interfaces
   - Hardware component lifecycle states

### Service Calls

- `controller_manager/list_controllers` - Get controller list and states
- `controller_manager/list_hardware_interfaces` - Get interface info
- `controller_manager/switch_controller` - Activate/deactivate controllers
- `controller_manager/load_controller` - Load new controller
- `controller_manager/unload_controller` - Unload controller
- Lifecycle services for state transitions

### Dependencies

- `controller_manager_msgs` - Service/message definitions
- `lifecycle_msgs` - Lifecycle state messages
- `hardware_interface` - Interface type definitions

## Feature: Remote AI Integration (Planned)

Allow users to integrate their own cloud AI accounts for enhanced capabilities:
- Claude (Anthropic) API integration
- OpenAI GPT integration
- Gemini (Google) integration
- Grok (xAI) integration

### Features

- Secure API key storage using system keychain
- Provider selection in settings
- Model selection per provider
- Usage tracking and cost estimation
- Fallback to local Ollama when offline

## Feature: Local AI Enhancements (Planned)

Improvements to the existing Ollama-based Local AI integration:

- Allow user to paste images to Local AI (multimodal support)
- Add token generation speed indicator in status bar (shown during generation)
- Set up dropdown with predetermined questions/prompts
- Link available ROS2 data to AI context automatically
- Make user and response chats fill only 80% of the width
- Add buffer at bottom to prevent half-rendered streaming tokens

## Feature: System Mapping Enhancements (Planned)

Improvements to the System Mapping panel:

- Double-click on "Extra Nodes in System" to add the node to the canvas automatically

## TF Tree Viewer Enhancements (Planned)

### Known Issues / Fixes Needed

- **View Mode Switching (Bug)**: The Tree View / Flat List combo box doesn't properly switch between views
  - **Current State**: Combo box exists but view doesn't change when selection changes
  - **Fix Required**: Implement `onViewModeChanged()` slot to rebuild tree in selected mode

### Future Enhancements

- **3D TF Tree Visualization**: Interactive 3D rendering of the transform tree
  - **OpenGL/Qt3D Rendering**: Embedded 3D viewport showing frame axes in 3D space
  - **Frame Axes Display**: RGB axes (X=Red, Y=Green, Z=Blue) at each frame origin
  - **Connection Lines**: Lines connecting parent-child frames showing hierarchy
  - **Interactive Camera**: Orbit, pan, zoom controls for 3D navigation
  - **Frame Labels**: Hoverable/clickable frame names in 3D view
  - **Animation**: Smooth interpolation showing transform changes over time
  - **Grid/Origin**: Reference grid plane and world origin marker
  - **Selective Rendering**: Show/hide specific branches or frame types
  - **Sync with Tree View**: Selecting frame in 3D highlights it in tree panel and vice versa

- URDF import: Load URDF and compare expected vs actual TF tree
- Transform recording: Record transforms to bag for replay
- Visual transform editor: Adjust transforms visually and publish corrections
- Multi-robot TF: Handle namespaced TF trees for multi-robot setups
- AR overlay: Show TF frames overlaid on camera images
- Performance profiler: Identify TF-related bottlenecks

## Feature: Unit Testing Framework (Planned)

Using gtest and a suite of defined tests to perform unit testing where it is most valuable for the app.

### Test Categories

- Core functionality tests (project save/load, code generation)
- Canvas operations (block creation, connections, grouping)
- Widget behavior tests
- ROS2 integration tests with mock environment

## Feature: Plot Panel Enhancements (Planned)

Future improvements to the Plot Panel:

- Bar Plot: For histogram-style data
- Multi-Axis: Secondary Y-axis for different scales
- Historical playback with slider
- Threshold lines: Horizontal reference lines for limits
- Anomaly highlighting: Color regions outside thresholds
- Measurement tools: Delta between two points
- Export to rosbag2
- FFT/frequency analysis view
- Correlation between multiple signals
- Triggered recording (start recording when threshold crossed)
- 2D plots (X-Y scatter of two topics)
- Expression evaluation (plot computed values like `topic1 - topic2`)

## Feature: Help Documentation Enhancements (Planned)

Future improvements to the in-program help system:

- Interactive Tutorials: Step-by-step guided walkthroughs with UI highlights
- "Tip of the Day" on startup (optional)
- Video tutorial links
- Offline documentation download
- Multiple language support (i18n)
- Integration with ROS2 documentation (ros.org links)

## Feature: Undo Function (Planned)

Add undo/redo support for canvas operations.

## Feature: Ollama Tool Usage Update (Planned)

Update Ollama integration to support native tool/function calling capabilities.

## Deployment

- Write a blog post announcing the release
- Package for ROS distribution
- Create installation documentation
