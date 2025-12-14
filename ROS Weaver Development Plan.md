# ROS Weaver Development Plan

This document contains planned features and future development items for ROS Weaver. For implemented features and current architecture, see [Architecture.md](Architecture.md).

## Development Phases

### Phase 1: Setup and Prototyping ✓ COMPLETED

- Fork or reference BranchForge's structure: Copy CMake setup, Qt project files
- Create base ROS2 package: ros2 pkg create --build-type ament_cmake ros_weaver
- Integrate Qt: Add find_package(Qt5 COMPONENTS Widgets Gui) in CMakeLists.txt
- Build MVP GUI: Simple QMainWindow with QGraphicsView canvas
- Test basic drag-drop of mock package blocks

### Phase 2: Core Features Implementation ✓ COMPLETED

- Implement Package Assembly: Search/integration with ROS Index API
- Node Editor: Custom QGraphicsItems for nodes, with pin connections
- Param Dashboard: QTreeView or custom forms linked to YAML
- Code Generation: Templates in C++ to output files
- Node Grouping: Unreal Engine-style comment boxes for organizing nodes

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

## Feature: GitHub Actions CI/CD (Planned)

### Overview

Implement continuous integration and deployment pipelines using GitHub Actions to automate building, testing, and releasing ROS Weaver.

### CI Pipeline (Continuous Integration)

1. **Build Workflow**: Triggered on push/PR to main branch
   - Build on Ubuntu 22.04 with ROS2 Humble
   - Build on Ubuntu 24.04 with ROS2 Jazzy
   - Cache colcon workspace and dependencies for faster builds
   - Upload build artifacts

2. **Test Workflow**: Run automated tests
   - Unit tests with `colcon test`
   - Code coverage reports (optional)
   - Linting with `ament_lint` tools

3. **Automated UI/Integration Testing**: Comprehensive user-facing tests
   - **Qt Test Framework**: Use QTest for widget and signal/slot testing
   - **GUI Automation**: Headless testing with `xvfb` (X Virtual Framebuffer)
   - **Test Categories**:
     - **Unit Tests**: Individual class/function testing (canvas operations, project serialization)
     - **Widget Tests**: UI component behavior (block creation, connection drawing, panel interactions)
     - **Integration Tests**: End-to-end workflows (load project → modify → save → verify)
     - **ROS2 Integration Tests**: Mock ROS2 environment for topic/TF testing
   - **Test Scenarios**:
     - Create/delete blocks and connections
     - Load/save project files
     - Node grouping operations
     - Topic viewer subscription and animation
     - Code generation output validation
     - TF tree discovery and display
   - **Regression Testing**: Automated screenshot comparison for visual regressions
   - **Performance Tests**: Canvas rendering with 100+ nodes, memory leak detection

4. **Static Analysis**: Code quality checks
   - `cppcheck` for C++ analysis
   - `clang-tidy` for additional checks
   - Qt-specific checks where applicable

### CD Pipeline (Continuous Deployment)

1. **Release Workflow**: Triggered on version tags (e.g., v1.0.0)
   - Build release binaries for supported platforms
   - Generate Debian packages (.deb)
   - Create GitHub Release with changelog
   - Upload packages as release assets

2. **Documentation Deployment** (optional)
   - Build and deploy documentation to GitHub Pages
   - Auto-generate API docs with Doxygen

### Workflow Files

```yaml
# .github/workflows/build.yml - Main CI workflow
# .github/workflows/release.yml - Release automation
# .github/workflows/docs.yml - Documentation (optional)
```

### Matrix Build Strategy

- **ROS2 Distributions**: Humble, Iron, Jazzy
- **Ubuntu Versions**: 22.04, 24.04
- **Build Types**: Debug, Release

### Caching Strategy

- Cache apt packages
- Cache colcon build directory
- Cache rosdep dependencies

## Feature: Live Topic Inspection and Data Flow Visualization (Planned Enhancements)

### Overview

The basic Topic Viewer is implemented. The following enhancements are planned:

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

#### 3. RViz Config Editor

Simple UI for configuring RViz displays without opening RViz.

#### 4. Canvas Integration

- Topics being visualized in RViz2 get a special indicator on canvas
- Connections with visualizable types show "RViz" badge

#### 5. Preset Configurations

**Built-in Presets:**
- Navigation: Map, Path, Pose, LaserScan, Robot
- SLAM: Map, LaserScan, TF, Robot
- Manipulation: Robot, PointCloud, Markers, TF
- Sensor Debug: All sensor topics visible

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

**Prerequisites:**
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_description`)
- Gazebo simulation environment
- Joy package for gamepad support
- Teleop packages (`teleop_twist_keyboard`, `teleop_twist_joy`)

**Learning Objectives:**
1. Connecting multiple publishers to a single subscriber
2. Understanding sensor topic data flow
3. Configuring gamepad/joystick input
4. Launching Gazebo simulation from ROS Weaver
5. Visualizing robot state with RViz2 integration

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
     - 🔴 Unconfigured
     - 🟡 Inactive
     - 🟢 Active
     - 🔵 Finalized
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

6. **Integration with Canvas**:
   - Link controllers to canvas blocks
   - Highlight blocks using specific controllers
   - Show controller status on block badges

### UI Layout

```
┌─ Controller Status ──────────────────────────────────────────────────────────┐
│ Controller Manager: /controller_manager                              [⟳ Refresh] │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│ Controllers                          │ Details                               │
│ ─────────────────────────────────   │ ──────────────────────────────────    │
│ 🟢 joint_state_broadcaster          │ Name: diff_drive_controller           │
│ 🟢 diff_drive_controller      ●     │ Type: diff_drive_controller/          │
│ 🟡 gripper_controller               │       DiffDriveController             │
│ 🔴 arm_controller                   │ State: Active                         │
│                                      │ Update Rate: 100 Hz                   │
│                                      │ ────────────────────────────────────  │
│                                      │ Command Interfaces:                   │
│                                      │   left_wheel/velocity                 │
│                                      │   right_wheel/velocity                │
│                                      │ State Interfaces:                     │
│                                      │   left_wheel/position                 │
│                                      │   left_wheel/velocity                 │
│                                      │   right_wheel/position                │
│                                      │   right_wheel/velocity                │
│                                      │ ────────────────────────────────────  │
│                                      │ [Configure] [Activate] [Deactivate]   │
│                                      │                                       │
├──────────────────────────────────────────────────────────────────────────────┤
│ Hardware Interfaces: 4 state, 2 command │ Status: All systems operational    │
└──────────────────────────────────────────────────────────────────────────────┘
```

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

### Menu Integration

- ROS2 > Show Controller Status (Ctrl+Shift+C)
- Toolbar button for quick access
- Status indicator in main status bar

### Future Enhancements

- Controller parameter tuning interface
- Real-time plotting of controller outputs
- Controller configuration file editor
- URDF joint/controller mapping visualization
- Controller performance metrics (jitter, latency)
- Integration with MoveIt2 for arm controllers

## Feature: In-Program Help Documentation (Planned)

### Overview

Provide comprehensive in-program help documentation accessible via menus, allowing users to learn ROS Weaver features without leaving the application.

### Features

1. **Help Menu Integration**:
   - Help > Getting Started - Quick start guide for new users
   - Help > User Manual - Full documentation browser
   - Help > Keyboard Shortcuts - Complete shortcut reference
   - Help > What's New - Changelog/release notes
   - Help > Video Tutorials - Links to tutorial videos (external)

2. **Context-Sensitive Help**:
   - F1 key shows help for currently focused widget/panel
   - "?" button on each panel header opens relevant help section
   - Tooltips with "Learn more..." links to detailed documentation
   - Right-click context menu "Help on this..." option

3. **Help Browser Window**:
   - Searchable documentation viewer
   - Table of contents sidebar navigation
   - Breadcrumb navigation
   - Print/export to PDF option
   - Bookmark favorite topics

4. **Documentation Sections**:
   - **Canvas Editor**: Block creation, connections, navigation, grouping
   - **Package Browser**: Searching, adding packages, templates
   - **Properties Panel**: Parameter editing, topics, TF tree
   - **Code Generation**: Wizard usage, templates, output options
   - **ROS2 Integration**: System scanning, topic monitoring, log viewing
   - **Project Management**: Save/load, examples, settings
   - **Keyboard Shortcuts**: Complete reference with search
   - **Troubleshooting**: Common issues and solutions

5. **Interactive Tutorials**:
   - Step-by-step guided walkthroughs
   - Highlight UI elements during tutorial
   - "Try it yourself" interactive exercises
   - Progress tracking for completed tutorials

6. **Quick Tips System**:
   - "Tip of the Day" on startup (optional)
   - Contextual tips based on user actions
   - Tips can be dismissed or saved for later

### Technical Implementation

```cpp
class HelpBrowser : public QDialog {
  Q_OBJECT

public:
  explicit HelpBrowser(QWidget* parent = nullptr);

  // Navigate to specific topic
  void showTopic(const QString& topicId);

  // Search documentation
  void search(const QString& query);

private:
  QTextBrowser* contentBrowser_;
  QTreeWidget* tocTree_;
  QLineEdit* searchEdit_;
  QListWidget* searchResults_;
};

class ContextHelp {
public:
  // Get help topic ID for a widget
  static QString getHelpTopic(QWidget* widget);

  // Show context help for widget
  static void showHelp(QWidget* widget);

  // Register widget with help topic
  static void registerWidget(QWidget* widget, const QString& topicId);
};
```

### Documentation Format

- **Source**: Markdown files in `docs/help/` directory
- **Build**: Convert to Qt Help format (.qhc/.qch) at build time
- **Fallback**: Render markdown directly if help files not compiled

### Menu Structure

```
Help
├── Getting Started               Ctrl+F1
├── User Manual...
├── Keyboard Shortcuts...         Ctrl+/
├── ─────────────────────
├── What's New
├── Video Tutorials               (opens browser)
├── ─────────────────────
├── Report Issue...               (opens GitHub)
├── ─────────────────────
├── About ROS Weaver
└── About Qt
```

### Future Enhancements

- Offline documentation download
- Multiple language support (i18n)
- User-contributed tips/documentation
- Integration with ROS2 documentation (ros.org links)
- AI-powered help assistant

## Feature: Embedded AI Assistant (Planned)

### Overview

Integrate an AI assistant directly into ROS Weaver to help users debug, understand, and optimize their robot systems. The assistant can access live robot data, logs, and project context to provide informed answers and suggestions.

### Supported AI Providers

- **Local LLMs** (Ollama) - Privacy-focused, offline option

  

## Maybe Future supported by using user's own accounts:

- **Claude** (Anthropic) - Recommended for code understanding and debugging
- **Gemini** (Google) - Alternative with multimodal capabilities
- **Grok** (xAI) - Alternative option
- **OpenAI GPT** - Widely available option
- 

### Features

1. **Chat Panel**:
   
   - Dedicated dockable panel for AI conversation
   - Markdown rendering for formatted responses
   - Code syntax highlighting in responses
   - Conversation history with search
   - Export chat to markdown/PDF
   - Multiple conversation threads
   
2. **Quick Action Buttons**:
   Pre-configured prompts for common debugging scenarios:

   **Diagnostics:**
   - "Why is my robot not launching?"
   - "What topics are published but not subscribed?"
   - "What topics are subscribed but not published?"
   - "Are there any TF tree issues?"
   - "Analyze recent error logs"
   - "Check for common misconfigurations"

   **Optimization:**
   - "Which nodes have high CPU usage?"
   - "Suggest bandwidth optimizations"
   - "Review my QoS settings"
   - "Identify potential bottlenecks"

   **Learning:**
   - "Explain this node's purpose"
   - "How does this topic flow work?"
   - "What does this error mean?"
   - "Suggest best practices for my setup"

3. **Context Access (Robot Data)**:
   The AI can access and analyze:
   - **Live Topics**: All active topics, types, publishers, subscribers, rates
   - **Node Graph**: Running nodes, their connections, namespaces
   - **TF Tree**: All transforms, rates, potential issues
   - **Parameters**: All node parameters and values
   - **Log Stream**: Recent /rosout messages, errors, warnings
   - **System State**: CPU, memory, network usage per node
   - **Project Data**: Canvas blocks, connections, generated code
   - **Launch Files**: Current launch configuration
   - **URDF/Robot Description**: Robot model if available
   - **Service/Action Status**: Available services and actions

4. **Proactive Suggestions**:
   - Alert when detecting common issues
   - Suggest fixes when errors appear in logs
   - Recommend optimizations based on system state
   - Notify about deprecated APIs or patterns

5. **Code Generation Assistance**:
   - Generate node code from natural language description
   - Create launch files from requirements
   - Write parameter YAML configurations
   - Generate message/service definitions
   - Explain and refactor existing code

6. **Visual Integration**:
   - Click any element on canvas → "Ask AI about this"
   - Highlight nodes/topics mentioned in AI responses
   - AI can reference specific canvas elements
   - Visual diff for suggested code changes

### UI Layout

```
┌─ AI Assistant ───────────────────────────────────────────────────────────────┐
│ Provider: [Claude ▼]  Model: [claude-sonnet-4-20250514 ▼]  [⚙ Settings]            │
├──────────────────────────────────────────────────────────────────────────────┤
│ ┌─ Quick Actions ──────────────────────────────────────────────────────────┐ │
│ │ [🔍 Why won't it launch?] [📡 Unsubscribed topics?] [🌳 TF issues?]     │ │
│ │ [📋 Analyze errors] [⚡ Performance check] [💡 Suggest improvements]     │ │
│ └──────────────────────────────────────────────────────────────────────────┘ │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─ You ─────────────────────────────────────────────────────────────────┐  │
│  │ Why is my robot not moving when I send cmd_vel commands?              │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌─ Claude ──────────────────────────────────────────────────────────────┐  │
│  │ I've analyzed your robot's current state. Here's what I found:        │  │
│  │                                                                        │  │
│  │ **Issue Detected:** The `/cmd_vel` topic has a publisher              │  │
│  │ (teleop_keyboard) but the subscriber (diff_drive_controller)          │  │
│  │ is using incompatible QoS settings.                                   │  │
│  │                                                                        │  │
│  │ **Details:**                                                           │  │
│  │ - Publisher QoS: RELIABLE, depth=10                                   │  │
│  │ - Subscriber QoS: BEST_EFFORT, depth=1                                │  │
│  │                                                                        │  │
│  │ **Suggested Fix:**                                                     │  │
│  │ ```python                                                              │  │
│  │ # Change subscriber QoS to match publisher                            │  │
│  │ qos = QoSProfile(reliability=RELIABLE, depth=10)                      │  │
│  │ ```                                                                    │  │
│  │                                                                        │  │
│  │ [📍 Show on Canvas] [📋 Copy Code] [🔧 Apply Fix]                      │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
├──────────────────────────────────────────────────────────────────────────────┤
│ │ Message...                                            [📎] [🎤] [Send] │  │
│ └────────────────────────────────────────────────────────────────────────┘  │
│ Context: 12 nodes │ 34 topics │ 156 TF frames │ 23 recent logs       [↻]   │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Technical Implementation

```cpp
class AIAssistantPanel : public QDockWidget {
  Q_OBJECT

public:
  explicit AIAssistantPanel(QWidget* parent = nullptr);

  // Set the AI provider
  void setProvider(AIProvider provider);

  // Send message with full robot context
  void sendMessage(const QString& message);

  // Quick action buttons
  void analyzeUnsubscribedTopics();
  void analyzeTFIssues();
  void analyzeRecentErrors();
  void checkPerformance();

signals:
  void highlightCanvasElement(const QString& elementId);
  void applyCodeSuggestion(const QString& filePath, const QString& code);

private:
  // Gather all robot context for AI
  QJsonObject gatherRobotContext();

  // Context components
  QJsonArray getActiveTopics();
  QJsonArray getNodeGraph();
  QJsonObject getTFTree();
  QJsonArray getRecentLogs();
  QJsonObject getSystemMetrics();
  QJsonObject getProjectData();
};

class AIProviderInterface {
public:
  virtual ~AIProviderInterface() = default;
  virtual QFuture<QString> complete(const QString& prompt,
                                     const QJsonObject& context) = 0;
  virtual void setApiKey(const QString& key) = 0;
  virtual void setModel(const QString& model) = 0;
};

class ClaudeProvider : public AIProviderInterface { /* ... */ };
class GeminiProvider : public AIProviderInterface { /* ... */ };
class OllamaProvider : public AIProviderInterface { /* ... */ };
```

### Context Gathering System

```cpp
struct RobotContext {
  // Live system state
  QList<TopicInfo> topics;          // All topics with pub/sub counts
  QList<NodeInfo> nodes;            // Running nodes with connections
  QList<TFFrameInfo> tfFrames;      // TF tree structure
  QList<ParameterInfo> parameters;  // All parameters
  QList<LogEntry> recentLogs;       // Last N log messages

  // Project context
  Project currentProject;           // Canvas blocks and connections
  QStringList generatedFiles;       // Recently generated code

  // System metrics
  QMap<QString, NodeMetrics> nodeMetrics;  // CPU, memory per node

  // Serialize for AI prompt
  QJsonObject toJson() const;
  QString toMarkdown() const;  // Human-readable summary
};
```

### Configuration

```yaml
ai_assistant:
  enabled: true
  provider: claude  # claude, gemini, grok, openai, ollama

  # API configuration (keys stored securely)
  claude:
    model: claude-sonnet-4-20250514
    max_tokens: 4096

  gemini:
    model: gemini-pro

  ollama:
    endpoint: http://localhost:11434
    model: llama3

  # Context settings
  context:
    include_topics: true
    include_nodes: true
    include_tf: true
    include_logs: true
    max_log_entries: 100
    include_parameters: true
    include_project: true

  # Privacy settings
  privacy:
    anonymize_namespaces: false
    exclude_topics: ["/camera/image_raw"]  # Don't send to cloud
    local_only_mode: false  # Use only Ollama

  # UI settings
  ui:
    show_quick_actions: true
    auto_gather_context: true
    stream_responses: true
```

### Security & Privacy

- **API Key Storage**: Secure storage using system keychain (libsecret/Keychain)
- **Local Mode**: Option to use only local LLMs (Ollama) for sensitive data
- **Data Filtering**: Exclude specific topics/data from AI context
- **Anonymization**: Option to anonymize node/topic names before sending
- **Audit Log**: Log what data was sent to AI providers

### Menu Integration

- AI > Show AI Assistant (Ctrl+Shift+A)
- AI > Quick Diagnosis > [submenu of quick actions]
- AI > Settings... (API keys, provider selection)
- Right-click any element → "Ask AI about this..."
- Toolbar button with status indicator

### Future Enhancements

- **Voice Input**: Microphone button for voice queries
- **Multimodal**: Send camera images to Gemini for visual debugging
- **Autonomous Actions**: AI can execute suggested fixes with approval
- **Learning**: Remember project-specific context across sessions
- **Team Knowledge Base**: Share AI insights across team (optional cloud sync)
- **Custom Prompts**: User-defined quick action buttons
- **ROS2 Documentation RAG**: Retrieve relevant docs for answers
- **Error Pattern Database**: Learn from common issues across users (opt-in)
- **Simulation Integration**: AI can suggest and run Gazebo tests
- **Natural Language Launch**: "Launch the navigation stack with this map"

### Local LLM Integration via Ollama (Recommended Primary Approach)

For an open-source, non-commercial project, prioritize local LLMs via Ollama to keep the tool fully offline-capable, free, and private.

#### Core Requirements

- **Backend**: Ollama (easy installation, OpenAI-compatible API at `http://localhost:11434`)
- **Offline**: No internet required after initial model download
- **Multi-model**: Support multiple models with smart defaults/recommendations
- **Optional**: Integration is optional (gracefully handle when Ollama not running)
- **Minimal Dependencies**: Use `requests` or `openai` Python client

#### Recommended Models (December 2025)

Focus on 7B-8B parameter models for good performance on 8-16GB RAM machines:

| Model | Ollama Tag | Use Case | RAM Usage |
|-------|-----------|----------|-----------|
| **Qwen2.5-Coder 7B** (Primary) | `qwen2.5-coder:7b` | Best coding performance, excels on HumanEval/LiveCodeBench | ~5-7GB |
| **Qwen3-Coder 8B** | `qwen3:8b` | Latest iteration with improved reasoning | ~5-7GB |
| **DeepSeek-Coder V2 7B** | `deepseek-coder:7b` | Excellent reasoning for complex tasks | ~5-7GB |
| **DeepSeek-R1 8B** | `deepseek-r1:8b` | Strong alternative with reasoning | ~5-7GB |
| **Llama 3.3 8B** (All-rounder) | `llama3.3:8b` | Balance of coding and general knowledge | ~5-7GB |
| **Phi-3 Mini 3.8B** (Fallback) | `phi3:mini` | For constrained hardware | ~3GB |
| **Gemma 2 2B** (Minimal) | `gemma2:2b` | Absolute minimum for low-end systems | ~2GB |

Use Q4_K_M or Q5_K_M quantization for optimal performance/quality balance.

#### Hardware Detection and Model Recommendation

Auto-scan user's machine on first run to recommend suitable model:

```python
import psutil
import platform
import subprocess

def get_hardware_info():
    """Detect system hardware capabilities."""
    ram_gb = psutil.virtual_memory().total / (1024 ** 3)
    gpu_info = "None"
    vram_gb = 0

    # Try torch for NVIDIA GPU detection
    try:
        import torch
        if torch.cuda.is_available():
            gpu_info = torch.cuda.get_device_name(0)
            vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024 ** 3)
    except ImportError:
        pass

    # Check for Apple Silicon (unified memory)
    if platform.system() == 'Darwin' and platform.machine().startswith('arm'):
        gpu_info = "Apple Silicon (Unified Memory)"
        vram_gb = ram_gb  # Unified memory

    return {
        "ram_gb": round(ram_gb, 1),
        "gpu": gpu_info,
        "vram_gb": round(vram_gb, 1)
    }

def recommend_model(info):
    """Recommend optimal model based on hardware."""
    if info["ram_gb"] < 8:
        return {
            "model": None,
            "warning": "Hardware too limited (<8GB RAM). Consider upgrading or using cloud alternatives.",
            "fallback": "gemma2:2b"
        }
    elif info["ram_gb"] < 12:
        return {
            "model": "phi3:mini",
            "note": "Lightweight model for constrained hardware"
        }
    elif info["vram_gb"] >= 8 or info["ram_gb"] >= 16:
        return {
            "model": "qwen2.5-coder:7b-q5_k_m",
            "note": "Best coding performance for your hardware"
        }
    else:
        return {
            "model": "qwen2.5-coder:7b-q4_k_m",
            "note": "Balanced model for CPU inference"
        }
```

#### Recommendation Logic

| RAM | GPU/VRAM | Recommendation |
|-----|----------|----------------|
| <8GB | Any | Not recommended; warn user, suggest `gemma2:2b` |
| 8-12GB | CPU only | `phi3:mini` or `qwen2.5-coder:7b-q4_k_m` |
| 12-16GB | Any | `qwen2.5-coder:7b-q4_k_m` |
| 16GB+ | 8GB+ VRAM | `qwen2.5-coder:7b-q5_k_m` or larger |
| Apple Silicon | 16GB+ Unified | `qwen2.5-coder:7b-q5_k_m` |

#### AI Feature Set

With local LLMs, provide these ROS2-specific capabilities:

1. **Code Generation**
   - Generate ROS2 node code from natural language descriptions
   - Create launch files from requirements
   - Generate message/service definitions
   - Autocomplete message field definitions

2. **Architecture Assistance**
   - Suggest package structures
   - Recommend node organization
   - Design topic hierarchies

3. **Learning & Debugging**
   - Explain ROS2 concepts in context
   - Analyze error logs and suggest fixes
   - Explain what nodes/topics do
   - Debugging assistance with live system context

#### Integration Implementation

```python
class OllamaProvider:
    """Local LLM provider using Ollama."""

    def __init__(self, endpoint="http://localhost:11434"):
        self.endpoint = endpoint
        self.model = "qwen2.5-coder:7b"

    def is_available(self) -> bool:
        """Check if Ollama is running."""
        try:
            response = requests.get(f"{self.endpoint}/api/tags", timeout=2)
            return response.status_code == 200
        except:
            return False

    def list_models(self) -> list:
        """Get available models."""
        response = requests.get(f"{self.endpoint}/api/tags")
        return [m["name"] for m in response.json().get("models", [])]

    def pull_model(self, model: str) -> bool:
        """Download a model."""
        response = requests.post(
            f"{self.endpoint}/api/pull",
            json={"name": model},
            stream=True
        )
        # Handle streaming progress...
        return response.status_code == 200

    def complete(self, prompt: str, system_prompt: str = None) -> str:
        """Generate completion."""
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})

        response = requests.post(
            f"{self.endpoint}/api/chat",
            json={
                "model": self.model,
                "messages": messages,
                "stream": False
            }
        )
        return response.json()["message"]["content"]

# ROS2-specific system prompts
ROS2_CODE_PROMPT = """You are a ROS2 expert assistant. Generate clean, idiomatic Python or C++ code
following ROS2 best practices. Use appropriate message types, follow naming conventions,
and include proper error handling. Target ROS2 Humble/Jazzy compatibility."""

ROS2_DEBUG_PROMPT = """You are a ROS2 debugging assistant. Analyze the provided system state,
logs, and error messages to identify issues. Suggest specific fixes with code examples.
Consider common problems like QoS mismatches, missing dependencies, and TF issues."""
```

#### Configuration

```yaml
ai_assistant:
  ollama:
    enabled: true
    endpoint: http://localhost:11434
    model: qwen2.5-coder:7b
    auto_detect_hardware: true
    auto_pull_recommended: true  # Auto-download recommended model
    temperature: 0.7
    max_tokens: 4096

    # Model-specific settings
    models:
      coding:
        model: qwen2.5-coder:7b
        temperature: 0.3  # Lower for code
      general:
        model: llama3.3:8b
        temperature: 0.7

    # Fallback chain
    fallback_models:
      - qwen2.5-coder:7b
      - deepseek-coder:7b
      - llama3.3:8b
      - phi3:mini
```

#### Setup Flow

1. **First Run Detection**
   - Check if Ollama is installed (`which ollama` or check service)
   - If not installed, show setup dialog with install instructions
   - Link to https://ollama.com for installation

2. **Hardware Scan**
   - Run hardware detection on first AI feature use
   - Display results and recommended model
   - Offer to auto-pull recommended model

3. **Model Management**
   - Show installed models in settings
   - Pull/delete models from UI
   - Model performance benchmarking option

4. **Graceful Degradation**
   - If Ollama not running, disable AI features with clear message
   - Option to start Ollama from within app (`ollama serve`)
   - Queue requests and notify when Ollama becomes available

#### Testing Requirements

- Test on low-end (8GB RAM, CPU-only) machines
- Test on mid-range (16GB RAM, integrated GPU)
- Test on high-end (32GB RAM, dedicated GPU)
- Verify generated ROS2 code compiles and runs
- Benchmark response times for different models

## Feature: Plot Panel (Planned)

### Overview

Real-time plotting of time-series data for tuning and spotting anomalies. Visualize sensor values, velocities, errors, battery levels, and any numeric ROS2 topic data over time.

### Features

1. **Topic Subscription**:
   - Subscribe to any numeric topic (Float32, Float64, Int32, etc.)
   - Extract fields from complex messages (e.g., `geometry_msgs/Twist.linear.x`)
   - Support for array indexing (e.g., `sensor_msgs/JointState.position[0]`)
   - Multiple topics on same plot with different colors
   - Auto-detection of plottable fields in message types

2. **Plot Types**:
   - **Line Plot**: Standard time-series line graph (default)
   - **Scatter Plot**: Discrete data points
   - **Step Plot**: For discrete state changes
   - **Bar Plot**: For histogram-style data
   - **Multi-Axis**: Secondary Y-axis for different scales

3. **Time Window Options**:
   - Sliding window (last N seconds): 10s, 30s, 60s, 5min
   - Fixed time range with scroll
   - Pause/resume to freeze display
   - Historical playback with slider

4. **Visual Customization**:
   - Line colors, styles (solid, dashed, dotted)
   - Line thickness
   - Legend position and visibility
   - Grid lines on/off
   - Dark/light theme support
   - Axis labels and titles

5. **Analysis Tools**:
   - **Statistics Display**: Min, max, mean, std dev, current value
   - **Threshold Lines**: Horizontal reference lines for limits
   - **Anomaly Highlighting**: Color regions outside thresholds
   - **Cursor Readout**: Click to see exact value at time point
   - **Zoom & Pan**: Mouse wheel zoom, drag to pan
   - **Measurement Tools**: Delta between two points

6. **Data Export**:
   - Export to CSV
   - Export to rosbag2
   - Screenshot plot as PNG/SVG
   - Copy data to clipboard

7. **Preset Configurations**:
   - Save/load plot configurations
   - Built-in presets:
     - Battery Monitor: voltage, current, percentage
     - Odometry: x, y, theta velocities
     - Joint States: positions, velocities, efforts
     - IMU: acceleration, angular velocity
     - PID Tuning: setpoint, feedback, error

### UI Layout

```
┌─ Plot Panel ─────────────────────────────────────────────────────────────────┐
│ [+ Add Topic] [⚙ Settings] [📷 Screenshot] [💾 Export] │ Window: [30s ▼] [▶||] │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  1.5 ┤                              ╭─╮                                      │
│      │                            ╭─╯ ╰─╮        ╭──╮                        │
│  1.0 ┤    ╭──────╮              ╭─╯     ╰──────╮╭╯  ╰╮                       │
│      │  ╭─╯      ╰─╮          ╭─╯              ╰╯    ╰─╮                     │
│  0.5 ┤╭─╯          ╰──────────╯                        ╰─────                │
│      │                                                                       │
│  0.0 ┼──────────────────────────────────────────────────────────────────────│
│     -30s        -25s        -20s        -15s        -10s        -5s      now │
│                                                                              │
├──────────────────────────────────────────────────────────────────────────────┤
│ ● /cmd_vel.linear.x  ● /odom.twist.twist.linear.x                           │
│ Current: 0.52 m/s │ Min: 0.00 │ Max: 1.48 │ Mean: 0.67 │ Std: 0.31          │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Technical Implementation

- **Plotting Library**: QCustomPlot or QtCharts for rendering
- **Data Buffer**: Ring buffer for efficient memory usage
- **Update Rate**: Configurable 10-60 Hz refresh
- **Threading**: Background thread for data collection, UI thread for rendering
- **Message Introspection**: Use `rosidl_typesupport_introspection_cpp` for field extraction

### Message Field Selection Dialog

```
┌─ Select Field to Plot ───────────────────────────────────────────────────────┐
│ Topic: /joint_states                                                          │
│ Type: sensor_msgs/msg/JointState                                              │
├───────────────────────────────────────────────────────────────────────────────┤
│ ▼ header                                                                      │
│   ├─ stamp.sec (int32)                                                        │
│   └─ stamp.nanosec (uint32)                                                   │
│ ▼ name[] (string[])                                                           │
│ ▼ position[] (float64[])                                                      │
│   ├─ [0] - joint1 ←────────────────────────────────────── [Select]            │
│   ├─ [1] - joint2                                                             │
│   └─ [2] - joint3                                                             │
│ ▼ velocity[] (float64[])                                                      │
│ ▼ effort[] (float64[])                                                        │
├───────────────────────────────────────────────────────────────────────────────┤
│ Selected: position[0]                              [Cancel] [Add to Plot]     │
└───────────────────────────────────────────────────────────────────────────────┘
```

### Dependencies

- `QCustomPlot` or `QtCharts` module
- Message introspection for field extraction

### Menu Integration

- ROS2 > Show Plot Panel (Ctrl+Shift+P)
- Right-click topic in Topic Viewer → "Plot this topic"
- Toolbar button for quick access

### Future Enhancements

- FFT/frequency analysis view
- Correlation between multiple signals
- Triggered recording (start recording when threshold crossed)
- Alarm/notification when values exceed limits
- Integration with rosbag2 for offline analysis
- 2D plots (X-Y scatter of two topics)
- Expression evaluation (plot computed values like `topic1 - topic2`)

## TF Tree Viewer Enhancements (Planned)

### Known Issues / Fixes Needed

- **View Mode Switching (Bug)**: The Tree View / Flat List combo box doesn't properly switch between views
  - **Current State**: Combo box exists but view doesn't change when selection changes
  - **Fix Required**: Implement `onViewModeChanged()` slot to rebuild tree in selected mode
  - **Tree View**: Hierarchical parent-child structure (current default)
  - **Flat List**: Alphabetically sorted flat list of all frames with parent column
  - **Table View** (future): Sortable columns for frame, parent, type, rate, status
  - **Graph View** (future): Visual node-and-edge graph like rqt_tf_tree

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
  - **Depth Visualization**: Color gradient based on tree depth
  - **Transform Gizmos**: Visual representation of rotation/translation at each frame
  - **Screenshot/Recording**: Export 3D view as image or animated GIF
  - **Sync with Tree View**: Selecting frame in 3D highlights it in tree panel and vice versa

- URDF import: Load URDF and compare expected vs actual TF tree
- Transform recording: Record transforms to bag for replay
- Visual transform editor: Adjust transforms visually and publish corrections
- Multi-robot TF: Handle namespaced TF trees for multi-robot setups
- AR overlay: Show TF frames overlaid on camera images
- Performance profiler: Identify TF-related bottlenecks

# Future Feature: Unit Testing (Planned)

Using gtest and a suite of define tests to perform unit testing where it is most valuable for the app.



## Planned Feature: Tutorial mode (Completed)

If selected it will pulse visuals over the UI in sequence to let a new user run an example and be shown all the features



## Robot Wizard ✓ COMPLETED

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



## Remote AI

* allow the user to integreate their own Claude Code account, or codex or similar.



## Local AI

* Allow user to Cut and paste images to Local AI
* Make the streaming in tokens not sit too low and be half un-rendered (add a buffer at the bottom)
* Add and option token generation speed to the lower bar Local AI status indicated. Only shown when the model is currently rendering tokens in ollama
* Set up a drop down with predetermined questions
* link the available data to the context of the local AI
* Create a system prompt for the Local AI and store it, show it in the setting pages and allow user to update it in the settings page.
* Make the user and response chats fill only 80% of the width



## System Mapping

In the System Matching panel, once a scan has been done if the user sees Extra Nodes in System and double clicks on them this should add the node to the canvas.



## Feature: Data linage

* every piece of displayed data should let the user right hand click to show where it originates from.
  * Where it sits in a text editable file, the option should be to include the option to open it to inspect or edit in VS Code
  * Where the data is generated or indirectly related to a source file, then the complete data linage and source file should be shown, with the option to edit where appropriate



## Bugs to fix

- [x] Plot panel is not full height
- [x] All topics default to being ticked for the topic panel (all start on for visualisation) 



## Deployment

* write a blog post





## Feature: Undo function.



## Feature:Update ollama version to allow tool usage
