# ROS2Weaver Feature Ideas & Roadmap

These are community and AI-suggested features to enhance ROS2Weaver, prioritized by potential impact and feasibility. Contributions are welcome!

## Implementation Status

### Completed Features (v1.1.0)

| Feature | Status | Notes |
|---------|--------|-------|
| Context-Sensitive Help | ✅ Done | F1 key, help browser, context tooltips |
| Inline Tooltips & Schema Viewer | ✅ Done | Message schema viewer panel |
| README Preview Panel | ✅ Done | Dockable panel for package docs |
| Export to PlantUML/Mermaid/Graphviz | ✅ Done | File → Export menu |
| Import from rqt_graph DOT | ✅ Done | File → Import menu |
| Import from CARET YAML | ✅ Done | File → Import menu |
| Project Save/Load (.weaver) | ✅ Done | JSON/YAML project files |
| One-click Gazebo/Ignition Launch | ✅ Done | Simulation menu |
| Rosbag Recording/Playback | ✅ Done | Rosbag Workbench panel |
| Static Analysis | ✅ Done | Tools → Run Analysis |
| Parameter Dashboard | ✅ Done | Properties panel |
| Undo/Redo Stack | ✅ Done | Edit menu, Ctrl+Z/Y |
| Architecture Diff View | ✅ Done | Compare project versions |
| Multi-canvas Tabs | ✅ Done | Tab-based canvas management |
| Canvas Grid Visualization | ✅ Done | View menu, snap-to-grid |
| Mini-map Navigation | ✅ Done | Dockable minimap panel |
| Auto-layout Algorithms | ✅ Done | Hierarchical, force-directed, circular, grid |
| Canvas Annotations | ✅ Done | Right-click → Add Annotation |
| Connection Bandwidth Visualization | ✅ Done | Live message rates on connections |
| Time-synced Playback | ✅ Done | Canvas highlights during bag playback |
| Node Health Dashboard | ✅ Done | CPU, memory, latency monitoring |
| QoS Profile Editor | ✅ Done | Visual QoS configuration |
| Launch File Generator | ✅ Done | Code generation wizard |
| Parameter Presets | ✅ Done | Save/load parameter configurations |
| Node Templates Library | ✅ Done | Pre-built node patterns |
| Workspace Browser Panel | ✅ Done | Browse ROS2 workspace packages |
| Natural Language Architecture Generation | ✅ Done | AI-powered architecture from description |
| Automated Error Fixing Suggestions | ✅ Done | AI suggestions via "Ask AI" button |
| Architecture Optimization Recommendations | ✅ Done | Tools → Optimize Architecture |
| Advanced Node Library Search | ✅ Done | Search tab with filters |

### Remaining Features

| Feature | Priority | Complexity |
|---------|----------|------------|
| Embedded mini-RViz / 3D view | Low | High |
| Multi-user collaboration | Low | High |
| Improved namespace/remapping visualization | Low | Low |

---

## Contributing Guidelines

### Branch Naming Convention
When implementing a feature from this roadmap, create a new branch using the following convention:

```bash
git checkout -b feature/<feature-name>
```

Examples:
- `feature/mini-rviz-view`
- `feature/ai-architecture-generation`
- `feature/multi-user-collab`

### Workflow
1. Create a new branch from `main` for each feature.
2. Implement the feature with appropriate tests.
3. Submit a pull request back to `main`.
4. Reference the relevant roadmap item in the PR description.

This keeps features isolated and makes code review easier.

---

## Feature Details

### Remaining: Embedded mini-RViz / 3D View
- Overlay live robot model and TF data
- Visualize sensor data in 3D
- Integration with existing TF Tree panel
- Would require significant Qt3D or RViz library integration

### Remaining: Multi-user Collaboration
- Real-time collaborative editing
- Presence indicators
- Conflict resolution
- Would require WebSocket server infrastructure

---

## Completed Feature Details

### AI-Powered Features ✅

#### Natural Language Architecture Generation
- Describe your robot system in plain language and get a generated architecture
- Uses Ollama integration for AI-powered suggestions
- Supports: robot type, use case, available sensors as input parameters
- Generates nodes, connections, and logical groupings automatically
- Example: "Create a ROS2 architecture for a mobile robot with SLAM and navigation"

#### Automated Error Fixing Suggestions
- "Ask AI" button in the Issue List panel for each detected issue
- Sends issue context to local LLM for targeted fix suggestions
- Provides structured suggestions with confidence scores
- Includes code examples for applicable fixes
- Integrates with static analysis results

#### Architecture Optimization Recommendations
- Tools → Optimize Architecture menu
- Analyzes current canvas architecture for:
  - Unused nodes and connections
  - Suboptimal QoS configurations
  - Missing common patterns
  - Performance bottlenecks
- Provides actionable recommendations with implementation guidance

#### Advanced Node Library Search
- Search tab in package browser with advanced filtering
- Category filters: Navigation, Perception, Manipulation, SLAM, Control, Drivers, Simulation, Utilities
- Message type filtering (common ROS2 message types)
- Feature filters: Has Inputs, Has Outputs, Has Services, Has Actions
- Double-click or "Add to Canvas" button to add nodes

### Documentation & Discovery Features ✅

#### Context-Sensitive ROS2 Documentation Lookup
- Hover or right-click on nodes, topics, services, actions, message types, or parameters to open a side panel/pop-up with relevant official documentation.
- Sources:
  - Official ROS2 docs (`https://docs.ros.org`) for core packages.
  - Local parsing of `.msg`, `.srv`, `.action` files for field descriptions.
  - Cached index for offline use.
- Global searchable docs bar (e.g., search "LaserScan fields" or "MoveIt parameters").

#### Inline Tooltips & Message Schema Viewer
- Quick hover tooltips showing message fields, types, and defaults.
- Click-to-expand full interactive schema tree (similar to `rqt_msg` or Foxglove message inspector).

#### Package/Node README & Description Preview
- Automatically load and display `README.md` or `package.xml` description for selected nodes/packages in a dockable panel.

### Export, Import & Interoperability ✅

- Export canvas diagrams to **PlantUML**, **Mermaid**, or **Graphviz** for documentation.
- Import existing architectures from:
  - `rqt_graph` DOT files.
  - CARET `architecture.yaml` files.
- Save/load full projects as `.weaver` JSON/YAML files.

### Simulation & Visualization Enhancements ✅

- One-click launch of **Gazebo** or **Ignition** using the robot configuration wizard output.
- Integrated **ros2 bag** recording/playback with canvas animation during replay.

### Validation & Debugging Tools ✅

- Static analysis:
  - Topic type mismatches.
  - Unused publishers/subscribers.
  - QoS policy incompatibilities.
  - Cyclic dependency detection.
  - Naming convention validation.
  - Security issue detection.
- Live parameter validation dashboard with override capabilities.
- Error highlighting and suggested fixes on the canvas.

### Collaboration & Version Control ✅

- Robust **undo/redo** stack for canvas operations.
- Git-aware project saving with visual diff support for canvas changes.

### Usability & Polish ✅

- Multi-canvas tabs for separate compositions (e.g., navigation stack vs. manipulation).
- Improved namespace and remapping visualization.
- Expanded keyboard shortcuts and customizable themes.

#### Canvas Grid Visualization (Blender-style)
- Configurable grid overlay with major and minor lines for precise node placement.
- Settings:
  - Toggle grid visibility on/off.
  - Snap-to-grid option for automatic alignment when moving nodes.
  - Adjustable granularity for major lines (e.g., every 100px, 200px).
  - Adjustable granularity for minor lines (e.g., subdivisions between major lines).
  - Customizable grid line colors and opacity.
- Visual style similar to Blender's 3D viewport grid for familiarity.

### Canvas Navigation & Layout ✅

#### Mini-map Navigation
- Small overview panel for navigating large architectures (similar to VS Code's minimap).
- Shows entire canvas with viewport indicator.
- Click-to-navigate and drag viewport rectangle.
- Useful for projects with many nodes spread across large canvas areas.

#### Auto-layout Algorithms
- One-click automatic layout options to organize nodes:
  - **Hierarchical** - Top-to-bottom or left-to-right data flow.
  - **Force-directed** - Physics-based spacing for balanced distribution.
  - **Circular** - Arrange nodes in a circle, useful for cyclic architectures.
  - **Grid** - Snap all nodes to a regular grid pattern.
- Option to layout only selected nodes or entire canvas.
- Undo support for layout operations.

#### Canvas Annotations & Sticky Notes
- Add text notes and documentation directly on the canvas.
- Sticky note widgets with customizable colors.
- Support for markdown formatting in notes.
- Link notes to specific nodes or connections.
- Useful for team communication and architecture documentation.

### Live Monitoring & Visualization ✅

#### Connection Bandwidth Visualization
- Show live message rates (Hz) on connection lines.
- Color coding for connection health:
  - Green: Healthy, receiving messages at expected rate.
  - Yellow: Degraded, lower than expected rate.
  - Red: Stale or dropped messages.
- Optional bandwidth display (KB/s, MB/s).
- Click connection to see detailed statistics (latency, jitter, queue depth).

#### Time-Synced Playback Visualization
- During bag playback, animate data flow on canvas connections.
- Visual pulse/glow effect showing which topics are active at each timestamp.
- Synchronized with timeline scrubbing.
- Helps understand temporal relationships between nodes during debugging.

#### Node Health Dashboard
- Real-time monitoring overlay for each node:
  - CPU usage percentage.
  - Memory consumption.
  - Callback execution latency.
  - Dropped message count.
- Heatmap visualization option (color nodes by resource usage).
- Alert thresholds with visual warnings.
- Historical graphs accessible via node context menu.

### Configuration & Code Generation ✅

#### QoS Profile Visual Editor
- Visual QoS configuration per connection/topic.
- Dropdown presets: Sensor Data, Services, Parameters, Default.
- Custom configuration with all QoS policies:
  - Reliability (reliable/best effort).
  - Durability (volatile/transient local).
  - History depth.
  - Deadline, lifespan, liveliness.
- Compatibility checker showing warnings for mismatched publisher/subscriber QoS.
- Generate QoS code snippets for C++/Python.

#### Launch File Generator
- Generate ROS2 launch files directly from canvas configuration.
- Support for Python and XML launch file formats.
- Include:
  - Node declarations with parameters.
  - Remappings from canvas connections.
  - Namespace configurations.
  - Composable node containers where applicable.
- Preview before export with syntax highlighting.

#### Parameter Presets
- Save named parameter configurations for quick switching.
- Example presets: "simulation", "real_robot", "debug", "competition".
- Per-node or global parameter overrides.
- Quick toggle between presets from toolbar.
- Export/import presets as YAML files.
- Diff view between presets.

### Node Library & Templates ✅

#### Node Templates Library
- Pre-configured templates for common ROS2 patterns:
  - **Sensor Fusion Node** - Multi-sensor input with synchronized output.
  - **State Machine** - Template with state transitions and events.
  - **Behavior Tree Executor** - BehaviorTree.CPP integration template.
  - **Bridge Node** - ROS1↔ROS2 bridge configuration.
  - **Lifecycle Node** - Managed node with state callbacks.
  - **Component Node** - Composable node template.
- Customizable templates with placeholder parameters.

#### Workspace Browser Panel
- Browse local ROS2 workspace packages.
- Tree view of installed packages organized by:
  - Workspace (overlay order).
  - Package type (nodes, libraries, messages).
- Drag-and-drop nodes from browser onto canvas.
- Quick search/filter within workspace.
- Show package dependencies and reverse dependencies.

### Comparison & Analysis ✅

#### Architecture Diff View
- Visually compare two project versions side-by-side.
- Highlighting for:
  - Added nodes/connections (green).
  - Removed nodes/connections (red).
  - Modified parameters (yellow).
- Git integration to compare with previous commits.
- Export diff report as HTML or markdown.
- Useful for code reviews and architecture evolution tracking.

---

Feel free to open issues or pull requests for any remaining features — they'd make ROS2Weaver an even more powerful all-in-one ROS2 development environment!
