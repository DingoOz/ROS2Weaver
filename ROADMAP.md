# ROS2Weaver Feature Ideas & Roadmap Suggestions

These are community and AI-suggested features to enhance ROS2Weaver, prioritized by potential impact and feasibility. Contributions are welcome!

## Contributing Guidelines

### Branch Naming Convention
When implementing a feature from this roadmap, create a new branch using the following convention:

```bash
git checkout -b feature/<feature-name>
```

Examples:
- `feature/mini-map-navigation`
- `feature/auto-layout-algorithms`
- `feature/canvas-grid-visualization`
- `feature/qos-profile-editor`
- `feature/launch-file-generator`

### Workflow
1. Create a new branch from `main` for each feature.
2. Implement the feature with appropriate tests.
3. Submit a pull request back to `main`.
4. Reference the relevant roadmap item in the PR description.

This keeps features isolated and makes code review easier.

## Documentation & Discovery Features (High Priority)

### 1. Context-Sensitive ROS2 Documentation Lookup
- Hover or right-click on nodes, topics, services, actions, message types, or parameters to open a side panel/pop-up with relevant official documentation.
- Sources:
  - Official ROS2 docs (`https://docs.ros.org`) for core packages.
  - Local parsing of `.msg`, `.srv`, `.action` files for field descriptions.
  - Cached index for offline use.
- Bonus: Global searchable docs bar (e.g., search "LaserScan fields" or "MoveIt parameters").

### 2. Inline Tooltips & Message Schema Viewer
- Quick hover tooltips showing message fields, types, and defaults.
- Click-to-expand full interactive schema tree (similar to `rqt_msg` or Foxglove message inspector).

### 3. Package/Node README & Description Preview
- Automatically load and display `README.md` or `package.xml` description for selected nodes/packages in a dockable panel.

## Export, Import & Interoperability

- Export canvas diagrams to **PlantUML**, **Mermaid**, or **Graphviz** for documentation.
- Import existing architectures from:
  - `rqt_graph` DOT files.
  - CARET `architecture.yaml` files.
- Save/load full projects as `.weaver` JSON/YAML files.

## Simulation & Visualization Enhancements

- One-click launch of **Gazebo** or **Ignition** using the robot configuration wizard output.
- Embedded mini-RViz or 3D view to overlay live robot model and TF data.
- Integrated **ros2 bag** recording/playback with canvas animation during replay.

## Validation & Debugging Tools

- Static analysis:
  - Topic type mismatches.
  - Unused publishers/subscribers.
  - QoS policy incompatibilities.
- Live parameter validation dashboard with override capabilities.
- Error highlighting and suggested fixes on the canvas.

## AI Assistant Expansions (Leveraging Existing Ollama Integration)

- Natural language architecture generation (e.g., "Add SLAM with Nav2 and connect to a lidar topic").
- Automated error fixing suggestions based on validation results.
- Architecture optimization recommendations.

## Collaboration & Version Control

- Robust **undo/redo** stack for canvas operations.
- Git-aware project saving with visual diff support for canvas changes.
- Multi-user collaboration potential (future).

## Usability & Polish

- Multi-canvas tabs for separate compositions (e.g., navigation stack vs. manipulation).
- Improved namespace and remapping visualization.
- Advanced node library search with filters (package, category, keywords).
- Expanded keyboard shortcuts and customizable themes.

### Canvas Grid Visualization (Blender-style)
- Configurable grid overlay with major and minor lines for precise node placement.
- Settings:
  - Toggle grid visibility on/off.
  - Snap-to-grid option for automatic alignment when moving nodes.
  - Adjustable granularity for major lines (e.g., every 100px, 200px).
  - Adjustable granularity for minor lines (e.g., subdivisions between major lines).
  - Customizable grid line colors and opacity.
- Visual style similar to Blender's 3D viewport grid for familiarity.

## Canvas Navigation & Layout

### Mini-map Navigation
- Small overview panel for navigating large architectures (similar to VS Code's minimap).
- Shows entire canvas with viewport indicator.
- Click-to-navigate and drag viewport rectangle.
- Useful for projects with many nodes spread across large canvas areas.

### Auto-layout Algorithms
- One-click automatic layout options to organize nodes:
  - **Hierarchical** - Top-to-bottom or left-to-right data flow.
  - **Force-directed** - Physics-based spacing for balanced distribution.
  - **Circular** - Arrange nodes in a circle, useful for cyclic architectures.
  - **Grid** - Snap all nodes to a regular grid pattern.
- Option to layout only selected nodes or entire canvas.
- Undo support for layout operations.

### Canvas Annotations & Sticky Notes
- Add text notes and documentation directly on the canvas.
- Sticky note widgets with customizable colors.
- Support for markdown formatting in notes.
- Link notes to specific nodes or connections.
- Useful for team communication and architecture documentation.

## Live Monitoring & Visualization

### Connection Bandwidth Visualization
- Show live message rates (Hz) on connection lines.
- Color coding for connection health:
  - Green: Healthy, receiving messages at expected rate.
  - Yellow: Degraded, lower than expected rate.
  - Red: Stale or dropped messages.
- Optional bandwidth display (KB/s, MB/s).
- Click connection to see detailed statistics (latency, jitter, queue depth).

### Time-Synced Playback Visualization
- During bag playback, animate data flow on canvas connections.
- Visual pulse/glow effect showing which topics are active at each timestamp.
- Synchronized with timeline scrubbing.
- Helps understand temporal relationships between nodes during debugging.

### Node Health Dashboard
- Real-time monitoring overlay for each node:
  - CPU usage percentage.
  - Memory consumption.
  - Callback execution latency.
  - Dropped message count.
- Heatmap visualization option (color nodes by resource usage).
- Alert thresholds with visual warnings.
- Historical graphs accessible via node context menu.

## Configuration & Code Generation

### QoS Profile Visual Editor
- Visual QoS configuration per connection/topic.
- Dropdown presets: Sensor Data, Services, Parameters, Default.
- Custom configuration with all QoS policies:
  - Reliability (reliable/best effort).
  - Durability (volatile/transient local).
  - History depth.
  - Deadline, lifespan, liveliness.
- Compatibility checker showing warnings for mismatched publisher/subscriber QoS.
- Generate QoS code snippets for C++/Python.

### Launch File Generator
- Generate ROS2 launch files directly from canvas configuration.
- Support for Python and XML launch file formats.
- Include:
  - Node declarations with parameters.
  - Remappings from canvas connections.
  - Namespace configurations.
  - Composable node containers where applicable.
- Preview before export with syntax highlighting.

### Parameter Presets
- Save named parameter configurations for quick switching.
- Example presets: "simulation", "real_robot", "debug", "competition".
- Per-node or global parameter overrides.
- Quick toggle between presets from toolbar.
- Export/import presets as YAML files.
- Diff view between presets.

## Node Library & Templates

### Node Templates Library
- Pre-configured templates for common ROS2 patterns:
  - **Sensor Fusion Node** - Multi-sensor input with synchronized output.
  - **State Machine** - Template with state transitions and events.
  - **Behavior Tree Executor** - BehaviorTree.CPP integration template.
  - **Bridge Node** - ROS1↔ROS2 bridge configuration.
  - **Lifecycle Node** - Managed node with state callbacks.
  - **Component Node** - Composable node template.
- Customizable templates with placeholder parameters.
- Community template sharing (future).

### Workspace Browser Panel
- Browse local ROS2 workspace packages.
- Tree view of installed packages organized by:
  - Workspace (overlay order).
  - Package type (nodes, libraries, messages).
- Drag-and-drop nodes from browser onto canvas.
- Quick search/filter within workspace.
- Show package dependencies and reverse dependencies.

## Comparison & Analysis

### Architecture Diff View
- Visually compare two project versions side-by-side.
- Highlighting for:
  - Added nodes/connections (green).
  - Removed nodes/connections (red).
  - Modified parameters (yellow).
- Git integration to compare with previous commits.
- Export diff report as HTML or markdown.
- Useful for code reviews and architecture evolution tracking.

## Prioritization Recommendation

### High Priority (Quick Wins)
1. Documentation lookup & tooltips (huge productivity boost).
2. Mini-map navigation (essential for large projects).
3. Canvas grid visualization with snap-to-grid.
4. Auto-layout algorithms (one-click organization).

### Medium Priority (High Value)
5. Connection bandwidth visualization (live monitoring).
6. Launch file generator (complements existing code gen).
7. QoS profile visual editor (prevents common issues).
8. Parameter presets (quick configuration switching).
9. Canvas annotations/sticky notes (team documentation).

### Lower Priority (Advanced Features)
10. Node templates library.
11. Workspace browser panel.
12. Architecture diff view.
13. Time-synced playback visualization.
14. Node health dashboard.

Feel free to open issues or pull requests for any of these — they'd make ROS2Weaver an even more powerful all-in-one ROS2 development environment!
