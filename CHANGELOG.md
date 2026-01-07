# Changelog

All notable changes to ROS2Weaver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - Unreleased

### Added

### Changed

### Fixed

## [1.1.0] - 2026-01-02

### Added

#### Multi-Canvas Tabs
- Added tabbed canvas interface for managing multiple compositions
- New Canvas menu with New (Ctrl+Shift+N), Close (Ctrl+W), Duplicate, and Rename (F2) actions
- Double-click tab bar to add new canvas or rename existing tabs
- Projects now save/load all canvas tabs with their contents
- Backward compatible with legacy single-canvas project files

#### Documentation & Discovery Features
- **Context-Sensitive ROS2 Documentation Lookup**: Hover or right-click on nodes, topics, services, actions, message types, or parameters to view official documentation
- **Message Schema Viewer**: Interactive schema tree for viewing message fields, types, and defaults (View > Panels > Schema Viewer)
- **Package README Preview**: Automatically load and display README.md or package.xml description for selected nodes/packages (View > Panels > Package Docs)

#### Export & Import
- **Export to PlantUML**: File > Export Diagram > Export to PlantUML
- **Export to Mermaid**: File > Export Diagram > Export to Mermaid
- **Export to Graphviz DOT**: File > Export Diagram > Export to Graphviz DOT
- **Import from rqt_graph DOT files**: File > Import > Import from rqt_graph DOT
- **Import from CARET architecture YAML**: File > Import > Import from CARET YAML

#### Simulation Integration
- **Simulation Launcher**: One-click launch of Gazebo or Ignition/Gazebo Sim (ROS2 > Simulation menu)
- Status feedback and stop simulation control

#### Validation & Debugging
- **Static Analyzer**: Detect topic type mismatches, unused publishers/subscribers, QoS incompatibilities (ROS2 > Run Static Analysis, Ctrl+Shift+A)
- **Issue List Panel**: View all analysis issues with severity, category, descriptions, and suggested fixes (View > Panels > Issues)
- Click issues to navigate to affected blocks on canvas
- "Ask AI" integration for help understanding and fixing issues
- **Live Parameter Validation Dashboard**: Real-time parameter monitoring and validation

#### AI Assistant Expansions
- **Natural Language Architecture Generation**: Generate architectures from text descriptions
- **Automated Error Fixing Suggestions**: AI-powered recommendations based on validation results
- **Architecture Optimization Recommendations**: AI suggestions for improving your ROS2 architecture

#### Usability & Polish
- **Theme Editor**: Create and customize color themes (Edit > Theme Editor)
- **Advanced Search Panel**: Filter packages by category, message types, features (Browser dock > Search tab)
- **Remapping Editor**: Visual editor for node namespace and topic remappings
- **Git Integration**: Git-aware project saving with awareness of repository state
- **Rosbag Playback Animation**: Canvas animation during rosbag replay

### Changed
- Zoom actions now properly reference current canvas in multi-tab environment
- Project save/load now supports multi-canvas serialization

### Fixed
- Fix deadlock in SlamPipelineManager::setParameter()
- Fix bugs found during code review
- Fix compiler warnings and add shutdown debugging
- Fix RosLogViewer level color comparison bug
- Fix startup delay caused by blocking ros2_control refresh
- Fix plot panel causing dock widget to resize on click

## [1.0.0] - 2025-12-27

### Added

#### UX Improvements (Phase 1)
- Toast notifications for non-blocking feedback
- Auto-save functionality with configurable intervals
- Command palette for quick access to all commands (Ctrl+Shift+P)
- Keyboard shortcuts dialog
- Zoom level indicator on canvas

#### MCP Integration
- MCP (Model Context Protocol) server framework
- Rosbag MCP server for bag file operations
- MCP Explorer panel for managing servers
- AI tools via MCP for canvas manipulation

#### Core Features
- Inline AI permission card for chat interactions
- Improved parameter validation UX
- Unified error handling system
- Theme colors applied across all widgets
- Unicode icons in output panel tabs
- Improved canvas connection drag visual feedback
- Improved log viewer visual hierarchy
- Improved status indicator visual design

### Fixed
- Various CI/CD workflow optimizations
- Signal tests disconnect before TearDown
- Lint job configuration improvements
