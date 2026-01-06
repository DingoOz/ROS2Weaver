#include "ros_weaver/widgets/help_browser.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QApplication>
#include <QDesktopServices>
#include <QRegularExpression>
#include <QScrollBar>

namespace ros_weaver {

HelpBrowser* HelpBrowser::instance_ = nullptr;

HelpBrowser* HelpBrowser::instance(QWidget* parent) {
  if (!instance_) {
    instance_ = new HelpBrowser(parent);
  }
  return instance_;
}

HelpBrowser::HelpBrowser(QWidget* parent)
  : QDialog(parent)
  , mainSplitter_(nullptr)
  , leftPanel_(nullptr)
  , leftStack_(nullptr)
  , tocTree_(nullptr)
  , searchResults_(nullptr)
  , searchEdit_(nullptr)
  , contentBrowser_(nullptr)
  , backButton_(nullptr)
  , forwardButton_(nullptr)
  , homeButton_(nullptr)
  , breadcrumbLabel_(nullptr)
  , historyIndex_(-1)
{
  setWindowTitle(tr("ROS Weaver Help"));
  setMinimumSize(900, 650);
  resize(1000, 700);

  setupUi();
  loadHelpContent();
  buildTableOfContents();

  // Show getting started by default
  showTopic("getting-started");
}

HelpBrowser::~HelpBrowser() {
  if (instance_ == this) {
    instance_ = nullptr;
  }
}

void HelpBrowser::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Navigation toolbar
  QHBoxLayout* navLayout = new QHBoxLayout();

  backButton_ = new QPushButton(tr("<"));
  backButton_->setToolTip(tr("Go Back"));
  backButton_->setFixedWidth(32);
  backButton_->setEnabled(false);
  connect(backButton_, &QPushButton::clicked, this, &HelpBrowser::onBackClicked);

  forwardButton_ = new QPushButton(tr(">"));
  forwardButton_->setToolTip(tr("Go Forward"));
  forwardButton_->setFixedWidth(32);
  forwardButton_->setEnabled(false);
  connect(forwardButton_, &QPushButton::clicked, this, &HelpBrowser::onForwardClicked);

  homeButton_ = new QPushButton(tr("Home"));
  homeButton_->setToolTip(tr("Go to Getting Started"));
  connect(homeButton_, &QPushButton::clicked, this, &HelpBrowser::onHomeClicked);

  breadcrumbLabel_ = new QLabel();
  breadcrumbLabel_->setStyleSheet("color: #666; font-size: 11px;");

  navLayout->addWidget(backButton_);
  navLayout->addWidget(forwardButton_);
  navLayout->addWidget(homeButton_);
  navLayout->addSpacing(16);
  navLayout->addWidget(breadcrumbLabel_, 1);
  mainLayout->addLayout(navLayout);

  // Main splitter
  mainSplitter_ = new QSplitter(Qt::Horizontal);

  // Left panel with TOC and search
  leftPanel_ = new QWidget();
  QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel_);
  leftLayout->setContentsMargins(0, 0, 0, 0);
  leftLayout->setSpacing(4);

  // Search box
  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search help..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged, this, &HelpBrowser::onSearchTextChanged);
  leftLayout->addWidget(searchEdit_);

  // Stacked widget for TOC / Search results
  leftStack_ = new QStackedWidget();

  // TOC tree
  tocTree_ = new QTreeWidget();
  tocTree_->setHeaderHidden(true);
  tocTree_->setIndentation(16);
  tocTree_->setAnimated(true);
  connect(tocTree_, &QTreeWidget::itemClicked, this, &HelpBrowser::onTocItemClicked);
  leftStack_->addWidget(tocTree_);

  // Search results list
  searchResults_ = new QListWidget();
  connect(searchResults_, &QListWidget::itemClicked, this, &HelpBrowser::onSearchResultClicked);
  leftStack_->addWidget(searchResults_);

  leftLayout->addWidget(leftStack_);
  leftPanel_->setMinimumWidth(200);
  leftPanel_->setMaximumWidth(300);

  // Content browser
  contentBrowser_ = new QTextBrowser();
  contentBrowser_->setOpenExternalLinks(false);
  contentBrowser_->setOpenLinks(false);
  connect(contentBrowser_, &QTextBrowser::anchorClicked, this, &HelpBrowser::onAnchorClicked);

  // Style the content browser
  contentBrowser_->setStyleSheet(R"(
    QTextBrowser {
      background-color: #fafafa;
      padding: 16px;
      font-size: 14px;
      line-height: 1.6;
    }
  )");

  mainSplitter_->addWidget(leftPanel_);
  mainSplitter_->addWidget(contentBrowser_);
  mainSplitter_->setStretchFactor(0, 0);
  mainSplitter_->setStretchFactor(1, 1);
  mainSplitter_->setSizes({250, 750});

  mainLayout->addWidget(mainSplitter_, 1);

  // Close button
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  QPushButton* closeButton = new QPushButton(tr("Close"));
  connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton);
  mainLayout->addLayout(buttonLayout);
}

void HelpBrowser::loadHelpContent() {
  // Getting Started
  topics_["getting-started"] = {
    "getting-started",
    tr("Getting Started"),
    tr(R"(# Getting Started with ROS Weaver

Welcome to ROS Weaver, a visual tool for assembling, extending, and iterating on ROS2 packages and nodes.

## Quick Start

1. **Create or Open a Project**
   - Use **File > New Project** to start fresh
   - Use **File > Open Project** to load an existing .weaver project
   - Try **File > Examples** to load sample projects

2. **Add Packages to Canvas**
   - Search for packages in the Package Browser (left panel)
   - Double-click a package to add it to the canvas
   - Drag packages around to organize your workspace

3. **Create Connections**
   - Click and drag from an output pin (right side) to an input pin (left side)
   - Connections represent topic subscriptions/publications

4. **Configure Parameters**
   - Select a block to view its parameters in the Properties panel
   - Edit parameters to customize node behavior

5. **Generate Code**
   - Use **File > Generate ROS2 Package** to create buildable code
   - Or use the Wizard for step-by-step code generation

## Interface Overview

- **Canvas** (center): Visual workspace for designing your ROS2 system
- **Package Browser** (left): Search and add ROS2 packages
- **Properties** (right): Configure parameters, view topics, TF tree
- **Output** (bottom): Build output, ROS logs, terminal

## Keyboard Shortcuts

Press **Ctrl+/** to see all keyboard shortcuts.

## Need More Help?

- Browse topics in the left panel
- Use the search box to find specific information
- Press **F1** anywhere for context-sensitive help
)"),
    "",
    {"start", "quick", "introduction", "overview", "basics"}
  };
  topicOrder_ << "getting-started";

  // Canvas Editor
  topics_["canvas-editor"] = {
    "canvas-editor",
    tr("Canvas Editor"),
    tr(R"(# Canvas Editor

The Canvas Editor is the central workspace where you design your ROS2 system visually.

## Navigation

- **Pan**: Middle-click and drag, or hold Space + left-click drag
- **Zoom**: Mouse wheel, or use View menu zoom options
- **Fit All**: Press **F** to fit all nodes in view
- **Reset Zoom**: Press **Ctrl+0** to reset zoom level

## Working with Blocks

### Adding Blocks
- Double-click a package in the Package Browser
- Blocks appear at the center of your current view

### Selecting Blocks
- Click to select a single block
- **Ctrl+Click** to add/remove from selection
- **Ctrl+A** to select all blocks

### Moving Blocks
- Drag selected blocks to move them
- Multiple selected blocks move together

### Deleting Blocks
- Select blocks and press **Delete**
- Or use **Edit > Delete**

## Creating Connections

1. Click on an output pin (right side of a block)
2. Drag to an input pin (left side of another block)
3. Release to create the connection

Connections represent topic subscriptions between nodes.

### Connection Colors
- Green animated lines indicate active data flow
- Gray lines indicate inactive connections

## Node Groups

Create comment boxes to organize related nodes:

1. Select multiple blocks
2. Right-click and choose "Group Selected"
3. Or use keyboard shortcut to create group

Groups can be:
- Renamed by double-clicking the title
- Resized by dragging corners
- Collapsed to save space
)"),
    "",
    {"canvas", "editor", "blocks", "nodes", "connections", "navigation", "pan", "zoom"}
  };
  topicOrder_ << "canvas-editor";

  // Package Browser
  topics_["package-browser"] = {
    "package-browser",
    tr("Package Browser"),
    tr(R"(# Package Browser

The Package Browser panel lets you search and add ROS2 packages to your canvas.

## Searching Packages

1. Type your search query in the search box
2. Press Enter or click Search
3. Results appear in the tree below

### Search Tips
- Search by package name: `turtlesim`
- Search by functionality: `navigation`
- Search by message type: `sensor_msgs`

## Adding Packages

- **Double-click** a package to add it to the canvas
- The block appears at the center of your current view

## Local Packages

The "Local Packages" section shows packages from your current workspace:
- Packages you've created
- Packages installed in your ROS2 environment

## Package Information

Hover over a package to see:
- Package description
- Maintainer information
- License type
)"),
    "",
    {"packages", "search", "browser", "add", "ros2"}
  };
  topicOrder_ << "package-browser";

  // Properties Panel
  topics_["properties-panel"] = {
    "properties-panel",
    tr("Properties Panel"),
    tr(R"(# Properties Panel

The Properties Panel on the right side shows details about the selected block and provides access to ROS2 system information.

## Tabs

### Parameters
Shows editable parameters for the selected node:
- Parameter name
- Current value
- Parameter type
- Description (if available)

Edit parameters by:
- Double-clicking the value
- Using spin boxes for numeric values
- Checkboxes for boolean values

### Topics
Lists all ROS2 topics in the system:
- Topic name
- Message type
- Publishers count
- Subscribers count

Click a topic to see more details.

### TF Tree
Displays the transform tree:
- Frame hierarchy
- Transform rates
- Frame status

Press **Ctrl+T** to quickly access the TF Tree.

## No Selection

When no block is selected, the Properties panel shows general project information.
)"),
    "",
    {"properties", "parameters", "topics", "tf", "transform", "settings"}
  };
  topicOrder_ << "properties-panel";

  // Code Generation
  topics_["code-generation"] = {
    "code-generation",
    tr("Code Generation"),
    tr(R"(# Code Generation

ROS Weaver can generate complete ROS2 packages from your visual design.

## Quick Generation

**File > Generate ROS2 Package** (Ctrl+G)

Generates a package with default settings to your chosen directory.

## Generation Wizard

**File > Generate ROS2 Package (Wizard)** (Ctrl+Shift+G)

Step-by-step wizard offering full customization:

### Step 1: Package Info
- Package name
- Description
- Version
- License
- Maintainer details

### Step 2: Build Options
- C++ or Python nodes
- Dependencies
- Build type

### Step 3: Output Location
- Directory selection
- File preview
- Overwrite options

## Generated Files

The generator creates:
- `package.xml` - Package manifest
- `CMakeLists.txt` - Build configuration
- `src/` - Node source files
- `launch/` - Launch files
- `config/` - Parameter YAML files

## Building Generated Code

After generation:
```bash
cd ~/your_ws
colcon build --packages-select your_package
source install/setup.bash
```
)"),
    "",
    {"generate", "code", "package", "build", "wizard", "cmake", "python", "cpp"}
  };
  topicOrder_ << "code-generation";

  // ROS2 Integration
  topics_["ros2-integration"] = {
    "ros2-integration",
    tr("ROS2 Integration"),
    tr(R"(# ROS2 Integration

ROS Weaver integrates deeply with your ROS2 environment.

## System Scanning

**ROS2 > Scan Running System** (Ctrl+Shift+R)

Discovers:
- Running nodes
- Active topics
- Published/subscribed connections
- TF frames

### Auto-Scan
Enable **ROS2 > Auto-Scan** to continuously monitor the system.

## System Mapping Panel

**ROS2 > Show System Mapping Panel** (Ctrl+Shift+M)

Compares your canvas with the running system:
- **Matched**: Blocks that exist in both
- **Missing from System**: Canvas blocks not running
- **Extra in System**: Running nodes not on canvas

## Topic Viewer

**ROS2 > Show Topic Viewer** (Ctrl+Shift+T)

Real-time topic monitoring:
- Subscribe to topics
- View message data
- Monitor publish rates
- Echo messages

## TF Tree

**ROS2 > Show TF Tree** (Ctrl+T)

Visualize transform frames:
- Hierarchical tree view
- Frame relationships
- Transform rates
- Static vs dynamic transforms

## ROS Logs

**ROS2 > Show ROS Logs** (Ctrl+Shift+L)

View rosout messages:
- Filter by severity (Debug, Info, Warn, Error, Fatal)
- Filter by node
- Search log content
- Analyze errors with AI
)"),
    "",
    {"ros2", "scan", "system", "topics", "tf", "logs", "nodes", "discovery"}
  };
  topicOrder_ << "ros2-integration";

  // Keyboard Shortcuts
  topics_["keyboard-shortcuts"] = {
    "keyboard-shortcuts",
    tr("Keyboard Shortcuts"),
    tr(R"(# Keyboard Shortcuts

## File Operations
| Shortcut | Action |
|----------|--------|
| Ctrl+N | New Project |
| Ctrl+O | Open Project |
| Ctrl+S | Save Project |
| Ctrl+Shift+S | Save Project As |
| Ctrl+G | Generate ROS2 Package |
| Ctrl+Shift+G | Generate (Wizard) |
| Ctrl+Shift+W | Robot Configuration Wizard |
| Ctrl+Q | Exit |

## Edit
| Shortcut | Action |
|----------|--------|
| Ctrl+Z | Undo |
| Ctrl+Y | Redo |
| Delete | Delete Selected |
| Ctrl+A | Select All |
| Ctrl+, | Settings |

## View
| Shortcut | Action |
|----------|--------|
| Ctrl++ | Zoom In |
| Ctrl+- | Zoom Out |
| Ctrl+0 | Reset Zoom |
| F | Fit All Nodes |

## ROS2
| Shortcut | Action |
|----------|--------|
| Ctrl+B | Build Workspace |
| Ctrl+L | Launch |
| Ctrl+Shift+R | Scan Running System |
| Ctrl+Shift+M | System Mapping Panel |
| Ctrl+Shift+T | Topic Viewer |
| Ctrl+T | TF Tree |
| Ctrl+Shift+L | ROS Logs |
| Ctrl+Shift+V | Launch RViz2 |

## External Tools
| Shortcut | Action |
|----------|--------|
| Ctrl+Shift+E | Open Project Folder in VS Code |

## Help
| Shortcut | Action |
|----------|--------|
| F1 | Context Help (for focused widget) |
| Ctrl+F1 | Getting Started |
| Ctrl+/ | Keyboard Shortcuts |

## Canvas Navigation
| Action | Control |
|--------|---------|
| Pan | Middle-click drag or Space+drag |
| Zoom | Mouse wheel |
| Select | Click |
| Multi-select | Ctrl+Click |
| Deselect | Click empty area |

## Guided Tour
| Key | Action |
|-----|--------|
| Right Arrow / Enter | Next step |
| Left Arrow | Previous step |
| Escape | Skip/Exit tour |
)"),
    "",
    {"shortcuts", "keyboard", "hotkeys", "keys", "commands"}
  };
  topicOrder_ << "keyboard-shortcuts";

  // Project Management
  topics_["project-management"] = {
    "project-management",
    tr("Project Management"),
    tr(R"(# Project Management

## Creating Projects

**File > New Project** (Ctrl+N)

Creates a blank canvas ready for your design.

## Saving Projects

**File > Save Project** (Ctrl+S)

Saves to `.weaver` format containing:
- Block positions and configurations
- Connections between nodes
- Parameter values
- Node group information

**File > Save Project As** (Ctrl+Shift+S)

Save to a new location or filename.

## Opening Projects

**File > Open Project** (Ctrl+O)

Load a previously saved `.weaver` project file.

## Example Projects

**File > Examples**

Pre-built example projects to learn from:

### TurtleBot3 Navigation
Complete navigation setup with:
- TurtleBot3 robot
- Navigation stack
- Localization

### Turtlesim Teleop
Simple example with:
- Turtlesim node
- Teleop keyboard control
- Demonstrates basic pub/sub

### Launch Examples
- **Launch Turtlesim Nodes**: Start the actual nodes
- **Launch TurtleBot3 Gazebo**: Full simulation

## Project Files

The `.weaver` format is YAML-based and human-readable:
```yaml
blocks:
  - id: "uuid"
    package: "turtlesim"
    position: [100, 200]
    parameters:
      background_r: 69
connections:
  - from_block: "uuid1"
    to_block: "uuid2"
    topic: "/turtle1/cmd_vel"
```
)"),
    "",
    {"project", "save", "open", "new", "examples", "weaver", "file"}
  };
  topicOrder_ << "project-management";

  // Troubleshooting
  topics_["troubleshooting"] = {
    "troubleshooting",
    tr("Troubleshooting"),
    tr(R"(# Troubleshooting

## Common Issues

### Nodes Not Appearing in Scan
- Ensure ROS2 environment is sourced
- Check that nodes are actually running: `ros2 node list`
- Verify network configuration for multi-machine setups

### Topic Data Not Showing
- Check topic exists: `ros2 topic list`
- Verify message type compatibility
- Ensure QoS settings are compatible

### Code Generation Fails
- Check output directory permissions
- Ensure valid package name (lowercase, underscores)
- Verify all required fields are filled

### Canvas Performance Issues
- Reduce number of visible nodes
- Disable auto-scan if not needed
- Close unused panels

### TF Tree Empty
- Ensure TF publishers are running
- Check for TF namespace issues
- Verify frame names

## Getting More Help

### Report Issues
Visit our GitHub repository to:
- Report bugs
- Request features
- View known issues

### ROS2 Documentation
For ROS2-specific questions:
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS Answers](https://answers.ros.org/)

### Local AI Assistant
Use the integrated AI assistant (if configured):
- Ask debugging questions
- Get code suggestions
- Analyze ROS logs
)"),
    "",
    {"troubleshooting", "problems", "issues", "bugs", "help", "fix", "error"}
  };
  topicOrder_ << "troubleshooting";

  // What's New
  topics_["whats-new"] = {
    "whats-new",
    tr("What's New"),
    tr(R"(# What's New in ROS Weaver

## Version 1.1.0 - Latest

### Multi-Canvas Tabs
- Open multiple canvas tabs for complex projects
- Drag and drop between canvases
- Rename and duplicate canvases
- Quick canvas switching with keyboard shortcuts

### Node Health Dashboard
- Real-time monitoring of node health metrics
- Track CPU usage, memory, latency, and message rates
- Configurable health thresholds (Healthy, Warning, Critical)
- Heatmap visualization to color canvas nodes by resource usage
- System health summary with overall status

### Workspace Browser Panel
- Browse and navigate ROS2 workspace packages
- Quick file preview and package information
- Drag packages directly onto canvas
- Search and filter workspace contents

### Node Templates Library
- Pre-built templates for common node patterns
- Publisher, subscriber, service, and action templates
- Custom template creation and management
- Quick instantiation with configurable parameters

### Architecture Diff View
- Compare project versions side-by-side
- Highlight added, removed, and modified nodes
- Track connection changes between versions
- Visual diff overlay on canvas

### Time-synced Playback Visualization
- Synchronized rosbag playback with canvas highlighting
- Visual indicators showing active topics during replay
- Playback progress bar with seek functionality
- Integration with Rosbag Workbench panel

### Rosbag Workbench
- Record, playback, and inspect rosbag files
- Integrated playback controls
- Message inspection and topic filtering
- Canvas synchronization during playback

### Canvas Annotations
- Add text annotations and notes to canvas
- Customizable colors and styles
- Documentation directly on your architecture

### Mini-map Navigation
- Bird's-eye view of large canvases
- Quick navigation to any area
- Viewport indicator

## Previous Updates

### Guided Tour
- Interactive walkthrough of the application
- Spotlight highlighting of UI elements
- Keyboard navigation support

### Data Lineage
- Track where data originates from
- View source files and line numbers
- Jump directly to VS Code for editing

### Robot Configuration Wizard
- Quick setup for common robot platforms
- TurtleBot3 support with ros2_control
- Navigation stack configuration

### Local AI Assistant
- Native tool calling via Ollama
- AI-powered canvas control
- Improved model recommendations

## Getting Help

- Press **F1** for context-sensitive help
- Use **Help > Guided Tour** for an introduction
- Check **Help > Video Tutorials** for online resources
- Report issues via **Help > Report Issue**
)"),
    "",
    {"new", "changelog", "version", "release", "updates", "features"}
  };
  topicOrder_ << "whats-new";

  // Local AI
  topics_["local-ai"] = {
    "local-ai",
    tr("Local AI Assistant"),
    tr(R"(# Local AI Assistant

ROS Weaver includes an optional local AI assistant powered by Ollama with native tool calling support.

## Setup

### Installing Ollama
1. Visit [ollama.com](https://ollama.com)
2. Download and install for your platform
3. Start Ollama service

### Recommended Models for Tool Calling
- **llama3.1:8b** - Excellent tool calling support
- **qwen2.5:7b** - Good all-rounder with tools
- **qwen2.5-coder:7b** - Best for coding tasks
- **mistral:7b** - Reliable tool support

## Using the AI Assistant

### Accessing the Chat
Open the AI chat panel in the Output area (bottom of window).

### Features
- Ask ROS2 questions
- Get debugging help
- Generate code snippets
- Analyze error logs
- **Control the canvas directly** (see AI Tools below)

### Example Prompts
- "How do I create a ROS2 publisher?"
- "What's wrong with this TF tree?"
- "Generate a Python subscriber for /cmd_vel"
- "Explain this error message"

## Settings

Access AI settings in **Edit > Settings**:
- Ollama endpoint URL
- Model selection
- System prompt customization
- Hardware detection

## Privacy

All AI processing happens locally on your machine:
- No data sent to cloud services
- Full control over your data
- Works offline after model download
)"),
    "",
    {"ai", "llm", "ollama", "assistant", "chat", "local"}
  };
  topicOrder_ << "local-ai";

  // AI Tools
  topics_["ai-tools"] = {
    "ai-tools",
    tr("AI Canvas Tools"),
    tr(R"(# AI Canvas Tools

The AI assistant can control the ROS Weaver canvas using native tool calling. Simply describe what you want in natural language.

## Available Tools

### load_example
Load example projects onto the canvas.

**Example prompts:**
- "Load the turtlesim example"
- "Show me the TurtleBot3 navigation example"
- "Open a demo project"

**Available examples:**
- `turtlesim_teleop` - Basic publisher/subscriber with turtlesim
- `turtlebot3_navigation` - Navigation with SLAM and Nav2

### add_block
Add ROS2 nodes to the canvas at a specified position.

**Example prompts:**
- "Add a turtlesim node"
- "Put a teleop_twist_keyboard node at position 300, 200"
- "Add the navigation stack nodes"

**Parameters:**
- package: ROS2 package name
- executable: Node executable name
- x, y: Position on canvas (optional)

### remove_block
Remove a node from the canvas by name.

**Example prompts:**
- "Remove the teleop node"
- "Delete turtlesim_node"
- "Take off the navigation node"

### set_parameter
Modify parameters on a node.

**Example prompts:**
- "Set the background color to blue"
- "Change the linear velocity to 2.0"
- "Update the robot name parameter"

**Parameters:**
- block_name: Target node name
- param_name: Parameter to change
- value: New value

### create_connection
Create topic connections between nodes.

**Example prompts:**
- "Connect the teleop to the turtlesim"
- "Wire up cmd_vel between these nodes"
- "Link the publisher to the subscriber"

**Parameters:**
- from_block: Source node name
- from_pin: Output pin name
- to_block: Target node name
- to_pin: Input pin name

### remove_connection
Remove topic connections.

**Example prompts:**
- "Disconnect the cmd_vel topic"
- "Remove the connection between teleop and turtle"

### create_group
Create a visual group around nodes.

**Example prompts:**
- "Group the navigation nodes together"
- "Create a group called 'Sensors'"

**Parameters:**
- title: Group name
- block_names: List of nodes to include

### get_project_state
Query the current canvas contents.

**Example prompts:**
- "What's on the canvas?"
- "Show me the current project state"
- "List all nodes and connections"

### get_block_info
Get detailed information about a specific node.

**Example prompts:**
- "Tell me about the turtlesim node"
- "What parameters does teleop have?"
- "Show details for the navigation node"

### list_available_packages
List ROS2 packages available to add.

**Example prompts:**
- "What packages can I use?"
- "List available ROS2 packages"
- "Show me navigation packages"

## Using AI Tools

### Natural Language
Just describe what you want:
- "Set up a basic turtlesim demo"
- "Add a camera node and connect it to the image processor"
- "Clean up the canvas and organize the nodes"

### Undo Support
AI actions can be undone:
- Click the **Undo** button that appears after actions
- The AI maintains an action history

### Permission Requests
Some actions require permission:
- A dialog will appear asking for confirmation
- You can approve individual actions or approve all for the session

## Tips

### Be Specific
- "Add turtlesim/turtlesim_node" is clearer than "add a turtle"
- Include positions if you have a preference

### Chain Commands
- "Load the turtlesim example and then add a second turtle node"
- The AI can handle multi-step requests

### Ask Questions First
- "What's on the canvas?" before making changes
- "What nodes are available in turtlesim?"

## Troubleshooting

### Tool Not Working
- Ensure you're using a model that supports tool calling
- Check that Ollama is running and connected
- Try rephrasing your request

### Permission Denied
- Some actions require explicit approval
- Check the permission dialog settings

### Unexpected Results
- Use "What's on the canvas?" to verify state
- Use the Undo button to revert changes
)"),
    "",
    {"ai", "tools", "canvas", "control", "commands", "automation"}
  };
  topicOrder_ << "ai-tools";

  // Plot Panel
  topics_["plot-panel"] = {
    "plot-panel",
    tr("Plot Panel"),
    tr(R"(# Plot Panel

The Plot Panel visualizes time-series data from ROS2 topics.

## Adding Topics to Plot

1. Click the "+" button in the plot panel
2. Select a topic from the list
3. Choose the field to plot (for complex messages)

## Controls

### Time Window
Select the time range to display:
- 10 seconds
- 30 seconds
- 60 seconds
- 5 minutes

### Play/Pause
- Pause to freeze the display
- Resume to continue live updates

### Export
- Save data to CSV
- Screenshot the plot as PNG

## Plot Features

### Multiple Series
Add multiple topics to the same plot with different colors.

### Legend
Shows all plotted series with current values.

### Statistics
View min, max, mean, and standard deviation.

### Zoom
Use mouse wheel to zoom in/out on the plot area.

## Supported Message Types

- Float32, Float64, Int32, etc.
- Complex messages with numeric fields
- Array elements (e.g., position[0])
)"),
    "",
    {"plot", "chart", "graph", "data", "visualization", "time series"}
  };
  topicOrder_ << "plot-panel";

  // Guided Tour
  topics_["guided-tour"] = {
    "guided-tour",
    tr("Guided Tour"),
    tr(R"(# Guided Tour

ROS Weaver includes an interactive guided tour to help you learn the application.

## Starting the Tour

**Help > Guided Tour** (or from the Welcome screen)

The tour walks you through each major component of the interface:

1. **Welcome** - Introduction to ROS Weaver
2. **Menu Bar** - Access all application functions
3. **Main Toolbar** - Quick access to common actions
4. **Visual Canvas** - Where you design your ROS2 system
5. **Package Browser** - Search and add packages
6. **Properties Panel** - Configure node parameters
7. **Output Panel** - Build output, logs, and terminal
8. **System Mapping** - Discover running ROS2 nodes
9. **Status Bar** - Connection status and feedback
10. **AI Assistant** - Chat with the local LLM
11. **Keyboard Shortcuts** - Tips for faster workflow

## Navigation

- **Next/Previous** buttons to move between steps
- **Skip** to exit the tour at any time
- **Escape** key to close the tour
- Arrow keys (Left/Right) to navigate

## Spotlight Feature

The tour uses a spotlight effect to highlight the current UI element:
- A red border highlights the focused area
- The rest of the screen is dimmed
- Click outside the highlight to skip the tour

## Restart the Tour

You can restart the tour anytime from **Help > Guided Tour**.
)"),
    "",
    {"tour", "tutorial", "introduction", "learn", "walkthrough", "guide"}
  };
  topicOrder_ << "guided-tour";

  // Data Lineage
  topics_["data-lineage"] = {
    "data-lineage",
    tr("Data Lineage"),
    tr(R"(# Data Lineage

ROS Weaver tracks where data comes from, helping you understand and debug your system.

## What is Data Lineage?

Data lineage shows the origin and transformation chain of any value in the system:
- Where a parameter value was defined
- Which file contains a configuration
- How data flows through your ROS2 system

## Viewing Data Origin

Right-click on parameters, topics, or other data to see:
- **Show Data Origin** - Opens the lineage dialog

The lineage dialog shows:
- **Primary Source** - Where the data originally came from
- **Derived From** - Intermediate sources in the chain
- File paths with line numbers
- Timestamps when available

## Source Types

Data can originate from:
- **Project File** - Your .weaver project file
- **YAML File** - Configuration and parameter files
- **Source Code** - C++ or Python files
- **ROS Topic** - Live data from the ROS2 system
- **ROS Parameter** - Parameter server values
- **User Input** - Values you entered manually
- **System Discovery** - Automatically detected nodes/topics
- **Generated** - Code generated by ROS Weaver

## VS Code Integration

From the lineage dialog:
- Click **Open in VS Code** to jump directly to the source
- Double-click any file path to open it
- **Copy Path** to copy the file location

This makes it easy to find and edit the original configuration.
)"),
    "",
    {"lineage", "origin", "source", "debug", "trace", "provenance"}
  };
  topicOrder_ << "data-lineage";

  // VS Code Integration
  topics_["vscode-integration"] = {
    "vscode-integration",
    tr("VS Code Integration"),
    tr(R"(# VS Code Integration

ROS Weaver integrates with Visual Studio Code for seamless code editing.

## Opening in VS Code

### Project Folder
**File > Open Project Folder in VS Code** (Ctrl+Shift+E)

Opens your entire project directory in VS Code.

### Generated Package
**File > Open Generated Package in VS Code**

After code generation, quickly open the generated ROS2 package.

### From Data Lineage
When viewing data origin, click **Open in VS Code** to jump directly to the source file and line number.

## Supported Editors

ROS Weaver detects these editors in order:
1. **code** - VS Code
2. **code-insiders** - VS Code Insiders
3. **codium** - VSCodium

## File Navigation

When opening files with line numbers, VS Code will:
- Open the file
- Scroll to the specified line
- Place the cursor at that location

## Requirements

- VS Code (or compatible) must be installed
- The `code` command must be in your PATH
- For most installations, this works automatically

## Tips

- Use Ctrl+Shift+E to quickly jump between ROS Weaver and VS Code
- The lineage feature helps you find where to make changes
- Generated code includes comments linking back to ROS Weaver
)"),
    "",
    {"vscode", "editor", "code", "visual studio", "ide", "edit"}
  };
  topicOrder_ << "vscode-integration";

  // Robot Configuration Wizard
  topics_["robot-wizard"] = {
    "robot-wizard",
    tr("Robot Configuration Wizard"),
    tr(R"(# Robot Configuration Wizard

The Robot Configuration Wizard helps you set up common robot configurations quickly.

## Starting the Wizard

**File > Robot Configuration Wizard** (Ctrl+Shift+W)

## Wizard Steps

### Step 1: Robot Type
Select your robot platform:
- TurtleBot3 (Burger, Waffle, Waffle Pi)
- Custom robot
- Generic differential drive
- Ackermann steering

### Step 2: Sensors
Configure sensors for your robot:
- LIDAR (2D/3D)
- Camera (RGB, Depth, RGBD)
- IMU
- Odometry sources

### Step 3: Navigation Stack
Choose navigation components:
- Nav2 (ROS2 Navigation)
- SLAM (mapping)
- Localization (AMCL, etc.)
- Path planning

### Step 4: Review
Review your configuration before generating.

## Generated Output

The wizard generates:
- Launch files for your robot
- Parameter YAML files
- URDF/Xacro modifications (if needed)
- Navigation configuration

## When to Use

The wizard is ideal when:
- Setting up a new robot project
- Adding navigation to an existing robot
- Learning ROS2 navigation concepts
- Creating a simulation environment
)"),
    "",
    {"robot", "wizard", "configuration", "setup", "turtlebot", "navigation"}
  };
  topicOrder_ << "robot-wizard";

  // Node Health Dashboard
  topics_["node-health-dashboard"] = {
    "node-health-dashboard",
    tr("Node Health Dashboard"),
    tr(R"(# Node Health Dashboard

The Node Health Dashboard provides real-time monitoring of ROS2 node health metrics.

## Opening the Dashboard

**View > Panels > Node Health Dashboard**

Or use the dockable panel on the right side.

## Features

### Health Metrics
The dashboard tracks key performance indicators for each node:
- **CPU Usage** - Percentage of CPU time consumed
- **Memory** - Memory usage in MB
- **Callback Latency** - Average callback execution time in ms
- **Dropped Messages** - Count of lost messages
- **Message Rate** - Publishing frequency in Hz

### Health Status Levels
Nodes are classified into three status levels:
- **Healthy** (Green) - All metrics within normal thresholds
- **Warning** (Orange) - Some metrics approaching limits
- **Critical** (Red) - Metrics exceeding safe thresholds

### System Health Summary
The top of the dashboard shows:
- Count of healthy, warning, and critical nodes
- Overall system health percentage bar
- Status color changes based on worst condition

## Using the Dashboard

### Syncing Nodes
1. Click **Sync Nodes** to populate the node list from canvas
2. The dashboard automatically discovers all package blocks
3. Node names are prefixed with "/" for ROS2 compatibility

### Start Monitoring
1. Click **Start Monitoring** to begin collecting metrics
2. The table updates in real-time with health data
3. Click **Stop Monitoring** to pause data collection

### Update Interval
Adjust the monitoring frequency:
- Use the spin box to set interval (100ms - 10000ms)
- Lower values provide more responsive updates
- Higher values reduce CPU overhead

## Heatmap Visualization

### Enabling Heatmap
Check the **Heatmap** checkbox to color canvas nodes based on health.

### Heatmap Modes
Select the visualization mode:
- **CPU Usage** - Color by CPU percentage (green to red)
- **Memory Usage** - Color by memory consumption
- **Health Status** - Color by overall health status

The canvas nodes display a semi-transparent overlay indicating their health.

## Thresholds

Default thresholds (configurable):
- **CPU Warning**: 70%, Critical: 90%
- **Memory Warning**: 500MB, Critical: 1000MB
- **Latency Warning**: 50ms, Critical: 100ms
- **Dropped Messages Warning**: 10, Critical: 100

## Tips

- Double-click a node row to view its history
- Use heatmap mode to quickly identify problem nodes
- Monitor during peak load to find bottlenecks
)"),
    "",
    {"health", "monitoring", "cpu", "memory", "metrics", "performance", "dashboard"}
  };
  topicOrder_ << "node-health-dashboard";

  // Workspace Browser
  topics_["workspace-browser"] = {
    "workspace-browser",
    tr("Workspace Browser"),
    tr(R"(# Workspace Browser

The Workspace Browser panel lets you explore and navigate your ROS2 workspace.

## Opening the Browser

**View > Panels > Workspace Browser**

## Features

### Directory Tree
Browse the complete structure of your ROS2 workspace:
- Source packages (`src/`)
- Build artifacts (`build/`)
- Install files (`install/`)
- Colcon workspace files

### Package Information
Click on a package to see:
- Package name and description
- Dependencies (build, exec, test)
- Maintainer information
- License type
- Version number

### File Preview
Select files to preview their contents:
- Source code with syntax highlighting
- Launch files
- Configuration YAML files
- CMakeLists.txt and package.xml

## Using the Browser

### Navigating
- Expand folders by clicking the arrow
- Double-click to open files in the preview pane
- Right-click for context menu options

### Adding to Canvas
- Drag a package folder onto the canvas
- The package block is created automatically
- Pins are populated from package interfaces

### Search and Filter
- Use the search box to filter files by name
- Filter by file type (*.cpp, *.py, *.launch.py)
- Quick navigation to specific packages

### Open in External Editor
Right-click a file and select:
- **Open in VS Code** - Edit in Visual Studio Code
- **Open in File Manager** - Browse in system file manager
- **Copy Path** - Copy file path to clipboard

## Workspace Types

The browser supports:
- **Colcon Workspaces** - Standard ROS2 workspace layout
- **Catkin Workspaces** - Legacy ROS1 workspaces (read-only)
- **Mixed Workspaces** - Overlayed workspace configurations

## Tips

- Pin frequently accessed packages for quick access
- Use the refresh button after building to see new files
- Drag multiple packages to canvas for batch import
)"),
    "",
    {"workspace", "browser", "files", "packages", "navigate", "explore"}
  };
  topicOrder_ << "workspace-browser";

  // Node Templates
  topics_["node-templates"] = {
    "node-templates",
    tr("Node Templates Library"),
    tr(R"(# Node Templates Library

The Node Templates Library provides pre-built node patterns for common ROS2 tasks.

## Opening the Library

**View > Panels > Node Templates**

## Built-in Templates

### Publisher Node
Creates a node that publishes messages:
- Configurable topic name and type
- Adjustable publish rate
- Sample message content

### Subscriber Node
Creates a node that subscribes to messages:
- Topic name and type selection
- Callback function template
- Message processing example

### Service Server
Creates a ROS2 service server:
- Service type selection
- Request/response handlers
- Error handling template

### Service Client
Creates a ROS2 service client:
- Async and sync call patterns
- Timeout handling
- Response processing

### Action Server
Creates a ROS2 action server:
- Goal handling
- Feedback publishing
- Result completion
- Cancellation support

### Action Client
Creates a ROS2 action client:
- Goal sending
- Feedback subscription
- Result handling

### Timer Node
Creates a node with periodic callbacks:
- Configurable timer period
- One-shot and repeating modes

### Lifecycle Node
Creates a managed lifecycle node:
- Configure, activate, deactivate states
- Transition callbacks
- State machine template

## Using Templates

### Instantiate a Template
1. Find the template in the library panel
2. Drag it onto the canvas
3. Configure the parameters in the dialog
4. Click **Create** to add the node

### Template Parameters
Each template has configurable options:
- Node name
- Topic/service/action names
- Message types
- Timer periods
- Namespace

### Preview Code
Click **Preview** to see the generated code before creating.

## Custom Templates

### Creating Custom Templates
1. Click **New Template** in the library
2. Define the template structure
3. Add configurable parameters
4. Save to your template library

### Template Format
Custom templates use a YAML format:
```yaml
name: My Custom Node
category: custom
parameters:
  - name: topic_name
    type: string
    default: "/my_topic"
code_template: |
  class MyNode : public rclcpp::Node { ... }
```

### Sharing Templates
- Export templates to share with team
- Import templates from files
- Templates are stored in ~/.ros_weaver/templates/

## Tips

- Use templates to ensure consistent code patterns
- Customize templates for your project's coding style
- Create templates for frequently used node types
)"),
    "",
    {"templates", "library", "publisher", "subscriber", "service", "action", "patterns"}
  };
  topicOrder_ << "node-templates";

  // Rosbag Workbench
  topics_["rosbag-workbench"] = {
    "rosbag-workbench",
    tr("Rosbag Workbench"),
    tr(R"(# Rosbag Workbench

The Rosbag Workbench provides tools for recording, playing, and analyzing rosbag files.

## Opening the Workbench

**View > Panels > Rosbag Workbench**

## Recording

### Start Recording
1. Click **Record** in the toolbar
2. Select topics to record (or all topics)
3. Choose output file location
4. Click **Start** to begin recording

### Recording Options
- **All Topics** - Record everything published
- **Selected Topics** - Choose specific topics
- **Split Size** - Split bags at specified size
- **Duration** - Stop after specified time

### Stop Recording
Click **Stop** to end recording. The bag file is saved automatically.

## Playback

### Loading a Bag
1. Click **Open** to browse for a bag file
2. Or drag a .db3 file onto the workbench

### Playback Controls
- **Play/Pause** - Start or pause playback
- **Stop** - Reset to beginning
- **Seek** - Drag the timeline to any position
- **Rate** - Adjust playback speed (0.1x to 10x)
- **Loop** - Continuously repeat playback

### Time-synced Visualization
During playback, the canvas shows:
- Active topics highlighted
- Data flow animations
- Connection activity indicators

## Inspection

### Message Browser
- View all topics in the bag
- See message counts and types
- Preview message content

### Topic Details
Select a topic to see:
- Message type and definition
- First/last timestamps
- Average publish rate
- Total message count

### Message Viewer
Double-click a topic to:
- Browse individual messages
- View timestamps
- Inspect field values
- Export to CSV

## Filtering

### Topic Filter
- Filter topics by name pattern
- Show/hide specific message types
- Focus on relevant data

### Time Range
- Set start and end times
- Extract portions of the bag
- Skip irrelevant sections

## Export

### Export Options
- **New Bag** - Export filtered data to new bag
- **CSV** - Export topic data to spreadsheet
- **MCAP** - Convert to MCAP format

## Integration with Canvas

### Canvas Sync
When playing a bag:
1. Enable **Sync with Canvas** option
2. Nodes publishing recorded topics are highlighted
3. Connection lines animate with playback
4. Time indicator shows current position

### Jump to Topic
Right-click a connection on canvas to:
- View topic in bag (if present)
- Seek to first message
- Show topic statistics

## Tips

- Use rate control to slow down fast topics
- Filter topics before export to reduce file size
- Enable canvas sync for visual debugging
- Use time range selection for large bags
)"),
    "",
    {"rosbag", "bag", "record", "playback", "messages", "topics", "replay"}
  };
  topicOrder_ << "rosbag-workbench";
}

void HelpBrowser::buildTableOfContents() {
  tocTree_->clear();

  // Create main categories
  QTreeWidgetItem* gettingStarted = new QTreeWidgetItem(tocTree_);
  gettingStarted->setText(0, tr("Getting Started"));
  gettingStarted->setData(0, Qt::UserRole, "getting-started");
  gettingStarted->setIcon(0, QApplication::style()->standardIcon(QStyle::SP_DialogHelpButton));

  QTreeWidgetItem* guidedTourItem = new QTreeWidgetItem(tocTree_);
  guidedTourItem->setText(0, tr("Guided Tour"));
  guidedTourItem->setData(0, Qt::UserRole, "guided-tour");
  guidedTourItem->setIcon(0, QApplication::style()->standardIcon(QStyle::SP_ArrowRight));

  QTreeWidgetItem* userGuide = new QTreeWidgetItem(tocTree_);
  userGuide->setText(0, tr("User Guide"));
  userGuide->setExpanded(true);

  QTreeWidgetItem* canvasItem = new QTreeWidgetItem(userGuide);
  canvasItem->setText(0, tr("Canvas Editor"));
  canvasItem->setData(0, Qt::UserRole, "canvas-editor");

  QTreeWidgetItem* packageItem = new QTreeWidgetItem(userGuide);
  packageItem->setText(0, tr("Package Browser"));
  packageItem->setData(0, Qt::UserRole, "package-browser");

  QTreeWidgetItem* propertiesItem = new QTreeWidgetItem(userGuide);
  propertiesItem->setText(0, tr("Properties Panel"));
  propertiesItem->setData(0, Qt::UserRole, "properties-panel");

  QTreeWidgetItem* codeGenItem = new QTreeWidgetItem(userGuide);
  codeGenItem->setText(0, tr("Code Generation"));
  codeGenItem->setData(0, Qt::UserRole, "code-generation");

  QTreeWidgetItem* robotWizardItem = new QTreeWidgetItem(userGuide);
  robotWizardItem->setText(0, tr("Robot Configuration Wizard"));
  robotWizardItem->setData(0, Qt::UserRole, "robot-wizard");

  QTreeWidgetItem* templatesItem = new QTreeWidgetItem(userGuide);
  templatesItem->setText(0, tr("Node Templates Library"));
  templatesItem->setData(0, Qt::UserRole, "node-templates");

  QTreeWidgetItem* workspaceItem = new QTreeWidgetItem(userGuide);
  workspaceItem->setText(0, tr("Workspace Browser"));
  workspaceItem->setData(0, Qt::UserRole, "workspace-browser");

  QTreeWidgetItem* projectItem = new QTreeWidgetItem(userGuide);
  projectItem->setText(0, tr("Project Management"));
  projectItem->setData(0, Qt::UserRole, "project-management");

  QTreeWidgetItem* ros2Item = new QTreeWidgetItem(tocTree_);
  ros2Item->setText(0, tr("ROS2 Integration"));
  ros2Item->setData(0, Qt::UserRole, "ros2-integration");
  ros2Item->setExpanded(true);

  QTreeWidgetItem* plotItem = new QTreeWidgetItem(ros2Item);
  plotItem->setText(0, tr("Plot Panel"));
  plotItem->setData(0, Qt::UserRole, "plot-panel");

  QTreeWidgetItem* lineageItem = new QTreeWidgetItem(ros2Item);
  lineageItem->setText(0, tr("Data Lineage"));
  lineageItem->setData(0, Qt::UserRole, "data-lineage");

  QTreeWidgetItem* healthItem = new QTreeWidgetItem(ros2Item);
  healthItem->setText(0, tr("Node Health Dashboard"));
  healthItem->setData(0, Qt::UserRole, "node-health-dashboard");

  QTreeWidgetItem* rosbagItem = new QTreeWidgetItem(ros2Item);
  rosbagItem->setText(0, tr("Rosbag Workbench"));
  rosbagItem->setData(0, Qt::UserRole, "rosbag-workbench");

  QTreeWidgetItem* aiItem = new QTreeWidgetItem(tocTree_);
  aiItem->setText(0, tr("Local AI Assistant"));
  aiItem->setData(0, Qt::UserRole, "local-ai");
  aiItem->setExpanded(true);

  QTreeWidgetItem* aiToolsItem = new QTreeWidgetItem(aiItem);
  aiToolsItem->setText(0, tr("AI Canvas Tools"));
  aiToolsItem->setData(0, Qt::UserRole, "ai-tools");

  QTreeWidgetItem* tools = new QTreeWidgetItem(tocTree_);
  tools->setText(0, tr("Tools & Integration"));
  tools->setExpanded(true);

  QTreeWidgetItem* vscodeItem = new QTreeWidgetItem(tools);
  vscodeItem->setText(0, tr("VS Code Integration"));
  vscodeItem->setData(0, Qt::UserRole, "vscode-integration");

  QTreeWidgetItem* reference = new QTreeWidgetItem(tocTree_);
  reference->setText(0, tr("Reference"));
  reference->setExpanded(true);

  QTreeWidgetItem* shortcutsItem = new QTreeWidgetItem(reference);
  shortcutsItem->setText(0, tr("Keyboard Shortcuts"));
  shortcutsItem->setData(0, Qt::UserRole, "keyboard-shortcuts");

  QTreeWidgetItem* troubleItem = new QTreeWidgetItem(reference);
  troubleItem->setText(0, tr("Troubleshooting"));
  troubleItem->setData(0, Qt::UserRole, "troubleshooting");

  QTreeWidgetItem* whatsNewItem = new QTreeWidgetItem(tocTree_);
  whatsNewItem->setText(0, tr("What's New"));
  whatsNewItem->setData(0, Qt::UserRole, "whats-new");
  whatsNewItem->setIcon(0, QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation));

  tocTree_->expandAll();
}

void HelpBrowser::showTopic(const QString& topicId) {
  if (!topics_.contains(topicId)) {
    return;
  }

  // Add to history if it's a new navigation
  if (currentTopicId_ != topicId) {
    // Remove any forward history
    while (history_.size() > historyIndex_ + 1) {
      history_.removeLast();
    }
    history_.append(topicId);
    historyIndex_ = history_.size() - 1;
  }

  currentTopicId_ = topicId;
  const HelpTopic& topic = topics_[topicId];

  // Update content
  QString html = getTopicHtml(topicId);
  contentBrowser_->setHtml(html);

  // Scroll to top
  contentBrowser_->verticalScrollBar()->setValue(0);

  // Update breadcrumb
  breadcrumbLabel_->setText(tr("Help > %1").arg(topic.title));

  // Update navigation buttons
  updateNavigationButtons();

  // Select in TOC
  std::function<void(QTreeWidgetItem*)> selectInToc = [&](QTreeWidgetItem* item) {
    for (int i = 0; i < item->childCount(); ++i) {
      QTreeWidgetItem* child = item->child(i);
      if (child->data(0, Qt::UserRole).toString() == topicId) {
        tocTree_->setCurrentItem(child);
        return;
      }
      selectInToc(child);
    }
  };

  for (int i = 0; i < tocTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = tocTree_->topLevelItem(i);
    if (item->data(0, Qt::UserRole).toString() == topicId) {
      tocTree_->setCurrentItem(item);
      break;
    }
    selectInToc(item);
  }

  emit topicChanged(topicId);
}

QString HelpBrowser::getTopicHtml(const QString& topicId) {
  if (!topics_.contains(topicId)) {
    return QString();
  }

  const HelpTopic& topic = topics_[topicId];
  QString html = renderMarkdownToHtml(topic.content);

  // Wrap in styled container
  return QString(R"(
<!DOCTYPE html>
<html>
<head>
<style>
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
  font-size: 14px;
  line-height: 1.6;
  color: #333;
  max-width: 800px;
  margin: 0;
  padding: 0;
}
h1 {
  color: #2c3e50;
  border-bottom: 2px solid #3498db;
  padding-bottom: 10px;
  margin-top: 0;
}
h2 {
  color: #2c3e50;
  margin-top: 30px;
}
h3 {
  color: #34495e;
  margin-top: 20px;
}
code {
  background-color: #f4f4f4;
  padding: 2px 6px;
  border-radius: 3px;
  font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
  font-size: 13px;
}
pre {
  background-color: #2d2d2d;
  color: #f8f8f2;
  padding: 16px;
  border-radius: 6px;
  overflow-x: auto;
}
pre code {
  background: none;
  padding: 0;
  color: inherit;
}
table {
  border-collapse: collapse;
  width: 100%;
  margin: 16px 0;
}
th, td {
  border: 1px solid #ddd;
  padding: 10px 12px;
  text-align: left;
}
th {
  background-color: #f5f5f5;
  font-weight: 600;
}
tr:nth-child(even) {
  background-color: #fafafa;
}
a {
  color: #3498db;
  text-decoration: none;
}
a:hover {
  text-decoration: underline;
}
strong {
  color: #2c3e50;
}
ul, ol {
  padding-left: 24px;
}
li {
  margin: 6px 0;
}
blockquote {
  border-left: 4px solid #3498db;
  margin: 16px 0;
  padding: 8px 16px;
  background-color: #f8f9fa;
}
</style>
</head>
<body>
%1
</body>
</html>
)").arg(html);
}

QString HelpBrowser::renderMarkdownToHtml(const QString& markdown) {
  QString html = markdown;

  // Headers
  html.replace(QRegularExpression("^### (.+)$", QRegularExpression::MultilineOption),
               "<h3>\\1</h3>");
  html.replace(QRegularExpression("^## (.+)$", QRegularExpression::MultilineOption),
               "<h2>\\1</h2>");
  html.replace(QRegularExpression("^# (.+)$", QRegularExpression::MultilineOption),
               "<h1>\\1</h1>");

  // Code blocks
  html.replace(QRegularExpression("```(\\w*)\\n([\\s\\S]*?)```"),
               "<pre><code class=\"\\1\">\\2</code></pre>");

  // Inline code
  html.replace(QRegularExpression("`([^`]+)`"), "<code>\\1</code>");

  // Bold
  html.replace(QRegularExpression("\\*\\*([^*]+)\\*\\*"), "<strong>\\1</strong>");

  // Links
  html.replace(QRegularExpression("\\[([^\\]]+)\\]\\(([^)]+)\\)"),
               "<a href=\"\\2\">\\1</a>");

  // Tables - simple handling using manual replacement
  QRegularExpression tableBlock("((?:^\\|.+\\|$\\n?)+)", QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator tableMatches = tableBlock.globalMatch(html);

  // Collect all table matches first (to avoid modifying string while iterating)
  QList<QPair<int, int>> tablePositions;
  QStringList tableReplacements;

  while (tableMatches.hasNext()) {
    QRegularExpressionMatch match = tableMatches.next();
    QString table = match.captured(1);
    QStringList rows = table.split('\n', Qt::SkipEmptyParts);

    if (!rows.isEmpty()) {
      QString result = "<table>";
      bool isHeader = true;
      for (const QString& row : rows) {
        if (row.contains("---")) continue;  // Skip separator row

        QString cells = row.mid(1, row.length() - 2);  // Remove leading/trailing |
        QStringList cellList = cells.split('|');

        result += "<tr>";
        for (const QString& cell : cellList) {
          QString tag = isHeader ? "th" : "td";
          result += QString("<%1>%2</%1>").arg(tag, cell.trimmed());
        }
        result += "</tr>";
        isHeader = false;
      }
      result += "</table>";

      tablePositions.append(qMakePair(match.capturedStart(), match.capturedLength()));
      tableReplacements.append(result);
    }
  }

  // Replace tables in reverse order to maintain positions
  for (int i = tablePositions.size() - 1; i >= 0; --i) {
    html.replace(tablePositions[i].first, tablePositions[i].second, tableReplacements[i]);
  }

  // Lists - unordered
  html.replace(QRegularExpression("^- (.+)$", QRegularExpression::MultilineOption),
               "<li>\\1</li>");

  // Wrap consecutive li elements in ul
  html.replace(QRegularExpression("((?:<li>[^<]+</li>\\s*)+)"), "<ul>\\1</ul>");

  // Numbered lists
  html.replace(QRegularExpression("^(\\d+)\\. (.+)$", QRegularExpression::MultilineOption),
               "<li>\\2</li>");

  // Paragraphs - wrap remaining text blocks
  html.replace(QRegularExpression("\\n\\n"), "</p><p>");
  if (!html.startsWith("<")) {
    html = "<p>" + html;
  }
  if (!html.endsWith(">")) {
    html = html + "</p>";
  }

  // Clean up empty paragraphs
  html.replace("<p></p>", "");
  html.replace("<p><h", "<h");
  html.replace("</h1></p>", "</h1>");
  html.replace("</h2></p>", "</h2>");
  html.replace("</h3></p>", "</h3>");
  html.replace("<p><ul>", "<ul>");
  html.replace("</ul></p>", "</ul>");
  html.replace("<p><table>", "<table>");
  html.replace("</table></p>", "</table>");
  html.replace("<p><pre>", "<pre>");
  html.replace("</pre></p>", "</pre>");

  return html;
}

void HelpBrowser::search(const QString& query) {
  searchResults_->clear();

  if (query.isEmpty()) {
    leftStack_->setCurrentWidget(tocTree_);
    return;
  }

  leftStack_->setCurrentWidget(searchResults_);

  QString lowerQuery = query.toLower();

  for (const QString& topicId : topicOrder_) {
    const HelpTopic& topic = topics_[topicId];

    bool match = false;

    // Check title
    if (topic.title.toLower().contains(lowerQuery)) {
      match = true;
    }

    // Check keywords
    for (const QString& keyword : topic.keywords) {
      if (keyword.toLower().contains(lowerQuery)) {
        match = true;
        break;
      }
    }

    // Check content
    if (topic.content.toLower().contains(lowerQuery)) {
      match = true;
    }

    if (match) {
      QListWidgetItem* item = new QListWidgetItem(topic.title);
      item->setData(Qt::UserRole, topicId);

      // Show snippet of matching content
      int pos = topic.content.toLower().indexOf(lowerQuery);
      if (pos >= 0) {
        int start = qMax(0, pos - 30);
        int end = qMin(topic.content.length(), pos + 50);
        QString snippet = "..." + topic.content.mid(start, end - start) + "...";
        snippet.replace('\n', ' ');
        item->setToolTip(snippet);
      }

      searchResults_->addItem(item);
    }
  }
}

void HelpBrowser::onTocItemClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString topicId = item->data(0, Qt::UserRole).toString();
  if (!topicId.isEmpty()) {
    showTopic(topicId);
  }
}

void HelpBrowser::onSearchTextChanged(const QString& text) {
  search(text);
}

void HelpBrowser::onSearchResultClicked(QListWidgetItem* item) {
  QString topicId = item->data(Qt::UserRole).toString();
  if (!topicId.isEmpty()) {
    showTopic(topicId);
  }
}

void HelpBrowser::onAnchorClicked(const QUrl& url) {
  QString scheme = url.scheme();

  if (scheme == "help") {
    // Internal help link
    showTopic(url.host());
  } else if (scheme == "http" || scheme == "https") {
    // External link
    QDesktopServices::openUrl(url);
  }
}

void HelpBrowser::onBackClicked() {
  if (historyIndex_ > 0) {
    historyIndex_--;
    currentTopicId_ = history_[historyIndex_];
    showTopic(currentTopicId_);
  }
}

void HelpBrowser::onForwardClicked() {
  if (historyIndex_ < history_.size() - 1) {
    historyIndex_++;
    currentTopicId_ = history_[historyIndex_];
    showTopic(currentTopicId_);
  }
}

void HelpBrowser::onHomeClicked() {
  showTopic("getting-started");
}

void HelpBrowser::updateNavigationButtons() {
  backButton_->setEnabled(historyIndex_ > 0);
  forwardButton_->setEnabled(historyIndex_ < history_.size() - 1);
}

}  // namespace ros_weaver
