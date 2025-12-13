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
| Ctrl+Shift+E | Open in VS Code |

## Help
| Shortcut | Action |
|----------|--------|
| F1 | Context Help |
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

## Version 0.1.0 (Current)

### Core Features
- Visual canvas for ROS2 package design
- Package browser with search
- Drag-and-drop block placement
- Topic connection visualization
- Parameter editing

### ROS2 Integration
- System scanning and discovery
- Real-time topic monitoring
- TF tree visualization
- ROS log viewer with filtering

### Code Generation
- Complete ROS2 package generation
- Generation wizard with customization
- C++ and Python node templates

### AI Integration
- Local LLM support via Ollama
- AI-powered log analysis
- Code suggestions

### Quality of Life
- Dark/Light theme support
- Plot panel for data visualization
- VS Code integration
- Keyboard shortcuts

## Planned Features

- RViz2 integration
- Interactive tutorials
- Unit testing framework
- Multi-robot support
- URDF visualization

Check the development plan for detailed roadmap.
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

ROS Weaver includes an optional local AI assistant powered by Ollama.

## Setup

### Installing Ollama
1. Visit [ollama.com](https://ollama.com)
2. Download and install for your platform
3. Start Ollama service

### Recommended Models
- **qwen2.5-coder:7b** - Best for coding tasks
- **llama3.3:8b** - Good all-rounder
- **phi3:mini** - For limited hardware

## Using the AI Assistant

### Accessing the Chat
Open the AI chat panel in the Output area (bottom of window).

### Features
- Ask ROS2 questions
- Get debugging help
- Generate code snippets
- Analyze error logs

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
}

void HelpBrowser::buildTableOfContents() {
  tocTree_->clear();

  // Create main categories
  QTreeWidgetItem* gettingStarted = new QTreeWidgetItem(tocTree_);
  gettingStarted->setText(0, tr("Getting Started"));
  gettingStarted->setData(0, Qt::UserRole, "getting-started");
  gettingStarted->setIcon(0, QApplication::style()->standardIcon(QStyle::SP_DialogHelpButton));

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

  QTreeWidgetItem* aiItem = new QTreeWidgetItem(tocTree_);
  aiItem->setText(0, tr("Local AI Assistant"));
  aiItem->setData(0, Qt::UserRole, "local-ai");

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
