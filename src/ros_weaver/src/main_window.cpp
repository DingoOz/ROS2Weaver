#include "ros_weaver/main_window.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/ros_package_index.hpp"
#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/core/external_editor.hpp"
#include "ros_weaver/core/topic_monitor.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"
#include "ros_weaver/widgets/output_panel.hpp"
#include "ros_weaver/widgets/topic_inspector.hpp"
#include "ros_weaver/widgets/ros_status_widget.hpp"
#include "ros_weaver/widgets/system_mapping_panel.hpp"
#include "ros_weaver/widgets/topic_viewer_panel.hpp"
#include "ros_weaver/core/system_discovery.hpp"
#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/wizards/package_wizard.hpp"

#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QAction>
#include <QIcon>
#include <QStandardPaths>
#include <QPushButton>
#include <QInputDialog>
#include <QRegularExpression>
#include <QDir>
#include <QFile>
#include <QDialog>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace ros_weaver {

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
  , canvas_(nullptr)
  , packageBrowserDock_(nullptr)
  , propertiesDock_(nullptr)
  , outputDock_(nullptr)
  , packageTree_(nullptr)
  , packageSearchEdit_(nullptr)
  , searchResultsItem_(nullptr)
  , localPackagesItem_(nullptr)
  , propertiesTab_(nullptr)
  , paramDashboard_(nullptr)
  , outputPanel_(nullptr)
  , packageIndex_(nullptr)
  , codeGenerator_(nullptr)
  , topicMonitor_(nullptr)
  , topicInspector_(nullptr)
  , liveMonitoringAction_(nullptr)
  , liveMonitoringEnabled_(false)
  , rosStatusWidget_(nullptr)
  , baseWindowTitle_("ROS Weaver - Visual ROS2 Package Editor")
  , systemDiscovery_(nullptr)
  , canvasMapper_(nullptr)
  , systemMappingPanel_(nullptr)
  , systemMappingDock_(nullptr)
  , scanSystemAction_(nullptr)
  , autoScanAction_(nullptr)
  , scanProgressBar_(nullptr)
  , topicViewerPanel_(nullptr)
  , topicViewerDock_(nullptr)
{
  setWindowTitle(baseWindowTitle_);
  setMinimumSize(1200, 800);

  // Initialize core components
  packageIndex_ = new RosPackageIndex(this);
  codeGenerator_ = new CodeGenerator(this);
  externalEditor_ = new ExternalEditor(this);

  // Initialize system discovery components
  systemDiscovery_ = new SystemDiscovery(this);
  canvasMapper_ = new CanvasMapper(this);

  // Connect system discovery signals
  connect(systemDiscovery_, &SystemDiscovery::scanStarted,
          this, &MainWindow::onScanStarted);
  connect(systemDiscovery_, &SystemDiscovery::scanProgress,
          this, &MainWindow::onScanProgress);
  connect(systemDiscovery_, &SystemDiscovery::scanCompleted,
          this, &MainWindow::onScanCompleted);
  connect(systemDiscovery_, &SystemDiscovery::scanTimedOut,
          this, &MainWindow::onScanTimedOut);
  connect(canvasMapper_, &CanvasMapper::mappingCompleted,
          this, &MainWindow::onMappingCompleted);

  // Connect signals
  connect(packageIndex_, &RosPackageIndex::searchResultsReady,
          this, &MainWindow::onPackageSearchResults);
  connect(codeGenerator_, &CodeGenerator::generationProgress,
          this, &MainWindow::onGenerationProgress);
  connect(codeGenerator_, &CodeGenerator::generationFinished,
          this, &MainWindow::onGenerationFinished);

  setupCentralWidget();
  setupMenuBar();
  setupToolBar();
  setupDockWidgets();
  setupStatusBar();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupMenuBar() {
  // File menu
  QMenu* fileMenu = menuBar()->addMenu(tr("&File"));

  QAction* newAction = fileMenu->addAction(tr("&New Project"));
  newAction->setShortcut(QKeySequence::New);
  connect(newAction, &QAction::triggered, this, &MainWindow::onNewProject);

  QAction* openAction = fileMenu->addAction(tr("&Open Project..."));
  openAction->setShortcut(QKeySequence::Open);
  connect(openAction, &QAction::triggered, this, &MainWindow::onOpenProject);

  QAction* saveAction = fileMenu->addAction(tr("&Save Project"));
  saveAction->setShortcut(QKeySequence::Save);
  connect(saveAction, &QAction::triggered, this, &MainWindow::onSaveProject);

  QAction* saveAsAction = fileMenu->addAction(tr("Save Project &As..."));
  saveAsAction->setShortcut(QKeySequence::SaveAs);
  connect(saveAsAction, &QAction::triggered, this, &MainWindow::onSaveProjectAs);

  fileMenu->addSeparator();

  // Examples submenu
  QMenu* examplesMenu = fileMenu->addMenu(tr("&Examples"));
  QAction* turtleBotAction = examplesMenu->addAction(tr("&TurtleBot3 Navigation"));
  connect(turtleBotAction, &QAction::triggered, this, &MainWindow::onLoadTurtleBotExample);

  QAction* turtlesimAction = examplesMenu->addAction(tr("Turtle&sim Teleop"));
  turtlesimAction->setToolTip(tr("Load the turtlesim teleop example - control a turtle with keyboard"));
  connect(turtlesimAction, &QAction::triggered, this, &MainWindow::onLoadTurtlesimExample);

  examplesMenu->addSeparator();

  QAction* launchTurtlesimAction = examplesMenu->addAction(tr("&Launch Turtlesim Nodes"));
  launchTurtlesimAction->setToolTip(tr("Launch the turtlesim_node and turtle_teleop_key nodes"));
  connect(launchTurtlesimAction, &QAction::triggered, this, &MainWindow::onLaunchTurtlesim);

  fileMenu->addSeparator();

  QAction* generateAction = fileMenu->addAction(tr("&Generate ROS2 Package..."));
  generateAction->setShortcut(tr("Ctrl+G"));
  generateAction->setToolTip(tr("Generate ROS2 package code from the current project"));
  connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateCode);

  QAction* generateWizardAction = fileMenu->addAction(tr("Generate ROS2 Package (&Wizard)..."));
  generateWizardAction->setShortcut(tr("Ctrl+Shift+G"));
  generateWizardAction->setToolTip(tr("Step-by-step wizard for generating ROS2 packages with full customization"));
  connect(generateWizardAction, &QAction::triggered, this, &MainWindow::onGenerateCodeWizard);

  fileMenu->addSeparator();

  // VS Code integration
  QAction* openProjectVSCodeAction = fileMenu->addAction(tr("Open Project Folder in VS Code"));
  openProjectVSCodeAction->setShortcut(tr("Ctrl+Shift+E"));
  openProjectVSCodeAction->setToolTip(tr("Open the current project folder in Visual Studio Code"));
  connect(openProjectVSCodeAction, &QAction::triggered, this, &MainWindow::onOpenProjectInVSCode);

  QAction* openGeneratedVSCodeAction = fileMenu->addAction(tr("Open Generated Package in VS Code"));
  openGeneratedVSCodeAction->setToolTip(tr("Open the last generated ROS2 package in Visual Studio Code"));
  connect(openGeneratedVSCodeAction, &QAction::triggered, this, &MainWindow::onOpenGeneratedPackageInVSCode);

  fileMenu->addSeparator();

  QAction* exitAction = fileMenu->addAction(tr("E&xit"));
  exitAction->setShortcut(QKeySequence::Quit);
  connect(exitAction, &QAction::triggered, this, &MainWindow::onExit);

  // Edit menu
  QMenu* editMenu = menuBar()->addMenu(tr("&Edit"));

  QAction* undoAction = editMenu->addAction(tr("&Undo"));
  undoAction->setShortcut(QKeySequence::Undo);
  undoAction->setEnabled(false);  // TODO: Implement undo

  QAction* redoAction = editMenu->addAction(tr("&Redo"));
  redoAction->setShortcut(QKeySequence::Redo);
  redoAction->setEnabled(false);  // TODO: Implement redo

  editMenu->addSeparator();

  QAction* deleteAction = editMenu->addAction(tr("&Delete"));
  deleteAction->setShortcut(QKeySequence::Delete);
  connect(deleteAction, &QAction::triggered, canvas_, &WeaverCanvas::deleteSelectedItems);

  editMenu->addSeparator();

  QAction* settingsAction = editMenu->addAction(tr("&Settings..."));
  settingsAction->setShortcut(tr("Ctrl+,"));
  settingsAction->setToolTip(tr("Open application settings"));
  connect(settingsAction, &QAction::triggered, this, &MainWindow::onOpenSettings);

  // View menu
  QMenu* viewMenu = menuBar()->addMenu(tr("&View"));

  QAction* zoomInAction = viewMenu->addAction(tr("Zoom &In"));
  zoomInAction->setShortcut(QKeySequence::ZoomIn);
  connect(zoomInAction, &QAction::triggered, canvas_, &WeaverCanvas::zoomIn);

  QAction* zoomOutAction = viewMenu->addAction(tr("Zoom &Out"));
  zoomOutAction->setShortcut(QKeySequence::ZoomOut);
  connect(zoomOutAction, &QAction::triggered, canvas_, &WeaverCanvas::zoomOut);

  QAction* resetZoomAction = viewMenu->addAction(tr("&Reset Zoom"));
  resetZoomAction->setShortcut(tr("Ctrl+0"));
  connect(resetZoomAction, &QAction::triggered, canvas_, &WeaverCanvas::resetZoom);

  viewMenu->addSeparator();

  QAction* fitAllAction = viewMenu->addAction(tr("&Fit All Nodes"));
  fitAllAction->setShortcut(tr("F"));
  fitAllAction->setToolTip(tr("Fit all nodes in the view"));
  connect(fitAllAction, &QAction::triggered, canvas_, &WeaverCanvas::fitToContents);

  viewMenu->addSeparator();

  // Panels submenu - toggles for dock widgets
  QMenu* panelsMenu = viewMenu->addMenu(tr("&Panels"));

  // These will be connected after dock widgets are created
  // We add them here and connect in setupDockWidgets
  QAction* showPackageBrowserAction = panelsMenu->addAction(tr("Package &Browser"));
  showPackageBrowserAction->setCheckable(true);
  showPackageBrowserAction->setChecked(true);
  showPackageBrowserAction->setObjectName("showPackageBrowserAction");

  QAction* showPropertiesAction = panelsMenu->addAction(tr("&Properties (Parameters)"));
  showPropertiesAction->setCheckable(true);
  showPropertiesAction->setChecked(true);
  showPropertiesAction->setObjectName("showPropertiesAction");

  QAction* showOutputAction = panelsMenu->addAction(tr("&Output"));
  showOutputAction->setCheckable(true);
  showOutputAction->setChecked(true);
  showOutputAction->setObjectName("showOutputAction");

  panelsMenu->addSeparator();

  QAction* resetLayoutAction = panelsMenu->addAction(tr("&Reset Layout"));
  resetLayoutAction->setToolTip(tr("Reset all panels to their default positions"));
  connect(resetLayoutAction, &QAction::triggered, this, [this]() {
    // Show all docks
    packageBrowserDock_->show();
    propertiesDock_->show();
    outputDock_->show();

    // Reset to default docked positions
    packageBrowserDock_->setFloating(false);
    propertiesDock_->setFloating(false);
    outputDock_->setFloating(false);

    addDockWidget(Qt::LeftDockWidgetArea, packageBrowserDock_);
    addDockWidget(Qt::RightDockWidgetArea, propertiesDock_);
    addDockWidget(Qt::BottomDockWidgetArea, outputDock_);

    // Update checkboxes
    findChild<QAction*>("showPackageBrowserAction")->setChecked(true);
    findChild<QAction*>("showPropertiesAction")->setChecked(true);
    findChild<QAction*>("showOutputAction")->setChecked(true);
  });

  // ROS2 menu
  QMenu* ros2Menu = menuBar()->addMenu(tr("&ROS2"));

  QAction* scanPackagesAction = ros2Menu->addAction(tr("&Scan Packages"));
  scanPackagesAction->setToolTip(tr("Scan workspace for ROS2 packages"));

  QAction* buildAction = ros2Menu->addAction(tr("&Build Workspace"));
  buildAction->setShortcut(tr("Ctrl+B"));

  QAction* launchAction = ros2Menu->addAction(tr("&Launch..."));
  launchAction->setShortcut(tr("Ctrl+L"));

  ros2Menu->addSeparator();

  // System discovery actions
  scanSystemAction_ = ros2Menu->addAction(tr("Scan &Running System"));
  scanSystemAction_->setShortcut(tr("Ctrl+Shift+R"));
  scanSystemAction_->setToolTip(tr("Scan for running ROS2 nodes and topics"));
  connect(scanSystemAction_, &QAction::triggered, this, &MainWindow::onScanSystem);

  autoScanAction_ = ros2Menu->addAction(tr("&Auto-Scan"));
  autoScanAction_->setCheckable(true);
  autoScanAction_->setToolTip(tr("Automatically scan system at regular intervals"));
  connect(autoScanAction_, &QAction::toggled, this, &MainWindow::onToggleAutoScan);

  QAction* showMappingPanelAction = ros2Menu->addAction(tr("Show System &Mapping Panel"));
  showMappingPanelAction->setShortcut(tr("Ctrl+Shift+M"));
  connect(showMappingPanelAction, &QAction::triggered, this, [this]() {
    if (systemMappingDock_) {
      systemMappingDock_->show();
      systemMappingDock_->raise();
    }
  });

  QAction* showTopicViewerAction = ros2Menu->addAction(tr("Show &Topic Viewer"));
  showTopicViewerAction->setShortcut(tr("Ctrl+Shift+T"));
  connect(showTopicViewerAction, &QAction::triggered, this, [this]() {
    // Show Properties dock and switch to Topics tab
    if (propertiesDock_) {
      propertiesDock_->show();
      propertiesDock_->raise();
    }
    if (propertiesTab_ && topicViewerPanel_) {
      propertiesTab_->setCurrentWidget(topicViewerPanel_);
    }
  });

  // Help menu
  QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));

  QAction* aboutAction = helpMenu->addAction(tr("&About ROS Weaver"));
  connect(aboutAction, &QAction::triggered, this, &MainWindow::onAbout);

  QAction* aboutQtAction = helpMenu->addAction(tr("About &Qt"));
  connect(aboutQtAction, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::setupToolBar() {
  QToolBar* mainToolBar = addToolBar(tr("Main Toolbar"));
  mainToolBar->setMovable(false);

  mainToolBar->addAction(tr("New"));
  mainToolBar->addAction(tr("Open"));
  mainToolBar->addAction(tr("Save"));
  mainToolBar->addSeparator();
  mainToolBar->addAction(tr("Build"));
  mainToolBar->addAction(tr("Launch"));
  mainToolBar->addSeparator();

  // Add scan system button to toolbar
  QAction* scanToolbarAction = mainToolBar->addAction(tr("Scan System"));
  scanToolbarAction->setToolTip(tr("Scan running ROS2 system (Ctrl+Shift+R)"));
  connect(scanToolbarAction, &QAction::triggered, this, &MainWindow::onScanSystem);
}

void MainWindow::setupDockWidgets() {
  // Package Browser Dock (left side)
  packageBrowserDock_ = new QDockWidget(tr("Package Browser"), this);
  packageBrowserDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  QWidget* browserWidget = new QWidget();
  QVBoxLayout* browserLayout = new QVBoxLayout(browserWidget);
  browserLayout->setContentsMargins(4, 4, 4, 4);
  browserLayout->setSpacing(4);

  // Search bar
  QHBoxLayout* searchLayout = new QHBoxLayout();
  packageSearchEdit_ = new QLineEdit();
  packageSearchEdit_->setPlaceholderText(tr("Search ROS packages..."));
  packageSearchEdit_->setClearButtonEnabled(true);
  connect(packageSearchEdit_, &QLineEdit::returnPressed, this, &MainWindow::onSearchPackages);

  QPushButton* searchButton = new QPushButton(tr("Search"));
  connect(searchButton, &QPushButton::clicked, this, &MainWindow::onSearchPackages);

  searchLayout->addWidget(packageSearchEdit_);
  searchLayout->addWidget(searchButton);
  browserLayout->addLayout(searchLayout);

  // Package tree
  packageTree_ = new QTreeWidget();
  packageTree_->setHeaderLabel(tr("Packages"));
  packageTree_->setDragEnabled(true);
  connect(packageTree_, &QTreeWidget::itemDoubleClicked,
          this, &MainWindow::onPackageItemDoubleClicked);

  // Search results section
  searchResultsItem_ = new QTreeWidgetItem(packageTree_);
  searchResultsItem_->setText(0, tr("Search Results"));
  searchResultsItem_->setExpanded(true);

  // Local packages section
  localPackagesItem_ = new QTreeWidgetItem(packageTree_);
  localPackagesItem_->setText(0, tr("Local Packages"));
  localPackagesItem_->setExpanded(true);

  // Templates section
  QTreeWidgetItem* templatesItem = new QTreeWidgetItem(packageTree_);
  templatesItem->setText(0, tr("Node Templates"));
  templatesItem->setExpanded(true);

  // Add template items with data for creating blocks
  QStringList templates = {
    "publisher_node", "subscriber_node", "relay_node",
    "service_server", "service_client",
    "action_server", "action_client",
    "sensor_fusion", "robot_controller", "image_processor"
  };

  for (const QString& tmpl : templates) {
    QTreeWidgetItem* item = new QTreeWidgetItem(templatesItem);
    item->setText(0, tmpl);
    item->setData(0, Qt::UserRole, tmpl);  // Store template name
    item->setToolTip(0, tr("Double-click to add to canvas"));
  }

  browserLayout->addWidget(packageTree_);
  packageBrowserDock_->setWidget(browserWidget);
  addDockWidget(Qt::LeftDockWidgetArea, packageBrowserDock_);

  // Properties Dock (right side) with Param Dashboard
  propertiesDock_ = new QDockWidget(tr("Properties"), this);
  propertiesDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  propertiesTab_ = new QTabWidget();

  // Parameters tab with ParamDashboard
  paramDashboard_ = new ParamDashboard();
  propertiesTab_->addTab(paramDashboard_, tr("Parameters"));

  // Topics tab with TopicViewerPanel
  topicViewerPanel_ = new TopicViewerPanel();
  topicViewerPanel_->setCanvas(canvas_);
  propertiesTab_->addTab(topicViewerPanel_, tr("Topics"));

  // Connect topic viewer signals
  connect(topicViewerPanel_, &TopicViewerPanel::topicSelected,
          this, &MainWindow::onTopicViewerTopicSelected);
  connect(topicViewerPanel_, &TopicViewerPanel::showTopicOnCanvas,
          this, &MainWindow::onShowTopicOnCanvas);
  connect(topicViewerPanel_, &TopicViewerPanel::echoTopicRequested,
          this, &MainWindow::onEchoTopicRequested);

  // Connect topic viewer rate updates to animate canvas connections
  connect(topicViewerPanel_, &TopicViewerPanel::messageReceived,
          this, [this](const QString& topicName, const QString&, double rate) {
    // Update connection lines that match this topic
    if (!canvas_) return;

    for (ConnectionLine* conn : canvas_->connections()) {
      if (conn->topicName() == topicName) {
        conn->setLiveMonitoringEnabled(true);
        conn->setMessageRate(rate);

        // Update activity state based on rate
        if (rate > ConnectionLine::HIGH_RATE_THRESHOLD) {
          conn->setActivityState(TopicActivityState::HighRate);
          conn->pulseActivity();  // Only pulse when active
        } else if (rate > 0) {
          conn->setActivityState(TopicActivityState::Active);
          conn->pulseActivity();  // Only pulse when active
        } else {
          conn->setActivityState(TopicActivityState::Inactive);
          // Don't pulse when inactive - let animation fade out
        }
      }
    }
  });

  propertiesDock_->setWidget(propertiesTab_);
  addDockWidget(Qt::RightDockWidgetArea, propertiesDock_);

  // Output Dock (bottom) - with tabs for Build Output, ROS Logs, and Terminal
  outputDock_ = new QDockWidget(tr("Output"), this);
  outputDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

  outputPanel_ = new OutputPanel();
  outputDock_->setWidget(outputPanel_);
  addDockWidget(Qt::BottomDockWidgetArea, outputDock_);

  // System Mapping Dock (right side, tabbed with Properties)
  systemMappingDock_ = new QDockWidget(tr("System Mapping"), this);
  systemMappingDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  systemMappingPanel_ = new SystemMappingPanel();
  systemMappingPanel_->setSystemDiscovery(systemDiscovery_);
  systemMappingPanel_->setCanvasMapper(canvasMapper_);

  connect(systemMappingPanel_, &SystemMappingPanel::blockSelected,
          this, &MainWindow::onMappingBlockSelected);
  connect(systemMappingPanel_, &SystemMappingPanel::scanRequested,
          this, &MainWindow::onScanSystem);

  systemMappingDock_->setWidget(systemMappingPanel_);
  addDockWidget(Qt::RightDockWidgetArea, systemMappingDock_);
  tabifyDockWidget(propertiesDock_, systemMappingDock_);

  propertiesDock_->raise();  // Make Properties the default visible tab

  // Connect panel visibility toggles from View > Panels menu
  QAction* showPackageBrowserAction = findChild<QAction*>("showPackageBrowserAction");
  QAction* showPropertiesAction = findChild<QAction*>("showPropertiesAction");
  QAction* showOutputAction = findChild<QAction*>("showOutputAction");

  if (showPackageBrowserAction) {
    connect(showPackageBrowserAction, &QAction::toggled, packageBrowserDock_, &QDockWidget::setVisible);
    connect(packageBrowserDock_, &QDockWidget::visibilityChanged, showPackageBrowserAction, &QAction::setChecked);
  }
  if (showPropertiesAction) {
    connect(showPropertiesAction, &QAction::toggled, propertiesDock_, &QDockWidget::setVisible);
    connect(propertiesDock_, &QDockWidget::visibilityChanged, showPropertiesAction, &QAction::setChecked);
  }
  if (showOutputAction) {
    connect(showOutputAction, &QAction::toggled, outputDock_, &QDockWidget::setVisible);
    connect(outputDock_, &QDockWidget::visibilityChanged, showOutputAction, &QAction::setChecked);
  }
}

void MainWindow::setupCentralWidget() {
  canvas_ = new WeaverCanvas(this);
  setCentralWidget(canvas_);

  // Connect canvas signals
  connect(canvas_, &WeaverCanvas::blockSelected, this, &MainWindow::onBlockSelected);

  // VS Code integration from canvas context menu
  connect(canvas_, &WeaverCanvas::openBlockInVSCodeRequested, this, &MainWindow::onOpenBlockInVSCode);

  // When user changes preferred YAML source via context menu, refresh the param dashboard
  connect(canvas_, &WeaverCanvas::blockYamlSourceChanged, this,
          [this](PackageBlock* block, const QString& /* yamlSource */) {
    // Refresh the param dashboard if this is the currently selected block
    if (paramDashboard_->currentBlock() == block) {
      paramDashboard_->setCurrentBlock(block);
    }
  });
}

void MainWindow::onBlockSelected(PackageBlock* block) {
  statusBar()->showMessage(tr("Selected: %1").arg(block ? block->packageName() : tr("None")));

  // Update param dashboard with selected block
  paramDashboard_->setCurrentBlock(block);
}

void MainWindow::setupStatusBar() {
  // Create ROS2 status widget
  rosStatusWidget_ = new RosStatusWidget(this);
  connect(rosStatusWidget_, &RosStatusWidget::titleBarUpdateRequested,
          this, &MainWindow::onRosStatusTitleBarUpdate);

  // Create scan progress bar (hidden by default)
  scanProgressBar_ = new QProgressBar(this);
  scanProgressBar_->setTextVisible(true);
  scanProgressBar_->setRange(0, 100);
  scanProgressBar_->setFixedWidth(200);
  scanProgressBar_->setVisible(false);

  // Add a stretch to push widgets to the right
  QWidget* spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  statusBar()->addWidget(spacer);

  // Add scan progress bar
  statusBar()->addPermanentWidget(scanProgressBar_);

  // Add ROS status widget to the right side of status bar
  statusBar()->addPermanentWidget(rosStatusWidget_);

  statusBar()->showMessage(tr("Ready"));
}

void MainWindow::onNewProject() {
  // Clear param dashboard first to avoid dangling pointer when canvas clears blocks
  paramDashboard_->setCurrentBlock(nullptr);
  canvas_->clearCanvas();
  currentProjectPath_.clear();
  baseWindowTitle_ = "ROS Weaver - Visual ROS2 Package Editor";
  setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());
  statusBar()->showMessage(tr("New project created"));
}

void MainWindow::onOpenProject() {
  QString fileName = QFileDialog::getOpenFileName(
    this,
    tr("Open ROS Weaver Project"),
    QString(),
    tr("ROS Weaver Projects (*.rwp);;All Files (*)")
  );

  if (!fileName.isEmpty()) {
    if (loadProject(fileName)) {
      currentProjectPath_ = fileName;
      baseWindowTitle_ = QString("ROS Weaver - %1").arg(QFileInfo(fileName).fileName());
      setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());
      statusBar()->showMessage(tr("Opened: %1").arg(fileName));
    }
  }
}

void MainWindow::onSaveProject() {
  if (currentProjectPath_.isEmpty()) {
    onSaveProjectAs();
    return;
  }

  if (saveProject(currentProjectPath_)) {
    statusBar()->showMessage(tr("Saved: %1").arg(currentProjectPath_));
  }
}

void MainWindow::onSaveProjectAs() {
  QString fileName = QFileDialog::getSaveFileName(
    this,
    tr("Save ROS Weaver Project"),
    QString(),
    tr("ROS Weaver Projects (*.rwp);;All Files (*)")
  );

  if (!fileName.isEmpty()) {
    // Ensure .rwp extension
    if (!fileName.endsWith(".rwp", Qt::CaseInsensitive)) {
      fileName += ".rwp";
    }

    if (saveProject(fileName)) {
      currentProjectPath_ = fileName;
      baseWindowTitle_ = QString("ROS Weaver - %1").arg(QFileInfo(fileName).fileName());
      setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());
      statusBar()->showMessage(tr("Saved: %1").arg(fileName));
    }
  }
}

bool MainWindow::saveProject(const QString& filePath) {
  Project project;
  canvas_->exportToProject(project);

  // Set metadata
  project.metadata().name = QFileInfo(filePath).baseName();

  if (!project.saveToFile(filePath)) {
    QMessageBox::warning(this, tr("Save Error"),
                         tr("Could not save project to %1").arg(filePath));
    return false;
  }

  return true;
}

bool MainWindow::loadProject(const QString& filePath) {
  QString errorMsg;
  Project project = Project::loadFromFile(filePath, &errorMsg);

  if (!errorMsg.isEmpty()) {
    QMessageBox::warning(this, tr("Load Error"), errorMsg);
    return false;
  }

  // Clear param dashboard before import (which clears the canvas)
  paramDashboard_->setCurrentBlock(nullptr);
  canvas_->importFromProject(project);
  return true;
}

void MainWindow::onLoadTurtleBotExample() {
  // Try to load from installed package share directory first
  QString examplePath;

  try {
    std::string packageShare = ament_index_cpp::get_package_share_directory("ros_weaver");
    examplePath = QString::fromStdString(packageShare) +
                  "/examples/turtlebot3_navigation/turtlebot3_navigation.rwp";
  } catch (const std::exception&) {
    // Fallback to source directory (for development)
    examplePath = QDir::currentPath() +
                  "/src/ros_weaver/examples/turtlebot3_navigation/turtlebot3_navigation.rwp";
  }

  // Check if file exists
  if (!QFile::exists(examplePath)) {
    // Try another fallback path for development
    QDir sourceDir(QDir::currentPath());
    if (sourceDir.exists("examples/turtlebot3_navigation/turtlebot3_navigation.rwp")) {
      examplePath = sourceDir.absoluteFilePath("examples/turtlebot3_navigation/turtlebot3_navigation.rwp");
    } else {
      QMessageBox::warning(this, tr("Example Not Found"),
        tr("Could not find the TurtleBot3 Navigation example.\n"
           "Please ensure the package is properly installed.\n\n"
           "Looked in:\n%1").arg(examplePath));
      return;
    }
  }

  // Load the project file
  QString errorMsg;
  Project project = Project::loadFromFile(examplePath, &errorMsg);

  if (!errorMsg.isEmpty()) {
    QMessageBox::warning(this, tr("Load Error"),
      tr("Failed to load TurtleBot3 example:\n%1").arg(errorMsg));
    return;
  }

  // Clear param dashboard before import (which clears the canvas)
  paramDashboard_->setCurrentBlock(nullptr);
  paramDashboard_->clearYamlFiles();

  // Import the project
  canvas_->importFromProject(project);
  currentProjectPath_.clear();
  baseWindowTitle_ = "ROS Weaver - TurtleBot3 Navigation (Example)";
  setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());

  // Determine YAML directory
  QString yamlDir;
  try {
    std::string packageShare = ament_index_cpp::get_package_share_directory("ros_weaver");
    yamlDir = QString::fromStdString(packageShare) + "/examples/turtlebot3_navigation/";
  } catch (const std::exception&) {
    yamlDir = QDir::currentPath() + "/src/ros_weaver/examples/turtlebot3_navigation/";
  }

  // Load YAML files for the project
  loadProjectYamlFiles(yamlDir);

  statusBar()->showMessage(tr("Loaded TurtleBot3 Navigation example"));

  // Append to output about YAML files
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Loaded TurtleBot3 Navigation example project.\n"));
  outputPanel_->appendBuildOutput(tr("Associated YAML configuration files:\n"));
  outputPanel_->appendBuildOutput(tr("  - nav2_params.yaml (Navigation2 stack parameters)\n"));
  outputPanel_->appendBuildOutput(tr("  - slam_toolbox_params.yaml (SLAM Toolbox parameters)\n"));
  outputPanel_->appendBuildOutput(tr("\nYAML files location: %1\n").arg(yamlDir));
  outputPanel_->appendBuildOutput(tr("\nSelect a node to see its parameters. Use the Source dropdown to switch between block parameters and YAML files."));
}

void MainWindow::onLoadTurtlesimExample() {
  // Try to load from installed package share directory first
  QString examplePath;

  try {
    std::string packageShare = ament_index_cpp::get_package_share_directory("ros_weaver");
    examplePath = QString::fromStdString(packageShare) +
                  "/examples/turtlesim_teleop/turtlesim_teleop.rwp";
  } catch (const std::exception&) {
    // Fallback to source directory (for development)
    examplePath = QDir::currentPath() +
                  "/src/ros_weaver/examples/turtlesim_teleop/turtlesim_teleop.rwp";
  }

  // Check if file exists
  if (!QFile::exists(examplePath)) {
    // Try another fallback path for development
    QDir sourceDir(QDir::currentPath());
    if (sourceDir.exists("examples/turtlesim_teleop/turtlesim_teleop.rwp")) {
      examplePath = sourceDir.absoluteFilePath("examples/turtlesim_teleop/turtlesim_teleop.rwp");
    } else {
      QMessageBox::warning(this, tr("Example Not Found"),
        tr("Could not find the Turtlesim Teleop example.\n"
           "Please ensure the package is properly installed.\n\n"
           "Looked in:\n%1").arg(examplePath));
      return;
    }
  }

  // Load the project file
  QString errorMsg;
  Project project = Project::loadFromFile(examplePath, &errorMsg);

  if (!errorMsg.isEmpty()) {
    QMessageBox::warning(this, tr("Load Error"),
      tr("Failed to load Turtlesim example:\n%1").arg(errorMsg));
    return;
  }

  // Clear param dashboard before import (which clears the canvas)
  paramDashboard_->setCurrentBlock(nullptr);
  paramDashboard_->clearYamlFiles();

  // Import the project
  canvas_->importFromProject(project);
  currentProjectPath_.clear();
  baseWindowTitle_ = "ROS Weaver - Turtlesim Teleop (Example)";
  setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());

  statusBar()->showMessage(tr("Loaded Turtlesim Teleop example"));

  // Append to output with launch instructions
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Loaded Turtlesim Teleop example project.\n\n"));
  outputPanel_->appendBuildOutput(tr("This example demonstrates:\n"));
  outputPanel_->appendBuildOutput(tr("  - turtlesim_node: A 2D turtle simulator window\n"));
  outputPanel_->appendBuildOutput(tr("  - turtle_teleop_key: Keyboard control for the turtle\n"));
  outputPanel_->appendBuildOutput(tr("  - Mimic system: Optional second turtle that mimics the first\n\n"));
  outputPanel_->appendBuildOutput(tr("To run this example:\n"));
  outputPanel_->appendBuildOutput(tr("  1. Go to File > Examples > Launch Turtlesim Nodes\n"));
  outputPanel_->appendBuildOutput(tr("  2. Or manually run in separate terminals:\n"));
  outputPanel_->appendBuildOutput(tr("     ros2 run turtlesim turtlesim_node\n"));
  outputPanel_->appendBuildOutput(tr("     ros2 run turtlesim turtle_teleop_key\n\n"));
  outputPanel_->appendBuildOutput(tr("Control the turtle with arrow keys in the teleop terminal.\n"));
  outputPanel_->appendBuildOutput(tr("Use the Topic Viewer (Ctrl+Shift+T) to see live data flow!"));
}

void MainWindow::onLaunchTurtlesim() {
  // Check if turtlesim is available
  QProcess checkProcess;
  checkProcess.start("ros2", QStringList() << "pkg" << "list");
  checkProcess.waitForFinished(5000);
  QString packages = checkProcess.readAllStandardOutput();

  if (!packages.contains("turtlesim")) {
    QMessageBox::warning(this, tr("Turtlesim Not Found"),
      tr("The turtlesim package is not installed.\n\n"
         "Install it with:\n"
         "  sudo apt install ros-jazzy-turtlesim\n\n"
         "Or for your ROS2 distribution:\n"
         "  sudo apt install ros-${ROS_DISTRO}-turtlesim"));
    return;
  }

  // Ask if user wants to clear canvas or add to existing
  QMessageBox::StandardButton reply = QMessageBox::question(this,
    tr("Add to Canvas"),
    tr("Do you want to clear the canvas and add turtlesim nodes?\n\n"
       "Click 'Yes' to clear and add nodes.\n"
       "Click 'No' to just launch without modifying the canvas."),
    QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

  if (reply == QMessageBox::Cancel) {
    return;
  }

  // Add nodes to canvas if user selected Yes
  if (reply == QMessageBox::Yes && canvas_) {
    canvas_->clearCanvas();
    paramDashboard_->setCurrentBlock(nullptr);

    // Create turtle_teleop_key block (publisher)
    QList<QPair<QString, QString>> teleopInputs;  // No inputs
    QList<QPair<QString, QString>> teleopOutputs;
    teleopOutputs.append(qMakePair(QString("cmd_vel"), QString("geometry_msgs/msg/Twist")));

    PackageBlock* teleopBlock = canvas_->addCustomBlock(
      "turtle_teleop_key",
      QPointF(-200, 0),
      teleopInputs,
      teleopOutputs
    );

    // Create turtlesim_node block (subscriber/publisher)
    QList<QPair<QString, QString>> turtleInputs;
    turtleInputs.append(qMakePair(QString("cmd_vel"), QString("geometry_msgs/msg/Twist")));

    QList<QPair<QString, QString>> turtleOutputs;
    turtleOutputs.append(qMakePair(QString("pose"), QString("turtlesim/msg/Pose")));
    turtleOutputs.append(qMakePair(QString("color_sensor"), QString("turtlesim/msg/Color")));

    PackageBlock* turtleBlock = canvas_->addCustomBlock(
      "turtlesim_node",
      QPointF(200, 0),
      turtleInputs,
      turtleOutputs
    );

    // Create connection between teleop and turtlesim
    if (teleopBlock && turtleBlock) {
      ConnectionLine* conn = canvas_->createConnection(teleopBlock, 0, turtleBlock, 0);
      if (conn) {
        conn->setTopicName("/turtle1/cmd_vel");
      }
    }

    // Update window title
    baseWindowTitle_ = "ROS Weaver - Turtlesim (Running)";
    setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());

    // Fit view to show all blocks
    canvas_->fitToContents();
  }

  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Launching Turtlesim nodes...\n\n"));

  // Launch turtlesim_node
  QProcess* turtlesimProc = new QProcess(this);
  turtlesimProc->setProcessChannelMode(QProcess::MergedChannels);

  connect(turtlesimProc, &QProcess::readyReadStandardOutput, [this, turtlesimProc]() {
    outputPanel_->appendBuildOutput(QString::fromUtf8(turtlesimProc->readAllStandardOutput()));
  });

  connect(turtlesimProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, turtlesimProc](int exitCode, QProcess::ExitStatus) {
    outputPanel_->appendBuildOutput(tr("\nTurtlesim node exited with code %1\n").arg(exitCode));
    turtlesimProc->deleteLater();
  });

  // Use bash to source ROS setup and run the node
  turtlesimProc->start("bash", QStringList() << "-c"
    << "source /opt/ros/jazzy/setup.bash && ros2 run turtlesim turtlesim_node");

  outputPanel_->appendBuildOutput(tr("Started: ros2 run turtlesim turtlesim_node\n"));
  outputPanel_->appendBuildOutput(tr("A turtle window should appear.\n\n"));

  // Give turtlesim_node time to start before launching teleop
  QTimer::singleShot(1500, this, [this]() {
    // Launch turtle_teleop_key in the terminal tab
    outputPanel_->appendBuildOutput(tr("To control the turtle, run in a terminal:\n"));
    outputPanel_->appendBuildOutput(tr("  ros2 run turtlesim turtle_teleop_key\n\n"));
    outputPanel_->appendBuildOutput(tr("Then use arrow keys to move the turtle!\n"));
    outputPanel_->appendBuildOutput(tr("  - Up arrow: Move forward\n"));
    outputPanel_->appendBuildOutput(tr("  - Down arrow: Move backward\n"));
    outputPanel_->appendBuildOutput(tr("  - Left/Right arrows: Rotate\n\n"));
    outputPanel_->appendBuildOutput(tr("Use Topic Viewer (Ctrl+Shift+T) to see /turtle1/cmd_vel and /turtle1/pose topics.\n"));
    outputPanel_->appendBuildOutput(tr("\nNodes displayed on canvas show the running system topology."));

    statusBar()->showMessage(tr("Turtlesim launched - use 'ros2 run turtlesim turtle_teleop_key' in a terminal to control"), 10000);
  });
}

void MainWindow::loadProjectYamlFiles(const QString& projectDir) {
  QDir dir(projectDir);
  if (!dir.exists()) {
    return;
  }

  QList<YamlFileInfo> yamlFiles;

  // Find all YAML files in the directory
  QStringList yamlFilters;
  yamlFilters << "*.yaml" << "*.yml";
  QStringList yamlFileNames = dir.entryList(yamlFilters, QDir::Files);

  for (const QString& fileName : yamlFileNames) {
    QString filePath = dir.absoluteFilePath(fileName);
    YamlFileInfo info;
    info.filePath = filePath;

    // Parse YAML to extract top-level node names
    QFile file(filePath);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QString content = QString::fromUtf8(file.readAll());
      file.close();

      try {
        YAML::Node root = YAML::Load(content.toStdString());
        for (auto it = root.begin(); it != root.end(); ++it) {
          QString key = QString::fromStdString(it->first.as<std::string>());
          // Check if this looks like a node section (has ros__parameters)
          if (it->second.IsMap()) {
            info.nodeNames.append(key);
          }
        }
      } catch (const YAML::Exception& e) {
        // Skip files that can't be parsed
        continue;
      }
    }

    if (!info.nodeNames.isEmpty()) {
      yamlFiles.append(info);
    }
  }

  // Pass to param dashboard
  paramDashboard_->setProjectDirectory(projectDir);
  paramDashboard_->setProjectYamlFiles(yamlFiles);

  // Pass to canvas for context menu
  QStringList yamlFilePaths;
  for (const YamlFileInfo& info : yamlFiles) {
    yamlFilePaths.append(info.filePath);
  }
  canvas_->setAvailableYamlFiles(yamlFilePaths);
}

void MainWindow::onExit() {
  close();
}

void MainWindow::onAbout() {
  QMessageBox::about(
    this,
    tr("About ROS Weaver"),
    tr("<h2>ROS Weaver</h2>"
       "<p>Version 0.1.0</p>"
       "<p>A visual tool for assembling, extending, and iterating on "
       "ROS2 packages and nodes.</p>"
       "<p>Inspired by visual programming tools like Unreal Engine's "
       "Blueprint and Houdini's node-based workflow.</p>")
  );
}

void MainWindow::onSearchPackages() {
  QString query = packageSearchEdit_->text().trimmed();
  if (query.isEmpty()) {
    return;
  }

  statusBar()->showMessage(tr("Searching for '%1'...").arg(query));
  packageIndex_->searchPackages(query);
}

void MainWindow::onPackageSearchResults(const QList<RosPackageInfo>& packages) {
  // Clear previous search results
  while (searchResultsItem_->childCount() > 0) {
    delete searchResultsItem_->takeChild(0);
  }

  // Add new results
  for (const RosPackageInfo& pkg : packages) {
    QTreeWidgetItem* item = new QTreeWidgetItem(searchResultsItem_);
    item->setText(0, pkg.name);
    item->setToolTip(0, pkg.description);
    item->setData(0, Qt::UserRole, pkg.name);
    item->setData(0, Qt::UserRole + 1, "search_result");
  }

  searchResultsItem_->setExpanded(true);
  statusBar()->showMessage(tr("Found %1 packages").arg(packages.size()));
}

void MainWindow::onPackageItemDoubleClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  if (!item || item->childCount() > 0) {
    return;  // Don't do anything for parent items
  }

  QString packageName = item->data(0, Qt::UserRole).toString();
  if (packageName.isEmpty()) {
    return;
  }

  // Add block to canvas at center of view
  QPointF centerPos = canvas_->mapToScene(canvas_->viewport()->rect().center());
  canvas_->addPackageBlock(packageName, centerPos);

  statusBar()->showMessage(tr("Added '%1' to canvas").arg(packageName));
}

void MainWindow::onGenerateCodeWizard() {
  // Export current project
  Project project;
  canvas_->exportToProject(project);

  if (project.blocks().isEmpty()) {
    QMessageBox::warning(this, tr("Empty Project"),
      tr("Cannot generate code from an empty project. Please add some nodes first."));
    return;
  }

  // Create and show the wizard
  PackageWizard wizard(project, this);

  // Connect to generation complete signal
  connect(&wizard, &PackageWizard::generationComplete,
          this, [this](bool success, const QString& path) {
    if (success) {
      lastGeneratedPackagePath_ = path;
      outputPanel_->appendBuildOutput(tr("\nPackage generated via wizard: %1").arg(path));
    }
  });

  if (wizard.exec() == QDialog::Accepted) {
    // Show success dialog with VS Code option
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(tr("Package Generated"));
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setText(tr("ROS2 package has been generated successfully!"));
    msgBox.setInformativeText(tr("Package location: %1").arg(wizard.generatedPackagePath()));

    QPushButton* openVSCodeBtn = msgBox.addButton(tr("Open in VS Code"), QMessageBox::ActionRole);
    QPushButton* openFolderBtn = msgBox.addButton(tr("Open Folder"), QMessageBox::ActionRole);
    msgBox.addButton(QMessageBox::Close);

    if (!ExternalEditor::isVSCodeAvailable()) {
      openVSCodeBtn->setEnabled(false);
      openVSCodeBtn->setToolTip(tr("VS Code not found on this system"));
    }

    msgBox.exec();

    if (msgBox.clickedButton() == openVSCodeBtn) {
      externalEditor_->openFolderInVSCode(wizard.generatedPackagePath());
    } else if (msgBox.clickedButton() == openFolderBtn) {
      ExternalEditor::openWithSystemDefault(wizard.generatedPackagePath());
    }
  }
}

void MainWindow::onGenerateCode() {
  // Get output directory from user
  QString outputDir = QFileDialog::getExistingDirectory(
    this,
    tr("Select Output Directory for Generated Package"),
    QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
  );

  if (outputDir.isEmpty()) {
    return;
  }

  // Get package name from user
  bool ok;
  QString packageName = QInputDialog::getText(
    this,
    tr("Package Name"),
    tr("Enter the name for the generated ROS2 package:"),
    QLineEdit::Normal,
    "my_ros_package",
    &ok
  );

  if (!ok || packageName.isEmpty()) {
    return;
  }

  // Validate package name
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (!validName.match(packageName).hasMatch()) {
    QMessageBox::warning(this, tr("Invalid Package Name"),
      tr("Package name must start with a lowercase letter and contain only "
         "lowercase letters, numbers, and underscores."));
    return;
  }

  // Set up generation options
  GeneratorOptions options;
  options.outputPath = outputDir;
  options.packageName = packageName;
  options.generateLaunchFile = true;
  options.generateTests = false;
  options.useCppStyle = true;
  options.rosDistro = "humble";

  // Save the path to the generated package for VS Code integration
  lastGeneratedPackagePath_ = outputDir + "/" + packageName;

  // Export current project
  Project project;
  canvas_->exportToProject(project);

  if (project.blocks().isEmpty()) {
    QMessageBox::warning(this, tr("Empty Project"),
      tr("Cannot generate code from an empty project. Please add some nodes first."));
    return;
  }

  // Show progress
  outputPanel_->showProgress(true);
  outputPanel_->progressBar()->setValue(0);
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Starting code generation...\n"));
  outputPanel_->showBuildTab();

  // Generate the package
  codeGenerator_->generatePackage(project, options);
}

void MainWindow::onGenerationProgress(int percent, const QString& message) {
  outputPanel_->progressBar()->setValue(percent);
  outputPanel_->appendBuildOutput(message);
}

void MainWindow::onGenerationFinished(bool success) {
  outputPanel_->showProgress(false);

  if (success) {
    outputPanel_->appendBuildOutput(tr("\nCode generation completed successfully!"));
    outputPanel_->appendBuildOutput(tr("You can now build the package with: colcon build --packages-select <package_name>"));

    // Create custom dialog with VS Code option
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(tr("Code Generation Complete"));
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setText(tr("ROS2 package has been generated successfully!"));
    msgBox.setInformativeText(
      tr("Package location: %1\n\n"
         "To build:\n"
         "  cd <workspace>\n"
         "  colcon build\n\n"
         "To run:\n"
         "  source install/setup.bash\n"
         "  ros2 launch <package_name> <package_name>_launch.py")
      .arg(lastGeneratedPackagePath_));

    QPushButton* openVSCodeBtn = msgBox.addButton(tr("Open in VS Code"), QMessageBox::ActionRole);
    QPushButton* openFolderBtn = msgBox.addButton(tr("Open Folder"), QMessageBox::ActionRole);
    msgBox.addButton(QMessageBox::Close);

    // Only enable VS Code button if available
    if (!ExternalEditor::isVSCodeAvailable()) {
      openVSCodeBtn->setEnabled(false);
      openVSCodeBtn->setToolTip(tr("VS Code not found on this system"));
    }

    msgBox.exec();

    if (msgBox.clickedButton() == openVSCodeBtn) {
      externalEditor_->openFolderInVSCode(lastGeneratedPackagePath_);
    } else if (msgBox.clickedButton() == openFolderBtn) {
      ExternalEditor::openWithSystemDefault(lastGeneratedPackagePath_);
    }
  } else {
    outputPanel_->appendBuildError(tr("Code generation failed: %1").arg(codeGenerator_->lastError()));
    QMessageBox::critical(this, tr("Code Generation Failed"),
      tr("Failed to generate ROS2 package:\n%1").arg(codeGenerator_->lastError()));
  }
}

void MainWindow::onOpenProjectInVSCode() {
  if (currentProjectPath_.isEmpty()) {
    QMessageBox::information(this, tr("No Project Open"),
      tr("Please save the project first before opening in VS Code."));
    return;
  }

  QFileInfo projectInfo(currentProjectPath_);
  QString projectDir = projectInfo.absolutePath();

  if (!ExternalEditor::isVSCodeAvailable()) {
    QMessageBox::warning(this, tr("VS Code Not Found"),
      tr("Visual Studio Code was not found on this system.\n\n"
         "Please install VS Code and ensure the 'code' command is available in your PATH."));
    return;
  }

  if (!externalEditor_->openFolderInVSCode(projectDir)) {
    QMessageBox::warning(this, tr("Failed to Open VS Code"),
      tr("Could not open VS Code:\n%1").arg(externalEditor_->lastError()));
  }
}

void MainWindow::onOpenGeneratedPackageInVSCode() {
  if (lastGeneratedPackagePath_.isEmpty()) {
    QMessageBox::information(this, tr("No Package Generated"),
      tr("No package has been generated yet.\n\n"
         "Use File → Generate ROS2 Package to generate a package first."));
    return;
  }

  QFileInfo packageInfo(lastGeneratedPackagePath_);
  if (!packageInfo.exists()) {
    QMessageBox::warning(this, tr("Package Not Found"),
      tr("The generated package folder no longer exists:\n%1").arg(lastGeneratedPackagePath_));
    lastGeneratedPackagePath_.clear();
    return;
  }

  if (!ExternalEditor::isVSCodeAvailable()) {
    QMessageBox::warning(this, tr("VS Code Not Found"),
      tr("Visual Studio Code was not found on this system.\n\n"
         "Please install VS Code and ensure the 'code' command is available in your PATH."));
    return;
  }

  if (!externalEditor_->openFolderInVSCode(lastGeneratedPackagePath_)) {
    QMessageBox::warning(this, tr("Failed to Open VS Code"),
      tr("Could not open VS Code:\n%1").arg(externalEditor_->lastError()));
  }
}

void MainWindow::onOpenBlockInVSCode(PackageBlock* block) {
  if (!block) return;

  QString packageName = block->packageName();

  // First check if VS Code is available
  if (!ExternalEditor::isVSCodeAvailable()) {
    QMessageBox::warning(this, tr("VS Code Not Found"),
      tr("Visual Studio Code was not found on this system.\n\n"
         "Please install VS Code and ensure the 'code' command is available in your PATH."));
    return;
  }

  // Try to find the package in ROS2 workspace
  if (packageIndex_) {
    QString packagePath = packageIndex_->findPackagePath(packageName);
    if (!packagePath.isEmpty()) {
      if (!externalEditor_->openFolderInVSCode(packagePath)) {
        QMessageBox::warning(this, tr("Failed to Open VS Code"),
          tr("Could not open VS Code:\n%1").arg(externalEditor_->lastError()));
      }
      return;
    }
  }

  // Package not found in ROS2 workspace - offer alternatives
  QMessageBox msgBox(this);
  msgBox.setWindowTitle(tr("Package Not Found"));
  msgBox.setIcon(QMessageBox::Information);
  msgBox.setText(tr("The package '%1' was not found in the ROS2 workspace.\n\n"
                    "This may be a custom node or the package hasn't been installed yet.")
                    .arg(packageName));

  QPushButton* openProjectBtn = msgBox.addButton(tr("Open Project Folder"), QMessageBox::ActionRole);
  QPushButton* generateBtn = msgBox.addButton(tr("Generate Package"), QMessageBox::ActionRole);
  msgBox.addButton(QMessageBox::Cancel);

  msgBox.exec();

  if (msgBox.clickedButton() == openProjectBtn) {
    onOpenProjectInVSCode();
  } else if (msgBox.clickedButton() == generateBtn) {
    onGenerateCodeWizard();
  }
}

// Live topic monitoring slots

void MainWindow::onToggleLiveMonitoring(bool enabled) {
  liveMonitoringEnabled_ = enabled;

  if (enabled) {
    // Create topic monitor if needed
    if (!topicMonitor_) {
      topicMonitor_ = new TopicMonitor(this);
      connect(topicMonitor_, &TopicMonitor::topicActivity,
              this, &MainWindow::onTopicActivity);
    }

    // Create topic inspector popup if needed
    if (!topicInspector_) {
      topicInspector_ = new TopicInspectorPopup(this);
      topicInspector_->setTopicMonitor(topicMonitor_);
      connect(topicInspector_, &TopicInspectorPopup::echoTopicRequested,
              this, &MainWindow::onEchoTopicRequested);
    }

    topicMonitor_->startMonitoring();
    statusBar()->showMessage(tr("Live topic monitoring enabled"));

    // Enable live monitoring on all connections
    if (canvas_) {
      for (ConnectionLine* conn : canvas_->connections()) {
        conn->setLiveMonitoringEnabled(true);
      }
    }
  } else {
    if (topicMonitor_) {
      topicMonitor_->stopMonitoring();
    }
    statusBar()->showMessage(tr("Live topic monitoring disabled"));

    // Disable live monitoring on all connections
    if (canvas_) {
      for (ConnectionLine* conn : canvas_->connections()) {
        conn->setLiveMonitoringEnabled(false);
      }
    }
  }

  if (liveMonitoringAction_) {
    liveMonitoringAction_->setChecked(enabled);
  }
}

void MainWindow::onConnectionClicked(ConnectionLine* connection) {
  if (!connection || !liveMonitoringEnabled_) {
    return;
  }

  // Create inspector popup if needed
  if (!topicInspector_) {
    topicInspector_ = new TopicInspectorPopup(this);
    if (topicMonitor_) {
      topicInspector_->setTopicMonitor(topicMonitor_);
    }
    connect(topicInspector_, &TopicInspectorPopup::echoTopicRequested,
            this, &MainWindow::onEchoTopicRequested);
  }

  // Show popup near the mouse cursor
  QPoint globalPos = QCursor::pos();
  topicInspector_->showForConnection(connection, globalPos);
}

void MainWindow::onConnectionDoubleClicked(ConnectionLine* connection) {
  if (!connection) {
    return;
  }

  // Double-click could open echo in output panel
  QString topicName = connection->topicName();
  if (!topicName.isEmpty()) {
    onEchoTopicRequested(topicName);
  }
}

void MainWindow::onTopicActivity(const QString& topicName, double rate) {
  // Update connection lines that match this topic
  if (!canvas_) return;

  for (ConnectionLine* conn : canvas_->connections()) {
    if (conn->topicName() == topicName) {
      conn->setMessageRate(rate);

      // Update activity state based on rate
      if (rate > ConnectionLine::HIGH_RATE_THRESHOLD) {
        conn->setActivityState(TopicActivityState::HighRate);
      } else if (rate > 0) {
        conn->setActivityState(TopicActivityState::Active);
      } else {
        conn->setActivityState(TopicActivityState::Inactive);
      }

      // Pulse the connection to show activity
      conn->pulseActivity();
    }
  }
}

void MainWindow::onEchoTopicRequested(const QString& topicName) {
  if (topicName.isEmpty()) {
    return;
  }

  // Output the echo request to the output panel
  if (outputPanel_) {
    outputPanel_->appendBuildOutput(tr("\n--- Echoing topic: %1 ---").arg(topicName));
    outputPanel_->appendBuildOutput(tr("(Topic echo functionality will be available in future update)\n"));
  }

  // Show output dock if hidden
  if (outputDock_ && !outputDock_->isVisible()) {
    outputDock_->show();
  }
}

void MainWindow::onRosStatusTitleBarUpdate(const QString& suffix) {
  setWindowTitle(baseWindowTitle_ + suffix);
}

void MainWindow::onOpenSettings() {
  QDialog dialog(this);
  dialog.setWindowTitle(tr("Settings"));
  dialog.setMinimumWidth(400);

  QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

  // ROS2 Status Display group
  QGroupBox* ros2Group = new QGroupBox(tr("ROS2 Status Display"), &dialog);
  QVBoxLayout* ros2Layout = new QVBoxLayout(ros2Group);

  // Show ROS2 connection status checkbox
  QCheckBox* showStatusCheck = new QCheckBox(tr("Show ROS2 connection status"), ros2Group);
  showStatusCheck->setChecked(rosStatusWidget_->isRos2StatusVisible());
  ros2Layout->addWidget(showStatusCheck);

  // Show ROS_DOMAIN_ID checkbox
  QCheckBox* showDomainCheck = new QCheckBox(tr("Show ROS_DOMAIN_ID"), ros2Group);
  showDomainCheck->setChecked(rosStatusWidget_->isDomainIdVisible());
  ros2Layout->addWidget(showDomainCheck);

  // Highlight non-default Domain ID checkbox
  QCheckBox* highlightDomainCheck = new QCheckBox(tr("Highlight non-default Domain ID"), ros2Group);
  highlightDomainCheck->setChecked(rosStatusWidget_->isHighlightNonDefaultDomain());
  ros2Layout->addWidget(highlightDomainCheck);

  ros2Layout->addSpacing(10);

  // Display location radio buttons
  QLabel* locationLabel = new QLabel(tr("Display location:"), ros2Group);
  ros2Layout->addWidget(locationLabel);

  QRadioButton* statusBarOnlyRadio = new QRadioButton(tr("Status bar only (default)"), ros2Group);
  QRadioButton* titleBarOnlyRadio = new QRadioButton(tr("Title bar only"), ros2Group);
  QRadioButton* bothRadio = new QRadioButton(tr("Both status bar and title bar"), ros2Group);

  StatusDisplayLocation currentLocation = rosStatusWidget_->displayLocation();
  statusBarOnlyRadio->setChecked(currentLocation == StatusDisplayLocation::StatusBarOnly);
  titleBarOnlyRadio->setChecked(currentLocation == StatusDisplayLocation::TitleBarOnly);
  bothRadio->setChecked(currentLocation == StatusDisplayLocation::Both);

  ros2Layout->addWidget(statusBarOnlyRadio);
  ros2Layout->addWidget(titleBarOnlyRadio);
  ros2Layout->addWidget(bothRadio);

  mainLayout->addWidget(ros2Group);

  // System Discovery group
  QGroupBox* discoveryGroup = new QGroupBox(tr("System Discovery"), &dialog);
  QVBoxLayout* discoveryLayout = new QVBoxLayout(discoveryGroup);

  // Scan timeout setting
  QHBoxLayout* timeoutLayout = new QHBoxLayout();
  QLabel* timeoutLabel = new QLabel(tr("Scan timeout (seconds):"), discoveryGroup);
  QSpinBox* timeoutSpinBox = new QSpinBox(discoveryGroup);
  timeoutSpinBox->setRange(1, 30);
  timeoutSpinBox->setValue(systemDiscovery_ ? systemDiscovery_->scanTimeout() : 5);
  timeoutSpinBox->setToolTip(tr("Maximum time to wait for ROS2 system discovery (1-30 seconds)"));
  timeoutLayout->addWidget(timeoutLabel);
  timeoutLayout->addWidget(timeoutSpinBox);
  timeoutLayout->addStretch();
  discoveryLayout->addLayout(timeoutLayout);

  // Auto-scan interval setting
  QHBoxLayout* intervalLayout = new QHBoxLayout();
  QLabel* intervalLabel = new QLabel(tr("Auto-scan interval (seconds):"), discoveryGroup);
  QSpinBox* intervalSpinBox = new QSpinBox(discoveryGroup);
  intervalSpinBox->setRange(5, 60);
  intervalSpinBox->setValue(systemDiscovery_ ? systemDiscovery_->autoScanInterval() : 10);
  intervalSpinBox->setToolTip(tr("Interval between automatic system scans when auto-scan is enabled (5-60 seconds)"));
  intervalLayout->addWidget(intervalLabel);
  intervalLayout->addWidget(intervalSpinBox);
  intervalLayout->addStretch();
  discoveryLayout->addLayout(intervalLayout);

  mainLayout->addWidget(discoveryGroup);

  // Add stretch to push buttons to bottom
  mainLayout->addStretch();

  // Dialog buttons
  QDialogButtonBox* buttonBox = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  mainLayout->addWidget(buttonBox);

  if (dialog.exec() == QDialog::Accepted) {
    // Apply settings
    rosStatusWidget_->setShowRos2Status(showStatusCheck->isChecked());
    rosStatusWidget_->setShowDomainId(showDomainCheck->isChecked());
    rosStatusWidget_->setHighlightNonDefaultDomain(highlightDomainCheck->isChecked());

    if (statusBarOnlyRadio->isChecked()) {
      rosStatusWidget_->setDisplayLocation(StatusDisplayLocation::StatusBarOnly);
    } else if (titleBarOnlyRadio->isChecked()) {
      rosStatusWidget_->setDisplayLocation(StatusDisplayLocation::TitleBarOnly);
    } else if (bothRadio->isChecked()) {
      rosStatusWidget_->setDisplayLocation(StatusDisplayLocation::Both);
    }

    // Apply system discovery settings
    if (systemDiscovery_) {
      systemDiscovery_->setScanTimeout(timeoutSpinBox->value());
      systemDiscovery_->setAutoScanInterval(intervalSpinBox->value());
    }
  }
}

// System discovery slots

void MainWindow::onScanSystem() {
  if (!systemDiscovery_) return;

  systemDiscovery_->scan();
}

void MainWindow::onToggleAutoScan(bool enabled) {
  if (systemDiscovery_) {
    systemDiscovery_->setAutoScanEnabled(enabled);
  }
  if (autoScanAction_) {
    autoScanAction_->setChecked(enabled);
  }
}

void MainWindow::onScanStarted() {
  if (scanProgressBar_) {
    scanProgressBar_->setValue(0);
    scanProgressBar_->setFormat(tr("Scanning..."));
    scanProgressBar_->setVisible(true);
  }
  if (scanSystemAction_) {
    scanSystemAction_->setEnabled(false);
  }
  statusBar()->showMessage(tr("Scanning ROS2 system..."));
}

void MainWindow::onScanProgress(int percent, const QString& message) {
  if (scanProgressBar_) {
    scanProgressBar_->setValue(percent);
    scanProgressBar_->setFormat(message);
  }
}

void MainWindow::onScanCompleted(const SystemGraph& graph) {
  if (scanProgressBar_) {
    scanProgressBar_->setVisible(false);
  }
  if (scanSystemAction_) {
    scanSystemAction_->setEnabled(true);
  }

  statusBar()->showMessage(
    tr("Scan complete: %1 nodes, %2 topics found")
      .arg(graph.nodes.size())
      .arg(graph.topics.size()),
    5000
  );

  // Trigger mapping with current canvas
  if (canvasMapper_ && canvas_) {
    Project project;
    canvas_->exportToProject(project);
    canvasMapper_->mapCanvasToSystem(project, graph);
  }
}

void MainWindow::onScanTimedOut() {
  if (scanProgressBar_) {
    scanProgressBar_->setVisible(false);
  }
  if (scanSystemAction_) {
    scanSystemAction_->setEnabled(true);
  }

  statusBar()->showMessage(tr("Scan timed out"), 5000);
}

void MainWindow::onMappingCompleted(const MappingResults& results) {
  // Update canvas blocks with mapping results
  if (!canvas_) return;

  // Get all package blocks from the canvas
  QList<PackageBlock*> blocks;
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    PackageBlock* block = dynamic_cast<PackageBlock*>(item);
    if (block) {
      blocks.append(block);
    }
  }

  // Apply mapping results to each block
  for (const BlockMappingResult& mapping : results.blockMappings) {
    for (PackageBlock* block : blocks) {
      if (block->id() == mapping.canvasBlockId) {
        block->updateMappingResult(mapping);
        break;
      }
    }
  }

  // Update the mapping panel (it connects directly via signal)
  // Show summary in status bar
  const auto& s = results.summary;
  statusBar()->showMessage(
    tr("Mapping: %1/%2 nodes matched, %3/%4 topics active")
      .arg(s.matchedBlocks).arg(s.totalCanvasBlocks)
      .arg(s.activeTopics).arg(s.totalCanvasTopics),
    5000
  );
}

void MainWindow::onMappingBlockSelected(const QUuid& blockId) {
  if (!canvas_) return;

  // Find and select the block on the canvas
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    PackageBlock* block = dynamic_cast<PackageBlock*>(item);
    if (block && block->id() == blockId) {
      canvas_->scene()->clearSelection();
      block->setSelected(true);
      canvas_->centerOn(block);
      break;
    }
  }
}

// Topic viewer slots

void MainWindow::onTopicViewerTopicSelected(const QString& topicName) {
  if (!canvas_) return;

  // Highlight connections that use this topic
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    ConnectionLine* connection = dynamic_cast<ConnectionLine*>(item);
    if (connection) {
      bool matches = connection->topicName() == topicName;
      connection->setHighlighted(matches);
    }
  }

  statusBar()->showMessage(tr("Selected topic: %1").arg(topicName), 3000);
}

void MainWindow::onShowTopicOnCanvas(const QString& topicName) {
  if (!canvas_) return;

  // Find and center on a connection that uses this topic
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    ConnectionLine* connection = dynamic_cast<ConnectionLine*>(item);
    if (connection && connection->topicName() == topicName) {
      canvas_->scene()->clearSelection();
      connection->setSelected(true);
      canvas_->centerOn(connection);

      // Also highlight the connected blocks
      if (connection->sourceBlock()) {
        connection->sourceBlock()->setSelected(true);
      }
      if (connection->targetBlock()) {
        connection->targetBlock()->setSelected(true);
      }

      statusBar()->showMessage(tr("Showing topic: %1").arg(topicName), 3000);
      return;
    }
  }

  statusBar()->showMessage(tr("Topic not found on canvas: %1").arg(topicName), 3000);
}

}  // namespace ros_weaver
