#include "ros_weaver/main_window.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/ros_package_index.hpp"
#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"

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
  , outputText_(nullptr)
  , progressBar_(nullptr)
  , packageIndex_(nullptr)
  , codeGenerator_(nullptr)
{
  setWindowTitle("ROS Weaver - Visual ROS2 Package Editor");
  setMinimumSize(1200, 800);

  // Initialize core components
  packageIndex_ = new RosPackageIndex(this);
  codeGenerator_ = new CodeGenerator(this);

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

  fileMenu->addSeparator();

  QAction* generateAction = fileMenu->addAction(tr("&Generate ROS2 Package..."));
  generateAction->setShortcut(tr("Ctrl+G"));
  generateAction->setToolTip(tr("Generate ROS2 package code from the current project"));
  connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateCode);

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

  // ROS2 menu
  QMenu* ros2Menu = menuBar()->addMenu(tr("&ROS2"));

  QAction* scanPackagesAction = ros2Menu->addAction(tr("&Scan Packages"));
  scanPackagesAction->setToolTip(tr("Scan workspace for ROS2 packages"));

  QAction* buildAction = ros2Menu->addAction(tr("&Build Workspace"));
  buildAction->setShortcut(tr("Ctrl+B"));

  QAction* launchAction = ros2Menu->addAction(tr("&Launch..."));
  launchAction->setShortcut(tr("Ctrl+L"));

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

  // Topics tab (placeholder for now)
  QWidget* topicsWidget = new QWidget();
  QVBoxLayout* topicsLayout = new QVBoxLayout(topicsWidget);
  QLabel* topicsLabel = new QLabel(tr("Topics and connections for the selected node will appear here."));
  topicsLabel->setWordWrap(true);
  topicsLayout->addWidget(topicsLabel);
  topicsLayout->addStretch();
  propertiesTab_->addTab(topicsWidget, tr("Topics"));

  propertiesDock_->setWidget(propertiesTab_);
  addDockWidget(Qt::RightDockWidgetArea, propertiesDock_);

  // Output Dock (bottom)
  outputDock_ = new QDockWidget(tr("Output"), this);
  outputDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

  QWidget* outputWidget = new QWidget();
  QVBoxLayout* outputLayout = new QVBoxLayout(outputWidget);
  outputLayout->setContentsMargins(4, 4, 4, 4);
  outputLayout->setSpacing(4);

  outputText_ = new QTextEdit();
  outputText_->setReadOnly(true);
  outputText_->setPlaceholderText(tr("Build and generation output will appear here..."));
  outputLayout->addWidget(outputText_);

  progressBar_ = new QProgressBar();
  progressBar_->setVisible(false);
  outputLayout->addWidget(progressBar_);

  outputDock_->setWidget(outputWidget);
  addDockWidget(Qt::BottomDockWidgetArea, outputDock_);
}

void MainWindow::setupCentralWidget() {
  canvas_ = new WeaverCanvas(this);
  setCentralWidget(canvas_);

  // Connect canvas signals
  connect(canvas_, &WeaverCanvas::blockSelected, this, &MainWindow::onBlockSelected);
}

void MainWindow::onBlockSelected(PackageBlock* block) {
  statusBar()->showMessage(tr("Selected: %1").arg(block ? block->packageName() : tr("None")));

  // Update param dashboard with selected block
  paramDashboard_->setCurrentBlock(block);
}

void MainWindow::setupStatusBar() {
  statusBar()->showMessage(tr("Ready"));
}

void MainWindow::onNewProject() {
  canvas_->clearCanvas();
  currentProjectPath_.clear();
  setWindowTitle("ROS Weaver - Visual ROS2 Package Editor");
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
      setWindowTitle(QString("ROS Weaver - %1").arg(QFileInfo(fileName).fileName()));
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
      setWindowTitle(QString("ROS Weaver - %1").arg(QFileInfo(fileName).fileName()));
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

  canvas_->importFromProject(project);
  return true;
}

void MainWindow::onLoadTurtleBotExample() {
  // Create a TurtleBot3 navigation example project programmatically
  Project project;
  project.metadata().name = "TurtleBot3 Navigation";
  project.metadata().description = "Example TurtleBot3 navigation system with SLAM";
  project.metadata().rosDistro = "humble";

  // Create blocks for TurtleBot3 navigation stack

  // 1. LiDAR sensor node
  BlockData lidarBlock;
  lidarBlock.id = QUuid::createUuid();
  lidarBlock.name = "lidar_sensor";
  lidarBlock.position = QPointF(-300, -100);
  PinData lidarOut;
  lidarOut.name = "scan";
  lidarOut.type = "output";
  lidarOut.dataType = "topic";
  lidarOut.messageType = "sensor_msgs/msg/LaserScan";
  lidarBlock.outputPins.append(lidarOut);
  project.addBlock(lidarBlock);

  // 2. Odometry publisher
  BlockData odomBlock;
  odomBlock.id = QUuid::createUuid();
  odomBlock.name = "odometry";
  odomBlock.position = QPointF(-300, 50);
  PinData odomOut;
  odomOut.name = "odom";
  odomOut.type = "output";
  odomOut.dataType = "topic";
  odomOut.messageType = "nav_msgs/msg/Odometry";
  odomBlock.outputPins.append(odomOut);
  project.addBlock(odomBlock);

  // 3. SLAM node (takes scan and odom, outputs map and tf)
  BlockData slamBlock;
  slamBlock.id = QUuid::createUuid();
  slamBlock.name = "slam_toolbox";
  slamBlock.position = QPointF(0, -50);
  PinData slamScanIn;
  slamScanIn.name = "scan";
  slamScanIn.type = "input";
  slamScanIn.dataType = "topic";
  slamScanIn.messageType = "sensor_msgs/msg/LaserScan";
  slamBlock.inputPins.append(slamScanIn);
  PinData slamOdomIn;
  slamOdomIn.name = "odom";
  slamOdomIn.type = "input";
  slamOdomIn.dataType = "topic";
  slamOdomIn.messageType = "nav_msgs/msg/Odometry";
  slamBlock.inputPins.append(slamOdomIn);
  PinData slamMapOut;
  slamMapOut.name = "map";
  slamMapOut.type = "output";
  slamMapOut.dataType = "topic";
  slamMapOut.messageType = "nav_msgs/msg/OccupancyGrid";
  slamBlock.outputPins.append(slamMapOut);
  project.addBlock(slamBlock);

  // 4. Nav2 Planner (takes map, outputs path)
  BlockData plannerBlock;
  plannerBlock.id = QUuid::createUuid();
  plannerBlock.name = "nav2_planner";
  plannerBlock.position = QPointF(300, -100);
  PinData plannerMapIn;
  plannerMapIn.name = "map";
  plannerMapIn.type = "input";
  plannerMapIn.dataType = "topic";
  plannerMapIn.messageType = "nav_msgs/msg/OccupancyGrid";
  plannerBlock.inputPins.append(plannerMapIn);
  PinData plannerGoalIn;
  plannerGoalIn.name = "goal_pose";
  plannerGoalIn.type = "input";
  plannerGoalIn.dataType = "topic";
  plannerGoalIn.messageType = "geometry_msgs/msg/PoseStamped";
  plannerBlock.inputPins.append(plannerGoalIn);
  PinData plannerPathOut;
  plannerPathOut.name = "path";
  plannerPathOut.type = "output";
  plannerPathOut.dataType = "topic";
  plannerPathOut.messageType = "nav_msgs/msg/Path";
  plannerBlock.outputPins.append(plannerPathOut);
  project.addBlock(plannerBlock);

  // 5. Nav2 Controller (takes path, outputs cmd_vel)
  BlockData controllerBlock;
  controllerBlock.id = QUuid::createUuid();
  controllerBlock.name = "nav2_controller";
  controllerBlock.position = QPointF(300, 100);
  PinData controllerPathIn;
  controllerPathIn.name = "path";
  controllerPathIn.type = "input";
  controllerPathIn.dataType = "topic";
  controllerPathIn.messageType = "nav_msgs/msg/Path";
  controllerBlock.inputPins.append(controllerPathIn);
  PinData controllerOdomIn;
  controllerOdomIn.name = "odom";
  controllerOdomIn.type = "input";
  controllerOdomIn.dataType = "topic";
  controllerOdomIn.messageType = "nav_msgs/msg/Odometry";
  controllerBlock.inputPins.append(controllerOdomIn);
  PinData controllerCmdOut;
  controllerCmdOut.name = "cmd_vel";
  controllerCmdOut.type = "output";
  controllerCmdOut.dataType = "topic";
  controllerCmdOut.messageType = "geometry_msgs/msg/Twist";
  controllerBlock.outputPins.append(controllerCmdOut);
  project.addBlock(controllerBlock);

  // 6. Motor driver (takes cmd_vel)
  BlockData motorBlock;
  motorBlock.id = QUuid::createUuid();
  motorBlock.name = "diff_drive";
  motorBlock.position = QPointF(600, 100);
  PinData motorCmdIn;
  motorCmdIn.name = "cmd_vel";
  motorCmdIn.type = "input";
  motorCmdIn.dataType = "topic";
  motorCmdIn.messageType = "geometry_msgs/msg/Twist";
  motorBlock.inputPins.append(motorCmdIn);
  project.addBlock(motorBlock);

  // 7. Goal pose publisher (user input)
  BlockData goalBlock;
  goalBlock.id = QUuid::createUuid();
  goalBlock.name = "goal_publisher";
  goalBlock.position = QPointF(0, -200);
  PinData goalOut;
  goalOut.name = "goal_pose";
  goalOut.type = "output";
  goalOut.dataType = "topic";
  goalOut.messageType = "geometry_msgs/msg/PoseStamped";
  goalBlock.outputPins.append(goalOut);
  project.addBlock(goalBlock);

  // Create connections
  // LiDAR -> SLAM
  ConnectionData lidarToSlam;
  lidarToSlam.id = QUuid::createUuid();
  lidarToSlam.sourceBlockId = lidarBlock.id;
  lidarToSlam.sourcePinIndex = 0;
  lidarToSlam.targetBlockId = slamBlock.id;
  lidarToSlam.targetPinIndex = 0;
  project.addConnection(lidarToSlam);

  // Odom -> SLAM
  ConnectionData odomToSlam;
  odomToSlam.id = QUuid::createUuid();
  odomToSlam.sourceBlockId = odomBlock.id;
  odomToSlam.sourcePinIndex = 0;
  odomToSlam.targetBlockId = slamBlock.id;
  odomToSlam.targetPinIndex = 1;
  project.addConnection(odomToSlam);

  // SLAM -> Planner
  ConnectionData slamToPlanner;
  slamToPlanner.id = QUuid::createUuid();
  slamToPlanner.sourceBlockId = slamBlock.id;
  slamToPlanner.sourcePinIndex = 0;
  slamToPlanner.targetBlockId = plannerBlock.id;
  slamToPlanner.targetPinIndex = 0;
  project.addConnection(slamToPlanner);

  // Goal -> Planner
  ConnectionData goalToPlanner;
  goalToPlanner.id = QUuid::createUuid();
  goalToPlanner.sourceBlockId = goalBlock.id;
  goalToPlanner.sourcePinIndex = 0;
  goalToPlanner.targetBlockId = plannerBlock.id;
  goalToPlanner.targetPinIndex = 1;
  project.addConnection(goalToPlanner);

  // Planner -> Controller
  ConnectionData plannerToController;
  plannerToController.id = QUuid::createUuid();
  plannerToController.sourceBlockId = plannerBlock.id;
  plannerToController.sourcePinIndex = 0;
  plannerToController.targetBlockId = controllerBlock.id;
  plannerToController.targetPinIndex = 0;
  project.addConnection(plannerToController);

  // Odom -> Controller
  ConnectionData odomToController;
  odomToController.id = QUuid::createUuid();
  odomToController.sourceBlockId = odomBlock.id;
  odomToController.sourcePinIndex = 0;
  odomToController.targetBlockId = controllerBlock.id;
  odomToController.targetPinIndex = 1;
  project.addConnection(odomToController);

  // Controller -> Motor
  ConnectionData controllerToMotor;
  controllerToMotor.id = QUuid::createUuid();
  controllerToMotor.sourceBlockId = controllerBlock.id;
  controllerToMotor.sourcePinIndex = 0;
  controllerToMotor.targetBlockId = motorBlock.id;
  controllerToMotor.targetPinIndex = 0;
  project.addConnection(controllerToMotor);

  // Import the project
  canvas_->importFromProject(project);
  currentProjectPath_.clear();
  setWindowTitle("ROS Weaver - TurtleBot3 Navigation (Example)");
  statusBar()->showMessage(tr("Loaded TurtleBot3 Navigation example"));
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

  // Export current project
  Project project;
  canvas_->exportToProject(project);

  if (project.blocks().isEmpty()) {
    QMessageBox::warning(this, tr("Empty Project"),
      tr("Cannot generate code from an empty project. Please add some nodes first."));
    return;
  }

  // Show progress
  progressBar_->setVisible(true);
  progressBar_->setValue(0);
  outputText_->clear();
  outputText_->append(tr("Starting code generation...\n"));

  // Generate the package
  codeGenerator_->generatePackage(project, options);
}

void MainWindow::onGenerationProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  outputText_->append(message);
}

void MainWindow::onGenerationFinished(bool success) {
  progressBar_->setVisible(false);

  if (success) {
    outputText_->append(tr("\nCode generation completed successfully!"));
    outputText_->append(tr("You can now build the package with: colcon build --packages-select <package_name>"));
    QMessageBox::information(this, tr("Code Generation Complete"),
      tr("ROS2 package has been generated successfully!\n\n"
         "To build:\n"
         "  cd <workspace>\n"
         "  colcon build\n\n"
         "To run:\n"
         "  source install/setup.bash\n"
         "  ros2 launch <package_name> <package_name>_launch.py"));
  } else {
    outputText_->append(tr("\nCode generation failed: %1").arg(codeGenerator_->lastError()));
    QMessageBox::critical(this, tr("Code Generation Failed"),
      tr("Failed to generate ROS2 package:\n%1").arg(codeGenerator_->lastError()));
  }
}

}  // namespace ros_weaver
