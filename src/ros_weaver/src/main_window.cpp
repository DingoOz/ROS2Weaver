#include "ros_weaver/main_window.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/ros_package_index.hpp"
#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/core/external_editor.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"
#include "ros_weaver/widgets/output_panel.hpp"
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
{
  setWindowTitle("ROS Weaver - Visual ROS2 Package Editor");
  setMinimumSize(1200, 800);

  // Initialize core components
  packageIndex_ = new RosPackageIndex(this);
  codeGenerator_ = new CodeGenerator(this);
  externalEditor_ = new ExternalEditor(this);

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

  // Output Dock (bottom) - with tabs for Build Output, ROS Logs, and Terminal
  outputDock_ = new QDockWidget(tr("Output"), this);
  outputDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

  outputPanel_ = new OutputPanel();
  outputDock_->setWidget(outputPanel_);
  addDockWidget(Qt::BottomDockWidgetArea, outputDock_);

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
  statusBar()->showMessage(tr("Ready"));
}

void MainWindow::onNewProject() {
  // Clear param dashboard first to avoid dangling pointer when canvas clears blocks
  paramDashboard_->setCurrentBlock(nullptr);
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
  setWindowTitle("ROS Weaver - TurtleBot3 Navigation (Example)");

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
      outputText_->append(tr("\nPackage generated via wizard: %1").arg(path));
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

}  // namespace ros_weaver
