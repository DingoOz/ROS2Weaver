#include "ros_weaver/main_window.hpp"
#include <rclcpp/rclcpp.hpp>
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/widgets/llm_chat_widget.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/ros_package_index.hpp"
#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/core/external_editor.hpp"
#include "ros_weaver/core/topic_monitor.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"
#include "ros_weaver/widgets/output_panel.hpp"
#include "ros_weaver/widgets/topic_inspector.hpp"
#include "ros_weaver/widgets/ros_status_widget.hpp"
#include "ros_weaver/widgets/local_ai_status_widget.hpp"
#include "ros_weaver/widgets/system_mapping_panel.hpp"
#include "ros_weaver/widgets/topic_viewer_panel.hpp"
#include "ros_weaver/widgets/tf_tree_panel.hpp"
#include "ros_weaver/widgets/plot_panel.hpp"
#include "ros_weaver/core/system_discovery.hpp"
#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include "ros_weaver/wizards/package_wizard.hpp"
#include "ros_weaver/wizards/robot_wizard.hpp"
#include "ros_weaver/widgets/ollama_settings_widget.hpp"
#include "ros_weaver/widgets/help_browser.hpp"
#include "ros_weaver/widgets/keyboard_shortcuts_dialog.hpp"
#include "ros_weaver/widgets/guided_tour.hpp"
#include "ros_weaver/widgets/topic_echo_dialog.hpp"
#include "ros_weaver/widgets/rosbag_workbench_panel.hpp"
#include "ros_weaver/core/bag_recorder.hpp"
#include "ros_weaver/widgets/mcp_explorer_panel.hpp"
#include "ros_weaver/widgets/ros_control_panel.hpp"
#include "ros_weaver/wizards/mcp_server_wizard.hpp"
#include "ros_weaver/core/mcp_manager.hpp"
#include "ros_weaver/core/mcp_providers.hpp"
#include "ros_weaver/core/context_help.hpp"
#include "ros_weaver/core/constants.hpp"
#include "ros_weaver/core/graph_exporter.hpp"
#include "ros_weaver/widgets/toast_notification.hpp"
#include "ros_weaver/widgets/command_palette.hpp"
#include "ros_weaver/widgets/empty_state.hpp"
#include "ros_weaver/core/error_handler.hpp"
#include "ros_weaver/widgets/schema_viewer_widget.hpp"
#include "ros_weaver/widgets/readme_preview_panel.hpp"
#include "ros_weaver/widgets/issue_list_panel.hpp"
#include "ros_weaver/widgets/advanced_search_panel.hpp"
#include "ros_weaver/widgets/theme_editor_dialog.hpp"
#include "ros_weaver/widgets/remapping_editor.hpp"
#include "ros_weaver/widgets/canvas_tab_widget.hpp"
#include "ros_weaver/widgets/minimap_panel.hpp"
#include "ros_weaver/widgets/node_templates_panel.hpp"
#include "ros_weaver/widgets/workspace_browser_panel.hpp"
#include "ros_weaver/widgets/node_health_dashboard.hpp"
#include "ros_weaver/widgets/diff_view_dialog.hpp"
#include "ros_weaver/widgets/message_inspector_panel.hpp"
#include "ros_weaver/widgets/lifecycle_panel.hpp"
#include "ros_weaver/widgets/remote_connection_dialog.hpp"
#include "ros_weaver/core/remote_connection_manager.hpp"
#include "ros_weaver/core/dot_importer.hpp"
#include "ros_weaver/core/caret_importer.hpp"
#include "ros_weaver/core/static_analyzer.hpp"
#include "ros_weaver/core/simulation_launcher.hpp"
#include "ros_weaver/widgets/launch_preview_dialog.hpp"
#include "ros_weaver/widgets/architecture_doc_dialog.hpp"
#include "ros_weaver/core/architecture_doc_generator.hpp"
#include "ros_weaver/widgets/preset_selector_widget.hpp"
#include "ros_weaver/widgets/scenario_editor_widget.hpp"
#include "ros_weaver/widgets/latency_heatmap_panel.hpp"
#include "ros_weaver/core/latency_tracker.hpp"
#include "ros_weaver/widgets/diagnostics_panel.hpp"
#include "ros_weaver/core/network_topology_manager.hpp"
#include "ros_weaver/widgets/network_topology_panel.hpp"
#include "ros_weaver/widgets/behavior_tree_panel.hpp"
#include "ros_weaver/widgets/dock_drop_overlay.hpp"
#include "ros_weaver/widgets/mission_planner_panel.hpp"
#ifdef HAVE_QT3D
#include "ros_weaver/widgets/urdf_viewer_panel.hpp"
#endif

#include <QApplication>
#include <QCloseEvent>
#include <QSettings>
#include <QDesktopServices>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QAction>
#include <QActionGroup>
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
#include <QComboBox>
#include <QRadioButton>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QColorDialog>
#include <QGridLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace ros_weaver {

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
  , canvasTabWidget_(nullptr)
  , canvas_(nullptr)
  , packageBrowserDock_(nullptr)
  , propertiesDock_(nullptr)
  , outputDock_(nullptr)
  , packageTree_(nullptr)
  , packageSearchEdit_(nullptr)
  , searchResultsItem_(nullptr)
  , localPackagesItem_(nullptr)
  , browserTab_(nullptr)
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
  , localAIStatusWidget_(nullptr)
  , baseWindowTitle_("ROS Weaver - Visual ROS2 Package Editor")
  , systemDiscovery_(nullptr)
  , canvasMapper_(nullptr)
  , systemMappingPanel_(nullptr)
  , scanSystemAction_(nullptr)
  , autoScanAction_(nullptr)
  , scanProgressBar_(nullptr)
  , topicViewerPanel_(nullptr)
  , topicViewerDock_(nullptr)
  , tfTreePanel_(nullptr)
  , plotPanel_(nullptr)
  , guidedTour_(nullptr)
  , undoStack_(nullptr)
  , undoAction_(nullptr)
  , redoAction_(nullptr)
  , rosbagWorkbenchPanel_(nullptr)
  , rosbagWorkbenchDock_(nullptr)
  , mcpExplorerPanel_(nullptr)
  , mcpExplorerDock_(nullptr)
  , rosControlPanel_(nullptr)
  , schemaViewerWidget_(nullptr)
  , schemaViewerDock_(nullptr)
  , readmePreviewPanel_(nullptr)
  , readmePreviewDock_(nullptr)
  , issueListPanel_(nullptr)
  , issueListDock_(nullptr)
  , advancedSearchPanel_(nullptr)
  , isProjectDirty_(false)
  , autoSaveTimer_(nullptr)
  , autoSaveEnabled_(true)
  , autoSaveIntervalMs_(constants::timing::AUTO_SAVE_INTERVAL_MS)
  , recentProjectsMenu_(nullptr)
  , commandPalette_(nullptr)
  , layoutPresetsMenu_(nullptr)
  , zoomIndicator_(nullptr)
  , messageInspectorPanel_(nullptr)
  , messageInspectorDock_(nullptr)
  , lifecyclePanel_(nullptr)
  , lifecycleDock_(nullptr)
  , remoteConnectionManager_(nullptr)
  , connectRemoteAction_(nullptr)
  , disconnectRemoteAction_(nullptr)
  , latencyTracker_(nullptr)
  , latencyHeatmapPanel_(nullptr)
  , latencyHeatmapDock_(nullptr)
  , scenarioEditorWidget_(nullptr)
  , scenarioEditorDock_(nullptr)
  , latencyHeatmapAction_(nullptr)
  , diagnosticsPanel_(nullptr)
  , diagnosticsDock_(nullptr)
  , networkTopologyManager_(nullptr)
  , networkTopologyPanel_(nullptr)
  , networkTopologyDock_(nullptr)
  , behaviorTreePanel_(nullptr)
  , behaviorTreeDock_(nullptr)
{
  setWindowTitle(baseWindowTitle_);
  setMinimumSize(constants::ui::MIN_WINDOW_WIDTH, constants::ui::MIN_WINDOW_HEIGHT);

  // Initialize core components
  packageIndex_ = new RosPackageIndex(this);
  codeGenerator_ = new CodeGenerator(this);
  externalEditor_ = new ExternalEditor(this);
  undoStack_ = new UndoStack(this);

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
  setupCommandPalette();

  // Initialize context-sensitive help system (F1 key support)
  ContextHelp::instance();

  // Initialize notification manager
  NotificationManager::instance().setParentWidget(this);

  // Setup auto-save timer
  autoSaveTimer_ = new QTimer(this);
  connect(autoSaveTimer_, &QTimer::timeout, this, &MainWindow::onAutoSave);
  if (autoSaveEnabled_) {
    autoSaveTimer_->start(autoSaveIntervalMs_);
  }

  // Connect canvas modification signals for dirty tracking
  connect(canvas_, &WeaverCanvas::connectionCreated, this, &MainWindow::onProjectModified);
  connect(canvas_, &WeaverCanvas::groupCreated, this, &MainWindow::onProjectModified);
  connect(canvas_, &WeaverCanvas::blockYamlSourceChanged, this, &MainWindow::onProjectModified);

  // Connect undo stack signals for dirty tracking
  connect(undoStack_, &UndoStack::indexChanged, this, &MainWindow::onProjectModified);
}

MainWindow::~MainWindow() {
  // CRITICAL: Shutdown ROS2 FIRST to unblock all spin threads
  // Spin threads check rclcpp::ok() in their loops - this makes them exit
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  // Explicitly delete dock widgets in controlled order to ensure clean shutdown.
  // This prevents issues with signal-slot connections to destroyed objects.
  delete behaviorTreeDock_;
  behaviorTreeDock_ = nullptr;
  delete networkTopologyDock_;
  networkTopologyDock_ = nullptr;
  delete diagnosticsDock_;
  diagnosticsDock_ = nullptr;
  delete lifecycleDock_;
  lifecycleDock_ = nullptr;
  delete messageInspectorDock_;
  messageInspectorDock_ = nullptr;
  delete mcpExplorerDock_;
  mcpExplorerDock_ = nullptr;
  delete latencyHeatmapDock_;
  latencyHeatmapDock_ = nullptr;
  delete workspaceBrowserDock_;
  workspaceBrowserDock_ = nullptr;
  delete nodeTemplatesDock_;
  nodeTemplatesDock_ = nullptr;
  delete minimapDock_;
  minimapDock_ = nullptr;
  delete issueListDock_;
  issueListDock_ = nullptr;
  delete readmePreviewDock_;
  readmePreviewDock_ = nullptr;
  delete schemaViewerDock_;
  schemaViewerDock_ = nullptr;
  delete nodeHealthDock_;
  nodeHealthDock_ = nullptr;
  delete scenarioEditorDock_;
  scenarioEditorDock_ = nullptr;
  delete rosbagWorkbenchDock_;
  rosbagWorkbenchDock_ = nullptr;
  delete outputDock_;
  outputDock_ = nullptr;
  delete propertiesDock_;
  propertiesDock_ = nullptr;
  delete packageBrowserDock_;
  packageBrowserDock_ = nullptr;
}

void MainWindow::setupMenuBar() {
  // Force Qt menu bar instead of native (fixes issues on some Linux desktops)
  menuBar()->setNativeMenuBar(false);

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

  // Export submenu
  QMenu* exportMenu = fileMenu->addMenu(tr("&Export Diagram"));

  QAction* exportPlantUMLAction = exportMenu->addAction(tr("Export to &PlantUML..."));
  exportPlantUMLAction->setToolTip(tr("Export the canvas diagram to PlantUML format"));
  connect(exportPlantUMLAction, &QAction::triggered, this, &MainWindow::onExportToPlantUML);

  QAction* exportMermaidAction = exportMenu->addAction(tr("Export to &Mermaid..."));
  exportMermaidAction->setToolTip(tr("Export the canvas diagram to Mermaid format for documentation"));
  connect(exportMermaidAction, &QAction::triggered, this, &MainWindow::onExportToMermaid);

  QAction* exportGraphvizAction = exportMenu->addAction(tr("Export to &Graphviz DOT..."));
  exportGraphvizAction->setToolTip(tr("Export the canvas diagram to Graphviz DOT format"));
  connect(exportGraphvizAction, &QAction::triggered, this, &MainWindow::onExportToGraphviz);

  exportMenu->addSeparator();

  QAction* generateLaunchAction = exportMenu->addAction(tr("Generate &Launch File..."));
  generateLaunchAction->setToolTip(tr("Generate a ROS2 launch file from the canvas configuration"));
  generateLaunchAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_L));
  connect(generateLaunchAction, &QAction::triggered, this, [this]() {
    Project project;
    canvas_->exportToProject(project);

    LaunchPreviewDialog dialog(&project, this);
    dialog.exec();
  });

  QAction* generateDocAction = exportMenu->addAction(tr("Generate &Documentation..."));
  generateDocAction->setToolTip(tr("Generate architecture documentation in Markdown, HTML, or PDF format"));
  generateDocAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_D));
  connect(generateDocAction, &QAction::triggered, this, &MainWindow::onGenerateDocumentation);

  // Import submenu
  QMenu* importMenu = fileMenu->addMenu(tr("&Import"));

  QAction* importDotAction = importMenu->addAction(tr("Import from rqt_graph &DOT..."));
  importDotAction->setToolTip(tr("Import a DOT file generated by rqt_graph"));
  connect(importDotAction, &QAction::triggered, this, &MainWindow::onImportFromDot);

  QAction* importCaretAction = importMenu->addAction(tr("Import from &CARET YAML..."));
  importCaretAction->setToolTip(tr("Import a CARET architecture YAML file"));
  connect(importCaretAction, &QAction::triggered, this, &MainWindow::onImportFromCaret);

  fileMenu->addSeparator();

  // Recent projects submenu
  recentProjectsMenu_ = fileMenu->addMenu(tr("&Recent Projects"));
  updateRecentProjectsMenu();

  fileMenu->addSeparator();

  // Examples submenu
  QMenu* examplesMenu = fileMenu->addMenu(tr("&Examples"));
  QAction* turtleBotAction = examplesMenu->addAction(tr("&TurtleBot3 Navigation"));
  connect(turtleBotAction, &QAction::triggered, this, &MainWindow::onLoadTurtleBotExample);

  QAction* turtlesimAction = examplesMenu->addAction(tr("Turtle&sim Teleop"));
  turtlesimAction->setToolTip(tr("Load the turtlesim teleop example - control a turtle with keyboard"));
  connect(turtlesimAction, &QAction::triggered, this, &MainWindow::onLoadTurtlesimExample);

  QAction* behaviorTreeAction = examplesMenu->addAction(tr("&Behavior Tree Patrol"));
  behaviorTreeAction->setToolTip(tr("Load the behavior tree patrol example - demonstrates Nav2 BT-driven navigation"));
  connect(behaviorTreeAction, &QAction::triggered, this, &MainWindow::onLoadBehaviorTreeExample);

#ifdef HAVE_QT3D
  QAction* turtleBot3URDFAction = examplesMenu->addAction(tr("TurtleBot3 &URDF Viewer"));
  turtleBot3URDFAction->setToolTip(tr("Load TurtleBot3 robot model in the URDF Viewer"));
  connect(turtleBot3URDFAction, &QAction::triggered, this, &MainWindow::onLoadTurtleBot3URDFExample);
#endif

  examplesMenu->addSeparator();

  QAction* launchTurtlesimAction = examplesMenu->addAction(tr("&Launch Turtlesim Nodes"));
  launchTurtlesimAction->setToolTip(tr("Launch the turtlesim_node and turtle_teleop_key nodes"));
  connect(launchTurtlesimAction, &QAction::triggered, this, &MainWindow::onLaunchTurtlesim);

  QAction* launchTurtleBot3Action = examplesMenu->addAction(tr("Launch TurtleBot3 &Gazebo"));
  launchTurtleBot3Action->setToolTip(tr("Launch TurtleBot3 simulation in Gazebo with SLAM and Nav2"));
  connect(launchTurtleBot3Action, &QAction::triggered, this, &MainWindow::onLaunchTurtleBot3Gazebo);

  fileMenu->addSeparator();

  QAction* generateAction = fileMenu->addAction(tr("&Generate ROS2 Package..."));
  generateAction->setShortcut(tr("Ctrl+G"));
  generateAction->setToolTip(tr("Generate ROS2 package code from the current project"));
  connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateCode);

  QAction* generateWizardAction = fileMenu->addAction(tr("Generate ROS2 Package (&Wizard)..."));
  generateWizardAction->setShortcut(tr("Ctrl+Shift+G"));
  generateWizardAction->setToolTip(tr("Step-by-step wizard for generating ROS2 packages with full customization"));
  connect(generateWizardAction, &QAction::triggered, this, &MainWindow::onGenerateCodeWizard);

  QAction* robotWizardAction = fileMenu->addAction(tr("&Robot Configuration Wizard..."));
  robotWizardAction->setShortcut(tr("Ctrl+R"));
  robotWizardAction->setToolTip(tr("Create a new robot with ros2_control support - configure joints, sensors, and controllers"));
  connect(robotWizardAction, &QAction::triggered, this, &MainWindow::onRobotWizard);

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

  undoAction_ = editMenu->addAction(tr("&Undo"));
  undoAction_->setShortcut(QKeySequence::Undo);
  undoAction_->setEnabled(false);
  connect(undoAction_, &QAction::triggered, undoStack_, &UndoStack::undo);
  connect(undoStack_, &UndoStack::canUndoChanged, undoAction_, &QAction::setEnabled);
  connect(undoStack_, &UndoStack::undoTextChanged, this, [this](const QString& text) {
    undoAction_->setText(text.isEmpty() ? tr("&Undo") : tr("&Undo %1").arg(text));
  });

  redoAction_ = editMenu->addAction(tr("&Redo"));
  redoAction_->setShortcut(QKeySequence::Redo);
  redoAction_->setEnabled(false);
  connect(redoAction_, &QAction::triggered, undoStack_, &UndoStack::redo);
  connect(undoStack_, &UndoStack::canRedoChanged, redoAction_, &QAction::setEnabled);
  connect(undoStack_, &UndoStack::redoTextChanged, this, [this](const QString& text) {
    redoAction_->setText(text.isEmpty() ? tr("&Redo") : tr("&Redo %1").arg(text));
  });

  editMenu->addSeparator();

  QAction* deleteAction = editMenu->addAction(tr("&Delete"));
  deleteAction->setShortcut(QKeySequence::Delete);
  connect(deleteAction, &QAction::triggered, canvas_, &WeaverCanvas::deleteSelectedItems);

  editMenu->addSeparator();

  // Command palette
  QAction* commandPaletteAction = editMenu->addAction(tr("Command &Palette..."));
  commandPaletteAction->setShortcut(tr("Ctrl+Shift+P"));
  commandPaletteAction->setToolTip(tr("Open command palette for quick access to all commands"));
  connect(commandPaletteAction, &QAction::triggered, this, &MainWindow::onShowCommandPalette);

  QAction* settingsAction = editMenu->addAction(tr("&Settings..."));
  settingsAction->setShortcut(tr("Ctrl+,"));
  settingsAction->setToolTip(tr("Open application settings"));
  connect(settingsAction, &QAction::triggered, this, &MainWindow::onOpenSettings);

  editMenu->addSeparator();

  QAction* themeEditorAction = editMenu->addAction(tr("&Theme Editor..."));
  themeEditorAction->setToolTip(tr("Create and customize color themes"));
  connect(themeEditorAction, &QAction::triggered, this, &MainWindow::onShowThemeEditor);

  editMenu->addSeparator();

  QAction* diffViewAction = editMenu->addAction(tr("Compare &Projects..."));
  diffViewAction->setToolTip(tr("Compare two project files to see differences"));
  connect(diffViewAction, &QAction::triggered, this, [this]() {
    // Open file dialogs to select two projects
    QString beforePath = QFileDialog::getOpenFileName(
      this,
      tr("Select 'Before' Project"),
      QString(),
      tr("ROS Weaver Projects (*.rwproj *.json);;All Files (*)")
    );
    if (beforePath.isEmpty()) return;

    QString afterPath = QFileDialog::getOpenFileName(
      this,
      tr("Select 'After' Project"),
      QString(),
      tr("ROS Weaver Projects (*.rwproj *.json);;All Files (*)")
    );
    if (afterPath.isEmpty()) return;

    DiffViewDialog dialog(this);
    dialog.compareFiles(beforePath, afterPath);
    dialog.exec();
  });

  // View menu
  QMenu* viewMenu = menuBar()->addMenu(tr("&View"));

  // Canvas submenu for multi-canvas management
  QMenu* canvasMenu = viewMenu->addMenu(tr("&Canvas"));

  QAction* newCanvasAction = canvasMenu->addAction(tr("&New Canvas"));
  newCanvasAction->setShortcut(tr("Ctrl+Shift+N"));
  newCanvasAction->setToolTip(tr("Create a new canvas tab"));
  connect(newCanvasAction, &QAction::triggered, this, &MainWindow::onNewCanvas);

  QAction* closeCanvasAction = canvasMenu->addAction(tr("&Close Canvas"));
  closeCanvasAction->setShortcut(tr("Ctrl+W"));
  closeCanvasAction->setToolTip(tr("Close the current canvas tab"));
  connect(closeCanvasAction, &QAction::triggered, this, &MainWindow::onCloseCanvas);

  QAction* duplicateCanvasAction = canvasMenu->addAction(tr("&Duplicate Canvas"));
  duplicateCanvasAction->setToolTip(tr("Duplicate the current canvas and its contents"));
  connect(duplicateCanvasAction, &QAction::triggered, this, &MainWindow::onDuplicateCanvas);

  canvasMenu->addSeparator();

  QAction* renameCanvasAction = canvasMenu->addAction(tr("&Rename Canvas..."));
  renameCanvasAction->setShortcut(tr("F2"));
  renameCanvasAction->setToolTip(tr("Rename the current canvas tab"));
  connect(renameCanvasAction, &QAction::triggered, this, &MainWindow::onRenameCanvas);

  viewMenu->addSeparator();

  QAction* zoomInAction = viewMenu->addAction(tr("Zoom &In"));
  zoomInAction->setShortcut(QKeySequence::ZoomIn);
  connect(zoomInAction, &QAction::triggered, this, [this]() {
    if (canvas_) canvas_->zoomIn();
  });

  QAction* zoomOutAction = viewMenu->addAction(tr("Zoom &Out"));
  zoomOutAction->setShortcut(QKeySequence::ZoomOut);
  connect(zoomOutAction, &QAction::triggered, this, [this]() {
    if (canvas_) canvas_->zoomOut();
  });

  QAction* resetZoomAction = viewMenu->addAction(tr("&Reset Zoom"));
  resetZoomAction->setShortcut(tr("Ctrl+0"));
  connect(resetZoomAction, &QAction::triggered, this, [this]() {
    if (canvas_) canvas_->resetZoom();
  });

  viewMenu->addSeparator();

  QAction* fitAllAction = viewMenu->addAction(tr("&Fit All Nodes"));
  fitAllAction->setShortcut(tr("F"));
  fitAllAction->setToolTip(tr("Fit all nodes in the view"));
  connect(fitAllAction, &QAction::triggered, this, [this]() {
    if (canvas_) canvas_->fitToContents();
  });

  viewMenu->addSeparator();

  // Grid submenu
  QMenu* gridMenu = viewMenu->addMenu(tr("&Grid"));

  QAction* showGridAction = gridMenu->addAction(tr("Show &Grid"));
  showGridAction->setCheckable(true);
  showGridAction->setChecked(true);  // Grid enabled by default
  showGridAction->setShortcut(tr("G"));
  showGridAction->setToolTip(tr("Toggle grid visibility"));
  connect(showGridAction, &QAction::triggered, this, [this](bool checked) {
    if (canvas_) canvas_->setGridEnabled(checked);
  });

  QAction* snapToGridAction = gridMenu->addAction(tr("&Snap to Grid"));
  snapToGridAction->setCheckable(true);
  snapToGridAction->setChecked(false);  // Snap disabled by default
  snapToGridAction->setShortcut(tr("Ctrl+G"));
  snapToGridAction->setToolTip(tr("Snap nodes to grid when moving"));
  connect(snapToGridAction, &QAction::triggered, this, [this](bool checked) {
    if (canvas_) canvas_->setSnapToGridEnabled(checked);
  });

  gridMenu->addSeparator();

  // Grid spacing submenu
  QMenu* gridSpacingMenu = gridMenu->addMenu(tr("Grid &Spacing"));

  QActionGroup* spacingGroup = new QActionGroup(this);
  spacingGroup->setExclusive(true);

  auto createSpacingAction = [&](int spacing) {
    QAction* action = gridSpacingMenu->addAction(tr("%1 px").arg(spacing));
    action->setCheckable(true);
    action->setChecked(spacing == 100);  // Default
    spacingGroup->addAction(action);
    connect(action, &QAction::triggered, this, [this, spacing]() {
      if (canvas_) canvas_->setGridMajorSpacing(spacing);
    });
  };

  createSpacingAction(50);
  createSpacingAction(100);
  createSpacingAction(150);
  createSpacingAction(200);

  // Grid subdivisions submenu
  QMenu* gridSubdivisionsMenu = gridMenu->addMenu(tr("Grid S&ubdivisions"));

  QActionGroup* subdivisionsGroup = new QActionGroup(this);
  subdivisionsGroup->setExclusive(true);

  auto createSubdivAction = [&](int subdivisions, const QString& label) {
    QAction* action = gridSubdivisionsMenu->addAction(label);
    action->setCheckable(true);
    action->setChecked(subdivisions == 5);  // Default
    subdivisionsGroup->addAction(action);
    connect(action, &QAction::triggered, this, [this, subdivisions]() {
      if (canvas_) canvas_->setGridMinorSubdivisions(subdivisions);
    });
  };

  createSubdivAction(2, tr("2 (coarse)"));
  createSubdivAction(4, tr("4"));
  createSubdivAction(5, tr("5 (default)"));
  createSubdivAction(10, tr("10 (fine)"));

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

  QAction* showRosbagWorkbenchAction = panelsMenu->addAction(tr("&Rosbag Workbench"));
  showRosbagWorkbenchAction->setCheckable(true);
  showRosbagWorkbenchAction->setChecked(false);  // Hidden by default
  showRosbagWorkbenchAction->setObjectName("showRosbagWorkbenchAction");
  showRosbagWorkbenchAction->setToolTip(tr("Show/hide rosbag playback and SLAM tuning workbench"));

  QAction* showMCPExplorerAction = panelsMenu->addAction(tr("&MCP Explorer"));
  showMCPExplorerAction->setCheckable(true);
  showMCPExplorerAction->setChecked(false);  // Hidden by default
  showMCPExplorerAction->setObjectName("showMCPExplorerAction");
  showMCPExplorerAction->setToolTip(tr("Show/hide MCP server explorer (star network visualization)"));

  QAction* showSchemaViewerAction = panelsMenu->addAction(tr("&Schema Viewer"));
  showSchemaViewerAction->setCheckable(true);
  showSchemaViewerAction->setChecked(false);  // Hidden by default
  showSchemaViewerAction->setObjectName("showSchemaViewerAction");
  showSchemaViewerAction->setToolTip(tr("Show/hide ROS2 message schema viewer"));

  QAction* showReadmePreviewAction = panelsMenu->addAction(tr("Package &Docs"));
  showReadmePreviewAction->setCheckable(true);
  showReadmePreviewAction->setChecked(false);  // Hidden by default
  showReadmePreviewAction->setObjectName("showReadmePreviewAction");
  showReadmePreviewAction->setToolTip(tr("Show/hide package README and documentation preview"));

  QAction* showIssueListAction = panelsMenu->addAction(tr("&Issues (Static Analysis)"));
  showIssueListAction->setCheckable(true);
  showIssueListAction->setChecked(false);  // Hidden by default
  showIssueListAction->setObjectName("showIssueListAction");
  showIssueListAction->setToolTip(tr("Show/hide static analysis issues panel"));

  QAction* showMinimapAction = panelsMenu->addAction(tr("&Minimap Navigation"));
  showMinimapAction->setCheckable(true);
  showMinimapAction->setChecked(false);  // Hidden by default
  showMinimapAction->setShortcut(tr("M"));
  showMinimapAction->setObjectName("showMinimapAction");
  showMinimapAction->setToolTip(tr("Show/hide minimap for navigating large canvases"));

  QAction* showNodeTemplatesAction = panelsMenu->addAction(tr("Node &Templates Library"));
  showNodeTemplatesAction->setCheckable(true);
  showNodeTemplatesAction->setChecked(false);  // Hidden by default
  showNodeTemplatesAction->setObjectName("showNodeTemplatesAction");
  showNodeTemplatesAction->setToolTip(tr("Show/hide node templates library for quick node creation"));

  QAction* showWorkspaceBrowserAction = panelsMenu->addAction(tr("&Workspace Browser"));
  showWorkspaceBrowserAction->setCheckable(true);
  showWorkspaceBrowserAction->setChecked(false);  // Hidden by default
  showWorkspaceBrowserAction->setObjectName("showWorkspaceBrowserAction");
  showWorkspaceBrowserAction->setToolTip(tr("Show/hide workspace browser for viewing ROS2 packages and files"));

  QAction* showNodeHealthAction = panelsMenu->addAction(tr("Node &Health Dashboard"));
  showNodeHealthAction->setCheckable(true);
  showNodeHealthAction->setChecked(false);  // Hidden by default
  showNodeHealthAction->setObjectName("showNodeHealthAction");
  showNodeHealthAction->setToolTip(tr("Show/hide node health dashboard for real-time monitoring"));

  QAction* showMessageInspectorAction = panelsMenu->addAction(tr("&Message Inspector"));
  showMessageInspectorAction->setCheckable(true);
  showMessageInspectorAction->setChecked(false);  // Hidden by default
  showMessageInspectorAction->setObjectName("showMessageInspectorAction");
  showMessageInspectorAction->setToolTip(tr("Show/hide message inspector for viewing live messages"));

  QAction* showLifecycleAction = panelsMenu->addAction(tr("&Lifecycle Nodes"));
  showLifecycleAction->setCheckable(true);
  showLifecycleAction->setChecked(false);  // Hidden by default
  showLifecycleAction->setObjectName("showLifecycleAction");
  showLifecycleAction->setToolTip(tr("Show/hide lifecycle node panel for managing lifecycle nodes"));

  QAction* showLatencyHeatmapAction = panelsMenu->addAction(tr("Latency &Heatmap"));
  showLatencyHeatmapAction->setCheckable(true);
  showLatencyHeatmapAction->setChecked(false);  // Hidden by default
  showLatencyHeatmapAction->setObjectName("showLatencyHeatmapAction");
  showLatencyHeatmapAction->setToolTip(tr("Show/hide latency heatmap for connection latency visualization"));

  QAction* showScenarioEditorAction = panelsMenu->addAction(tr("&Scenario Editor"));
  showScenarioEditorAction->setCheckable(true);
  showScenarioEditorAction->setChecked(false);  // Hidden by default
  showScenarioEditorAction->setObjectName("showScenarioEditorAction");
  showScenarioEditorAction->setToolTip(tr("Show/hide scenario editor for creating and running test scenarios"));

  QAction* showDiagnosticsAction = panelsMenu->addAction(tr("&Diagnostics (ros2 doctor)"));
  showDiagnosticsAction->setCheckable(true);
  showDiagnosticsAction->setChecked(false);  // Hidden by default
  showDiagnosticsAction->setObjectName("showDiagnosticsAction");
  showDiagnosticsAction->setToolTip(tr("Show/hide system diagnostics panel (ros2 doctor)"));

  QAction* showNetworkTopologyAction = panelsMenu->addAction(tr("&Network Topology"));
  showNetworkTopologyAction->setCheckable(true);
  showNetworkTopologyAction->setChecked(false);  // Hidden by default
  showNetworkTopologyAction->setObjectName("showNetworkTopologyAction");
  showNetworkTopologyAction->setToolTip(tr("Show/hide DDS network topology view"));

  QAction* showBehaviorTreeAction = panelsMenu->addAction(tr("&Behavior Tree"));
  showBehaviorTreeAction->setCheckable(true);
  showBehaviorTreeAction->setChecked(false);  // Hidden by default
  showBehaviorTreeAction->setObjectName("showBehaviorTreeAction");
  showBehaviorTreeAction->setToolTip(tr("Show/hide behavior tree visualization panel"));

  QAction* showMissionPlannerAction = panelsMenu->addAction(tr("&Mission Planner"));
  showMissionPlannerAction->setCheckable(true);
  showMissionPlannerAction->setChecked(false);  // Hidden by default
  showMissionPlannerAction->setObjectName("showMissionPlannerAction");
  showMissionPlannerAction->setToolTip(tr("Show/hide mission waypoint planner panel"));

#ifdef HAVE_QT3D
  QAction* showURDFViewerAction = panelsMenu->addAction(tr("&URDF Viewer"));
  showURDFViewerAction->setCheckable(true);
  showURDFViewerAction->setChecked(false);  // Hidden by default
  showURDFViewerAction->setObjectName("showURDFViewerAction");
  showURDFViewerAction->setShortcut(QKeySequence("Ctrl+Shift+U"));
  showURDFViewerAction->setToolTip(tr("Show/hide URDF visualization and editing panel"));
#endif

  panelsMenu->addSeparator();

  QAction* dockAllPanelsAction = panelsMenu->addAction(tr("&Dock All Floating Panels"));
  dockAllPanelsAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_D));
  dockAllPanelsAction->setToolTip(tr("Dock all floating panels back to their default positions"));
  connect(dockAllPanelsAction, &QAction::triggered, this, &MainWindow::onDockAllPanels);

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

  // Arrange menu - layout algorithms
  QMenu* arrangeMenu = menuBar()->addMenu(tr("&Arrange"));

  QMenu* autoLayoutMenu = arrangeMenu->addMenu(tr("&Auto-Layout"));

  QAction* hierarchicalLayoutAction = autoLayoutMenu->addAction(tr("&Hierarchical (Top-Down)"));
  hierarchicalLayoutAction->setToolTip(tr("Arrange nodes in hierarchical layers based on data flow"));
  connect(hierarchicalLayoutAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyHierarchicalLayout(false, false);
      statusBar()->showMessage(tr("Applied hierarchical layout"), 3000);
    }
  });

  QAction* hierarchicalLRLayoutAction = autoLayoutMenu->addAction(tr("Hierarchical (&Left-Right)"));
  hierarchicalLRLayoutAction->setToolTip(tr("Arrange nodes in hierarchical layers from left to right"));
  connect(hierarchicalLRLayoutAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyHierarchicalLayout(false, true);
      statusBar()->showMessage(tr("Applied left-to-right hierarchical layout"), 3000);
    }
  });

  QAction* forceDirectedAction = autoLayoutMenu->addAction(tr("&Force-Directed"));
  forceDirectedAction->setToolTip(tr("Apply physics-based layout for balanced distribution"));
  connect(forceDirectedAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyForceDirectedLayout(false);
      statusBar()->showMessage(tr("Applied force-directed layout"), 3000);
    }
  });

  QAction* circularLayoutAction = autoLayoutMenu->addAction(tr("&Circular"));
  circularLayoutAction->setToolTip(tr("Arrange nodes in a circle"));
  connect(circularLayoutAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyCircularLayout(false);
      statusBar()->showMessage(tr("Applied circular layout"), 3000);
    }
  });

  QAction* gridLayoutAction = autoLayoutMenu->addAction(tr("&Grid"));
  gridLayoutAction->setToolTip(tr("Arrange nodes in a regular grid pattern"));
  connect(gridLayoutAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyGridLayout(false);
      statusBar()->showMessage(tr("Applied grid layout"), 3000);
    }
  });

  arrangeMenu->addSeparator();

  // Layout selected only submenu
  QMenu* layoutSelectedMenu = arrangeMenu->addMenu(tr("Layout &Selected"));

  QAction* hierarchicalSelectedAction = layoutSelectedMenu->addAction(tr("&Hierarchical"));
  connect(hierarchicalSelectedAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyHierarchicalLayout(true, false);
      statusBar()->showMessage(tr("Applied hierarchical layout to selection"), 3000);
    }
  });

  QAction* forceSelectedAction = layoutSelectedMenu->addAction(tr("&Force-Directed"));
  connect(forceSelectedAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyForceDirectedLayout(true);
      statusBar()->showMessage(tr("Applied force-directed layout to selection"), 3000);
    }
  });

  QAction* circularSelectedAction = layoutSelectedMenu->addAction(tr("&Circular"));
  connect(circularSelectedAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyCircularLayout(true);
      statusBar()->showMessage(tr("Applied circular layout to selection"), 3000);
    }
  });

  QAction* gridSelectedAction = layoutSelectedMenu->addAction(tr("&Grid"));
  connect(gridSelectedAction, &QAction::triggered, this, [this]() {
    if (canvas_) {
      canvas_->applyGridLayout(true);
      statusBar()->showMessage(tr("Applied grid layout to selection"), 3000);
    }
  });

  arrangeMenu->addSeparator();

  QAction* alignHorizontalAction = arrangeMenu->addAction(tr("Align &Horizontal Centers"));
  alignHorizontalAction->setShortcut(tr("Ctrl+Shift+H"));
  alignHorizontalAction->setToolTip(tr("Align selected nodes horizontally"));
  connect(alignHorizontalAction, &QAction::triggered, this, [this]() {
    if (!canvas_) return;
    auto blocks = canvas_->selectedBlocks();
    if (blocks.size() < 2) return;

    // Calculate average Y position
    qreal avgY = 0;
    for (PackageBlock* block : blocks) {
      avgY += block->pos().y();
    }
    avgY /= blocks.size();

    // Align all blocks to average Y
    for (PackageBlock* block : blocks) {
      block->setPos(block->pos().x(), avgY);
    }
    statusBar()->showMessage(tr("Aligned %1 nodes horizontally").arg(blocks.size()), 3000);
  });

  QAction* alignVerticalAction = arrangeMenu->addAction(tr("Align &Vertical Centers"));
  alignVerticalAction->setShortcut(tr("Ctrl+Shift+V"));
  alignVerticalAction->setToolTip(tr("Align selected nodes vertically"));
  connect(alignVerticalAction, &QAction::triggered, this, [this]() {
    if (!canvas_) return;
    auto blocks = canvas_->selectedBlocks();
    if (blocks.size() < 2) return;

    // Calculate average X position
    qreal avgX = 0;
    for (PackageBlock* block : blocks) {
      avgX += block->pos().x();
    }
    avgX /= blocks.size();

    // Align all blocks to average X
    for (PackageBlock* block : blocks) {
      block->setPos(avgX, block->pos().y());
    }
    statusBar()->showMessage(tr("Aligned %1 nodes vertically").arg(blocks.size()), 3000);
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
    // Show Browser dock and switch to System Mapping tab
    if (packageBrowserDock_) {
      packageBrowserDock_->show();
      packageBrowserDock_->raise();
    }
    if (browserTab_ && systemMappingPanel_) {
      browserTab_->setCurrentWidget(systemMappingPanel_);
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

  QAction* showTFTreeAction = ros2Menu->addAction(tr("Show T&F Tree"));
  showTFTreeAction->setShortcut(tr("Ctrl+T"));
  connect(showTFTreeAction, &QAction::triggered, this, [this]() {
    // Show Properties dock and switch to TF Tree tab
    if (propertiesDock_) {
      propertiesDock_->show();
      propertiesDock_->raise();
    }
    if (propertiesTab_ && tfTreePanel_) {
      propertiesTab_->setCurrentWidget(tfTreePanel_);
      // Start listening if not already
      if (!tfTreePanel_->isListening()) {
        tfTreePanel_->startListening();
      }
    }
  });

  QAction* showRosLogsAction = ros2Menu->addAction(tr("Show ROS &Logs"));
  showRosLogsAction->setShortcut(tr("Ctrl+Shift+L"));
  connect(showRosLogsAction, &QAction::triggered, this, [this]() {
    // Show Output dock and switch to ROS Logs tab
    if (outputDock_) {
      outputDock_->show();
      outputDock_->raise();
    }
    if (outputPanel_) {
      outputPanel_->showRosLogTab();
      // Start listening if not already
      if (outputPanel_->rosLogViewer() && !outputPanel_->rosLogViewer()->isListening()) {
        outputPanel_->rosLogViewer()->startListening();
      }
    }
  });

  ros2Menu->addSeparator();

  // External tools
  QAction* launchRviz2Action = ros2Menu->addAction(tr("Launch R&Viz2"));
  launchRviz2Action->setShortcut(tr("Ctrl+Shift+V"));
  launchRviz2Action->setToolTip(tr("Launch RViz2 visualization tool"));
  connect(launchRviz2Action, &QAction::triggered, this, &MainWindow::onLaunchRviz2);

  ros2Menu->addSeparator();

  // Simulation submenu
  QMenu* simMenu = ros2Menu->addMenu(tr("&Simulation"));

  QAction* launchGazeboAction = simMenu->addAction(tr("Launch &Gazebo"));
  launchGazeboAction->setToolTip(tr("Launch Gazebo simulation with empty world"));
  connect(launchGazeboAction, &QAction::triggered, this, &MainWindow::onLaunchGazebo);

  QAction* launchIgnitionAction = simMenu->addAction(tr("Launch &Ignition/Gazebo Sim"));
  launchIgnitionAction->setToolTip(tr("Launch Ignition Gazebo (Gazebo Sim) with empty world"));
  connect(launchIgnitionAction, &QAction::triggered, this, &MainWindow::onLaunchIgnition);

  simMenu->addSeparator();

  QAction* stopSimAction = simMenu->addAction(tr("&Stop Simulation"));
  stopSimAction->setToolTip(tr("Stop the running simulation"));
  connect(stopSimAction, &QAction::triggered, this, &MainWindow::onStopSimulation);

  // Connect simulation launcher signals
  connect(&SimulationLauncher::instance(), &SimulationLauncher::simulationStarted, this,
          [this]() {
    statusBar()->showMessage(tr("Simulation started"), 3000);
  });

  connect(&SimulationLauncher::instance(), &SimulationLauncher::simulationStopped, this,
          [this]() {
    statusBar()->showMessage(tr("Simulation stopped"), 3000);
  });

  connect(&SimulationLauncher::instance(), &SimulationLauncher::errorOccurred, this,
          [this](const QString& error) {
    QMessageBox::warning(this, tr("Simulation Error"), error);
  });

  ros2Menu->addSeparator();

  // Static Analysis action
  QAction* runAnalysisAction = ros2Menu->addAction(tr("Run Static &Analysis"));
  runAnalysisAction->setShortcut(tr("Ctrl+Shift+A"));
  runAnalysisAction->setToolTip(tr("Analyze the architecture for potential issues"));
  connect(runAnalysisAction, &QAction::triggered, this, &MainWindow::onRunAnalysis);

  // Connection menu
  QMenu* connectionMenu = menuBar()->addMenu(tr("&Connection"));

  connectRemoteAction_ = connectionMenu->addAction(tr("Connect to &Robot..."));
  connectRemoteAction_->setShortcut(tr("Ctrl+R"));
  connectRemoteAction_->setToolTip(tr("Connect to a remote robot via SSH"));
  connect(connectRemoteAction_, &QAction::triggered, this, &MainWindow::onShowRemoteConnectionDialog);

  disconnectRemoteAction_ = connectionMenu->addAction(tr("&Disconnect"));
  disconnectRemoteAction_->setEnabled(false);
  disconnectRemoteAction_->setToolTip(tr("Disconnect from the remote robot"));
  connect(disconnectRemoteAction_, &QAction::triggered, this, &MainWindow::onDisconnectRemote);

  connectionMenu->addSeparator();

  // Recent connections submenu
  QMenu* recentConnectionsMenu = connectionMenu->addMenu(tr("Recent Connections"));
  recentConnectionsMenu->setEnabled(false);  // Will be enabled when profiles exist

  // Help menu
  QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));

  QAction* gettingStartedAction = helpMenu->addAction(tr("&Getting Started"));
  gettingStartedAction->setShortcut(tr("Ctrl+F1"));
  gettingStartedAction->setToolTip(tr("Quick start guide for new users"));
  connect(gettingStartedAction, &QAction::triggered, this, &MainWindow::onShowGettingStarted);

  QAction* guidedTourAction = helpMenu->addAction(tr("Guided &Tour..."));
  guidedTourAction->setShortcut(tr("Ctrl+Shift+T"));
  guidedTourAction->setToolTip(tr("Interactive tour of the application features"));
  connect(guidedTourAction, &QAction::triggered, this, &MainWindow::onShowGuidedTour);

  QAction* userManualAction = helpMenu->addAction(tr("&User Manual..."));
  userManualAction->setToolTip(tr("Browse the full documentation"));
  connect(userManualAction, &QAction::triggered, this, &MainWindow::onShowUserManual);

  QAction* shortcutsAction = helpMenu->addAction(tr("&Keyboard Shortcuts..."));
  shortcutsAction->setShortcut(tr("Ctrl+/"));
  shortcutsAction->setToolTip(tr("View all keyboard shortcuts"));
  connect(shortcutsAction, &QAction::triggered, this, &MainWindow::onShowKeyboardShortcuts);

  helpMenu->addSeparator();

  QAction* whatsNewAction = helpMenu->addAction(tr("What's &New"));
  whatsNewAction->setToolTip(tr("See recent changes and new features"));
  connect(whatsNewAction, &QAction::triggered, this, &MainWindow::onShowWhatsNew);

  QAction* videoTutorialsAction = helpMenu->addAction(tr("&Video Tutorials"));
  videoTutorialsAction->setToolTip(tr("Open video tutorials (external browser)"));
  connect(videoTutorialsAction, &QAction::triggered, this, []() {
    QDesktopServices::openUrl(QUrl("https://github.com/DingoOz/ROS2Weaver/wiki/Tutorials"));
  });

  helpMenu->addSeparator();

  QAction* reportIssueAction = helpMenu->addAction(tr("&Report Issue..."));
  reportIssueAction->setToolTip(tr("Report a bug or request a feature"));
  connect(reportIssueAction, &QAction::triggered, this, &MainWindow::onReportIssue);

  helpMenu->addSeparator();

  QAction* aboutAction = helpMenu->addAction(tr("&About ROS Weaver"));
  connect(aboutAction, &QAction::triggered, this, &MainWindow::onAbout);

  QAction* aboutQtAction = helpMenu->addAction(tr("About &Qt"));
  connect(aboutQtAction, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::setupToolBar() {
  QToolBar* mainToolBar = addToolBar(tr("Main Toolbar"));
  mainToolBar->setMovable(false);

  QAction* newToolbarAction = mainToolBar->addAction(tr("New"));
  newToolbarAction->setToolTip(tr("New Project (Ctrl+N)"));
  connect(newToolbarAction, &QAction::triggered, this, &MainWindow::onNewProject);

  QAction* openToolbarAction = mainToolBar->addAction(tr("Open"));
  openToolbarAction->setToolTip(tr("Open Project (Ctrl+O)"));
  connect(openToolbarAction, &QAction::triggered, this, &MainWindow::onOpenProject);

  QAction* saveToolbarAction = mainToolBar->addAction(tr("Save"));
  saveToolbarAction->setToolTip(tr("Save Project (Ctrl+S)"));
  connect(saveToolbarAction, &QAction::triggered, this, &MainWindow::onSaveProject);

  mainToolBar->addSeparator();

  // Add scan system button to toolbar
  QAction* scanToolbarAction = mainToolBar->addAction(tr("Scan System"));
  scanToolbarAction->setToolTip(tr("Scan running ROS2 system (Ctrl+Shift+R)"));
  connect(scanToolbarAction, &QAction::triggered, this, &MainWindow::onScanSystem);

  // Add parameter preset selector
  mainToolBar->addSeparator();
  PresetSelectorWidget* presetSelector = new PresetSelectorWidget(this);
  mainToolBar->addWidget(presetSelector);
}

void MainWindow::setupDockWidgets() {
  // Browser Dock (left side) - contains Package Browser and System Mapping tabs
  packageBrowserDock_ = new QDockWidget(tr("Browser"), this);
  packageBrowserDock_->setObjectName("packageBrowserDock");
  packageBrowserDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  browserTab_ = new QTabWidget();

  // Package Browser tab
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
  browserTab_->addTab(browserWidget, tr("Packages"));

  // Advanced Search tab
  advancedSearchPanel_ = new AdvancedSearchPanel();
  browserTab_->addTab(advancedSearchPanel_, tr("Search"));

  // Connect advanced search signals
  connect(advancedSearchPanel_, &AdvancedSearchPanel::addToCanvasRequested,
          this, [this](const SearchResult& result) {
    if (canvas_) {
      // Create a new block based on the search result at center of view
      QPointF center = canvas_->mapToScene(canvas_->viewport()->rect().center());
      canvas_->addPackageBlock(result.packageName, center);
      statusBar()->showMessage(tr("Added '%1' to canvas").arg(result.name), 3000);
    }
  });

  // System Mapping tab
  systemMappingPanel_ = new SystemMappingPanel();
  systemMappingPanel_->setSystemDiscovery(systemDiscovery_);
  systemMappingPanel_->setCanvasMapper(canvasMapper_);
  browserTab_->addTab(systemMappingPanel_, tr("System Mapping"));

  connect(systemMappingPanel_, &SystemMappingPanel::blockSelected,
          this, &MainWindow::onMappingBlockSelected);
  connect(systemMappingPanel_, &SystemMappingPanel::scanRequested,
          this, &MainWindow::onScanSystem);

  packageBrowserDock_->setWidget(browserTab_);
  addDockWidget(Qt::LeftDockWidgetArea, packageBrowserDock_);

  // Properties Dock (right side) with Param Dashboard
  propertiesDock_ = new QDockWidget(tr("Properties"), this);
  propertiesDock_->setObjectName("propertiesDock");
  propertiesDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  propertiesTab_ = new QTabWidget();

  // Parameters tab with ParamDashboard
  paramDashboard_ = new ParamDashboard();
  paramDashboard_->setUndoStack(undoStack_);
  paramDashboard_->setCanvas(canvas_);  // canvas_ is set from setupCentralWidget()
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

  // TF Tree tab
  tfTreePanel_ = new TFTreePanel();
  tfTreePanel_->setCanvas(canvas_);
  propertiesTab_->addTab(tfTreePanel_, tr("TF Tree"));

  // Plot panel tab
  plotPanel_ = new PlotPanel();
  plotPanel_->setCanvas(canvas_);
  propertiesTab_->addTab(plotPanel_, tr("Plots"));

  // ROS Control panel tab
  rosControlPanel_ = new RosControlPanel();
  rosControlPanel_->setCanvas(canvas_);
  propertiesTab_->addTab(rosControlPanel_, tr("Controllers"));

  // Connect ROS Control panel signals
  connect(rosControlPanel_, &RosControlPanel::controllerStateChanged,
          this, [this](const QString& name, const QString& oldState, const QString& newState) {
    statusBar()->showMessage(tr("Controller '%1' changed: %2 -> %3")
        .arg(name, oldState, newState), 5000);
  });

  connect(rosControlPanel_, &RosControlPanel::errorOccurred,
          this, [this](const QString& error) {
    statusBar()->showMessage(tr("ros_control error: %1").arg(error), 5000);
  });

  // Connect TF tree signals
  connect(tfTreePanel_, &TFTreePanel::frameSelected,
          this, [this](const QString& frameName) {
    // Could highlight blocks that use this frame
    statusBar()->showMessage(tr("Selected frame: %1").arg(frameName), 3000);
  });

  connect(tfTreePanel_, &TFTreePanel::showFrameOnCanvas,
          this, [this](const QString& frameName) {
    // Find and highlight blocks that use this frame
    if (!canvas_) return;
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      PackageBlock* block = dynamic_cast<PackageBlock*>(item);
      if (!block) continue;
      for (const auto& param : block->parameters()) {
        if (param.currentValue.toString().contains(frameName)) {
          canvas_->centerOn(block);
          block->setBlockSelected(true);
          statusBar()->showMessage(tr("Found frame '%1' in block '%2'")
            .arg(frameName, block->packageName()), 3000);
          return;
        }
      }
    }
    statusBar()->showMessage(tr("Frame '%1' not found in any block parameters").arg(frameName), 3000);
  });

  connect(tfTreePanel_, &TFTreePanel::openYamlAtLine,
          this, [this](const QString& filePath, int lineNumber) {
    // Open YAML file in VS Code at specific line
    if (externalEditor_) {
      externalEditor_->openFileInVSCodeAtLine(filePath, lineNumber);
    }
  });

  propertiesDock_->setWidget(propertiesTab_);
  addDockWidget(Qt::RightDockWidgetArea, propertiesDock_);

  // Output Dock (bottom) - with tabs for Build Output, ROS Logs, and Terminal
  outputDock_ = new QDockWidget(tr("Output"), this);
  outputDock_->setObjectName("outputDock");
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

  // Rosbag Workbench Dock (bottom, initially hidden)
  rosbagWorkbenchDock_ = new QDockWidget(tr("Rosbag Workbench"), this);
  rosbagWorkbenchDock_->setObjectName("rosbagWorkbenchDock");
  rosbagWorkbenchDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

  // Create panel with deferred initialization to avoid blocking startup
  rosbagWorkbenchPanel_ = new RosbagWorkbenchPanel();
  rosbagWorkbenchPanel_->setCanvas(canvas_);
  rosbagWorkbenchPanel_->setTopicViewerPanel(topicViewerPanel_);
  rosbagWorkbenchPanel_->setPlotPanel(plotPanel_);
  rosbagWorkbenchPanel_->setParamDashboard(paramDashboard_);

  rosbagWorkbenchDock_->setWidget(rosbagWorkbenchPanel_);
  addDockWidget(Qt::BottomDockWidgetArea, rosbagWorkbenchDock_);
  rosbagWorkbenchDock_->hide();  // Hidden by default

  // Connect rosbag workbench visibility toggle
  QAction* showRosbagWorkbenchAction = findChild<QAction*>("showRosbagWorkbenchAction");
  if (showRosbagWorkbenchAction) {
    connect(showRosbagWorkbenchAction, &QAction::toggled, rosbagWorkbenchDock_, &QDockWidget::setVisible);
    connect(rosbagWorkbenchDock_, &QDockWidget::visibilityChanged, showRosbagWorkbenchAction, &QAction::setChecked);
  }

  // MCP Explorer Dock (floating/undocked by default as per plan)
  mcpExplorerDock_ = new QDockWidget(tr("MCP Explorer"), this);
  mcpExplorerDock_->setObjectName("mcpExplorerDock");
  mcpExplorerDock_->setAllowedAreas(Qt::AllDockWidgetAreas);

  mcpExplorerPanel_ = new MCPExplorerPanel();
  mcpExplorerDock_->setWidget(mcpExplorerPanel_);

  // Add as floating window initially (undocked)
  addDockWidget(Qt::RightDockWidgetArea, mcpExplorerDock_);
  mcpExplorerDock_->setFloating(true);
  mcpExplorerDock_->resize(500, 400);
  mcpExplorerDock_->hide();  // Hidden by default, shown via View menu

  // Connect MCP explorer signals
  connect(mcpExplorerPanel_, &MCPExplorerPanel::addServerRequested, this,
          [this]() {
    MCPServerWizard wizard(this);
    if (wizard.exec() == QDialog::Accepted) {
      MCPServerConfig config = wizard.resultConfig();
      auto server = MCPManager::instance().createServer(config);
      if (server) {
        MCPManager::instance().registerServer(server);
      }
    }
  });

  connect(mcpExplorerPanel_, &MCPExplorerPanel::serverSettingsRequested, this,
          [this](const QUuid& serverId) {
    auto server = MCPManager::instance().server(serverId);
    if (server) {
      QMessageBox::information(this, tr("Server Settings"),
          tr("Settings for: %1\n\nType: %2\nStatus: %3")
          .arg(server->name())
          .arg(server->serverType())
          .arg(server->isConnected() ? "Connected" : "Disconnected"));
    }
  });

  // Connect MCP explorer visibility toggle
  QAction* showMCPExplorerAction = findChild<QAction*>("showMCPExplorerAction");
  if (showMCPExplorerAction) {
    connect(showMCPExplorerAction, &QAction::toggled, mcpExplorerDock_, &QDockWidget::setVisible);
    connect(mcpExplorerDock_, &QDockWidget::visibilityChanged, showMCPExplorerAction, &QAction::setChecked);
  }

  // Schema Viewer Dock (for viewing ROS2 message schemas)
  schemaViewerDock_ = new QDockWidget(tr("Schema Viewer"), this);
  schemaViewerDock_->setObjectName("schemaViewerDock");
  schemaViewerDock_->setAllowedAreas(Qt::AllDockWidgetAreas);

  schemaViewerWidget_ = new SchemaViewerWidget();
  schemaViewerDock_->setWidget(schemaViewerWidget_);

  addDockWidget(Qt::RightDockWidgetArea, schemaViewerDock_);
  schemaViewerDock_->hide();  // Hidden by default

  // Connect schema viewer visibility toggle
  QAction* showSchemaViewerAction = findChild<QAction*>("showSchemaViewerAction");
  if (showSchemaViewerAction) {
    connect(showSchemaViewerAction, &QAction::toggled, schemaViewerDock_, &QDockWidget::setVisible);
    connect(schemaViewerDock_, &QDockWidget::visibilityChanged, showSchemaViewerAction, &QAction::setChecked);
  }

  // README Preview Dock (for viewing package READMEs)
  readmePreviewDock_ = new QDockWidget(tr("Package Docs"), this);
  readmePreviewDock_->setObjectName("readmePreviewDock");
  readmePreviewDock_->setAllowedAreas(Qt::AllDockWidgetAreas);

  readmePreviewPanel_ = new ReadmePreviewPanel();
  readmePreviewDock_->setWidget(readmePreviewPanel_);

  addDockWidget(Qt::RightDockWidgetArea, readmePreviewDock_);
  readmePreviewDock_->hide();  // Hidden by default

  // Connect README preview visibility toggle
  QAction* showReadmePreviewAction = findChild<QAction*>("showReadmePreviewAction");
  if (showReadmePreviewAction) {
    connect(showReadmePreviewAction, &QAction::toggled, readmePreviewDock_, &QDockWidget::setVisible);
    connect(readmePreviewDock_, &QDockWidget::visibilityChanged, showReadmePreviewAction, &QAction::setChecked);
  }

  // Issue List Dock (for static analysis results)
  issueListDock_ = new QDockWidget(tr("Issues"), this);
  issueListDock_->setObjectName("issueListDock");
  issueListDock_->setAllowedAreas(Qt::AllDockWidgetAreas);

  issueListPanel_ = new IssueListPanel();
  issueListDock_->setWidget(issueListPanel_);

  addDockWidget(Qt::BottomDockWidgetArea, issueListDock_);
  issueListDock_->hide();  // Hidden by default

  // Connect issue list signals
  connect(issueListPanel_, &IssueListPanel::navigateToBlock, this, &MainWindow::onNavigateToIssue);
  connect(issueListPanel_, &IssueListPanel::askAIAboutIssue, this,
          [this](const AnalysisIssue& issue) {
    if (outputPanel_->llmChatWidget()) {
      outputPanel_->showLLMChatTab();
      outputPanel_->llmChatWidget()->sendMessage(
          tr("Help me understand and fix this issue:\n\n**%1** (%2)\n\n%3\n\nSuggestion: %4")
          .arg(issue.title)
          .arg(StaticAnalyzer::categoryToString(issue.category))
          .arg(issue.description)
          .arg(issue.suggestion));
    }
  });

  // Connect issue list visibility toggle
  QAction* showIssueListAction = findChild<QAction*>("showIssueListAction");
  if (showIssueListAction) {
    connect(showIssueListAction, &QAction::toggled, issueListDock_, &QDockWidget::setVisible);
    connect(issueListDock_, &QDockWidget::visibilityChanged, showIssueListAction, &QAction::setChecked);
  }

  // Minimap Navigation Dock (right side, small panel for navigation)
  minimapDock_ = new QDockWidget(tr("Minimap"), this);
  minimapDock_->setObjectName("minimapDock");
  minimapDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  minimapDock_->setFeatures(QDockWidget::DockWidgetMovable |
                            QDockWidget::DockWidgetFloatable |
                            QDockWidget::DockWidgetClosable);

  minimapPanel_ = new MinimapPanel();
  minimapPanel_->setCanvas(canvas_);
  minimapDock_->setWidget(minimapPanel_);

  // Add to right dock area
  addDockWidget(Qt::RightDockWidgetArea, minimapDock_);
  minimapDock_->setFloating(false);
  minimapDock_->hide();  // Hidden by default, shown via View menu

  // Connect minimap visibility toggle
  QAction* showMinimapAction = findChild<QAction*>("showMinimapAction");
  if (showMinimapAction) {
    connect(showMinimapAction, &QAction::toggled, minimapDock_, &QDockWidget::setVisible);
    connect(minimapDock_, &QDockWidget::visibilityChanged, showMinimapAction, &QAction::setChecked);
  }

  // Node Templates panel
  nodeTemplatesDock_ = new QDockWidget(tr("Node Templates"), this);
  nodeTemplatesDock_->setObjectName("nodeTemplatesDock");
  nodeTemplatesDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  nodeTemplatesDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                  QDockWidget::DockWidgetFloatable |
                                  QDockWidget::DockWidgetClosable);

  nodeTemplatesPanel_ = new NodeTemplatesPanel(this);
  nodeTemplatesDock_->setWidget(nodeTemplatesPanel_);

  // Add to left dock area, tabified with package browser
  addDockWidget(Qt::LeftDockWidgetArea, nodeTemplatesDock_);
  tabifyDockWidget(packageBrowserDock_, nodeTemplatesDock_);
  nodeTemplatesDock_->hide();  // Hidden by default

  // Connect template selection to canvas node creation
  connect(nodeTemplatesPanel_, &NodeTemplatesPanel::templateSelected, this,
          [this](const NodeTemplate& tmpl) {
    if (canvas_) {
      // Create a new block based on the template
      PackageBlock* block = canvas_->addPackageBlock(tmpl.displayName, QPointF(200, 200));
      if (block) {
        // Add input pins from template
        for (const auto& pin : tmpl.inputPins) {
          block->addInputPin(pin.name, Pin::DataType::Topic, pin.messageType);
        }
        // Add output pins from template
        for (const auto& pin : tmpl.outputPins) {
          block->addOutputPin(pin.name, Pin::DataType::Topic, pin.messageType);
        }
        // Add parameters from template
        QList<BlockParamData> blockParams;
        for (const auto& param : tmpl.parameters) {
          BlockParamData pd;
          pd.name = param.name;
          pd.defaultValue = param.defaultValue;
          pd.currentValue = param.defaultValue;
          pd.type = param.type;
          pd.description = param.description;
          blockParams.append(pd);
        }
        block->setParameters(blockParams);
      }
    }
  });

  // Connect node templates visibility toggle
  QAction* showNodeTemplatesAction = findChild<QAction*>("showNodeTemplatesAction");
  if (showNodeTemplatesAction) {
    connect(showNodeTemplatesAction, &QAction::toggled, nodeTemplatesDock_, &QDockWidget::setVisible);
    connect(nodeTemplatesDock_, &QDockWidget::visibilityChanged, showNodeTemplatesAction, &QAction::setChecked);
  }

  // Workspace Browser panel
  workspaceBrowserDock_ = new QDockWidget(tr("Workspace Browser"), this);
  workspaceBrowserDock_->setObjectName("workspaceBrowserDock");
  workspaceBrowserDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  workspaceBrowserDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                     QDockWidget::DockWidgetFloatable |
                                     QDockWidget::DockWidgetClosable);

  workspaceBrowserPanel_ = new WorkspaceBrowserPanel(this);
  workspaceBrowserDock_->setWidget(workspaceBrowserPanel_);

  // Add to left dock area, tabified with package browser
  addDockWidget(Qt::LeftDockWidgetArea, workspaceBrowserDock_);
  tabifyDockWidget(packageBrowserDock_, workspaceBrowserDock_);
  workspaceBrowserDock_->hide();  // Hidden by default

  // Connect workspace browser signals
  connect(workspaceBrowserPanel_, &WorkspaceBrowserPanel::launchFileSelected, this,
          [this](const QString& filePath) {
    // Open launch file in external editor
    if (externalEditor_) {
      externalEditor_->openFileInVSCode(filePath, true);
    } else {
      ExternalEditor::openInDefaultEditor(filePath);
    }
  });

  connect(workspaceBrowserPanel_, &WorkspaceBrowserPanel::openInEditorRequested, this,
          [this](const QString& filePath) {
    if (externalEditor_) {
      externalEditor_->openFileInVSCode(filePath, true);
    } else {
      ExternalEditor::openInDefaultEditor(filePath);
    }
  });

  // Connect workspace browser visibility toggle
  QAction* showWorkspaceBrowserAction = findChild<QAction*>("showWorkspaceBrowserAction");
  if (showWorkspaceBrowserAction) {
    connect(showWorkspaceBrowserAction, &QAction::toggled, workspaceBrowserDock_, &QDockWidget::setVisible);
    connect(workspaceBrowserDock_, &QDockWidget::visibilityChanged, showWorkspaceBrowserAction, &QAction::setChecked);
  }

  // Node Health Dashboard
  nodeHealthDock_ = new QDockWidget(tr("Node Health"), this);
  nodeHealthDock_->setObjectName("nodeHealthDock");
  nodeHealthDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  nodeHealthDock_->setFeatures(QDockWidget::DockWidgetMovable |
                               QDockWidget::DockWidgetFloatable |
                               QDockWidget::DockWidgetClosable);

  nodeHealthDashboard_ = new NodeHealthDashboard(this);
  nodeHealthDashboard_->setCanvas(canvas_);
  nodeHealthDock_->setWidget(nodeHealthDashboard_);

  // Add to right dock area
  addDockWidget(Qt::RightDockWidgetArea, nodeHealthDock_);
  nodeHealthDock_->hide();  // Hidden by default

  // Connect node health dashboard visibility toggle
  QAction* showNodeHealthAction = findChild<QAction*>("showNodeHealthAction");
  if (showNodeHealthAction) {
    connect(showNodeHealthAction, &QAction::toggled, nodeHealthDock_, &QDockWidget::setVisible);
    connect(nodeHealthDock_, &QDockWidget::visibilityChanged, showNodeHealthAction, &QAction::setChecked);
  }

  // Connect static analyzer signals
  connect(&StaticAnalyzer::instance(), &StaticAnalyzer::analysisCompleted,
          this, &MainWindow::onAnalysisCompleted);

  // Initialize MCP Manager and load configuration
  MCPManager::instance().loadConfiguration();
  MCPManager::instance().setCanvas(canvas_);

  // Connect any existing rosbag MCP servers to the workbench panel
  connectRosbagMCPServers();

  // Also connect when new servers are registered
  connect(&MCPManager::instance(), &MCPManager::serverRegistered, this,
          [this](const QUuid& id) {
    auto server = MCPManager::instance().server(id);
    if (server && server->serverType() == "rosbag") {
      connectRosbagMCPServer(std::dynamic_pointer_cast<RosbagMCPServer>(server));
    }
  });

  // Setup AI integration between canvas and LLM chat
  if (outputPanel_->llmChatWidget() && canvas_) {
    LLMChatWidget* llmChat = outputPanel_->llmChatWidget();

    // Set the canvas on the LLM chat widget for AI tools
    llmChat->setCanvas(canvas_);

    // Connect canvas "Ask AI about this..." signals to LLM chat
    connect(canvas_, &WeaverCanvas::askAIAboutBlock, this,
            [this, llmChat](PackageBlock* block) {
      outputPanel_->showLLMChatTab();
      llmChat->askAboutBlock(block);
    });

    connect(canvas_, &WeaverCanvas::askAIAboutConnection, this,
            [this, llmChat](ConnectionLine* connection) {
      outputPanel_->showLLMChatTab();
      llmChat->askAboutConnection(connection);
    });

    connect(canvas_, &WeaverCanvas::askAIAboutGroup, this,
            [this, llmChat](NodeGroup* group) {
      outputPanel_->showLLMChatTab();
      llmChat->askAboutGroup(group);
    });

    connect(canvas_, &WeaverCanvas::askAIAboutPin, this,
            [this, llmChat](PackageBlock* block, int pinIndex, bool isOutput) {
      outputPanel_->showLLMChatTab();
      llmChat->askAboutPin(block, pinIndex, isOutput);
    });

    // Connect load example signal from AI to main window
    connect(llmChat, &LLMChatWidget::loadExampleRequested, this,
            [this](const QString& exampleName) {
      if (exampleName == "turtlesim_teleop") {
        onLoadTurtlesimExample();
      } else if (exampleName == "turtlebot3_navigation") {
        onLoadTurtleBotExample();
      }
    });
  }

  // Message Inspector Panel
  messageInspectorDock_ = new QDockWidget(tr("Message Inspector"), this);
  messageInspectorDock_->setObjectName("messageInspectorDock");
  messageInspectorDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  messageInspectorDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                     QDockWidget::DockWidgetFloatable |
                                     QDockWidget::DockWidgetClosable);

  messageInspectorPanel_ = new MessageInspectorPanel(this);
  messageInspectorPanel_->setCanvas(canvas_);
  messageInspectorDock_->setWidget(messageInspectorPanel_);

  addDockWidget(Qt::RightDockWidgetArea, messageInspectorDock_);
  messageInspectorDock_->hide();  // Hidden by default

  // Connect message inspector visibility toggle
  QAction* showMessageInspectorAction = findChild<QAction*>("showMessageInspectorAction");
  if (showMessageInspectorAction) {
    connect(showMessageInspectorAction, &QAction::toggled, messageInspectorDock_, &QDockWidget::setVisible);
    connect(messageInspectorDock_, &QDockWidget::visibilityChanged, showMessageInspectorAction, &QAction::setChecked);
  }

  // Lifecycle Panel
  lifecycleDock_ = new QDockWidget(tr("Lifecycle Nodes"), this);
  lifecycleDock_->setObjectName("lifecycleDock");
  lifecycleDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  lifecycleDock_->setFeatures(QDockWidget::DockWidgetMovable |
                              QDockWidget::DockWidgetFloatable |
                              QDockWidget::DockWidgetClosable);

  lifecyclePanel_ = new LifecyclePanel(this);
  lifecyclePanel_->setCanvas(canvas_);
  lifecycleDock_->setWidget(lifecyclePanel_);

  addDockWidget(Qt::RightDockWidgetArea, lifecycleDock_);
  lifecycleDock_->hide();  // Hidden by default

  // Connect lifecycle panel visibility toggle
  QAction* showLifecycleAction = findChild<QAction*>("showLifecycleAction");
  if (showLifecycleAction) {
    connect(showLifecycleAction, &QAction::toggled, lifecycleDock_, &QDockWidget::setVisible);
    connect(lifecycleDock_, &QDockWidget::visibilityChanged, showLifecycleAction, &QAction::setChecked);
  }

  // Latency Heatmap Panel
  latencyHeatmapDock_ = new QDockWidget(tr("Latency Heatmap"), this);
  latencyHeatmapDock_->setObjectName("latencyHeatmapDock");
  latencyHeatmapDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  latencyHeatmapDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                    QDockWidget::DockWidgetFloatable |
                                    QDockWidget::DockWidgetClosable);

  latencyTracker_ = new LatencyTracker(this);
  latencyHeatmapPanel_ = new LatencyHeatmapPanel(this);
  latencyHeatmapPanel_->setCanvas(canvas_);
  latencyHeatmapPanel_->setLatencyTracker(latencyTracker_);
  latencyHeatmapDock_->setWidget(latencyHeatmapPanel_);

  addDockWidget(Qt::RightDockWidgetArea, latencyHeatmapDock_);
  latencyHeatmapDock_->hide();  // Hidden by default

  // Connect latency heatmap signals
  connect(latencyHeatmapPanel_, &LatencyHeatmapPanel::heatmapEnabledChanged,
          this, &MainWindow::onToggleLatencyHeatmap);
  connect(latencyTracker_, &LatencyTracker::latencyAlert,
          this, &MainWindow::onLatencyAlert);

  QAction* showLatencyHeatmapAction = findChild<QAction*>("showLatencyHeatmapAction");
  if (showLatencyHeatmapAction) {
    connect(showLatencyHeatmapAction, &QAction::toggled, latencyHeatmapDock_, &QDockWidget::setVisible);
    connect(latencyHeatmapDock_, &QDockWidget::visibilityChanged, showLatencyHeatmapAction, &QAction::setChecked);
  }

  // Scenario Editor Panel
  scenarioEditorDock_ = new QDockWidget(tr("Scenario Editor"), this);
  scenarioEditorDock_->setObjectName("scenarioEditorDock");
  scenarioEditorDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  scenarioEditorDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                    QDockWidget::DockWidgetFloatable |
                                    QDockWidget::DockWidgetClosable);

  scenarioEditorWidget_ = new ScenarioEditorWidget(this);
  scenarioEditorDock_->setWidget(scenarioEditorWidget_);

  addDockWidget(Qt::BottomDockWidgetArea, scenarioEditorDock_);
  scenarioEditorDock_->hide();  // Hidden by default

  // Connect scenario editor signals
  connect(scenarioEditorWidget_, &ScenarioEditorWidget::scenarioLoaded,
          this, &MainWindow::onScenarioLoaded);
  connect(scenarioEditorWidget_, &ScenarioEditorWidget::scenarioCompleted,
          this, &MainWindow::onScenarioCompleted);

  QAction* showScenarioEditorAction = findChild<QAction*>("showScenarioEditorAction");
  if (showScenarioEditorAction) {
    connect(showScenarioEditorAction, &QAction::toggled, scenarioEditorDock_, &QDockWidget::setVisible);
    connect(scenarioEditorDock_, &QDockWidget::visibilityChanged, showScenarioEditorAction, &QAction::setChecked);
  }

  // Diagnostics Panel (ros2 doctor)
  diagnosticsDock_ = new QDockWidget(tr("Diagnostics"), this);
  diagnosticsDock_->setObjectName("diagnosticsDock");
  diagnosticsDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  diagnosticsDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                 QDockWidget::DockWidgetFloatable |
                                 QDockWidget::DockWidgetClosable);

  diagnosticsPanel_ = new DiagnosticsPanel(this);
  diagnosticsDock_->setWidget(diagnosticsPanel_);

  addDockWidget(Qt::RightDockWidgetArea, diagnosticsDock_);
  tabifyDockWidget(nodeHealthDock_, diagnosticsDock_);
  diagnosticsDock_->hide();  // Hidden by default

  QAction* showDiagnosticsAction = findChild<QAction*>("showDiagnosticsAction");
  if (showDiagnosticsAction) {
    connect(showDiagnosticsAction, &QAction::toggled, diagnosticsDock_, &QDockWidget::setVisible);
    connect(diagnosticsDock_, &QDockWidget::visibilityChanged, showDiagnosticsAction, &QAction::setChecked);
  }

  // Network Topology Panel (DDS network view)
  networkTopologyDock_ = new QDockWidget(tr("Network Topology"), this);
  networkTopologyDock_->setObjectName("networkTopologyDock");
  networkTopologyDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  networkTopologyDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                     QDockWidget::DockWidgetFloatable |
                                     QDockWidget::DockWidgetClosable);

  networkTopologyManager_ = new NetworkTopologyManager(this);
  networkTopologyPanel_ = new NetworkTopologyPanel(this);
  networkTopologyPanel_->setCanvas(canvas_);
  networkTopologyPanel_->setNetworkTopologyManager(networkTopologyManager_);
  networkTopologyDock_->setWidget(networkTopologyPanel_);

  addDockWidget(Qt::RightDockWidgetArea, networkTopologyDock_);
  tabifyDockWidget(diagnosticsDock_, networkTopologyDock_);
  networkTopologyDock_->hide();  // Hidden by default

  QAction* showNetworkTopologyAction = findChild<QAction*>("showNetworkTopologyAction");
  if (showNetworkTopologyAction) {
    connect(showNetworkTopologyAction, &QAction::toggled, networkTopologyDock_, &QDockWidget::setVisible);
    connect(networkTopologyDock_, &QDockWidget::visibilityChanged, showNetworkTopologyAction, &QAction::setChecked);
  }

  // Connect canvas integration signal
  connect(networkTopologyPanel_, &NetworkTopologyPanel::nodeSelectedOnCanvas,
          this, [this](const QString& nodeName) {
    // Find and highlight the node on canvas
    if (!canvas_) return;
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      PackageBlock* block = dynamic_cast<PackageBlock*>(item);
      if (block && block->packageName() == nodeName) {
        canvas_->scene()->clearSelection();
        block->setSelected(true);
        canvas_->centerOn(block);
        break;
      }
    }
  });

  // Behavior Tree Panel
  behaviorTreeDock_ = new QDockWidget(tr("Behavior Tree"), this);
  behaviorTreeDock_->setObjectName("behaviorTreeDock");
  behaviorTreeDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  behaviorTreeDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                  QDockWidget::DockWidgetFloatable |
                                  QDockWidget::DockWidgetClosable);

  behaviorTreePanel_ = new BehaviorTreePanel(this);
  behaviorTreeDock_->setWidget(behaviorTreePanel_);

  addDockWidget(Qt::RightDockWidgetArea, behaviorTreeDock_);
  tabifyDockWidget(networkTopologyDock_, behaviorTreeDock_);
  behaviorTreeDock_->hide();  // Hidden by default

  QAction* showBehaviorTreeAction = findChild<QAction*>("showBehaviorTreeAction");
  if (showBehaviorTreeAction) {
    connect(showBehaviorTreeAction, &QAction::toggled, behaviorTreeDock_, &QDockWidget::setVisible);
    connect(behaviorTreeDock_, &QDockWidget::visibilityChanged, showBehaviorTreeAction, &QAction::setChecked);
  }

  // Mission Planner Panel
  missionPlannerDock_ = new QDockWidget(tr("Mission Planner"), this);
  missionPlannerDock_->setObjectName("missionPlannerDock");
  missionPlannerDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  missionPlannerDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                    QDockWidget::DockWidgetFloatable |
                                    QDockWidget::DockWidgetClosable);

  missionPlannerPanel_ = new MissionPlannerPanel(this);
  missionPlannerPanel_->setBehaviorTreeEditor(behaviorTreePanel_);
  missionPlannerDock_->setWidget(missionPlannerPanel_);

  addDockWidget(Qt::RightDockWidgetArea, missionPlannerDock_);
  tabifyDockWidget(behaviorTreeDock_, missionPlannerDock_);
  missionPlannerDock_->hide();  // Hidden by default

  QAction* showMissionPlannerAction = findChild<QAction*>("showMissionPlannerAction");
  if (showMissionPlannerAction) {
    connect(showMissionPlannerAction, &QAction::toggled, missionPlannerDock_, &QDockWidget::setVisible);
    connect(missionPlannerDock_, &QDockWidget::visibilityChanged, showMissionPlannerAction, &QAction::setChecked);
  }

#ifdef HAVE_QT3D
  // URDF Viewer Panel
  urdfViewerDock_ = new QDockWidget(tr("URDF Viewer"), this);
  urdfViewerDock_->setObjectName("urdfViewerDock");
  urdfViewerDock_->setAllowedAreas(Qt::AllDockWidgetAreas);
  urdfViewerDock_->setFeatures(QDockWidget::DockWidgetMovable |
                                QDockWidget::DockWidgetFloatable |
                                QDockWidget::DockWidgetClosable);

  urdfViewerPanel_ = new URDFViewerPanel(this);
  urdfViewerDock_->setWidget(urdfViewerPanel_);

  addDockWidget(Qt::RightDockWidgetArea, urdfViewerDock_);
  tabifyDockWidget(missionPlannerDock_, urdfViewerDock_);
  urdfViewerDock_->hide();  // Hidden by default

  QAction* showURDFViewerAction = findChild<QAction*>("showURDFViewerAction");
  if (showURDFViewerAction) {
    connect(showURDFViewerAction, &QAction::toggled, urdfViewerDock_, &QDockWidget::setVisible);
    connect(urdfViewerDock_, &QDockWidget::visibilityChanged, showURDFViewerAction, &QAction::setChecked);
  }
#endif

  // Initialize dock drag filter for Ctrl+drag docking
  dockDragFilter_ = new DockDragFilter(this, this);

  // Install dock drag filter on all dock widgets
  for (QDockWidget* dock : findChildren<QDockWidget*>()) {
    dock->installEventFilter(dockDragFilter_);
  }

  // Initialize Remote Connection Manager
  remoteConnectionManager_ = new RemoteConnectionManager(this);

  connect(remoteConnectionManager_, &RemoteConnectionManager::connectionEstablished,
          this, &MainWindow::onRemoteConnectionEstablished);
  connect(remoteConnectionManager_, &RemoteConnectionManager::connectionFailed,
          this, &MainWindow::onRemoteConnectionFailed);
  connect(remoteConnectionManager_, &RemoteConnectionManager::connectionLost,
          this, &MainWindow::onRemoteConnectionLost);
}

void MainWindow::setupCentralWidget() {
  // Create the multi-canvas tab widget
  canvasTabWidget_ = new CanvasTabWidget(this);
  canvasTabWidget_->setObjectName("canvasTabWidget");
  canvasTabWidget_->setUndoStack(undoStack_);
  setCentralWidget(canvasTabWidget_);

  // Get the initial canvas (default "Main" canvas created by CanvasTabWidget)
  canvas_ = canvasTabWidget_->currentCanvas();

  // Connect tab widget signals - these forward from active canvas
  connect(canvasTabWidget_, &CanvasTabWidget::blockSelected, this, &MainWindow::onBlockSelected);
  connect(canvasTabWidget_, &CanvasTabWidget::openBlockInVSCodeRequested, this, &MainWindow::onOpenBlockInVSCode);

  // When user changes preferred YAML source via context menu, refresh the param dashboard
  connect(canvasTabWidget_, &CanvasTabWidget::blockYamlSourceChanged, this,
          [this](PackageBlock* block, const QString& /* yamlSource */) {
    // Refresh the param dashboard if this is the currently selected block
    if (paramDashboard_->currentBlock() == block) {
      paramDashboard_->setCurrentBlock(block);
    }
  });

  // Connect connection line signals when new connections are created
  connect(canvasTabWidget_, &CanvasTabWidget::connectionCreated, this,
          [this](ConnectionLine* connection) {
    if (!connection) return;

    // Connect double-click to show topic echo dialog
    connect(connection, &ConnectionLine::doubleClicked,
            this, &MainWindow::onConnectionDoubleClicked);

    // Connect single-click for topic inspector (when live monitoring enabled)
    connect(connection, &ConnectionLine::clicked,
            this, &MainWindow::onConnectionClicked);
  });

  // Update canvas_ pointer when tab changes
  connect(canvasTabWidget_, &CanvasTabWidget::currentCanvasChanged,
          this, &MainWindow::onCurrentCanvasChanged);
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

  // Add LocalAI status widget next to ROS status
  localAIStatusWidget_ = new LocalAIStatusWidget(this);
  statusBar()->addPermanentWidget(localAIStatusWidget_);

  statusBar()->showMessage(tr("Ready"));
}

void MainWindow::onNewProject() {
  // Clear param dashboard first to avoid dangling pointer when canvas clears blocks
  paramDashboard_->setCurrentBlock(nullptr);
  canvasTabWidget_->clearAll();
  canvas_ = canvasTabWidget_->currentCanvas();  // Update canvas pointer
  undoStack_->clear();
  currentProjectPath_.clear();
  baseWindowTitle_ = "ROS Weaver - Visual ROS2 Package Editor";
  setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());
  statusBar()->showMessage(tr("New project created"));
  NotificationManager::instance().showSuccess(tr("New project created"));
}

void MainWindow::onOpenProject() {
  QString fileName = QFileDialog::getOpenFileName(
    this,
    tr("Open ROS Weaver Project"),
    QString(),
    tr("ROS Weaver Projects (*.rwp);;All Files (*)"),
    nullptr,
    QFileDialog::DontUseNativeDialog
  );

  if (!fileName.isEmpty()) {
    if (loadProject(fileName)) {
      currentProjectPath_ = fileName;
      baseWindowTitle_ = QString("ROS Weaver - %1").arg(QFileInfo(fileName).fileName());
      setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());
      statusBar()->showMessage(tr("Opened: %1").arg(fileName));
      NotificationManager::instance().showSuccess(tr("Project opened successfully"));
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
    tr("ROS Weaver Projects (*.rwp);;All Files (*)"),
    nullptr,
    QFileDialog::DontUseNativeDialog
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
      NotificationManager::instance().showSuccess(tr("Project saved successfully"));
    }
  }
}

void MainWindow::onExportToPlantUML() {
  QString fileName = QFileDialog::getSaveFileName(
    this,
    tr("Export to PlantUML"),
    QString(),
    GraphExporter::fileFilter(ExportFormat::PlantUML)
  );

  if (!fileName.isEmpty()) {
    if (!fileName.endsWith(".puml", Qt::CaseInsensitive) &&
        !fileName.endsWith(".plantuml", Qt::CaseInsensitive)) {
      fileName += ".puml";
    }

    Project project;
    canvas_->exportToProject(project);

    ExportOptions options;
    options.title = project.metadata().name;

    if (GraphExporter::instance().exportToFile(project, fileName, ExportFormat::PlantUML, options)) {
      statusBar()->showMessage(tr("Exported to: %1").arg(fileName));
      NotificationManager::instance().showSuccess(tr("Diagram exported to PlantUML"));
    } else {
      ErrorHandler::showError(tr("Could not export to %1").arg(fileName));
    }
  }
}

void MainWindow::onExportToMermaid() {
  QString fileName = QFileDialog::getSaveFileName(
    this,
    tr("Export to Mermaid"),
    QString(),
    GraphExporter::fileFilter(ExportFormat::Mermaid)
  );

  if (!fileName.isEmpty()) {
    if (!fileName.endsWith(".mmd", Qt::CaseInsensitive) &&
        !fileName.endsWith(".mermaid", Qt::CaseInsensitive) &&
        !fileName.endsWith(".md", Qt::CaseInsensitive)) {
      fileName += ".mmd";
    }

    Project project;
    canvas_->exportToProject(project);

    ExportOptions options;
    options.title = project.metadata().name;

    if (GraphExporter::instance().exportToFile(project, fileName, ExportFormat::Mermaid, options)) {
      statusBar()->showMessage(tr("Exported to: %1").arg(fileName));
      NotificationManager::instance().showSuccess(tr("Diagram exported to Mermaid"));
    } else {
      ErrorHandler::showError(tr("Could not export to %1").arg(fileName));
    }
  }
}

void MainWindow::onExportToGraphviz() {
  QString fileName = QFileDialog::getSaveFileName(
    this,
    tr("Export to Graphviz DOT"),
    QString(),
    GraphExporter::fileFilter(ExportFormat::Graphviz)
  );

  if (!fileName.isEmpty()) {
    if (!fileName.endsWith(".dot", Qt::CaseInsensitive) &&
        !fileName.endsWith(".gv", Qt::CaseInsensitive)) {
      fileName += ".dot";
    }

    Project project;
    canvas_->exportToProject(project);

    ExportOptions options;
    options.title = project.metadata().name;

    if (GraphExporter::instance().exportToFile(project, fileName, ExportFormat::Graphviz, options)) {
      statusBar()->showMessage(tr("Exported to: %1").arg(fileName));
      NotificationManager::instance().showSuccess(tr("Diagram exported to Graphviz DOT"));
    } else {
      ErrorHandler::showError(tr("Could not export to %1").arg(fileName));
    }
  }
}

void MainWindow::onGenerateDocumentation() {
  Project project;
  canvas_->exportToProject(project);

  ArchitectureDocDialog dialog(&project, this);
  dialog.exec();
}

void MainWindow::onImportFromDot() {
  QString fileName = QFileDialog::getOpenFileName(
    this,
    tr("Import rqt_graph DOT File"),
    QString(),
    tr("DOT Files (*.dot *.gv);;All Files (*)")
  );

  if (fileName.isEmpty()) {
    return;
  }

  DotGraph graph = DotImporter::instance().parseFile(fileName);

  if (!graph.isValid) {
    ErrorHandler::showError(tr("Could not parse DOT file: %1").arg(
        graph.errors.isEmpty() ? tr("Unknown error") : graph.errors.first()));
    return;
  }

  int nodesCreated = DotImporter::instance().importToCanvas(graph, canvas_);

  if (nodesCreated > 0) {
    statusBar()->showMessage(tr("Imported %1 nodes from DOT file").arg(nodesCreated));
    NotificationManager::instance().showSuccess(
        tr("Imported %1 nodes from rqt_graph").arg(nodesCreated));
    setProjectDirty(true);
  } else {
    ErrorHandler::showError(tr("No nodes could be imported from the DOT file"));
  }
}

void MainWindow::onImportFromCaret() {
  QString fileName = QFileDialog::getOpenFileName(
    this,
    tr("Import CARET Architecture YAML"),
    QString(),
    tr("YAML Files (*.yaml *.yml);;All Files (*)")
  );

  if (fileName.isEmpty()) {
    return;
  }

  CaretArchitecture arch = CaretImporter::instance().parseFile(fileName);

  if (!arch.isValid) {
    ErrorHandler::showError(tr("Could not parse CARET YAML: %1").arg(
        arch.errors.isEmpty() ? tr("Unknown error") : arch.errors.first()));
    return;
  }

  int nodesCreated = CaretImporter::instance().importToCanvas(arch, canvas_);

  if (nodesCreated > 0) {
    statusBar()->showMessage(tr("Imported %1 nodes from CARET architecture").arg(nodesCreated));
    NotificationManager::instance().showSuccess(
        tr("Imported %1 nodes from CARET").arg(nodesCreated));
    setProjectDirty(true);
  } else {
    ErrorHandler::showError(tr("No nodes could be imported from the CARET file"));
  }
}

bool MainWindow::saveProject(const QString& filePath) {
  Project project;
  canvasTabWidget_->exportToProject(project);

  // Set metadata
  project.metadata().name = QFileInfo(filePath).baseName();

  if (!project.saveToFile(filePath)) {
    ErrorHandler::showError(tr("Could not save project to %1").arg(filePath));
    return false;
  }

  return true;
}

bool MainWindow::loadProject(const QString& filePath) {
  QString errorMsg;
  Project project = Project::loadFromFile(filePath, &errorMsg);

  if (!errorMsg.isEmpty()) {
    ErrorHandler::showError(errorMsg);
    return false;
  }

  // Clear param dashboard before import (which clears the canvas)
  paramDashboard_->setCurrentBlock(nullptr);
  canvasTabWidget_->importFromProject(project);
  canvas_ = canvasTabWidget_->currentCanvas();  // Update canvas pointer

  // Clear undo stack after loading - loaded state is the new baseline
  undoStack_->clear();
  undoStack_->setClean();
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

void MainWindow::onLoadBehaviorTreeExample() {
  // Try to load from installed package share directory first
  QString examplePath;
  QString btXmlPath;

  try {
    std::string packageShare = ament_index_cpp::get_package_share_directory("ros_weaver");
    examplePath = QString::fromStdString(packageShare) +
                  "/examples/behavior_tree_patrol/behavior_tree_patrol.rwp";
    btXmlPath = QString::fromStdString(packageShare) +
                "/examples/behavior_tree_patrol/patrol_behavior.xml";
  } catch (const std::exception&) {
    // Fallback to source directory (for development)
    examplePath = QDir::currentPath() +
                  "/src/ros_weaver/examples/behavior_tree_patrol/behavior_tree_patrol.rwp";
    btXmlPath = QDir::currentPath() +
                "/src/ros_weaver/examples/behavior_tree_patrol/patrol_behavior.xml";
  }

  // Check if file exists, try multiple fallback paths for development
  if (!QFile::exists(examplePath)) {
    QStringList searchPaths;
    QDir currentDir(QDir::currentPath());

    // Try current directory
    searchPaths << currentDir.absoluteFilePath("examples/behavior_tree_patrol/behavior_tree_patrol.rwp");

    // Try going up from build directory (handles running from build/ros_weaver)
    QDir parentDir = currentDir;
    for (int i = 0; i < 3; ++i) {
      if (parentDir.cdUp()) {
        searchPaths << parentDir.absoluteFilePath("src/ros_weaver/examples/behavior_tree_patrol/behavior_tree_patrol.rwp");
      }
    }

    bool found = false;
    for (const QString& path : searchPaths) {
      if (QFile::exists(path)) {
        examplePath = path;
        btXmlPath = QFileInfo(path).absolutePath() + "/patrol_behavior.xml";
        found = true;
        break;
      }
    }

    if (!found) {
      QMessageBox::warning(this, tr("Example Not Found"),
        tr("Could not find the Behavior Tree Patrol example.\n"
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
      tr("Failed to load Behavior Tree Patrol example:\n%1").arg(errorMsg));
    return;
  }

  // Clear param dashboard before import (which clears the canvas)
  paramDashboard_->setCurrentBlock(nullptr);
  paramDashboard_->clearYamlFiles();

  // Import the project
  canvas_->importFromProject(project);
  currentProjectPath_.clear();
  baseWindowTitle_ = "ROS Weaver - Behavior Tree Patrol (Example)";
  setWindowTitle(baseWindowTitle_ + rosStatusWidget_->titleBarSuffix());

  statusBar()->showMessage(tr("Loaded Behavior Tree Patrol example"));

  // Show the Behavior Tree panel and load the example behavior tree
  if (behaviorTreeDock_) {
    behaviorTreeDock_->show();
    behaviorTreeDock_->raise();  // Bring to front
  }

  // Load the behavior tree XML into the panel
  if (behaviorTreePanel_ && QFile::exists(btXmlPath)) {
    behaviorTreePanel_->loadFromFile(btXmlPath);
  }

  // Append to output with example information
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Loaded Behavior Tree Patrol example project.\n\n"));
  outputPanel_->appendBuildOutput(tr("This example demonstrates:\n"));
  outputPanel_->appendBuildOutput(tr("  - Nav2 Behavior Tree Navigator for autonomous patrol\n"));
  outputPanel_->appendBuildOutput(tr("  - Waypoint-based patrol route with 4 locations\n"));
  outputPanel_->appendBuildOutput(tr("  - Battery monitoring with dock return behavior\n"));
  outputPanel_->appendBuildOutput(tr("  - Recovery behaviors for navigation failures\n"));
  outputPanel_->appendBuildOutput(tr("  - Parallel status monitoring during patrol\n\n"));
  outputPanel_->appendBuildOutput(tr("Behavior Tree Structure:\n"));
  outputPanel_->appendBuildOutput(tr("  - System health checks (battery, localization, map)\n"));
  outputPanel_->appendBuildOutput(tr("  - ReactiveFallback for priority-based behaviors\n"));
  outputPanel_->appendBuildOutput(tr("  - Waypoint sequence: Kitchen -> Living Room -> Entrance -> Office\n"));
  outputPanel_->appendBuildOutput(tr("  - Docking sub-tree for charging when battery is low\n\n"));
  outputPanel_->appendBuildOutput(tr("The Behavior Tree panel is now open on the right.\n"));
  outputPanel_->appendBuildOutput(tr("Explore the patrol_behavior.xml tree structure visually!\n\n"));
  outputPanel_->appendBuildOutput(tr("BT XML location: %1\n").arg(btXmlPath));
}

#ifdef HAVE_QT3D
void MainWindow::onLoadTurtleBot3URDFExample() {
  // Try to find TurtleBot3 description package
  QString urdfPath;
  QString modelName = "burger";  // Default model

  // Check TURTLEBOT3_MODEL environment variable
  QString tbModel = qgetenv("TURTLEBOT3_MODEL");
  if (!tbModel.isEmpty()) {
    modelName = tbModel;
  }

  // Try to find the URDF from the turtlebot3_description package
  try {
    std::string packageShare = ament_index_cpp::get_package_share_directory("turtlebot3_description");
    urdfPath = QString::fromStdString(packageShare) +
               "/urdf/turtlebot3_" + modelName + ".urdf";
  } catch (const std::exception&) {
    // Package not found
  }

  // Check if file exists
  if (urdfPath.isEmpty() || !QFile::exists(urdfPath)) {
    // Try common fallback paths
    QStringList searchPaths;
    searchPaths << QString("/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_%1.urdf").arg(modelName);
    searchPaths << QString("/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_%1.urdf").arg(modelName);

    bool found = false;
    for (const QString& path : searchPaths) {
      if (QFile::exists(path)) {
        urdfPath = path;
        found = true;
        break;
      }
    }

    if (!found) {
      QMessageBox::warning(this, tr("TurtleBot3 Not Found"),
        tr("Could not find TurtleBot3 description package.\n\n"
           "Install it with:\n"
           "  sudo apt install ros-jazzy-turtlebot3-description\n\n"
           "Or for your ROS2 distribution:\n"
           "  sudo apt install ros-${ROS_DISTRO}-turtlebot3-description\n\n"
           "Current TURTLEBOT3_MODEL: %1").arg(modelName.isEmpty() ? "(not set)" : modelName));
      return;
    }
  }

  // Show the URDF viewer panel
  urdfViewerDock_->show();
  urdfViewerDock_->raise();

  // Load the URDF
  urdfViewerPanel_->loadURDF(urdfPath);

  // Update the menu action checkbox
  QAction* showURDFViewerAction = findChild<QAction*>("showURDFViewerAction");
  if (showURDFViewerAction) {
    showURDFViewerAction->setChecked(true);
  }

  // Show instructions in output panel
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Loaded TurtleBot3 URDF: %1\n\n").arg(urdfPath));
  outputPanel_->appendBuildOutput(tr("URDF Viewer Controls:\n"));
  outputPanel_->appendBuildOutput(tr("  Mouse Controls:\n"));
  outputPanel_->appendBuildOutput(tr("    - Right-click drag: Orbit camera\n"));
  outputPanel_->appendBuildOutput(tr("    - Alt + Left-click drag: Orbit camera (Maya-style)\n"));
  outputPanel_->appendBuildOutput(tr("    - Shift + Right-click drag: Pan camera\n"));
  outputPanel_->appendBuildOutput(tr("    - Scroll wheel: Zoom in/out\n"));
  outputPanel_->appendBuildOutput(tr("    - Left-click: Select joint\n"));
  outputPanel_->appendBuildOutput(tr("    - Ctrl + Left-click: Multi-select joints\n\n"));
  outputPanel_->appendBuildOutput(tr("  Keyboard Shortcuts:\n"));
  outputPanel_->appendBuildOutput(tr("    - X: Rotate selected joint around X axis (+90°)\n"));
  outputPanel_->appendBuildOutput(tr("    - Y: Rotate selected joint around Y axis (+90°)\n"));
  outputPanel_->appendBuildOutput(tr("    - Z: Rotate selected joint around Z axis (+90°)\n"));
  outputPanel_->appendBuildOutput(tr("    - Shift+X/Y/Z: Rotate in opposite direction (-90°)\n"));
  outputPanel_->appendBuildOutput(tr("    - Home: Reset camera view\n"));
  outputPanel_->appendBuildOutput(tr("    - Ctrl+A: Select all joints\n"));
  outputPanel_->appendBuildOutput(tr("    - Escape: Clear selection\n\n"));
  outputPanel_->appendBuildOutput(tr("The tree view shows the robot's kinematic structure.\n"));
  outputPanel_->appendBuildOutput(tr("Click on joints to see their axis indicators (RGB = XYZ).\n"));

  statusBar()->showMessage(tr("Loaded TurtleBot3 %1 URDF in viewer").arg(modelName), 5000);
}
#endif

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

  // Auto-start ROS Logger to capture log messages
  if (outputPanel_->rosLogViewer() && !outputPanel_->rosLogViewer()->isListening()) {
    outputPanel_->rosLogViewer()->startListening();
  }

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

void MainWindow::onLaunchTurtleBot3Gazebo() {
  // Check if required TurtleBot3 packages are available
  QProcess checkProcess;
  checkProcess.start("ros2", QStringList() << "pkg" << "list");
  checkProcess.waitForFinished(5000);
  QString packages = checkProcess.readAllStandardOutput();

  QStringList missingPackages;
  if (!packages.contains("turtlebot3_gazebo")) {
    missingPackages << "turtlebot3_gazebo";
  }
  if (!packages.contains("nav2_bringup")) {
    missingPackages << "nav2_bringup";
  }
  if (!packages.contains("slam_toolbox")) {
    missingPackages << "slam_toolbox";
  }

  if (!missingPackages.isEmpty()) {
    QString distro = qgetenv("ROS_DISTRO");
    if (distro.isEmpty()) distro = "jazzy";

    QMessageBox::warning(this, tr("Missing Packages"),
      tr("The following packages are required but not installed:\n\n"
         "  %1\n\n"
         "Install TurtleBot3 packages with:\n"
         "  sudo apt install ros-%2-turtlebot3*\n\n"
         "Install Navigation2 packages with:\n"
         "  sudo apt install ros-%2-navigation2 ros-%2-nav2-bringup\n\n"
         "Install SLAM Toolbox with:\n"
         "  sudo apt install ros-%2-slam-toolbox")
         .arg(missingPackages.join(", "), distro));
    return;
  }

  // Check TURTLEBOT3_MODEL environment variable
  QString tbModel = qgetenv("TURTLEBOT3_MODEL");
  if (tbModel.isEmpty()) {
    QMessageBox::StandardButton reply = QMessageBox::question(this,
      tr("TurtleBot3 Model"),
      tr("TURTLEBOT3_MODEL environment variable is not set.\n\n"
         "Would you like to launch with 'burger' model?\n\n"
         "You can set this permanently with:\n"
         "  export TURTLEBOT3_MODEL=burger"),
      QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

    if (reply == QMessageBox::Cancel) {
      return;
    } else if (reply == QMessageBox::No) {
      QMessageBox::information(this, tr("Manual Setup Required"),
        tr("Please set TURTLEBOT3_MODEL and try again:\n\n"
           "  export TURTLEBOT3_MODEL=burger  (or waffle, waffle_pi)"));
      return;
    }
    tbModel = "burger";
  }

  // Create dialog to select what to launch
  QDialog launchDialog(this);
  launchDialog.setWindowTitle(tr("Launch TurtleBot3 Gazebo Simulation"));
  launchDialog.setMinimumWidth(400);

  QVBoxLayout* layout = new QVBoxLayout(&launchDialog);

  QLabel* infoLabel = new QLabel(tr("Select components to launch:"), &launchDialog);
  layout->addWidget(infoLabel);

  QGroupBox* componentGroup = new QGroupBox(tr("Components"), &launchDialog);
  QVBoxLayout* componentLayout = new QVBoxLayout(componentGroup);

  QCheckBox* gazeboCheck = new QCheckBox(tr("Gazebo Simulation (required)"), componentGroup);
  gazeboCheck->setChecked(true);
  gazeboCheck->setEnabled(false);  // Always launch Gazebo
  componentLayout->addWidget(gazeboCheck);

  QCheckBox* slamCheck = new QCheckBox(tr("SLAM Toolbox (mapping)"), componentGroup);
  slamCheck->setChecked(true);
  slamCheck->setToolTip(tr("Enable SLAM for simultaneous localization and mapping"));
  componentLayout->addWidget(slamCheck);

  QCheckBox* nav2Check = new QCheckBox(tr("Nav2 Navigation Stack"), componentGroup);
  nav2Check->setChecked(true);
  nav2Check->setToolTip(tr("Enable Navigation2 for autonomous path planning"));
  componentLayout->addWidget(nav2Check);

  QCheckBox* rvizCheck = new QCheckBox(tr("RViz2 Visualization"), componentGroup);
  rvizCheck->setChecked(true);
  rvizCheck->setToolTip(tr("Launch RViz2 to visualize robot state, map, and navigation"));
  componentLayout->addWidget(rvizCheck);

  layout->addWidget(componentGroup);

  // World selection
  QGroupBox* worldGroup = new QGroupBox(tr("Gazebo World"), &launchDialog);
  QVBoxLayout* worldLayout = new QVBoxLayout(worldGroup);

  QRadioButton* emptyWorldRadio = new QRadioButton(tr("Empty World"), worldGroup);
  QRadioButton* tb3WorldRadio = new QRadioButton(tr("TurtleBot3 World (obstacles)"), worldGroup);
  QRadioButton* houseWorldRadio = new QRadioButton(tr("TurtleBot3 House"), worldGroup);
  tb3WorldRadio->setChecked(true);

  worldLayout->addWidget(emptyWorldRadio);
  worldLayout->addWidget(tb3WorldRadio);
  worldLayout->addWidget(houseWorldRadio);

  layout->addWidget(worldGroup);

  // Add to canvas option
  QCheckBox* addToCanvasCheck = new QCheckBox(tr("Load TurtleBot3 Navigation project on canvas"), &launchDialog);
  addToCanvasCheck->setChecked(true);
  layout->addWidget(addToCanvasCheck);

  // Dialog buttons
  QDialogButtonBox* buttonBox = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &launchDialog);
  connect(buttonBox, &QDialogButtonBox::accepted, &launchDialog, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, &launchDialog, &QDialog::reject);
  layout->addWidget(buttonBox);

  if (launchDialog.exec() != QDialog::Accepted) {
    return;
  }

  // Determine which world to use
  QString worldArg;
  if (emptyWorldRadio->isChecked()) {
    worldArg = "empty_world.launch.py";
  } else if (tb3WorldRadio->isChecked()) {
    worldArg = "turtlebot3_world.launch.py";
  } else if (houseWorldRadio->isChecked()) {
    worldArg = "turtlebot3_house.launch.py";
  }

  // Load example project if requested
  if (addToCanvasCheck->isChecked()) {
    onLoadTurtleBotExample();
  }

  // Setup output
  outputPanel_->clearBuildOutput();
  outputPanel_->appendBuildOutput(tr("Launching TurtleBot3 Gazebo Simulation...\n"));
  outputPanel_->appendBuildOutput(tr("Model: %1\n").arg(tbModel));
  outputPanel_->appendBuildOutput(tr("World: %1\n\n").arg(worldArg));

  // Auto-start ROS Logger to capture log messages
  if (outputPanel_->rosLogViewer() && !outputPanel_->rosLogViewer()->isListening()) {
    outputPanel_->rosLogViewer()->startListening();
  }

  QString rosDistro = qgetenv("ROS_DISTRO");
  if (rosDistro.isEmpty()) rosDistro = "jazzy";

  // Launch Gazebo simulation
  QProcess* gazeboProc = new QProcess(this);
  gazeboProc->setProcessChannelMode(QProcess::MergedChannels);

  connect(gazeboProc, &QProcess::readyReadStandardOutput, [this, gazeboProc]() {
    outputPanel_->appendBuildOutput(QString::fromUtf8(gazeboProc->readAllStandardOutput()));
  });

  connect(gazeboProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, gazeboProc](int exitCode, QProcess::ExitStatus) {
    outputPanel_->appendBuildOutput(tr("\nGazebo process exited with code %1\n").arg(exitCode));
    gazeboProc->deleteLater();
  });

  QString gazeboCmd = QString(
    "source /opt/ros/%1/setup.bash && "
    "export TURTLEBOT3_MODEL=%2 && "
    "ros2 launch turtlebot3_gazebo %3"
  ).arg(rosDistro, tbModel, worldArg);

  gazeboProc->start("bash", QStringList() << "-c" << gazeboCmd);
  outputPanel_->appendBuildOutput(tr("Started: ros2 launch turtlebot3_gazebo %1\n").arg(worldArg));
  outputPanel_->appendBuildOutput(tr("Waiting for Gazebo to initialize...\n\n"));

  // Launch SLAM, Nav2, and RViz after a delay to let Gazebo start
  int launchDelay = 5000;  // 5 seconds

  if (slamCheck->isChecked()) {
    QTimer::singleShot(launchDelay, this, [this, rosDistro, tbModel]() {
      QProcess* slamProc = new QProcess(this);
      slamProc->setProcessChannelMode(QProcess::MergedChannels);

      connect(slamProc, &QProcess::readyReadStandardOutput, [this, slamProc]() {
        outputPanel_->appendBuildOutput(QString::fromUtf8(slamProc->readAllStandardOutput()));
      });

      connect(slamProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
              [this, slamProc](int exitCode, QProcess::ExitStatus) {
        outputPanel_->appendBuildOutput(tr("\nSLAM process exited with code %1\n").arg(exitCode));
        slamProc->deleteLater();
      });

      QString slamCmd = QString(
        "source /opt/ros/%1/setup.bash && "
        "export TURTLEBOT3_MODEL=%2 && "
        "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"
      ).arg(rosDistro, tbModel);

      slamProc->start("bash", QStringList() << "-c" << slamCmd);
      outputPanel_->appendBuildOutput(tr("Started: SLAM Toolbox\n"));
    });
    launchDelay += 2000;
  }

  if (nav2Check->isChecked()) {
    QTimer::singleShot(launchDelay, this, [this, rosDistro, tbModel]() {
      QProcess* nav2Proc = new QProcess(this);
      nav2Proc->setProcessChannelMode(QProcess::MergedChannels);

      connect(nav2Proc, &QProcess::readyReadStandardOutput, [this, nav2Proc]() {
        outputPanel_->appendBuildOutput(QString::fromUtf8(nav2Proc->readAllStandardOutput()));
      });

      connect(nav2Proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
              [this, nav2Proc](int exitCode, QProcess::ExitStatus) {
        outputPanel_->appendBuildOutput(tr("\nNav2 process exited with code %1\n").arg(exitCode));
        nav2Proc->deleteLater();
      });

      QString nav2Cmd = QString(
        "source /opt/ros/%1/setup.bash && "
        "export TURTLEBOT3_MODEL=%2 && "
        "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"
      ).arg(rosDistro, tbModel);

      nav2Proc->start("bash", QStringList() << "-c" << nav2Cmd);
      outputPanel_->appendBuildOutput(tr("Started: Navigation2 Stack\n"));
    });
    launchDelay += 2000;
  }

  if (rvizCheck->isChecked()) {
    QTimer::singleShot(launchDelay, this, [this, rosDistro, tbModel]() {
      QProcess* rvizProc = new QProcess(this);
      rvizProc->setProcessChannelMode(QProcess::MergedChannels);

      connect(rvizProc, &QProcess::readyReadStandardOutput, [this, rvizProc]() {
        outputPanel_->appendBuildOutput(QString::fromUtf8(rvizProc->readAllStandardOutput()));
      });

      connect(rvizProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
              [this, rvizProc](int exitCode, QProcess::ExitStatus) {
        outputPanel_->appendBuildOutput(tr("\nRViz process exited with code %1\n").arg(exitCode));
        rvizProc->deleteLater();
      });

      QString rvizCmd = QString(
        "source /opt/ros/%1/setup.bash && "
        "export TURTLEBOT3_MODEL=%2 && "
        "ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true"
      ).arg(rosDistro, tbModel);

      rvizProc->start("bash", QStringList() << "-c" << rvizCmd);
      outputPanel_->appendBuildOutput(tr("Started: RViz2\n"));
    });
    launchDelay += 1000;
  }

  // Show instructions after all components have started
  QTimer::singleShot(launchDelay + 2000, this, [this]() {
    outputPanel_->appendBuildOutput(tr("\n=== TurtleBot3 Simulation Ready ===\n\n"));
    outputPanel_->appendBuildOutput(tr("To control the robot with keyboard:\n"));
    outputPanel_->appendBuildOutput(tr("  ros2 run turtlebot3_teleop teleop_keyboard\n\n"));
    outputPanel_->appendBuildOutput(tr("To set a 2D navigation goal in RViz:\n"));
    outputPanel_->appendBuildOutput(tr("  1. Click '2D Goal Pose' button in RViz toolbar\n"));
    outputPanel_->appendBuildOutput(tr("  2. Click and drag on the map to set goal position and orientation\n\n"));
    outputPanel_->appendBuildOutput(tr("To save the map after mapping:\n"));
    outputPanel_->appendBuildOutput(tr("  ros2 run nav2_map_server map_saver_cli -f ~/my_map\n\n"));
    outputPanel_->appendBuildOutput(tr("Use the TF Tree tab (Ctrl+T) to visualize transforms!\n"));
    outputPanel_->appendBuildOutput(tr("Use the Topic Viewer (Ctrl+Shift+T) to see live data flow.\n"));

    statusBar()->showMessage(tr("TurtleBot3 Gazebo simulation launched"), 10000);
  });
}

void MainWindow::onLaunchRviz2() {
  // Check if rviz2 is available
  QProcess checkProcess;
  checkProcess.start("which", QStringList() << "rviz2");
  checkProcess.waitForFinished(5000);

  if (checkProcess.exitCode() != 0) {
    // Try checking via ros2 pkg
    checkProcess.start("ros2", QStringList() << "pkg" << "list");
    checkProcess.waitForFinished(5000);
    QString output = checkProcess.readAllStandardOutput();
    if (!output.contains("rviz2")) {
      QMessageBox::warning(this, tr("RViz2 Not Found"),
        tr("RViz2 does not appear to be installed.\n\n"
           "Please install it using:\n"
           "  sudo apt install ros-${ROS_DISTRO}-rviz2"));
      return;
    }
  }

  // Create a dialog for launch options
  QDialog dialog(this);
  dialog.setWindowTitle(tr("Launch RViz2"));
  dialog.setMinimumWidth(400);

  QVBoxLayout* layout = new QVBoxLayout(&dialog);

  // Description
  QLabel* descLabel = new QLabel(tr("Choose how to launch RViz2:"));
  layout->addWidget(descLabel);

  // Options group
  QGroupBox* optionsGroup = new QGroupBox(tr("Launch Options"));
  QVBoxLayout* optionsLayout = new QVBoxLayout(optionsGroup);

  QRadioButton* defaultOption = new QRadioButton(tr("Default configuration"));
  defaultOption->setToolTip(tr("Launch RViz2 with its default configuration"));
  defaultOption->setChecked(true);
  optionsLayout->addWidget(defaultOption);

  QRadioButton* emptyOption = new QRadioButton(tr("Empty configuration"));
  emptyOption->setToolTip(tr("Launch RViz2 with no displays configured"));
  optionsLayout->addWidget(emptyOption);

  QRadioButton* customOption = new QRadioButton(tr("Load custom .rviz file"));
  customOption->setToolTip(tr("Launch RViz2 with a custom configuration file"));
  optionsLayout->addWidget(customOption);

  // Custom file selection
  QHBoxLayout* fileLayout = new QHBoxLayout();
  QLineEdit* fileEdit = new QLineEdit();
  fileEdit->setPlaceholderText(tr("Select a .rviz configuration file..."));
  fileEdit->setEnabled(false);
  QPushButton* browseButton = new QPushButton(tr("Browse..."));
  browseButton->setEnabled(false);
  fileLayout->addWidget(fileEdit);
  fileLayout->addWidget(browseButton);
  optionsLayout->addLayout(fileLayout);

  layout->addWidget(optionsGroup);

  // Enable/disable file selection based on custom option
  connect(customOption, &QRadioButton::toggled, [fileEdit, browseButton](bool checked) {
    fileEdit->setEnabled(checked);
    browseButton->setEnabled(checked);
  });

  // Browse button action
  connect(browseButton, &QPushButton::clicked, [this, fileEdit]() {
    QString fileName = QFileDialog::getOpenFileName(this,
      tr("Select RViz2 Configuration"),
      QDir::homePath(),
      tr("RViz2 Config Files (*.rviz);;All Files (*)"));
    if (!fileName.isEmpty()) {
      fileEdit->setText(fileName);
    }
  });

  // Dialog buttons
  QDialogButtonBox* buttonBox = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttonBox);

  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  // Validate custom file if selected
  QString configFile;
  if (customOption->isChecked()) {
    configFile = fileEdit->text();
    if (configFile.isEmpty()) {
      QMessageBox::warning(this, tr("No File Selected"),
        tr("Please select a .rviz configuration file."));
      return;
    }
    if (!QFile::exists(configFile)) {
      QMessageBox::warning(this, tr("File Not Found"),
        tr("The selected configuration file does not exist:\n%1").arg(configFile));
      return;
    }
  }

  // Build the launch command
  QString rosDistro = qEnvironmentVariable("ROS_DISTRO", "jazzy");
  QString rvizCmd;

  if (emptyOption->isChecked()) {
    // Launch with empty config
    rvizCmd = QString("source /opt/ros/%1/setup.bash && rviz2").arg(rosDistro);
  } else if (customOption->isChecked()) {
    // Launch with custom config
    rvizCmd = QString("source /opt/ros/%1/setup.bash && rviz2 -d \"%2\"")
      .arg(rosDistro, configFile);
  } else {
    // Default - just launch rviz2
    rvizCmd = QString("source /opt/ros/%1/setup.bash && rviz2").arg(rosDistro);
  }

  // Launch RViz2 as a separate process
  QProcess* rvizProc = new QProcess(this);
  rvizProc->setProcessChannelMode(QProcess::MergedChannels);

  connect(rvizProc, &QProcess::readyReadStandardOutput, [this, rvizProc]() {
    QString output = QString::fromUtf8(rvizProc->readAllStandardOutput());
    // Filter out excessive Qt/Ogre debug messages to keep output clean
    if (!output.contains("[DEBUG]") && !output.contains("Ogre")) {
      outputPanel_->appendBuildOutput(output);
    }
  });

  connect(rvizProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, rvizProc](int exitCode, QProcess::ExitStatus) {
    outputPanel_->appendBuildOutput(tr("\nRViz2 process exited with code %1\n").arg(exitCode));
    rvizProc->deleteLater();
    statusBar()->showMessage(tr("RViz2 closed"), 3000);
  });

  connect(rvizProc, &QProcess::errorOccurred, [this, rvizProc](QProcess::ProcessError error) {
    QString errorMsg;
    switch (error) {
      case QProcess::FailedToStart:
        errorMsg = tr("Failed to start RViz2. Make sure it is installed.");
        break;
      case QProcess::Crashed:
        errorMsg = tr("RViz2 crashed unexpectedly.");
        break;
      default:
        errorMsg = tr("RViz2 encountered an error.");
        break;
    }
    outputPanel_->appendBuildOutput(tr("\nError: %1\n").arg(errorMsg));
    rvizProc->deleteLater();
  });

  // Start the process
  rvizProc->start("bash", QStringList() << "-c" << rvizCmd);

  // Show output panel and update status
  if (outputDock_) {
    outputDock_->show();
    outputDock_->raise();
  }
  outputPanel_->showBuildTab();
  outputPanel_->appendBuildOutput(tr("\n=== Launching RViz2 ===\n"));
  if (!configFile.isEmpty()) {
    outputPanel_->appendBuildOutput(tr("Configuration: %1\n").arg(configFile));
  }
  outputPanel_->appendBuildOutput(tr("Command: %1\n\n").arg(rvizCmd));

  statusBar()->showMessage(tr("RViz2 launched"), 5000);
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
       "Blueprint and Houdini's node-based workflow.</p>"
       "<hr>"
       "<p><b>Developed by:</b> Nigel Hungerford-Symes</p>"
       "<hr>"
       "<p><small><b>Disclaimer:</b> This software is provided \"as is\" without "
       "warranty of any kind. Use at your own risk. The developers are not "
       "responsible for any damage to hardware, software, or robotic systems "
       "that may result from using this application. Always test robot missions "
       "in a safe environment before deployment.</small></p>")
  );
}

void MainWindow::onShowGettingStarted() {
  HelpBrowser* browser = HelpBrowser::instance(this);
  browser->showTopic("getting-started");
  browser->show();
  browser->raise();
  browser->activateWindow();
}

void MainWindow::onShowUserManual() {
  HelpBrowser* browser = HelpBrowser::instance(this);
  browser->show();
  browser->raise();
  browser->activateWindow();
}

void MainWindow::onShowKeyboardShortcuts() {
  KeyboardShortcutsDialog dialog(this);
  dialog.exec();
}

void MainWindow::onShowWhatsNew() {
  HelpBrowser* browser = HelpBrowser::instance(this);
  browser->showTopic("whats-new");
  browser->show();
  browser->raise();
  browser->activateWindow();
}

void MainWindow::onShowGuidedTour() {
  if (!guidedTour_) {
    guidedTour_ = new GuidedTour(this);
  }

  if (!guidedTour_->isRunning()) {
    guidedTour_->start();
  }
}

void MainWindow::onReportIssue() {
  QDesktopServices::openUrl(QUrl("https://github.com/DingoOz/ROS2Weaver/issues/new"));
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

void MainWindow::onRobotWizard() {
  // Create and show the robot configuration wizard
  RobotWizard wizard(this);

  if (wizard.exec() == QDialog::Accepted) {
    // Show success dialog with options
    QString generatedPath = wizard.generatedPackagePath();
    if (!generatedPath.isEmpty()) {
      lastGeneratedPackagePath_ = generatedPath;
      outputPanel_->appendBuildOutput(tr("\nRobot package generated: %1").arg(generatedPath));

      QMessageBox msgBox(this);
      msgBox.setWindowTitle(tr("Robot Package Generated"));
      msgBox.setIcon(QMessageBox::Information);
      msgBox.setText(tr("Robot description package has been generated successfully!"));
      msgBox.setInformativeText(tr("Package location: %1\n\nThe package includes:\n"
                                   "- URDF/Xacro robot description\n"
                                   "- ros2_control configuration\n"
                                   "- Controller configuration\n"
                                   "- Launch files").arg(generatedPath));

      QPushButton* openVSCodeBtn = msgBox.addButton(tr("Open in VS Code"), QMessageBox::ActionRole);
      QPushButton* openFolderBtn = msgBox.addButton(tr("Open Folder"), QMessageBox::ActionRole);
      msgBox.addButton(QMessageBox::Close);

      if (!ExternalEditor::isVSCodeAvailable()) {
        openVSCodeBtn->setEnabled(false);
        openVSCodeBtn->setToolTip(tr("VS Code not found on this system"));
      }

      msgBox.exec();

      if (msgBox.clickedButton() == openVSCodeBtn) {
        externalEditor_->openFolderInVSCode(generatedPath);
      } else if (msgBox.clickedButton() == openFolderBtn) {
        ExternalEditor::openWithSystemDefault(generatedPath);
      }
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
    ErrorHandler::show(ErrorSeverity::Critical, tr("Code Generation Failed"),
      tr("Failed to generate ROS2 package:\n%1").arg(codeGenerator_->lastError()), this);
  }
}

void MainWindow::onOpenProjectInVSCode() {
  if (currentProjectPath_.isEmpty()) {
    ErrorHandler::showInfo(tr("Please save the project first before opening in VS Code."));
    return;
  }

  QFileInfo projectInfo(currentProjectPath_);
  QString projectDir = projectInfo.absolutePath();

  if (!ExternalEditor::isVSCodeAvailable()) {
    ErrorHandler::showWarning(tr("VS Code not found. Please install it and ensure 'code' is in PATH."));
    return;
  }

  if (!externalEditor_->openFolderInVSCode(projectDir)) {
    ErrorHandler::showError(tr("Could not open VS Code: %1").arg(externalEditor_->lastError()));
  }
}

void MainWindow::onOpenGeneratedPackageInVSCode() {
  if (lastGeneratedPackagePath_.isEmpty()) {
    ErrorHandler::showInfo(tr("No package generated yet. Use File → Generate ROS2 Package first."));
    return;
  }

  QFileInfo packageInfo(lastGeneratedPackagePath_);
  if (!packageInfo.exists()) {
    ErrorHandler::showWarning(tr("Generated package folder no longer exists."));
    lastGeneratedPackagePath_.clear();
    return;
  }

  if (!ExternalEditor::isVSCodeAvailable()) {
    ErrorHandler::showWarning(tr("VS Code not found. Please install it and ensure 'code' is in PATH."));
    return;
  }

  if (!externalEditor_->openFolderInVSCode(lastGeneratedPackagePath_)) {
    ErrorHandler::showError(tr("Could not open VS Code: %1").arg(externalEditor_->lastError()));
  }
}

void MainWindow::onOpenBlockInVSCode(PackageBlock* block) {
  if (!block) return;

  QString packageName = block->packageName();

  // First check if VS Code is available
  if (!ExternalEditor::isVSCodeAvailable()) {
    ErrorHandler::showWarning(tr("VS Code not found. Please install it and ensure 'code' is in PATH."));
    return;
  }

  // Try to find the package in ROS2 workspace
  if (packageIndex_) {
    QString packagePath = packageIndex_->findPackagePath(packageName);
    if (!packagePath.isEmpty()) {
      if (!externalEditor_->openFolderInVSCode(packagePath)) {
        ErrorHandler::showError(tr("Could not open VS Code: %1").arg(externalEditor_->lastError()));
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

  QString topicName = connection->topicName();
  if (topicName.isEmpty()) {
    return;
  }

  // Open floating topic echo dialog
  TopicEchoDialog* dialog = new TopicEchoDialog(topicName, this);
  dialog->show();
  dialog->raise();
  dialog->activateWindow();

  // Also log to output panel (existing behavior)
  onEchoTopicRequested(topicName);
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
  dialog.setMinimumSize(500, 550);

  QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

  // Create tab widget for settings pages
  QTabWidget* settingsTabs = new QTabWidget(&dialog);

  // ==================== General Settings Tab ====================
  QWidget* generalTab = new QWidget();
  QVBoxLayout* generalLayout = new QVBoxLayout(generalTab);

  // Appearance group
  QGroupBox* appearanceGroup = new QGroupBox(tr("Appearance"), generalTab);
  QHBoxLayout* appearanceLayout = new QHBoxLayout(appearanceGroup);

  QLabel* themeLabel = new QLabel(tr("Theme:"), appearanceGroup);
  QComboBox* themeCombo = new QComboBox(appearanceGroup);
  themeCombo->addItem(ThemeManager::themeName(Theme::Dark), static_cast<int>(Theme::Dark));
  themeCombo->addItem(ThemeManager::themeName(Theme::Light), static_cast<int>(Theme::Light));

  // Set current theme
  Theme currentTheme = ThemeManager::instance().currentTheme();
  themeCombo->setCurrentIndex(themeCombo->findData(static_cast<int>(currentTheme)));

  appearanceLayout->addWidget(themeLabel);
  appearanceLayout->addWidget(themeCombo);
  appearanceLayout->addStretch();

  generalLayout->addWidget(appearanceGroup);

  // ROS2 Status Display group
  QGroupBox* ros2Group = new QGroupBox(tr("ROS2 Status Display"), generalTab);
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

  generalLayout->addWidget(ros2Group);

  // System Discovery group
  QGroupBox* discoveryGroup = new QGroupBox(tr("System Discovery"), generalTab);
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

  generalLayout->addWidget(discoveryGroup);

  // ROS Logger Colors group
  QGroupBox* logColorGroup = new QGroupBox(tr("ROS Logger Colors"), generalTab);
  QGridLayout* colorLayout = new QGridLayout(logColorGroup);

  // Get current colors from the ROS log viewer
  LogLevelColors currentColors;
  if (outputPanel_ && outputPanel_->rosLogViewer()) {
    currentColors = outputPanel_->rosLogViewer()->logLevelColors();
  }

  // Create color buttons for each log level
  std::map<QString, QPushButton*> colorButtons;
  std::map<QString, QColor> selectedColors;
  QStringList levels = RosLogViewer::logLevels();

  auto createColorButton = [](const QColor& color) -> QPushButton* {
    QPushButton* btn = new QPushButton();
    btn->setFixedSize(60, 24);
    btn->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(color.name()));
    return btn;
  };

  int row = 0;
  for (const QString& level : levels) {
    QColor initialColor;
    if (level == "DEBUG") initialColor = currentColors.debugColor;
    else if (level == "INFO") initialColor = currentColors.infoColor;
    else if (level == "WARN") initialColor = currentColors.warnColor;
    else if (level == "ERROR") initialColor = currentColors.errorColor;
    else if (level == "FATAL") initialColor = currentColors.fatalColor;

    selectedColors[level] = initialColor;

    QLabel* label = new QLabel(level, logColorGroup);
    QPushButton* colorBtn = createColorButton(initialColor);
    colorButtons[level] = colorBtn;

    connect(colorBtn, &QPushButton::clicked, [colorBtn, level, &selectedColors]() {
      QColor newColor = QColorDialog::getColor(selectedColors[level], colorBtn,
        QObject::tr("Select color for %1").arg(level));
      if (newColor.isValid()) {
        selectedColors[level] = newColor;
        colorBtn->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(newColor.name()));
      }
    });

    colorLayout->addWidget(label, row, 0);
    colorLayout->addWidget(colorBtn, row, 1);
    row++;
  }

  // Reset to defaults button
  QPushButton* resetColorsBtn = new QPushButton(tr("Reset to Defaults"), logColorGroup);
  connect(resetColorsBtn, &QPushButton::clicked, [&selectedColors, &colorButtons, &currentColors]() {
    LogLevelColors defaults;
    selectedColors["DEBUG"] = defaults.debugColor;
    selectedColors["INFO"] = defaults.infoColor;
    selectedColors["WARN"] = defaults.warnColor;
    selectedColors["ERROR"] = defaults.errorColor;
    selectedColors["FATAL"] = defaults.fatalColor;

    colorButtons["DEBUG"]->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(defaults.debugColor.name()));
    colorButtons["INFO"]->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(defaults.infoColor.name()));
    colorButtons["WARN"]->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(defaults.warnColor.name()));
    colorButtons["ERROR"]->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(defaults.errorColor.name()));
    colorButtons["FATAL"]->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(defaults.fatalColor.name()));
  });
  colorLayout->addWidget(resetColorsBtn, row, 0, 1, 2);

  generalLayout->addWidget(logColorGroup);

  // ==================== Plot Settings Group ====================
  QGroupBox* plotGroup = new QGroupBox(tr("Plot Settings"), generalTab);
  QVBoxLayout* plotLayout = new QVBoxLayout(plotGroup);

  // Line thickness setting
  QHBoxLayout* thicknessLayout = new QHBoxLayout();
  QLabel* thicknessLabel = new QLabel(tr("Line thickness:"), plotGroup);
  QSpinBox* thicknessSpinBox = new QSpinBox(plotGroup);
  thicknessSpinBox->setRange(1, 10);
  thicknessSpinBox->setValue(plotPanel_ ? plotPanel_->lineThickness() : 2);
  thicknessSpinBox->setToolTip(tr("Thickness of plot lines (1-10 pixels)"));
  thicknessLayout->addWidget(thicknessLabel);
  thicknessLayout->addWidget(thicknessSpinBox);
  thicknessLayout->addStretch();
  plotLayout->addLayout(thicknessLayout);

  // Color palette
  QLabel* colorPaletteLabel = new QLabel(tr("Default plot colors:"), plotGroup);
  plotLayout->addWidget(colorPaletteLabel);

  QGridLayout* plotColorLayout = new QGridLayout();
  QList<QColor> currentPalette = plotPanel_ ? plotPanel_->colorPalette() : QList<QColor>();
  std::map<int, QPushButton*> plotColorButtons;
  std::map<int, QColor> plotSelectedColors;

  auto createPlotColorButton = [](const QColor& color) -> QPushButton* {
    QPushButton* btn = new QPushButton();
    btn->setFixedSize(40, 24);
    btn->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(color.name()));
    return btn;
  };

  // Show first 8 colors (the default palette size)
  int numColors = qMin(currentPalette.size(), 8);
  for (int i = 0; i < numColors; ++i) {
    plotSelectedColors[i] = currentPalette[i];
    QPushButton* colorBtn = createPlotColorButton(currentPalette[i]);
    plotColorButtons[i] = colorBtn;

    connect(colorBtn, &QPushButton::clicked, [colorBtn, i, &plotSelectedColors]() {
      QColor newColor = QColorDialog::getColor(plotSelectedColors[i], colorBtn,
        QObject::tr("Select plot color %1").arg(i + 1));
      if (newColor.isValid()) {
        plotSelectedColors[i] = newColor;
        colorBtn->setStyleSheet(QString("background-color: %1; border: 1px solid #555;").arg(newColor.name()));
      }
    });

    plotColorLayout->addWidget(colorBtn, i / 4, i % 4);
  }

  plotLayout->addLayout(plotColorLayout);

  // Reset plot colors button
  QPushButton* resetPlotColorsBtn = new QPushButton(tr("Reset to Defaults"), plotGroup);
  connect(resetPlotColorsBtn, &QPushButton::clicked, [&plotSelectedColors, &plotColorButtons]() {
    QList<QColor> defaults = {
      QColor(0, 114, 189), QColor(217, 83, 25), QColor(237, 177, 32), QColor(126, 47, 142),
      QColor(119, 172, 48), QColor(77, 190, 238), QColor(162, 20, 47), QColor(0, 128, 128)
    };
    for (int i = 0; i < 8 && i < defaults.size(); ++i) {
      plotSelectedColors[i] = defaults[i];
      if (plotColorButtons.count(i)) {
        plotColorButtons[i]->setStyleSheet(
          QString("background-color: %1; border: 1px solid #555;").arg(defaults[i].name()));
      }
    }
  });
  plotLayout->addWidget(resetPlotColorsBtn);

  generalLayout->addWidget(plotGroup);

  // Add stretch to push remaining space to bottom
  generalLayout->addStretch();

  settingsTabs->addTab(generalTab, tr("General"));

  // ==================== Network Topology Settings Tab ====================
  QWidget* networkTopoTab = new QWidget();
  QVBoxLayout* networkTopoLayout = new QVBoxLayout(networkTopoTab);

  // Auto-refresh group
  QGroupBox* topoAutoRefreshGroup = new QGroupBox(tr("Auto-Refresh"), networkTopoTab);
  QVBoxLayout* topoArLayout = new QVBoxLayout(topoAutoRefreshGroup);

  QCheckBox* topoAutoRefreshCheck = new QCheckBox(tr("Enable auto-refresh"), topoAutoRefreshGroup);
  topoAutoRefreshCheck->setChecked(networkTopologyManager_ ?
      networkTopologyManager_->isAutoRefreshEnabled() : false);
  topoAutoRefreshCheck->setToolTip(tr("Automatically refresh network topology at the specified interval"));
  topoArLayout->addWidget(topoAutoRefreshCheck);

  QHBoxLayout* topoIntervalLayout = new QHBoxLayout();
  QLabel* topoIntervalLabel = new QLabel(tr("Refresh interval (seconds):"), topoAutoRefreshGroup);
  QSpinBox* topoIntervalSpin = new QSpinBox(topoAutoRefreshGroup);
  topoIntervalSpin->setRange(10, 300);
  topoIntervalSpin->setValue(networkTopologyManager_ ?
      networkTopologyManager_->autoRefreshInterval() : 30);
  topoIntervalSpin->setToolTip(tr("Interval between automatic topology scans (10-300 seconds)"));
  topoIntervalLayout->addWidget(topoIntervalLabel);
  topoIntervalLayout->addWidget(topoIntervalSpin);
  topoIntervalLayout->addStretch();
  topoArLayout->addLayout(topoIntervalLayout);

  networkTopoLayout->addWidget(topoAutoRefreshGroup);

  // Display options group
  QGroupBox* topoDisplayGroup = new QGroupBox(tr("Display Options"), networkTopoTab);
  QVBoxLayout* topoDisplayLayout = new QVBoxLayout(topoDisplayGroup);

  QCheckBox* topoShowBandwidthCheck = new QCheckBox(tr("Show bandwidth between hosts"), topoDisplayGroup);
  topoShowBandwidthCheck->setChecked(networkTopologyManager_ ?
      networkTopologyManager_->showBandwidth() : true);
  topoShowBandwidthCheck->setToolTip(tr("Display estimated bandwidth on host connections"));
  topoDisplayLayout->addWidget(topoShowBandwidthCheck);

  QHBoxLayout* topoViewModeLayout = new QHBoxLayout();
  QLabel* topoViewModeLabel = new QLabel(tr("Default view mode:"), topoDisplayGroup);
  QComboBox* topoViewModeCombo = new QComboBox(topoDisplayGroup);
  topoViewModeCombo->addItem(tr("Graph"), "graph");
  topoViewModeCombo->addItem(tr("Table"), "table");
  QString currentViewMode = networkTopologyManager_ ? networkTopologyManager_->viewMode() : "graph";
  topoViewModeCombo->setCurrentIndex(currentViewMode == "table" ? 1 : 0);
  topoViewModeCombo->setToolTip(tr("Default view when opening the Network Topology panel"));
  topoViewModeLayout->addWidget(topoViewModeLabel);
  topoViewModeLayout->addWidget(topoViewModeCombo);
  topoViewModeLayout->addStretch();
  topoDisplayLayout->addLayout(topoViewModeLayout);

  networkTopoLayout->addWidget(topoDisplayGroup);
  networkTopoLayout->addStretch();

  settingsTabs->addTab(networkTopoTab, tr("Network Topology"));

  // ==================== Local LLM Settings Tab ====================
  OllamaSettingsWidget* ollamaSettingsWidget = new OllamaSettingsWidget();
  ollamaSettingsWidget->setLocalAIStatusWidget(localAIStatusWidget_);
  settingsTabs->addTab(ollamaSettingsWidget, tr("Local LLM"));

  // Add tabs to main layout
  mainLayout->addWidget(settingsTabs);

  // Dialog buttons
  QDialogButtonBox* buttonBox = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  mainLayout->addWidget(buttonBox);

  if (dialog.exec() == QDialog::Accepted) {
    // Apply theme setting
    Theme selectedTheme = static_cast<Theme>(themeCombo->currentData().toInt());
    ThemeManager::instance().setTheme(selectedTheme);

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

    // Apply log colors
    if (outputPanel_ && outputPanel_->rosLogViewer()) {
      LogLevelColors newColors;
      newColors.debugColor = selectedColors["DEBUG"];
      newColors.infoColor = selectedColors["INFO"];
      newColors.warnColor = selectedColors["WARN"];
      newColors.errorColor = selectedColors["ERROR"];
      newColors.fatalColor = selectedColors["FATAL"];
      outputPanel_->rosLogViewer()->setLogLevelColors(newColors);
    }

    // Apply plot settings
    if (plotPanel_) {
      plotPanel_->setLineThickness(thicknessSpinBox->value());

      QList<QColor> newPalette;
      for (int i = 0; i < 8; ++i) {
        if (plotSelectedColors.count(i)) {
          newPalette.append(plotSelectedColors[i]);
        }
      }
      if (!newPalette.isEmpty()) {
        plotPanel_->setColorPalette(newPalette);
      }
    }

    // Apply Network Topology settings
    if (networkTopologyManager_) {
      networkTopologyManager_->setAutoRefreshEnabled(topoAutoRefreshCheck->isChecked());
      networkTopologyManager_->setAutoRefreshInterval(topoIntervalSpin->value());
      networkTopologyManager_->setShowBandwidth(topoShowBandwidthCheck->isChecked());
      networkTopologyManager_->setViewMode(topoViewModeCombo->currentData().toString());
    }

    // Apply Ollama/Local LLM settings
    ollamaSettingsWidget->applySettings();
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

void MainWindow::connectRosbagMCPServers() {
  // Connect all existing rosbag MCP servers to the workbench panel
  for (const auto& server : MCPManager::instance().allServers()) {
    if (server->serverType() == "rosbag") {
      connectRosbagMCPServer(std::dynamic_pointer_cast<RosbagMCPServer>(server));
    }
  }
}

void MainWindow::connectRosbagMCPServer(std::shared_ptr<RosbagMCPServer> server) {
  if (!server || !rosbagWorkbenchPanel_) return;

  BagRecorder* recorder = rosbagWorkbenchPanel_->bagRecorder();
  if (!recorder) return;

  // Connect MCP server signals to the bag recorder
  connect(server.get(), &RosbagMCPServer::startRecordingRequested, recorder,
          [recorder](const QString& outputPath, const QStringList& topics, const QString& storageFormat) {
    if (!outputPath.isEmpty()) {
      recorder->setOutputPath(outputPath);
    }
    if (!topics.isEmpty()) {
      recorder->setTopicsToRecord(topics);
      recorder->setRecordAllTopics(false);
    } else {
      recorder->setRecordAllTopics(true);
    }
    recorder->setStorageFormat(storageFormat.isEmpty() ? "sqlite3" : storageFormat);
    recorder->startRecording();
  });

  connect(server.get(), &RosbagMCPServer::stopRecordingRequested, recorder, &BagRecorder::stopRecording);
  connect(server.get(), &RosbagMCPServer::pauseRecordingRequested, recorder, &BagRecorder::pauseRecording);
  connect(server.get(), &RosbagMCPServer::resumeRecordingRequested, recorder, &BagRecorder::resumeRecording);

  // Connect recorder signals back to MCP server to keep state in sync
  connect(recorder, &BagRecorder::recordingStarted, server.get(),
          [server](const QString& path) {
    server->setRecordingState(true, path);
  });

  connect(recorder, &BagRecorder::recordingStopped, server.get(),
          [server]() {
    server->setRecordingState(false);
  });

  connect(recorder, &BagRecorder::recordingPaused, server.get(),
          [server]() {
    server->setPausedState(true);
  });

  connect(recorder, &BagRecorder::recordingResumed, server.get(),
          [server]() {
    server->setPausedState(false);
  });

  // Set the default output path from the current recorder setting
  if (!recorder->outputPath().isEmpty()) {
    server->setDefaultOutputPath(recorder->outputPath());
  }
}

// =============================================================================
// Project Dirty Tracking and Auto-Save
// =============================================================================

void MainWindow::onProjectModified() {
  setProjectDirty(true);
}

void MainWindow::setProjectDirty(bool dirty) {
  if (isProjectDirty_ != dirty) {
    isProjectDirty_ = dirty;
    updateWindowTitle();
  }
}

void MainWindow::updateWindowTitle() {
  QString title = baseWindowTitle_;

  // Add project name if available
  if (!currentProjectPath_.isEmpty()) {
    QFileInfo info(currentProjectPath_);
    title = info.baseName() + " - " + baseWindowTitle_;
  }

  // Add dirty indicator
  if (isProjectDirty_) {
    title = "* " + title;
  }

  // Add ROS status suffix if configured
  if (rosStatusWidget_) {
    QString suffix = rosStatusWidget_->titleBarSuffix();
    if (!suffix.isEmpty()) {
      title += suffix;
    }
  }

  setWindowTitle(title);
}

bool MainWindow::maybeSave() {
  if (!isProjectDirty_) {
    return true;
  }

  QMessageBox::StandardButton ret = QMessageBox::warning(
      this,
      tr("Unsaved Changes"),
      tr("The project has been modified.\nDo you want to save your changes?"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Save);

  switch (ret) {
    case QMessageBox::Save:
      onSaveProject();
      return !isProjectDirty_;  // Return true only if save succeeded
    case QMessageBox::Discard:
      return true;
    case QMessageBox::Cancel:
    default:
      return false;
  }
}

void MainWindow::onAutoSave() {
  if (!isProjectDirty_ || currentProjectPath_.isEmpty()) {
    return;
  }

  // Save to auto-save file
  QString autoSavePath = currentProjectPath_ + ".autosave";
  if (saveProject(autoSavePath)) {
    Toast.showInfo(tr("Auto-saved"), 2000);
  }
}

void MainWindow::closeEvent(QCloseEvent* event) {
  if (maybeSave()) {
    // Clean up auto-save file
    if (!currentProjectPath_.isEmpty()) {
      QString autoSavePath = currentProjectPath_ + ".autosave";
      QFile::remove(autoSavePath);
    }
    event->accept();
    QApplication::quit();  // Force the event loop to exit
  } else {
    event->ignore();
  }
}

// =============================================================================
// Recent Projects
// =============================================================================

void MainWindow::addToRecentProjects(const QString& path) {
  QSettings settings("ROS Weaver", "ROS Weaver");
  QStringList recent = settings.value("RecentProjects").toStringList();

  // Remove if already exists, add to front
  recent.removeAll(path);
  recent.prepend(path);

  // Limit to max
  while (recent.size() > MAX_RECENT_PROJECTS) {
    recent.removeLast();
  }

  settings.setValue("RecentProjects", recent);
  updateRecentProjectsMenu();
}

QStringList MainWindow::getRecentProjects() const {
  QSettings settings("ROS Weaver", "ROS Weaver");
  return settings.value("RecentProjects").toStringList();
}

void MainWindow::updateRecentProjectsMenu() {
  if (!recentProjectsMenu_) return;

  recentProjectsMenu_->clear();

  QStringList recent = getRecentProjects();
  if (recent.isEmpty()) {
    QAction* emptyAction = recentProjectsMenu_->addAction(tr("(No recent projects)"));
    emptyAction->setEnabled(false);
  } else {
    for (int i = 0; i < recent.size(); ++i) {
      QString path = recent[i];
      QFileInfo info(path);
      QString label = QString("%1. %2").arg(i + 1).arg(info.fileName());

      QAction* action = recentProjectsMenu_->addAction(label);
      action->setData(path);
      action->setToolTip(path);
      connect(action, &QAction::triggered, this, &MainWindow::onOpenRecentProject);
    }

    recentProjectsMenu_->addSeparator();
    QAction* clearAction = recentProjectsMenu_->addAction(tr("Clear Recent Projects"));
    connect(clearAction, &QAction::triggered, this, &MainWindow::onClearRecentProjects);
  }
}

void MainWindow::onOpenRecentProject() {
  QAction* action = qobject_cast<QAction*>(sender());
  if (!action) return;

  QString path = action->data().toString();
  if (!path.isEmpty() && QFile::exists(path)) {
    if (maybeSave()) {
      loadProject(path);
    }
  } else {
    Toast.showError(tr("Project file not found: %1").arg(path));
    // Remove from recent projects
    QSettings settings("ROS Weaver", "ROS Weaver");
    QStringList recent = settings.value("RecentProjects").toStringList();
    recent.removeAll(path);
    settings.setValue("RecentProjects", recent);
    updateRecentProjectsMenu();
  }
}

void MainWindow::onClearRecentProjects() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.remove("RecentProjects");
  updateRecentProjectsMenu();
  Toast.showInfo(tr("Recent projects cleared"));
}

// =============================================================================
// Command Palette
// =============================================================================

void MainWindow::setupCommandPalette() {
  commandPalette_ = new CommandPalette(this);
  registerCommands();

  // Connect command executed signal for any custom handling
  connect(commandPalette_, &CommandPalette::commandExecuted,
          this, [this](const QString& id) {
    Q_UNUSED(id)
    // Could log command usage or track analytics here
  });
}

void MainWindow::registerCommands() {
  if (!commandPalette_) return;

  // File commands
  commandPalette_->addCommand("file.new", tr("New Project"), tr("File"),
                              findChild<QAction*>("newAction"));
  commandPalette_->addCommand("file.open", tr("Open Project"), tr("File"),
                              findChild<QAction*>("openAction"));
  commandPalette_->addCommand("file.save", tr("Save Project"), tr("File"),
                              findChild<QAction*>("saveAction"));
  commandPalette_->addCommand("file.saveAs", tr("Save Project As"), tr("File"),
                              findChild<QAction*>("saveAsAction"));
  commandPalette_->addCommand("file.generate", tr("Generate ROS2 Package"), tr("File"),
                              nullptr, tr("Generate code from current project"));

  // Edit commands
  commandPalette_->addCommand("edit.undo", tr("Undo"), tr("Edit"), undoAction_);
  commandPalette_->addCommand("edit.redo", tr("Redo"), tr("Edit"), redoAction_);

  // View commands
  commandPalette_->addCommand("view.zoomIn", tr("Zoom In"), tr("View"), nullptr);
  commandPalette_->addCommand("view.zoomOut", tr("Zoom Out"), tr("View"), nullptr);
  commandPalette_->addCommand("view.resetZoom", tr("Reset Zoom"), tr("View"), nullptr);
  commandPalette_->addCommand("view.fitAll", tr("Fit All Nodes"), tr("View"), nullptr);

  // ROS2 commands
  commandPalette_->addCommand("ros2.scan", tr("Scan Running System"), tr("ROS2"),
                              scanSystemAction_);
  commandPalette_->addCommand("ros2.build", tr("Build Workspace"), tr("ROS2"), nullptr);
  commandPalette_->addCommand("ros2.rviz", tr("Launch RViz2"), tr("ROS2"), nullptr);

  // Panels
  commandPalette_->addCommand("panel.output", tr("Show Output Panel"), tr("Panels"), nullptr);
  commandPalette_->addCommand("panel.topics", tr("Show Topic Viewer"), tr("Panels"), nullptr);
  commandPalette_->addCommand("panel.tf", tr("Show TF Tree"), tr("Panels"), nullptr);
  commandPalette_->addCommand("panel.params", tr("Show Parameters"), tr("Panels"), nullptr);
  commandPalette_->addCommand("panel.logs", tr("Show ROS Logs"), tr("Panels"), nullptr);

  // Help
  commandPalette_->addCommand("help.shortcuts", tr("Keyboard Shortcuts"), tr("Help"), nullptr);
  commandPalette_->addCommand("help.tour", tr("Guided Tour"), tr("Help"), nullptr);
  commandPalette_->addCommand("help.manual", tr("User Manual"), tr("Help"), nullptr);
}

void MainWindow::onShowCommandPalette() {
  if (commandPalette_) {
    commandPalette_->showPalette();
  }
}

// =============================================================================
// Panel Management
// =============================================================================

void MainWindow::onDockAllPanels() {
  int dockedCount = 0;

  // Find all dock widgets and dock any that are floating
  QList<QDockWidget*> dockWidgets = findChildren<QDockWidget*>();

  for (QDockWidget* dock : dockWidgets) {
    if (dock && dock->isFloating() && dock->isVisible()) {
      // Dock it back - Qt remembers the last docked position
      dock->setFloating(false);
      dockedCount++;
    }
  }

  if (dockedCount > 0) {
    statusBar()->showMessage(tr("Docked %1 floating panel(s)").arg(dockedCount), 3000);
  } else {
    statusBar()->showMessage(tr("No floating panels to dock"), 2000);
  }
}

// =============================================================================
// Layout Presets
// =============================================================================

void MainWindow::onSaveLayoutPreset() {
  bool ok;
  QString name = QInputDialog::getText(this, tr("Save Layout Preset"),
                                        tr("Preset name:"), QLineEdit::Normal,
                                        tr("My Layout"), &ok);
  if (ok && !name.isEmpty()) {
    saveLayoutState(name);
    Toast.showSuccess(tr("Layout saved: %1").arg(name));
  }
}

void MainWindow::onLoadLayoutPreset(const QString& name) {
  restoreLayoutState(name);
  Toast.showInfo(tr("Layout loaded: %1").arg(name));
}

void MainWindow::saveLayoutState(const QString& name) {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("LayoutPresets");
  settings.beginGroup(name);

  settings.setValue("geometry", saveGeometry());
  settings.setValue("state", saveState());

  settings.endGroup();
  settings.endGroup();

  // Update presets list
  if (!layoutPresetNames_.contains(name)) {
    layoutPresetNames_.append(name);
    settings.setValue("LayoutPresets/names", layoutPresetNames_);
  }
}

void MainWindow::restoreLayoutState(const QString& name) {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("LayoutPresets");
  settings.beginGroup(name);

  QByteArray geometry = settings.value("geometry").toByteArray();
  QByteArray state = settings.value("state").toByteArray();

  if (!geometry.isEmpty()) {
    restoreGeometry(geometry);
  }
  if (!state.isEmpty()) {
    restoreState(state);
  }

  settings.endGroup();
  settings.endGroup();
}

// Static Analysis slots
void MainWindow::onRunAnalysis() {
  if (!canvas_) return;

  // Get the current project from canvas
  Project project;
  canvas_->exportToProject(project);

  // Show the issues panel
  if (issueListDock_) {
    issueListDock_->show();
    issueListDock_->raise();
  }

  // Run analysis
  statusBar()->showMessage(tr("Running static analysis..."), 0);
  AnalysisResult result = StaticAnalyzer::instance().analyze(project);

  // Update the issue list panel
  if (issueListPanel_) {
    issueListPanel_->setAnalysisResult(result);
  }

  statusBar()->showMessage(tr("Analysis complete: %1 errors, %2 warnings")
      .arg(result.errorCount).arg(result.warningCount), 5000);
}

void MainWindow::onAnalysisCompleted(const AnalysisResult& result) {
  if (issueListPanel_) {
    issueListPanel_->setAnalysisResult(result);
  }
}

void MainWindow::onNavigateToIssue(const QUuid& blockId) {
  if (!canvas_) return;

  // Find the block and center on it
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    PackageBlock* block = dynamic_cast<PackageBlock*>(item);
    if (block && block->id() == blockId) {
      canvas_->centerOn(block);
      block->setBlockSelected(true);
      return;
    }
  }
}

// Theme Editor slot
void MainWindow::onShowThemeEditor() {
  ThemeEditorDialog dialog(this);
  connect(&dialog, &ThemeEditorDialog::themeApplied, this,
          [this](const QString& themeName) {
    // Map theme name to Theme enum
    Theme theme = Theme::Dark;  // Default
    if (themeName.toLower() == "light") {
      theme = Theme::Light;
    } else if (themeName.toLower() == "dark") {
      theme = Theme::Dark;
    } else if (themeName.toLower().contains("high") && themeName.toLower().contains("contrast")) {
      theme = Theme::HighContrast;
    }
    ThemeManager::instance().setTheme(theme);
    statusBar()->showMessage(tr("Theme '%1' applied").arg(themeName), 3000);
  });
  dialog.exec();
}

// Simulation Launcher slots
void MainWindow::onLaunchGazebo() {
  SimulationLauncher& launcher = SimulationLauncher::instance();

  if (!launcher.isGazeboAvailable()) {
    QMessageBox::warning(this, tr("Gazebo Not Found"),
        tr("Gazebo Classic is not installed or not in PATH.\n"
           "Please install it with: sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs"));
    return;
  }

  statusBar()->showMessage(tr("Launching Gazebo..."), 0);
  if (!launcher.launchEmptyWorld(SimulatorType::Gazebo)) {
    QMessageBox::warning(this, tr("Launch Failed"),
        tr("Failed to launch Gazebo. Check the output for details."));
  }
}

void MainWindow::onLaunchIgnition() {
  SimulationLauncher& launcher = SimulationLauncher::instance();

  if (!launcher.isIgnitionAvailable()) {
    QMessageBox::warning(this, tr("Ignition/Gazebo Sim Not Found"),
        tr("Ignition Gazebo (Gazebo Sim) is not installed or not in PATH.\n"
           "Please install it with: sudo apt install ros-$ROS_DISTRO-ros-gz"));
    return;
  }

  statusBar()->showMessage(tr("Launching Ignition Gazebo..."), 0);
  if (!launcher.launchEmptyWorld(SimulatorType::Ignition)) {
    QMessageBox::warning(this, tr("Launch Failed"),
        tr("Failed to launch Ignition Gazebo. Check the output for details."));
  }
}

void MainWindow::onStopSimulation() {
  SimulationLauncher& launcher = SimulationLauncher::instance();

  if (!launcher.isRunning()) {
    statusBar()->showMessage(tr("No simulation is running"), 3000);
    return;
  }

  launcher.stop();
}

// Remapping Editor slot
void MainWindow::onEditRemappings(PackageBlock* block) {
  if (!block) return;

  // Create a dialog containing the remapping editor
  QDialog dialog(this);
  dialog.setWindowTitle(tr("Edit Remappings - %1").arg(block->packageName()));
  dialog.setMinimumSize(500, 400);

  QVBoxLayout* layout = new QVBoxLayout(&dialog);

  RemappingEditor* editor = new RemappingEditor(&dialog);
  editor->setUndoStack(undoStack_);
  editor->setCanvas(canvas_);
  editor->setBlock(block);
  layout->addWidget(editor);

  QDialogButtonBox* buttons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addWidget(buttons);

  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() == QDialog::Accepted) {
    editor->applyChanges();
    setProjectDirty(true);
    statusBar()->showMessage(tr("Remappings updated for '%1'").arg(block->packageName()), 3000);
  }
}

// Canvas tab management slots
void MainWindow::onNewCanvas() {
  if (!canvasTabWidget_) return;

  WeaverCanvas* newCanvas = canvasTabWidget_->addCanvas();
  if (newCanvas) {
    canvas_ = newCanvas;
    setProjectDirty(true);
    statusBar()->showMessage(tr("New canvas created"), 3000);
  }
}

void MainWindow::onCloseCanvas() {
  if (!canvasTabWidget_) return;

  if (canvasTabWidget_->canvasCount() <= 1) {
    QMessageBox::information(this, tr("Cannot Close"),
        tr("Cannot close the last canvas. At least one canvas must remain."));
    return;
  }

  canvasTabWidget_->closeCurrentCanvas();
  canvas_ = canvasTabWidget_->currentCanvas();
  setProjectDirty(true);
}

void MainWindow::onDuplicateCanvas() {
  if (!canvasTabWidget_) return;

  canvasTabWidget_->duplicateCurrentCanvas();
  canvas_ = canvasTabWidget_->currentCanvas();
  setProjectDirty(true);
  statusBar()->showMessage(tr("Canvas duplicated"), 3000);
}

void MainWindow::onRenameCanvas() {
  if (!canvasTabWidget_) return;

  int index = canvasTabWidget_->currentIndex();
  QString currentName = canvasTabWidget_->tabName(index);

  bool ok;
  QString newName = QInputDialog::getText(this, tr("Rename Canvas"),
      tr("Canvas name:"), QLineEdit::Normal, currentName, &ok);

  if (ok && !newName.isEmpty() && newName != currentName) {
    canvasTabWidget_->renameTab(index, newName);
    setProjectDirty(true);
    statusBar()->showMessage(tr("Canvas renamed to '%1'").arg(newName), 3000);
  }
}

void MainWindow::onCurrentCanvasChanged(WeaverCanvas* canvas) {
  canvas_ = canvas;

  // Update param dashboard - clear selection and update canvas reference when switching canvases
  if (paramDashboard_) {
    paramDashboard_->setCanvas(canvas);
    paramDashboard_->setCurrentBlock(nullptr);
  }

  // Update status bar
  if (canvasTabWidget_) {
    int index = canvasTabWidget_->currentIndex();
    QString name = canvasTabWidget_->tabName(index);
    statusBar()->showMessage(tr("Switched to canvas: %1").arg(name), 2000);
  }
}

void MainWindow::onShowRemoteConnectionDialog() {
  RemoteConnectionDialog dialog(remoteConnectionManager_, this);

  connect(&dialog, &RemoteConnectionDialog::connectionRequested,
          this, [this](const RobotProfile& profile) {
    statusBar()->showMessage(tr("Connecting to %1...").arg(profile.name));
    remoteConnectionManager_->connectToRobot(profile);
  });

  dialog.exec();
}

void MainWindow::onRemoteConnectionEstablished(const QString& robotName) {
  connectRemoteAction_->setEnabled(false);
  disconnectRemoteAction_->setEnabled(true);

  // Update window title
  setWindowTitle(QString("%1 - Connected to %2").arg(baseWindowTitle_, robotName));

  statusBar()->showMessage(tr("Connected to %1").arg(robotName), 5000);

  // Show toast notification
  Toast.showSuccess(tr("Connected to %1").arg(robotName));
}

void MainWindow::onRemoteConnectionFailed(const QString& error) {
  connectRemoteAction_->setEnabled(true);
  disconnectRemoteAction_->setEnabled(false);

  statusBar()->showMessage(tr("Connection failed: %1").arg(error), 5000);

  QMessageBox::warning(this, tr("Connection Failed"),
                       tr("Failed to connect to remote robot:\n%1").arg(error));
}

void MainWindow::onRemoteConnectionLost() {
  connectRemoteAction_->setEnabled(true);
  disconnectRemoteAction_->setEnabled(false);

  // Reset window title
  updateWindowTitle();

  statusBar()->showMessage(tr("Connection lost"), 5000);

  Toast.showWarning(tr("Connection to remote robot lost"));
}

void MainWindow::onDisconnectRemote() {
  if (remoteConnectionManager_->isConnected()) {
    QString robotName = remoteConnectionManager_->connectedRobotName();
    remoteConnectionManager_->disconnect();

    connectRemoteAction_->setEnabled(true);
    disconnectRemoteAction_->setEnabled(false);

    // Reset window title
    updateWindowTitle();

    statusBar()->showMessage(tr("Disconnected from %1").arg(robotName), 3000);
  }
}

// Latency Heatmap slots

void MainWindow::onToggleLatencyHeatmap(bool enabled) {
  if (enabled) {
    statusBar()->showMessage(tr("Latency heatmap enabled"), 3000);
  } else {
    statusBar()->showMessage(tr("Latency heatmap disabled"), 3000);
  }
}

void MainWindow::onLatencyAlert(const QString& connectionId, double latencyMs, double thresholdMs) {
  Q_UNUSED(connectionId)
  Q_UNUSED(thresholdMs)

  Toast.showWarning(tr("High latency detected: %1 ms").arg(latencyMs, 0, 'f', 1));
}

// Scenario Editor slots

void MainWindow::onScenarioLoaded(const QString& name) {
  statusBar()->showMessage(tr("Loaded scenario: %1").arg(name), 3000);
}

void MainWindow::onScenarioCompleted(bool success) {
  if (success) {
    Toast.showSuccess(tr("Scenario completed successfully"));
  } else {
    Toast.showError(tr("Scenario failed"));
  }
}

}  // namespace ros_weaver
