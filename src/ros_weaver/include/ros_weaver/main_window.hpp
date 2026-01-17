#ifndef ROS_WEAVER_MAIN_WINDOW_HPP
#define ROS_WEAVER_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QDockWidget>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTabWidget>
#include <QLineEdit>
#include <QProgressBar>
#include <QTextEdit>
#include <QAction>
#include <QTimer>
#include <QLabel>
#include <memory>

#include "ros_weaver/core/system_discovery.hpp"
#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/core/undo/undo_stack.hpp"

namespace ros_weaver {

class WeaverCanvas;
class Project;
class ParamDashboard;
class RosPackageIndex;
class CodeGenerator;
class PackageBlock;
class ExternalEditor;
class PackageWizard;
class RobotWizard;
class OutputPanel;
class TopicMonitor;
class TopicInspectorPopup;
class ConnectionLine;
class RosStatusWidget;
class LocalAIStatusWidget;
class SystemDiscovery;
class CanvasMapper;
class SystemMappingPanel;
class TopicViewerPanel;
class TFTreePanel;
class PlotPanel;
class GuidedTour;
class RosbagWorkbenchPanel;
class MCPExplorerPanel;
class RosControlPanel;
class RosbagMCPServer;
class CommandPalette;
class SchemaViewerWidget;
class ReadmePreviewPanel;
class IssueListPanel;
class AdvancedSearchPanel;
class ThemeEditorDialog;
class RemappingEditor;
class CanvasTabWidget;
class MinimapPanel;
class NodeTemplatesPanel;
class WorkspaceBrowserPanel;
class NodeHealthDashboard;
class MessageInspectorPanel;
class LifecyclePanel;
class RemoteConnectionManager;
class ScenarioEditorWidget;
class LatencyHeatmapPanel;
class LatencyTracker;
class DiagnosticsPanel;
class NetworkTopologyManager;
class NetworkTopologyPanel;
class BehaviorTreePanel;
class MissionPlannerPanel;
#ifdef HAVE_QT3D
class URDFViewerPanel;
#endif
class DockDragFilter;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

private slots:
  void onNewProject();
  void onOpenProject();
  void onSaveProject();
  void onSaveProjectAs();
  void onExportToPlantUML();
  void onExportToMermaid();
  void onExportToGraphviz();
  void onGenerateDocumentation();
  void onImportFromDot();
  void onImportFromCaret();
  void onGenerateCode();
  void onGenerateCodeWizard();
  void onRobotWizard();
  void onExit();
  void onAbout();

  // Help menu
  void onShowGettingStarted();
  void onShowUserManual();
  void onShowKeyboardShortcuts();
  void onShowWhatsNew();
  void onShowGuidedTour();
  void onReportIssue();

  // Example projects
  void onLoadTurtleBotExample();
  void onLoadTurtlesimExample();
  void onLoadBehaviorTreeExample();
  void onLaunchTurtlesim();
  void onLaunchTurtleBot3Gazebo();

  // External tools
  void onLaunchRviz2();

  // Package search
  void onSearchPackages();
  void onPackageSearchResults(const QList<struct RosPackageInfo>& packages);
  void onPackageItemDoubleClicked(QTreeWidgetItem* item, int column);

  // Block selection
  void onBlockSelected(PackageBlock* block);

  // Code generation progress
  void onGenerationProgress(int percent, const QString& message);
  void onGenerationFinished(bool success);

  // VS Code integration
  void onOpenProjectInVSCode();
  void onOpenGeneratedPackageInVSCode();
  void onOpenBlockInVSCode(PackageBlock* block);

  // Live topic monitoring
  void onToggleLiveMonitoring(bool enabled);
  void onConnectionClicked(ConnectionLine* connection);
  void onConnectionDoubleClicked(ConnectionLine* connection);
  void onTopicActivity(const QString& topicName, double rate);
  void onEchoTopicRequested(const QString& topicName);

  // Settings
  void onOpenSettings();
  void onRosStatusTitleBarUpdate(const QString& suffix);

  // System discovery
  void onScanSystem();
  void onToggleAutoScan(bool enabled);
  void onScanStarted();
  void onScanProgress(int percent, const QString& message);
  void onScanCompleted(const SystemGraph& graph);
  void onScanTimedOut();
  void onMappingCompleted(const MappingResults& results);
  void onMappingBlockSelected(const QUuid& blockId);

  // Topic viewer
  void onTopicViewerTopicSelected(const QString& topicName);
  void onShowTopicOnCanvas(const QString& topicName);

  // Project dirty tracking
  void onProjectModified();
  void onAutoSave();

  // Command palette
  void onShowCommandPalette();

  // Layout presets
  void onSaveLayoutPreset();
  void onLoadLayoutPreset(const QString& name);

  // Panel management
  void onDockAllPanels();

  // Recent projects
  void onOpenRecentProject();
  void onClearRecentProjects();

  // Static analysis
  void onRunAnalysis();
  void onAnalysisCompleted(const struct AnalysisResult& result);
  void onNavigateToIssue(const QUuid& blockId);

  // Theme editor
  void onShowThemeEditor();

  // Simulation launcher
  void onLaunchGazebo();
  void onLaunchIgnition();
  void onStopSimulation();

  // Remapping editor
  void onEditRemappings(PackageBlock* block);

  // Canvas tab management
  void onNewCanvas();
  void onCloseCanvas();
  void onDuplicateCanvas();
  void onRenameCanvas();
  void onCurrentCanvasChanged(WeaverCanvas* canvas);

  // Remote connection
  void onShowRemoteConnectionDialog();
  void onRemoteConnectionEstablished(const QString& robotName);
  void onRemoteConnectionFailed(const QString& error);
  void onRemoteConnectionLost();
  void onDisconnectRemote();

  // Latency heatmap
  void onToggleLatencyHeatmap(bool enabled);
  void onLatencyAlert(const QString& connectionId, double latencyMs, double thresholdMs);

  // Scenario editor
  void onScenarioLoaded(const QString& name);
  void onScenarioCompleted(bool success);

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  void setupMenuBar();
  void setupToolBar();
  void setupDockWidgets();
  void setupCentralWidget();
  void setupStatusBar();
  void setupPackageSearch();

  bool saveProject(const QString& filePath);
  bool loadProject(const QString& filePath);
  void loadProjectYamlFiles(const QString& projectDir);

  // Dirty tracking and auto-save
  void setProjectDirty(bool dirty);
  void updateWindowTitle();
  bool maybeSave();  // Returns false if user cancels

  // Recent projects management
  void addToRecentProjects(const QString& path);
  void updateRecentProjectsMenu();
  QStringList getRecentProjects() const;

  // Command palette setup
  void setupCommandPalette();
  void registerCommands();

  // Layout management
  void saveLayoutState(const QString& name);
  void restoreLayoutState(const QString& name);

  // MCP Rosbag integration
  void connectRosbagMCPServers();
  void connectRosbagMCPServer(std::shared_ptr<RosbagMCPServer> server);

  // Multi-canvas tab widget
  CanvasTabWidget* canvasTabWidget_;

  // Convenience pointer to current canvas (updated when tab changes)
  WeaverCanvas* canvas_;

  // Dock widgets
  QDockWidget* packageBrowserDock_;
  QDockWidget* propertiesDock_;
  QDockWidget* outputDock_;

  // Package browser
  QTreeWidget* packageTree_;
  QLineEdit* packageSearchEdit_;
  QTreeWidgetItem* searchResultsItem_;
  QTreeWidgetItem* localPackagesItem_;

  // Browser panel tabs (left side)
  QTabWidget* browserTab_;

  // Properties panel with param dashboard
  QTabWidget* propertiesTab_;
  ParamDashboard* paramDashboard_;

  // Output panel with tabs (Build, ROS Logs, Terminal)
  OutputPanel* outputPanel_;

  // Core components
  RosPackageIndex* packageIndex_;
  CodeGenerator* codeGenerator_;
  ExternalEditor* externalEditor_;

  // Live topic monitoring
  TopicMonitor* topicMonitor_;
  TopicInspectorPopup* topicInspector_;
  QAction* liveMonitoringAction_;
  bool liveMonitoringEnabled_;

  // Current project state
  QString currentProjectPath_;
  QString lastGeneratedPackagePath_;  // Path to last generated package

  // ROS2 status display
  RosStatusWidget* rosStatusWidget_;
  LocalAIStatusWidget* localAIStatusWidget_;
  QString baseWindowTitle_;

  // System discovery
  SystemDiscovery* systemDiscovery_;
  CanvasMapper* canvasMapper_;
  SystemMappingPanel* systemMappingPanel_;
  QAction* scanSystemAction_;
  QAction* autoScanAction_;
  QProgressBar* scanProgressBar_;

  // Topic viewer
  TopicViewerPanel* topicViewerPanel_;
  QDockWidget* topicViewerDock_;

  // TF Tree viewer
  TFTreePanel* tfTreePanel_;

  // Plot panel
  PlotPanel* plotPanel_;

  // Guided tour
  GuidedTour* guidedTour_;

  // Undo/Redo system
  UndoStack* undoStack_;
  QAction* undoAction_;
  QAction* redoAction_;

  // Rosbag Workbench
  RosbagWorkbenchPanel* rosbagWorkbenchPanel_;
  QDockWidget* rosbagWorkbenchDock_;

  // MCP Explorer
  MCPExplorerPanel* mcpExplorerPanel_;
  QDockWidget* mcpExplorerDock_;

  // ROS Control Panel
  RosControlPanel* rosControlPanel_;

  // Schema Viewer
  SchemaViewerWidget* schemaViewerWidget_;
  QDockWidget* schemaViewerDock_;

  // README Preview Panel
  ReadmePreviewPanel* readmePreviewPanel_;
  QDockWidget* readmePreviewDock_;

  // Issue List Panel (static analysis)
  IssueListPanel* issueListPanel_;
  QDockWidget* issueListDock_;

  // Advanced Search Panel
  AdvancedSearchPanel* advancedSearchPanel_;

  // Project dirty tracking
  bool isProjectDirty_;

  // Auto-save functionality
  QTimer* autoSaveTimer_;
  bool autoSaveEnabled_;
  int autoSaveIntervalMs_;

  // Recent projects
  QMenu* recentProjectsMenu_;
  static constexpr int MAX_RECENT_PROJECTS = 10;

  // Command palette
  CommandPalette* commandPalette_;

  // Layout presets
  QMenu* layoutPresetsMenu_;
  QStringList layoutPresetNames_;

  // Zoom indicator
  QLabel* zoomIndicator_;

  // Minimap navigation
  MinimapPanel* minimapPanel_;
  QDockWidget* minimapDock_;

  // Node templates panel
  NodeTemplatesPanel* nodeTemplatesPanel_;
  QDockWidget* nodeTemplatesDock_;

  // Workspace browser panel
  WorkspaceBrowserPanel* workspaceBrowserPanel_;
  QDockWidget* workspaceBrowserDock_;

  // Node health dashboard
  NodeHealthDashboard* nodeHealthDashboard_;
  QDockWidget* nodeHealthDock_;

  // Message inspector panel
  MessageInspectorPanel* messageInspectorPanel_;
  QDockWidget* messageInspectorDock_;

  // Lifecycle panel
  LifecyclePanel* lifecyclePanel_;
  QDockWidget* lifecycleDock_;

  // Remote connection
  RemoteConnectionManager* remoteConnectionManager_;
  QAction* connectRemoteAction_;
  QAction* disconnectRemoteAction_;

  // Latency heatmap and scenario editor
  LatencyTracker* latencyTracker_;
  LatencyHeatmapPanel* latencyHeatmapPanel_;
  QDockWidget* latencyHeatmapDock_;
  ScenarioEditorWidget* scenarioEditorWidget_;
  QDockWidget* scenarioEditorDock_;
  QAction* latencyHeatmapAction_;

  // Diagnostics panel (ros2 doctor)
  DiagnosticsPanel* diagnosticsPanel_;
  QDockWidget* diagnosticsDock_;

  // Network topology panel (DDS network view)
  NetworkTopologyManager* networkTopologyManager_;
  NetworkTopologyPanel* networkTopologyPanel_;
  QDockWidget* networkTopologyDock_;

  // Behavior tree panel
  BehaviorTreePanel* behaviorTreePanel_;
  QDockWidget* behaviorTreeDock_;

  // Dock drag filter for Ctrl+drag docking
  DockDragFilter* dockDragFilter_;

  // Mission planner panel
  MissionPlannerPanel* missionPlannerPanel_;
  QDockWidget* missionPlannerDock_;

  // URDF viewer panel (only when Qt3D is available)
#ifdef HAVE_QT3D
  URDFViewerPanel* urdfViewerPanel_;
  QDockWidget* urdfViewerDock_;
#endif
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MAIN_WINDOW_HPP
