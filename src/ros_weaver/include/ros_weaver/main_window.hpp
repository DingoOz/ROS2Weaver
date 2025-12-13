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

#include "ros_weaver/core/system_discovery.hpp"
#include "ros_weaver/core/canvas_mapper.hpp"

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

  // Central canvas for visual editing
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
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MAIN_WINDOW_HPP
