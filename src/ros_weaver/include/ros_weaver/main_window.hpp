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

namespace ros_weaver {

class WeaverCanvas;
class Project;
class ParamDashboard;
class RosPackageIndex;
class CodeGenerator;
class PackageBlock;

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
  void onExit();
  void onAbout();

  // Example projects
  void onLoadTurtleBotExample();

  // Package search
  void onSearchPackages();
  void onPackageSearchResults(const QList<struct RosPackageInfo>& packages);
  void onPackageItemDoubleClicked(QTreeWidgetItem* item, int column);

  // Block selection
  void onBlockSelected(PackageBlock* block);

  // Code generation progress
  void onGenerationProgress(int percent, const QString& message);
  void onGenerationFinished(bool success);

private:
  void setupMenuBar();
  void setupToolBar();
  void setupDockWidgets();
  void setupCentralWidget();
  void setupStatusBar();
  void setupPackageSearch();

  bool saveProject(const QString& filePath);
  bool loadProject(const QString& filePath);

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

  // Properties panel with param dashboard
  QTabWidget* propertiesTab_;
  ParamDashboard* paramDashboard_;

  // Output text
  QTextEdit* outputText_;
  QProgressBar* progressBar_;

  // Core components
  RosPackageIndex* packageIndex_;
  CodeGenerator* codeGenerator_;

  // Current project state
  QString currentProjectPath_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MAIN_WINDOW_HPP
