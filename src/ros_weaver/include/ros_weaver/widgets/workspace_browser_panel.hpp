#ifndef ROS_WEAVER_WIDGETS_WORKSPACE_BROWSER_PANEL_HPP
#define ROS_WEAVER_WIDGETS_WORKSPACE_BROWSER_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QFileSystemWatcher>
#include <QDir>
#include <QMap>
#include <QSet>

namespace ros_weaver {

/**
 * @brief Information about a ROS2 package in the workspace
 */
struct WorkspacePackageInfo {
  QString name;
  QString path;
  QString version;
  QString description;
  QString maintainer;
  QString license;
  QStringList buildDepends;
  QStringList execDepends;
  QStringList launchFiles;
  QStringList configFiles;
  QStringList sourceFiles;
  QString buildType;  // "ament_cmake", "ament_python", etc.
  bool isColconPackage = true;
};

/**
 * @brief Panel for browsing the ROS2 workspace
 *
 * Features:
 * - Tree view of workspace packages
 * - Show package metadata (package.xml info)
 * - Browse launch files, config files, source files
 * - Quick search/filter
 * - Open files in external editor
 * - Show/hide colcon/ROS packages
 * - Multiple workspace support
 */
class WorkspaceBrowserPanel : public QWidget {
  Q_OBJECT

public:
  explicit WorkspaceBrowserPanel(QWidget* parent = nullptr);
  ~WorkspaceBrowserPanel() override = default;

signals:
  /**
   * @brief Emitted when a launch file is selected
   */
  void launchFileSelected(const QString& filePath);

  /**
   * @brief Emitted when a config file is selected
   */
  void configFileSelected(const QString& filePath);

  /**
   * @brief Emitted when a package is selected for viewing/editing
   */
  void packageSelected(const WorkspacePackageInfo& package);

  /**
   * @brief Emitted when user wants to open a file in external editor
   */
  void openInEditorRequested(const QString& filePath);

public slots:
  /**
   * @brief Set the workspace path to browse
   */
  void setWorkspacePath(const QString& path);

  /**
   * @brief Refresh the workspace view
   */
  void refresh();

  /**
   * @brief Set the search/filter text
   */
  void setFilterText(const QString& text);

  /**
   * @brief Add an additional workspace to browse
   */
  void addWorkspace(const QString& path);

  /**
   * @brief Remove a workspace from browsing
   */
  void removeWorkspace(const QString& path);

private slots:
  void onSearchTextChanged(const QString& text);
  void onWorkspaceChanged(int index);
  void onItemClicked(QTreeWidgetItem* item, int column);
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onRefreshClicked();
  void onAddWorkspaceClicked();
  void onRemoveWorkspaceClicked();
  void onFileChanged(const QString& path);
  void onContextMenu(const QPoint& pos);

private:
  void setupUi();
  void scanWorkspace(const QString& path);
  void populateTree();
  void populateWithFilter(const QString& filter);
  void addPackageToTree(const WorkspacePackageInfo& pkg);
  WorkspacePackageInfo parsePackageXml(const QString& packagePath);
  QStringList findLaunchFiles(const QString& packagePath);
  QStringList findConfigFiles(const QString& packagePath);
  QStringList findSourceFiles(const QString& packagePath);
  QTreeWidgetItem* createFileItem(const QString& filePath, const QString& fileType);
  QString getFileIcon(const QString& extension);

  // UI components
  QLineEdit* searchEdit_ = nullptr;
  QComboBox* workspaceCombo_ = nullptr;
  QTreeWidget* treeWidget_ = nullptr;
  QPushButton* refreshButton_ = nullptr;
  QPushButton* addWorkspaceButton_ = nullptr;
  QPushButton* removeWorkspaceButton_ = nullptr;
  QLabel* statusLabel_ = nullptr;

  // Workspace data
  QStringList workspacePaths_;
  QString currentWorkspace_;
  QMap<QString, QList<WorkspacePackageInfo>> workspacePackages_;

  // File system watching for auto-refresh
  QFileSystemWatcher* fileWatcher_ = nullptr;

  // Item roles
  static constexpr int FilePathRole = Qt::UserRole;
  static constexpr int FileTypeRole = Qt::UserRole + 1;
  static constexpr int PackageNameRole = Qt::UserRole + 2;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_WORKSPACE_BROWSER_PANEL_HPP
