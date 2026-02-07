#ifndef ROS_WEAVER_WIDGETS_URDF_EDITOR_PANEL_HPP
#define ROS_WEAVER_WIDGETS_URDF_EDITOR_PANEL_HPP

#include <QWidget>
#include <QSplitter>
#include <QPushButton>
#include <QLabel>
#include <QToolBar>
#include <QUndoStack>
#include <memory>

#include "ros_weaver/core/urdf_model.hpp"

namespace ros_weaver {

// Forward declarations
class URDFParser;
class URDFHierarchyTree;
class URDFViewport3D;
class URDFPropertiesPanel;

/**
 * @brief Main panel for the URDF Realtime Editor
 *
 * Provides a visual editor for URDF/xacro files with:
 * - Hierarchy tree view (left panel)
 * - 3D viewport with interactive gizmo (center panel)
 * - Properties panel for links and joints (right panel)
 * - Toolbar for file operations and view controls
 */
class URDFEditorPanel : public QWidget {
  Q_OBJECT

public:
  explicit URDFEditorPanel(QWidget* parent = nullptr);
  ~URDFEditorPanel() override;

  // File operations
  bool openFile(const QString& filePath);
  bool saveFile(const QString& filePath = QString());
  bool saveFileAs();
  void closeFile();

  bool hasUnsavedChanges() const;
  QString currentFilePath() const;

  // Model access
  URDFModel* model() const;
  bool hasModel() const;

  // Undo/redo access
  QUndoStack* undoStack() const { return undoStack_; }

  // Component access
  URDFHierarchyTree* hierarchyTree() const { return hierarchyTree_; }
  URDFViewport3D* viewport() const { return viewport_; }
  URDFPropertiesPanel* propertiesPanel() const { return propertiesPanel_; }

signals:
  void fileOpened(const QString& filePath);
  void fileSaved(const QString& filePath);
  void fileClosed();
  void modelModified();
  void selectionChanged(const QString& elementName, bool isJoint);
  void statusMessage(const QString& message, int timeout = 0);

public slots:
  void onOpenFile();
  void onSaveFile();
  void onSaveFileAs();
  void onCloseFile();
  void onReloadFile();

  // View controls
  void onResetView();
  void onToggleGrid(bool visible);
  void onToggleWireframe(bool wireframe);
  void onFocusSelected();

  // Selection from hierarchy
  void onHierarchySelectionChanged(const QString& name, bool isJoint);

  // Selection from 3D viewport
  void onViewportSelectionChanged(const QString& name, bool isJoint);

  // Property changes
  void onPropertyChanged();

private slots:
  void onModelModified();
  void onParseProgress(int percent, const QString& message);
  void onParseError(const QString& error);
  void onWorkspaceNotBuilt(const QString& workspacePath, const QString& buildCommand);

private:
  void setupUi();
  void setupToolbar();
  void setupLeftPanel();
  void setupCenterPanel();
  void setupRightPanel();
  void setupConnections();

  void updateUiState();
  void updateWindowTitle();
  bool maybeSave();  // Returns false if user cancels

  void selectElement(const QString& name, bool isJoint);

  // Core components
  std::unique_ptr<URDFModel> model_;
  URDFParser* parser_ = nullptr;
  QUndoStack* undoStack_ = nullptr;

  // Current file path
  QString currentFilePath_;

  // UI - Main layout
  QSplitter* mainSplitter_ = nullptr;

  // UI - Toolbar
  QToolBar* toolbar_ = nullptr;
  QPushButton* openButton_ = nullptr;
  QPushButton* saveButton_ = nullptr;
  QPushButton* saveAsButton_ = nullptr;
  QPushButton* reloadButton_ = nullptr;
  QPushButton* undoButton_ = nullptr;
  QPushButton* redoButton_ = nullptr;
  QPushButton* resetViewButton_ = nullptr;
  QPushButton* focusSelectedButton_ = nullptr;
  QPushButton* showLabelsButton_ = nullptr;
  QPushButton* showLinesButton_ = nullptr;
  QPushButton* showAxesButton_ = nullptr;
  QLabel* statusLabel_ = nullptr;

  // UI - Left panel (hierarchy)
  QWidget* leftPanel_ = nullptr;
  URDFHierarchyTree* hierarchyTree_ = nullptr;

  // UI - Center panel (3D viewport)
  QWidget* centerPanel_ = nullptr;
  URDFViewport3D* viewport_ = nullptr;

  // UI - Right panel (properties)
  QWidget* rightPanel_ = nullptr;
  URDFPropertiesPanel* propertiesPanel_ = nullptr;

  // Currently selected element
  QString selectedElementName_;
  bool selectedIsJoint_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_URDF_EDITOR_PANEL_HPP
