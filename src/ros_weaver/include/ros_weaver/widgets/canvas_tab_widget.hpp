#ifndef ROS_WEAVER_WIDGETS_CANVAS_TAB_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_CANVAS_TAB_WIDGET_HPP

#include <QTabWidget>
#include <QMap>
#include <QUuid>

namespace ros_weaver {

class WeaverCanvas;
class Project;
class UndoStack;
class PackageBlock;
class ConnectionLine;
class NodeGroup;

// Data structure for a canvas tab
struct CanvasTabData {
  QUuid id;
  QString name;
  QString description;
  // Canvas state is managed by WeaverCanvas itself
};

// Tabbed widget for managing multiple canvases
class CanvasTabWidget : public QTabWidget {
  Q_OBJECT

public:
  explicit CanvasTabWidget(QWidget* parent = nullptr);
  ~CanvasTabWidget() override;

  // Tab management
  WeaverCanvas* addCanvas(const QString& name = "Canvas");
  void removeCanvas(int index);
  void closeCurrentCanvas();
  WeaverCanvas* currentCanvas() const;
  WeaverCanvas* canvasAt(int index) const;
  int canvasCount() const;

  // Get canvas by ID
  WeaverCanvas* canvasById(const QUuid& id) const;
  QUuid currentCanvasId() const;

  // Tab naming
  void renameTab(int index, const QString& name);
  void renameCurrentTab(const QString& name);
  QString tabName(int index) const;

  // Project serialization - export/import all canvases
  void exportToProject(Project& project) const;
  void importFromProject(const Project& project);

  // Clear all canvases
  void clearAll();

  // Undo stack management
  void setUndoStack(UndoStack* undoStack);

  // Get all canvases
  QList<WeaverCanvas*> allCanvases() const;

  // Duplicate current canvas
  void duplicateCurrentCanvas();

signals:
  // Forward signals from active canvas
  void blockSelected(PackageBlock* block);
  void blockDoubleClicked(PackageBlock* block);
  void connectionCreated(ConnectionLine* connection);
  void groupCreated(NodeGroup* group);
  void canvasCleared();
  void blockYamlSourceChanged(PackageBlock* block, const QString& yamlSource);
  void openBlockInVSCodeRequested(PackageBlock* block);
  void askAIAboutBlock(PackageBlock* block);
  void askAIAboutConnection(ConnectionLine* connection);
  void askAIAboutGroup(NodeGroup* group);
  void askAIAboutPin(PackageBlock* block, int pinIndex, bool isOutput);

  // Tab-specific signals
  void currentCanvasChanged(WeaverCanvas* canvas);
  void tabAdded(int index);
  void tabRemoved(int index);
  void tabRenamed(int index, const QString& name);

private slots:
  void onTabCloseRequested(int index);
  void onCurrentChanged(int index);
  void onTabDoubleClicked(int index);

  // Forward signals from canvas
  void forwardBlockSelected(PackageBlock* block);
  void forwardBlockDoubleClicked(PackageBlock* block);
  void forwardConnectionCreated(ConnectionLine* connection);
  void forwardGroupCreated(NodeGroup* group);
  void forwardCanvasCleared();

private:
  void setupUI();
  void connectCanvasSignals(WeaverCanvas* canvas);
  void disconnectCanvasSignals(WeaverCanvas* canvas);
  QString generateUniqueName(const QString& baseName) const;

  // Map of canvas ID to canvas
  QMap<QUuid, WeaverCanvas*> canvases_;

  // Tab data
  QMap<int, CanvasTabData> tabData_;

  // Undo stack shared by all canvases
  UndoStack* undoStack_;

  // Counter for unique canvas names
  int canvasCounter_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_CANVAS_TAB_WIDGET_HPP
