#ifndef ROS_WEAVER_CANVAS_WEAVER_CANVAS_HPP
#define ROS_WEAVER_CANVAS_WEAVER_CANVAS_HPP

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QContextMenuEvent>
#include <QResizeEvent>
#include <QMap>
#include <QRubberBand>
#include <QLabel>

namespace ros_weaver {

class PackageBlock;
class ConnectionLine;
class NodeGroup;
class Project;
class UndoStack;

class WeaverCanvas : public QGraphicsView {
  Q_OBJECT

public:
  explicit WeaverCanvas(QWidget* parent = nullptr);
  ~WeaverCanvas() override;

  QGraphicsScene* scene() const { return scene_; }

  // Add a package block to the canvas
  PackageBlock* addPackageBlock(const QString& packageName, const QPointF& pos);

  // Remove a package block
  void removePackageBlock(PackageBlock* block);

  // Create a connection between blocks with specific pins
  ConnectionLine* createConnection(PackageBlock* source, int sourcePin,
                                   PackageBlock* target, int targetPin);

  // Remove a connection
  void removeConnection(ConnectionLine* connection);

  // Clear all items from canvas
  void clearCanvas();

  // Delete all selected items
  void deleteSelectedItems();

  // Duplicate selected items
  void duplicateSelectedItems();

  // Select all items
  void selectAllItems();

  // Ungroup selected groups
  void ungroupSelectedGroups();

  // Node group management
  NodeGroup* createNodeGroup(const QString& title = "Group");
  NodeGroup* createGroupFromSelection(const QString& title = "Group");
  void removeNodeGroup(NodeGroup* group);
  QList<NodeGroup*> nodeGroups() const { return nodeGroups_; }

  // Get all connections on the canvas
  QList<ConnectionLine*> connections() const;

  // Zoom controls
  void zoomIn();
  void zoomOut();
  void resetZoom();
  void fitToContents();

  // Grid settings
  bool isGridEnabled() const { return gridEnabled_; }
  void setGridEnabled(bool enabled);
  bool isSnapToGridEnabled() const { return snapToGridEnabled_; }
  void setSnapToGridEnabled(bool enabled);
  int gridMajorSpacing() const { return gridMajorSpacing_; }
  void setGridMajorSpacing(int spacing);
  int gridMinorSubdivisions() const { return gridMinorSubdivisions_; }
  void setGridMinorSubdivisions(int subdivisions);
  QColor gridMajorColor() const { return gridMajorColor_; }
  void setGridMajorColor(const QColor& color);
  QColor gridMinorColor() const { return gridMinorColor_; }
  void setGridMinorColor(const QColor& color);
  qreal gridOpacity() const { return gridOpacity_; }
  void setGridOpacity(qreal opacity);

  // Snap a position to the grid
  QPointF snapToGrid(const QPointF& pos) const;

  // Project serialization
  void exportToProject(Project& project) const;
  void importFromProject(const Project& project);

  // Create block with custom pins (for loading projects)
  PackageBlock* addCustomBlock(const QString& name, const QPointF& pos,
                               const QList<QPair<QString, QString>>& inputPins,
                               const QList<QPair<QString, QString>>& outputPins);

  // Set available YAML files for context menu
  void setAvailableYamlFiles(const QStringList& yamlFiles);

  // Undo/redo support
  void setUndoStack(UndoStack* undoStack);
  UndoStack* undoStack() const { return undoStack_; }

  // Flag for undo command execution (to prevent recursive command creation)
  bool isExecutingCommand() const { return isExecutingCommand_; }
  void setExecutingCommand(bool executing) { isExecutingCommand_ = executing; }

signals:
  void blockSelected(PackageBlock* block);
  void blockDoubleClicked(PackageBlock* block);
  void connectionCreated(ConnectionLine* connection);
  void groupCreated(NodeGroup* group);
  void canvasCleared();
  void blockYamlSourceChanged(PackageBlock* block, const QString& yamlSource);
  void openBlockInVSCodeRequested(PackageBlock* block);
  // AI context menu signals
  void askAIAboutBlock(PackageBlock* block);
  void askAIAboutConnection(ConnectionLine* connection);
  void askAIAboutGroup(NodeGroup* group);
  void askAIAboutPin(PackageBlock* block, int pinIndex, bool isOutput);
  // Grid settings signals
  void gridEnabledChanged(bool enabled);
  void snapToGridEnabledChanged(bool enabled);
  void gridSettingsChanged();

protected:
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private slots:
  void onPinHovered(PackageBlock* block, int pinIndex, bool isOutput);
  void onPinUnhovered(PackageBlock* block);
  void onBlockMoveFinished(PackageBlock* block);

private:
  void setupScene();
  void drawGrid();
  void connectBlockSignals(PackageBlock* block);

  // Find block and pin at scene position
  PackageBlock* blockAtPos(const QPointF& scenePos);
  bool findPinAtPos(const QPointF& scenePos, PackageBlock*& block,
                    int& pinIndex, bool& isOutput);

  // Update temporary connection line during drag
  void updateTempConnection(const QPointF& endPos);

  // Check if a connection would be valid
  bool isValidConnection(PackageBlock* source, int sourcePin,
                         PackageBlock* target, int targetPin);

  // Update zoom level indicator
  void updateZoomIndicator();
  void positionZoomIndicator();

  QGraphicsScene* scene_;
  double zoomFactor_;
  bool isPanning_;
  QPoint lastPanPoint_;

  // Connection dragging state
  bool isDraggingConnection_;
  QGraphicsPathItem* tempConnectionPath_;
  PackageBlock* connectionSourceBlock_;
  int connectionSourcePin_;
  bool connectionFromOutput_;  // true if dragging from output, false if from input
  QColor dragConnectionColor_;

  // For disconnecting - store the original connection being modified
  ConnectionLine* disconnectingLine_;

  // Track currently highlighted connections from pin hover
  QList<ConnectionLine*> pinHighlightedConnections_;

  // Rubber band selection
  bool isRubberBandSelecting_;
  QPoint rubberBandOrigin_;
  QRubberBand* rubberBand_;

  // Node groups
  QList<NodeGroup*> nodeGroups_;

  // Available YAML files for context menu
  QStringList availableYamlFiles_;

  // Undo/redo support
  UndoStack* undoStack_;
  bool isExecutingCommand_;

  // Space key for panning
  bool isSpacePressed_;

  // Zoom indicator
  QLabel* zoomIndicator_;

  // Grid settings
  bool gridEnabled_;
  bool snapToGridEnabled_;
  int gridMajorSpacing_;
  int gridMinorSubdivisions_;
  QColor gridMajorColor_;
  QColor gridMinorColor_;
  qreal gridOpacity_;

  static constexpr double MIN_ZOOM = 0.1;
  static constexpr double MAX_ZOOM = 5.0;
  static constexpr double ZOOM_STEP = 0.1;
  static constexpr int DEFAULT_GRID_MAJOR_SPACING = 100;
  static constexpr int DEFAULT_GRID_MINOR_SUBDIVISIONS = 5;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_WEAVER_CANVAS_HPP
