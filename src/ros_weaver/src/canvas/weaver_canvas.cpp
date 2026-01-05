#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/lineage_provider.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include "ros_weaver/widgets/lineage_dialog.hpp"
#include "ros_weaver/core/undo/undo_stack.hpp"
#include "ros_weaver/core/undo/commands/add_block_command.hpp"
#include "ros_weaver/core/undo/commands/remove_block_command.hpp"
#include "ros_weaver/core/undo/commands/move_block_command.hpp"
#include "ros_weaver/core/undo/commands/add_connection_command.hpp"
#include "ros_weaver/core/undo/commands/remove_connection_command.hpp"
#include "ros_weaver/core/undo/commands/add_group_command.hpp"
#include "ros_weaver/core/undo/commands/remove_group_command.hpp"
#include "ros_weaver/core/undo/commands/macro_command.hpp"

#include <QMenu>
#include <QInputDialog>
#include <QMimeData>
#include <QDrag>
#include <QScrollBar>
#include <QGraphicsPathItem>
#include <QSet>
#include <QMap>
#include <QFileInfo>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

WeaverCanvas::WeaverCanvas(QWidget* parent)
  : QGraphicsView(parent)
  , scene_(nullptr)
  , zoomFactor_(1.0)
  , isPanning_(false)
  , isDraggingConnection_(false)
  , tempConnectionPath_(nullptr)
  , connectionSourceBlock_(nullptr)
  , connectionSourcePin_(-1)
  , connectionFromOutput_(true)
  , disconnectingLine_(nullptr)
  , isRubberBandSelecting_(false)
  , rubberBand_(nullptr)
  , undoStack_(nullptr)
  , isExecutingCommand_(false)
  , isSpacePressed_(false)
  , zoomIndicator_(nullptr)
  , gridEnabled_(true)
  , snapToGridEnabled_(false)
  , gridMajorSpacing_(DEFAULT_GRID_MAJOR_SPACING)
  , gridMinorSubdivisions_(DEFAULT_GRID_MINOR_SUBDIVISIONS)
  , gridMajorColor_(QColor(70, 70, 70))
  , gridMinorColor_(QColor(50, 50, 50))
  , gridOpacity_(1.0)
{
  setupScene();

  // Configure view settings
  setRenderHint(QPainter::Antialiasing, true);
  setRenderHint(QPainter::SmoothPixmapTransform, true);
  setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorUnderMouse);

  // Enable drag and drop
  setAcceptDrops(true);
  setDragMode(QGraphicsView::NoDrag);  // We handle dragging manually

  // Enable focus for keyboard events
  setFocusPolicy(Qt::StrongFocus);

  // Set background color
  setBackgroundBrush(QBrush(QColor(35, 35, 35)));

  // Set scene rect large enough for comfortable editing
  scene_->setSceneRect(-5000, -5000, 10000, 10000);

  // Center the view
  centerOn(0, 0);

  // Create zoom indicator label
  zoomIndicator_ = new QLabel(this);
  zoomIndicator_->setStyleSheet(
    "QLabel {"
    "  color: rgba(255, 255, 255, 80);"
    "  background: transparent;"
    "  font-size: 11px;"
    "  padding: 4px 8px;"
    "}"
  );
  zoomIndicator_->setAttribute(Qt::WA_TransparentForMouseEvents);
  updateZoomIndicator();
}

WeaverCanvas::~WeaverCanvas() {
  std::cerr << "WeaverCanvas destructor" << std::endl;
}

void WeaverCanvas::setUndoStack(UndoStack* undoStack) {
  undoStack_ = undoStack;
}

// Helper to create BlockData from a PackageBlock
static BlockData createBlockData(PackageBlock* block) {
  BlockData blockData;
  blockData.id = block->id();
  blockData.name = block->packageName();
  blockData.position = block->pos();

  // Export input pins
  for (const Pin& pin : block->inputPins()) {
    PinData pinData;
    pinData.name = pin.name;
    pinData.type = "input";
    switch (pin.dataType) {
      case Pin::DataType::Topic: pinData.dataType = "topic"; break;
      case Pin::DataType::Service: pinData.dataType = "service"; break;
      case Pin::DataType::Action: pinData.dataType = "action"; break;
      case Pin::DataType::Parameter: pinData.dataType = "parameter"; break;
    }
    pinData.messageType = pin.messageType;
    blockData.inputPins.append(pinData);
  }

  // Export output pins
  for (const Pin& pin : block->outputPins()) {
    PinData pinData;
    pinData.name = pin.name;
    pinData.type = "output";
    switch (pin.dataType) {
      case Pin::DataType::Topic: pinData.dataType = "topic"; break;
      case Pin::DataType::Service: pinData.dataType = "service"; break;
      case Pin::DataType::Action: pinData.dataType = "action"; break;
      case Pin::DataType::Parameter: pinData.dataType = "parameter"; break;
    }
    pinData.messageType = pin.messageType;
    blockData.outputPins.append(pinData);
  }

  // Export parameters
  blockData.parameters = block->parameters();
  blockData.preferredYamlSource = block->preferredYamlSource();

  return blockData;
}

// Helper to create ConnectionData from a ConnectionLine
static ConnectionData createConnectionData(ConnectionLine* conn) {
  ConnectionData connData;
  connData.id = conn->id();
  // Guard against null blocks (could happen if connection is in detached state)
  connData.sourceBlockId = conn->sourceBlock() ? conn->sourceBlock()->id() : QUuid();
  connData.sourcePinIndex = conn->sourcePinIndex();
  connData.targetBlockId = conn->targetBlock() ? conn->targetBlock()->id() : QUuid();
  connData.targetPinIndex = conn->targetPinIndex();
  return connData;
}

// Helper to create NodeGroupData from a NodeGroup
static NodeGroupData createGroupData(NodeGroup* group) {
  NodeGroupData groupData;
  groupData.id = group->id();
  groupData.title = group->title();
  groupData.position = group->pos();
  groupData.size = group->size();
  groupData.color = group->color();
  for (PackageBlock* node : group->containedNodes()) {
    groupData.containedNodeIds.append(node->id());
  }
  return groupData;
}

void WeaverCanvas::setupScene() {
  scene_ = new QGraphicsScene(this);
  setScene(scene_);
  drawGrid();

  // Connect to scene selection changes to emit blockSelected signal
  connect(scene_, &QGraphicsScene::selectionChanged, this, [this]() {
    QList<QGraphicsItem*> selected = scene_->selectedItems();
    PackageBlock* selectedBlock = nullptr;

    // Find the first selected PackageBlock
    for (QGraphicsItem* item : selected) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        selectedBlock = block;
        break;
      }
    }

    emit blockSelected(selectedBlock);
  });
}

void WeaverCanvas::drawGrid() {
  if (!gridEnabled_) {
    // Solid background when grid is disabled
    setBackgroundBrush(QBrush(QColor(35, 35, 35)));
    return;
  }

  // Calculate minor grid spacing
  int minorSpacing = gridMajorSpacing_ / gridMinorSubdivisions_;
  if (minorSpacing < 5) minorSpacing = 5;  // Minimum spacing for visibility

  // Create a pixmap that tiles seamlessly - size matches one major grid cell
  int tileSize = gridMajorSpacing_;
  QPixmap gridPixmap(tileSize, tileSize);
  gridPixmap.fill(QColor(35, 35, 35));

  QPainter painter(&gridPixmap);
  painter.setRenderHint(QPainter::Antialiasing, false);

  // Apply opacity
  QColor minorColor = gridMinorColor_;
  minorColor.setAlphaF(minorColor.alphaF() * gridOpacity_);
  QColor majorColor = gridMajorColor_;
  majorColor.setAlphaF(majorColor.alphaF() * gridOpacity_);

  // Draw minor grid lines
  painter.setPen(QPen(minorColor, 1));
  for (int i = minorSpacing; i < tileSize; i += minorSpacing) {
    // Vertical minor lines
    painter.drawLine(i, 0, i, tileSize);
    // Horizontal minor lines
    painter.drawLine(0, i, tileSize, i);
  }

  // Draw major grid lines (at the edges of the tile, thicker and brighter)
  painter.setPen(QPen(majorColor, 1));
  // Left edge (major vertical line)
  painter.drawLine(0, 0, 0, tileSize);
  // Top edge (major horizontal line)
  painter.drawLine(0, 0, tileSize, 0);

  // Draw origin indicator (slightly brighter at 0,0)
  painter.setPen(QPen(majorColor.lighter(120), 2));
  painter.drawPoint(0, 0);

  painter.end();

  setBackgroundBrush(QBrush(gridPixmap));
}

void WeaverCanvas::setGridEnabled(bool enabled) {
  if (gridEnabled_ != enabled) {
    gridEnabled_ = enabled;
    drawGrid();
    emit gridEnabledChanged(enabled);
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setSnapToGridEnabled(bool enabled) {
  if (snapToGridEnabled_ != enabled) {
    snapToGridEnabled_ = enabled;
    emit snapToGridEnabledChanged(enabled);
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setGridMajorSpacing(int spacing) {
  if (spacing >= 20 && spacing <= 500 && gridMajorSpacing_ != spacing) {
    gridMajorSpacing_ = spacing;
    drawGrid();
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setGridMinorSubdivisions(int subdivisions) {
  if (subdivisions >= 1 && subdivisions <= 10 && gridMinorSubdivisions_ != subdivisions) {
    gridMinorSubdivisions_ = subdivisions;
    drawGrid();
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setGridMajorColor(const QColor& color) {
  if (gridMajorColor_ != color) {
    gridMajorColor_ = color;
    drawGrid();
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setGridMinorColor(const QColor& color) {
  if (gridMinorColor_ != color) {
    gridMinorColor_ = color;
    drawGrid();
    emit gridSettingsChanged();
  }
}

void WeaverCanvas::setGridOpacity(qreal opacity) {
  opacity = qBound(0.0, opacity, 1.0);
  if (!qFuzzyCompare(gridOpacity_, opacity)) {
    gridOpacity_ = opacity;
    drawGrid();
    emit gridSettingsChanged();
  }
}

QPointF WeaverCanvas::snapToGrid(const QPointF& pos) const {
  if (!snapToGridEnabled_ || gridMajorSpacing_ <= 0) {
    return pos;
  }

  // Calculate minor grid spacing for finer snapping
  int snapSpacing = gridMajorSpacing_ / gridMinorSubdivisions_;
  if (snapSpacing < 5) snapSpacing = 5;

  // Round to nearest grid point
  qreal snappedX = std::round(pos.x() / snapSpacing) * snapSpacing;
  qreal snappedY = std::round(pos.y() / snapSpacing) * snapSpacing;

  return QPointF(snappedX, snappedY);
}

PackageBlock* WeaverCanvas::addPackageBlock(const QString& packageName, const QPointF& pos) {
  PackageBlock* block = new PackageBlock(packageName);
  block->setPos(pos);
  scene_->addItem(block);

  // Connect signals for pin hover highlighting
  connectBlockSignals(block);

  // Add pins based on node type
  if (packageName == "publisher_node") {
    // Publisher: no inputs, one or more outputs
    block->addOutputPin("topic_out", Pin::DataType::Topic, "std_msgs/msg/String");
  }
  else if (packageName == "subscriber_node") {
    // Subscriber: one or more inputs, no outputs
    block->addInputPin("topic_in", Pin::DataType::Topic, "std_msgs/msg/String");
  }
  else if (packageName == "relay_node") {
    // Relay: takes input, produces output
    block->addInputPin("input", Pin::DataType::Topic, "std_msgs/msg/String");
    block->addOutputPin("output", Pin::DataType::Topic, "std_msgs/msg/String");
  }
  else if (packageName == "service_server") {
    // Service server: provides a service
    block->addInputPin("request", Pin::DataType::Service, "std_srvs/srv/Trigger");
    block->addOutputPin("response", Pin::DataType::Service, "std_srvs/srv/Trigger");
  }
  else if (packageName == "service_client") {
    // Service client: calls a service
    block->addOutputPin("request", Pin::DataType::Service, "std_srvs/srv/Trigger");
    block->addInputPin("response", Pin::DataType::Service, "std_srvs/srv/Trigger");
  }
  else if (packageName == "action_server") {
    // Action server: multiple inputs/outputs for goal, feedback, result
    block->addInputPin("goal", Pin::DataType::Action, "");
    block->addOutputPin("feedback", Pin::DataType::Action, "");
    block->addOutputPin("result", Pin::DataType::Action, "");
  }
  else if (packageName == "action_client") {
    // Action client: sends goals, receives feedback/result
    block->addOutputPin("goal", Pin::DataType::Action, "");
    block->addInputPin("feedback", Pin::DataType::Action, "");
    block->addInputPin("result", Pin::DataType::Action, "");
  }
  else if (packageName == "sensor_fusion") {
    // Example: multiple inputs, one output
    block->addInputPin("lidar", Pin::DataType::Topic, "sensor_msgs/msg/LaserScan");
    block->addInputPin("camera", Pin::DataType::Topic, "sensor_msgs/msg/Image");
    block->addInputPin("imu", Pin::DataType::Topic, "sensor_msgs/msg/Imu");
    block->addOutputPin("fused_data", Pin::DataType::Topic, "");
  }
  else if (packageName == "robot_controller") {
    // Example: multiple inputs and outputs
    block->addInputPin("cmd_vel", Pin::DataType::Topic, "geometry_msgs/msg/Twist");
    block->addInputPin("odom", Pin::DataType::Topic, "nav_msgs/msg/Odometry");
    block->addInputPin("scan", Pin::DataType::Topic, "sensor_msgs/msg/LaserScan");
    block->addOutputPin("motor_left", Pin::DataType::Topic, "std_msgs/msg/Float64");
    block->addOutputPin("motor_right", Pin::DataType::Topic, "std_msgs/msg/Float64");
    block->addOutputPin("status", Pin::DataType::Topic, "std_msgs/msg/String");
  }
  else if (packageName == "image_processor") {
    // Example: image processing node
    block->addInputPin("image_raw", Pin::DataType::Topic, "sensor_msgs/msg/Image");
    block->addInputPin("camera_info", Pin::DataType::Topic, "sensor_msgs/msg/CameraInfo");
    block->addOutputPin("image_rect", Pin::DataType::Topic, "sensor_msgs/msg/Image");
    block->addOutputPin("detections", Pin::DataType::Topic, "vision_msgs/msg/Detection2DArray");
  }
  else {
    // Default: generic node with one input and one output
    block->addInputPin("input", Pin::DataType::Topic, "std_msgs/msg/String");
    block->addOutputPin("output", Pin::DataType::Topic, "std_msgs/msg/String");
  }

  // Push undo command if not executing a command (to prevent recursion)
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new AddBlockCommand(this, createBlockData(block)));
  }

  return block;
}

void WeaverCanvas::connectBlockSignals(PackageBlock* block) {
  connect(block, &PackageBlock::pinHovered, this, &WeaverCanvas::onPinHovered);
  connect(block, &PackageBlock::pinUnhovered, this, &WeaverCanvas::onPinUnhovered);
  connect(block, &PackageBlock::blockMoveFinished, this, &WeaverCanvas::onBlockMoveFinished);
}

void WeaverCanvas::onPinHovered(PackageBlock* block, int pinIndex, bool isOutput) {
  // Clear any previous highlighting
  for (ConnectionLine* conn : pinHighlightedConnections_) {
    conn->setPinHighlighted(false);
  }
  pinHighlightedConnections_.clear();

  // Get all connections for this pin and highlight them
  QList<ConnectionLine*> connections = block->connectionsForPin(pinIndex, isOutput);
  for (ConnectionLine* conn : connections) {
    conn->setPinHighlighted(true);
    pinHighlightedConnections_.append(conn);
  }
}

void WeaverCanvas::onPinUnhovered(PackageBlock* block) {
  Q_UNUSED(block)

  // Clear all pin highlighting
  for (ConnectionLine* conn : pinHighlightedConnections_) {
    conn->setPinHighlighted(false);
  }
  pinHighlightedConnections_.clear();
}

void WeaverCanvas::onBlockMoveFinished(PackageBlock* block) {
  if (!block) return;

  QPointF oldPos = block->moveStartPos();
  QPointF newPos = block->pos();

  // Apply snap-to-grid if enabled
  if (snapToGridEnabled_) {
    QPointF snappedPos = snapToGrid(newPos);
    if (snappedPos != newPos) {
      block->setPos(snappedPos);
      newPos = snappedPos;
    }
  }

  // Push MoveBlockCommand if position actually changed
  if (undoStack_ && !isExecutingCommand_ && oldPos != newPos) {
    undoStack_->push(new MoveBlockCommand(this, block->id(), oldPos, newPos));
  }

  // Check if block was dragged into a group
  for (NodeGroup* group : nodeGroups_) {
    bool wasInGroup = group->containedNodes().contains(block);
    bool isNowInGroup = group->containsNode(block);

    if (!wasInGroup && isNowInGroup) {
      // Block was dragged into this group
      group->addNode(block);
    } else if (wasInGroup && !isNowInGroup) {
      // Block was dragged out of this group
      group->removeNode(block);
    }
  }
}

void WeaverCanvas::removePackageBlock(PackageBlock* block) {
  if (!block) return;

  // Gather data for undo BEFORE modifying anything
  BlockData blockData;
  QList<ConnectionData> connectionDataList;

  if (undoStack_ && !isExecutingCommand_) {
    blockData = createBlockData(block);

    // Gather connection data for all connections attached to this block
    for (ConnectionLine* conn : block->connections()) {
      connectionDataList.append(createConnectionData(conn));
    }
  }

  // Remove all connections associated with this block
  // Set isExecutingCommand_ to prevent individual RemoveConnectionCommands
  bool wasExecutingCommand = isExecutingCommand_;
  isExecutingCommand_ = true;

  QList<ConnectionLine*> connections = block->connections();
  for (ConnectionLine* conn : connections) {
    removeConnection(conn);
  }

  // Remove from any groups
  for (NodeGroup* group : nodeGroups_) {
    group->removeNode(block);
  }

  scene_->removeItem(block);
  delete block;

  isExecutingCommand_ = wasExecutingCommand;

  // Push undo command after deletion (since command skips first redo)
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new RemoveBlockCommand(this, blockData, connectionDataList));
  }
}

ConnectionLine* WeaverCanvas::createConnection(PackageBlock* source, int sourcePin,
                                                PackageBlock* target, int targetPin) {
  if (!source || !target || source == target) return nullptr;

  // Validate pin indices
  if (sourcePin < 0 || sourcePin >= source->outputPins().size()) return nullptr;
  if (targetPin < 0 || targetPin >= target->inputPins().size()) return nullptr;

  // Check type compatibility
  if (!isValidConnection(source, sourcePin, target, targetPin)) {
    return nullptr;
  }

  // Create the connection
  ConnectionLine* connection = new ConnectionLine(source, sourcePin, target, targetPin);

  // Set connection color based on data type
  connection->setConnectionColor(source->pinColor(false, sourcePin));

  // Set topic name from source pin name (for animation matching)
  const QList<Pin>& pins = source->outputPins();
  if (sourcePin < pins.size()) {
    QString pinName = pins[sourcePin].name;
    // Ensure topic name starts with / for ROS convention
    QString topicName = pinName.startsWith('/') ? pinName : ("/" + pinName);
    connection->setTopicName(topicName);
  }

  scene_->addItem(connection);

  source->addConnection(connection);
  target->addConnection(connection);

  // Push undo command after connection is fully created
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new AddConnectionCommand(this, createConnectionData(connection)));
  }

  emit connectionCreated(connection);
  return connection;
}

void WeaverCanvas::removeConnection(ConnectionLine* connection) {
  if (!connection) return;

  // Gather data for undo BEFORE modifying anything
  ConnectionData connData;
  if (undoStack_ && !isExecutingCommand_) {
    connData = createConnectionData(connection);
  }

  // Remove from blocks
  if (connection->sourceBlock()) {
    connection->sourceBlock()->removeConnection(connection);
  }
  if (connection->targetBlock()) {
    connection->targetBlock()->removeConnection(connection);
  }

  scene_->removeItem(connection);
  delete connection;

  // Push undo command after deletion (since command skips first redo)
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new RemoveConnectionCommand(this, connData));
  }
}

QList<ConnectionLine*> WeaverCanvas::connections() const {
  QList<ConnectionLine*> result;
  for (QGraphicsItem* item : scene_->items()) {
    if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      result.append(conn);
    }
  }
  return result;
}

void WeaverCanvas::clearCanvas() {
  // Clear all tracked pointers before scene_->clear() deletes the objects
  pinHighlightedConnections_.clear();
  nodeGroups_.clear();

  // Reset connection dragging state in case we're mid-drag
  if (tempConnectionPath_) {
    // Note: tempConnectionPath_ will be deleted by scene_->clear()
    tempConnectionPath_ = nullptr;
  }
  isDraggingConnection_ = false;
  connectionSourceBlock_ = nullptr;
  connectionSourcePin_ = -1;
  disconnectingLine_ = nullptr;

  // CRITICAL: Detach all connections from blocks BEFORE scene_->clear()
  // This prevents use-after-free when ConnectionLine destructors run
  // after PackageBlock objects have already been deleted (Qt deletes
  // scene items in undefined order)
  for (QGraphicsItem* item : scene_->items()) {
    if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      conn->detach();
    }
  }

  scene_->clear();
  drawGrid();
  emit canvasCleared();
}

void WeaverCanvas::deleteSelectedItems() {
  QList<QGraphicsItem*> selected = scene_->selectedItems();

  // First collect all items to delete (to avoid issues with iterator invalidation)
  QList<PackageBlock*> blocksToDelete;
  QList<ConnectionLine*> connectionsToDelete;
  QList<NodeGroup*> groupsToDelete;

  for (QGraphicsItem* item : selected) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      blocksToDelete.append(block);
    } else if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      connectionsToDelete.append(conn);
    } else if (NodeGroup* group = dynamic_cast<NodeGroup*>(item)) {
      groupsToDelete.append(group);
    }
  }

  // Count total items to delete
  int totalItems = blocksToDelete.size() + connectionsToDelete.size() + groupsToDelete.size();
  if (totalItems == 0) return;

  // Use macro for multiple deletions so they can be undone together
  if (undoStack_ && totalItems > 1) {
    undoStack_->beginMacro(tr("Delete %1 Items").arg(totalItems));
  }

  // Delete connections first
  for (ConnectionLine* conn : connectionsToDelete) {
    removeConnection(conn);
  }

  // Then delete blocks (this will also remove their connections)
  for (PackageBlock* block : blocksToDelete) {
    removePackageBlock(block);
  }

  // Then delete groups
  for (NodeGroup* group : groupsToDelete) {
    removeNodeGroup(group);
  }

  // End macro if we started one
  if (undoStack_ && totalItems > 1) {
    undoStack_->endMacro();
  }
}

void WeaverCanvas::duplicateSelectedItems() {
  QList<QGraphicsItem*> selected = scene_->selectedItems();

  QList<PackageBlock*> blocksToDuplicate;
  for (QGraphicsItem* item : selected) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      blocksToDuplicate.append(block);
    }
  }

  if (blocksToDuplicate.isEmpty()) return;

  // Use macro for multiple duplications
  if (undoStack_ && blocksToDuplicate.size() > 1) {
    undoStack_->beginMacro(tr("Duplicate %1 Items").arg(blocksToDuplicate.size()));
  }

  // Deselect all first
  scene_->clearSelection();

  // Duplicate each block with an offset
  constexpr double OFFSET = 30.0;
  for (PackageBlock* original : blocksToDuplicate) {
    // Helper to convert Pin::DataType to string
    auto dataTypeToString = [](Pin::DataType dt) -> QString {
      switch (dt) {
        case Pin::DataType::Topic: return "topic";
        case Pin::DataType::Service: return "service";
        case Pin::DataType::Action: return "action";
        case Pin::DataType::Parameter: return "parameter";
        default: return "topic";
      }
    };

    // Create input and output pin lists
    QList<QPair<QString, QString>> inputPinList;
    QList<QPair<QString, QString>> outputPinList;

    for (const Pin& pin : original->inputPins()) {
      inputPinList.append(qMakePair(dataTypeToString(pin.dataType), pin.name));
    }
    for (const Pin& pin : original->outputPins()) {
      outputPinList.append(qMakePair(dataTypeToString(pin.dataType), pin.name));
    }

    // Create duplicated block at offset position
    QPointF newPos = original->pos() + QPointF(OFFSET, OFFSET);
    PackageBlock* duplicate = addCustomBlock(
        original->packageName(),
        newPos,
        inputPinList,
        outputPinList
    );

    if (duplicate) {
      duplicate->setSelected(true);
      // Copy parameters if any
      if (original->hasParameters()) {
        duplicate->setParameters(original->parameters());
      }
    }
  }

  // End macro if we started one
  if (undoStack_ && blocksToDuplicate.size() > 1) {
    undoStack_->endMacro();
  }
}

void WeaverCanvas::selectAllItems() {
  for (QGraphicsItem* item : scene_->items()) {
    if (dynamic_cast<PackageBlock*>(item) ||
        dynamic_cast<ConnectionLine*>(item) ||
        dynamic_cast<NodeGroup*>(item)) {
      item->setSelected(true);
    }
  }
}

void WeaverCanvas::ungroupSelectedGroups() {
  QList<QGraphicsItem*> selected = scene_->selectedItems();

  QList<NodeGroup*> groupsToUngroup;
  for (QGraphicsItem* item : selected) {
    if (NodeGroup* group = dynamic_cast<NodeGroup*>(item)) {
      groupsToUngroup.append(group);
    }
  }

  if (groupsToUngroup.isEmpty()) return;

  // Use macro for multiple ungroupings
  if (undoStack_ && groupsToUngroup.size() > 1) {
    undoStack_->beginMacro(tr("Ungroup %1 Groups").arg(groupsToUngroup.size()));
  }

  for (NodeGroup* group : groupsToUngroup) {
    removeNodeGroup(group);
  }

  // End macro if we started one
  if (undoStack_ && groupsToUngroup.size() > 1) {
    undoStack_->endMacro();
  }
}

NodeGroup* WeaverCanvas::createNodeGroup(const QString& title) {
  NodeGroup* group = new NodeGroup(title);
  scene_->addItem(group);
  nodeGroups_.append(group);
  emit groupCreated(group);
  return group;
}

NodeGroup* WeaverCanvas::createGroupFromSelection(const QString& title) {
  QList<QGraphicsItem*> selected = scene_->selectedItems();

  // Collect selected PackageBlocks
  QList<PackageBlock*> selectedBlocks;
  for (QGraphicsItem* item : selected) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      selectedBlocks.append(block);
    }
  }

  if (selectedBlocks.isEmpty()) {
    return nullptr;
  }

  // Create the group (suppress undo for this intermediate step)
  bool wasExecutingCommand = isExecutingCommand_;
  isExecutingCommand_ = true;
  NodeGroup* group = createNodeGroup(title);
  isExecutingCommand_ = wasExecutingCommand;

  // Add all selected blocks to the group
  for (PackageBlock* block : selectedBlocks) {
    group->addNode(block);
  }

  // Fit group to contents (use larger padding for single node)
  qreal padding = selectedBlocks.size() == 1 ? 50.0 : 30.0;
  group->fitToContents(padding);

  // Push undo command after group is fully set up
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new AddGroupCommand(this, createGroupData(group)));
  }

  return group;
}

void WeaverCanvas::removeNodeGroup(NodeGroup* group) {
  if (!group) return;

  // Gather data for undo BEFORE modifying anything
  NodeGroupData groupData;
  if (undoStack_ && !isExecutingCommand_) {
    groupData = createGroupData(group);
  }

  nodeGroups_.removeOne(group);
  scene_->removeItem(group);
  delete group;

  // Push undo command after deletion (since command skips first redo)
  if (undoStack_ && !isExecutingCommand_) {
    undoStack_->push(new RemoveGroupCommand(this, groupData));
  }
}

void WeaverCanvas::zoomIn() {
  if (zoomFactor_ < MAX_ZOOM) {
    zoomFactor_ += ZOOM_STEP;
    setTransform(QTransform::fromScale(zoomFactor_, zoomFactor_));
    updateZoomIndicator();
  }
}

void WeaverCanvas::zoomOut() {
  if (zoomFactor_ > MIN_ZOOM) {
    zoomFactor_ -= ZOOM_STEP;
    setTransform(QTransform::fromScale(zoomFactor_, zoomFactor_));
    updateZoomIndicator();
  }
}

void WeaverCanvas::resetZoom() {
  zoomFactor_ = 1.0;
  setTransform(QTransform());
  updateZoomIndicator();
}

void WeaverCanvas::fitToContents() {
  // Get bounding rect of all items (excluding background)
  QRectF itemsBounds;
  bool hasItems = false;

  for (QGraphicsItem* item : scene_->items()) {
    // Only consider PackageBlock items (skip connections and background)
    if (dynamic_cast<PackageBlock*>(item)) {
      if (!hasItems) {
        itemsBounds = item->sceneBoundingRect();
        hasItems = true;
      } else {
        itemsBounds = itemsBounds.united(item->sceneBoundingRect());
      }
    }
  }

  if (!hasItems) {
    // No items, just reset to center
    resetZoom();
    centerOn(0, 0);
    return;
  }

  // Add some padding around the content
  qreal padding = 50;
  itemsBounds.adjust(-padding, -padding, padding, padding);

  // Fit the view to show all items
  fitInView(itemsBounds, Qt::KeepAspectRatio);

  // Update zoom factor to match the current transform
  zoomFactor_ = transform().m11();

  // Clamp zoom factor to valid range
  if (zoomFactor_ < MIN_ZOOM) {
    zoomFactor_ = MIN_ZOOM;
    setTransform(QTransform::fromScale(zoomFactor_, zoomFactor_));
    centerOn(itemsBounds.center());
  } else if (zoomFactor_ > MAX_ZOOM) {
    zoomFactor_ = MAX_ZOOM;
    setTransform(QTransform::fromScale(zoomFactor_, zoomFactor_));
    centerOn(itemsBounds.center());
  }

  updateZoomIndicator();
}

PackageBlock* WeaverCanvas::blockAtPos(const QPointF& scenePos) {
  QList<QGraphicsItem*> items = scene_->items(scenePos);
  for (QGraphicsItem* item : items) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      return block;
    }
  }
  return nullptr;
}

bool WeaverCanvas::findPinAtPos(const QPointF& scenePos, PackageBlock*& block,
                                 int& pinIndex, bool& isOutput) {
  QList<QGraphicsItem*> items = scene_->items(scenePos);
  for (QGraphicsItem* item : items) {
    if (PackageBlock* b = dynamic_cast<PackageBlock*>(item)) {
      QPointF localPos = b->mapFromScene(scenePos);

      // Check output pins first (right side)
      int outputIdx = b->outputPinAtPos(localPos);
      if (outputIdx >= 0) {
        block = b;
        pinIndex = outputIdx;
        isOutput = true;
        return true;
      }

      // Check input pins (left side)
      int inputIdx = b->inputPinAtPos(localPos);
      if (inputIdx >= 0) {
        block = b;
        pinIndex = inputIdx;
        isOutput = false;
        return true;
      }
    }
  }
  return false;
}

void WeaverCanvas::updateTempConnection(const QPointF& endPos) {
  if (!tempConnectionPath_ || !connectionSourceBlock_) return;

  auto& theme = ThemeManager::instance();

  QPointF startPos;
  if (connectionFromOutput_) {
    startPos = connectionSourceBlock_->outputPinScenePos(connectionSourcePin_);
  } else {
    startPos = connectionSourceBlock_->inputPinScenePos(connectionSourcePin_);
  }

  // Calculate bezier curve control points
  qreal dx = std::abs(endPos.x() - startPos.x());
  qreal offset = std::max(50.0, dx * 0.5);

  QPointF ctrl1, ctrl2;
  if (connectionFromOutput_) {
    ctrl1 = QPointF(startPos.x() + offset, startPos.y());
    ctrl2 = QPointF(endPos.x() - offset, endPos.y());
  } else {
    ctrl1 = QPointF(startPos.x() - offset, startPos.y());
    ctrl2 = QPointF(endPos.x() + offset, endPos.y());
  }

  QPainterPath path;
  path.moveTo(startPos);
  path.cubicTo(ctrl1, ctrl2, endPos);

  tempConnectionPath_->setPath(path);

  // Check if we're hovering over a valid target
  PackageBlock* targetBlock = nullptr;
  int targetPin = -1;
  bool targetIsOutput = false;

  if (findPinAtPos(endPos, targetBlock, targetPin, targetIsOutput)) {
    // Check if same pin type (output->output or input->input = invalid)
    bool sameType = (connectionFromOutput_ && targetIsOutput) ||
                    (!connectionFromOutput_ && !targetIsOutput);

    // Check if connecting to self
    bool selfConnect = (targetBlock == connectionSourceBlock_);

    if (sameType || selfConnect) {
      // Invalid connection - red with X pattern
      QPen pen(theme.errorColor(), 3, Qt::DotLine);
      tempConnectionPath_->setPen(pen);
      return;
    }

    // Valid target direction: output -> input or input -> output
    bool validTarget = (connectionFromOutput_ && !targetIsOutput) ||
                       (!connectionFromOutput_ && targetIsOutput);

    if (validTarget && targetBlock != connectionSourceBlock_) {
      // Check type compatibility
      bool compatible = false;
      if (connectionFromOutput_) {
        compatible = isValidConnection(connectionSourceBlock_, connectionSourcePin_,
                                       targetBlock, targetPin);
      } else {
        compatible = isValidConnection(targetBlock, targetPin,
                                       connectionSourceBlock_, connectionSourcePin_);
      }

      if (compatible) {
        // Valid connection - bright green solid line, thicker
        QPen pen(theme.successColor(), 3);
        tempConnectionPath_->setPen(pen);
        return;
      } else {
        // Incompatible types - red dashed line
        QPen pen(theme.errorColor(), 2, Qt::DashLine);
        tempConnectionPath_->setPen(pen);
        return;
      }
    }
  }

  // Default state (not hovering over any pin) - neutral dashed line
  QColor neutralColor = dragConnectionColor_;
  neutralColor.setAlpha(180);
  QPen pen(neutralColor, 2, Qt::DashLine);
  tempConnectionPath_->setPen(pen);
}

bool WeaverCanvas::isValidConnection(PackageBlock* source, int sourcePin,
                                      PackageBlock* target, int targetPin) {
  if (!source || !target || source == target) return false;
  if (sourcePin < 0 || sourcePin >= source->outputPins().size()) return false;
  if (targetPin < 0 || targetPin >= target->inputPins().size()) return false;

  const Pin& outPin = source->outputPins()[sourcePin];
  const Pin& inPin = target->inputPins()[targetPin];

  return PackageBlock::canConnect(outPin, inPin);
}

void WeaverCanvas::wheelEvent(QWheelEvent* event) {
  // Zoom with Ctrl+scroll
  if (event->modifiers() & Qt::ControlModifier) {
    if (event->angleDelta().y() > 0) {
      zoomIn();
    } else {
      zoomOut();
    }
    event->accept();
  } else {
    QGraphicsView::wheelEvent(event);
  }
}

void WeaverCanvas::mousePressEvent(QMouseEvent* event) {
  // Ensure we have focus for keyboard events
  setFocus();

  // Middle mouse button for panning
  if (event->button() == Qt::MiddleButton) {
    isPanning_ = true;
    lastPanPoint_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }

  // Space + Left click for panning (alternative)
  if (event->button() == Qt::LeftButton && isSpacePressed_) {
    isPanning_ = true;
    lastPanPoint_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }

  // Left click - check for pin interaction
  if (event->button() == Qt::LeftButton) {
    QPointF scenePos = mapToScene(event->pos());

    PackageBlock* block = nullptr;
    int pinIndex = -1;
    bool isOutput = false;

    if (findPinAtPos(scenePos, block, pinIndex, isOutput)) {
      // Start dragging a new connection
      isDraggingConnection_ = true;
      connectionSourceBlock_ = block;
      connectionSourcePin_ = pinIndex;
      connectionFromOutput_ = isOutput;
      disconnectingLine_ = nullptr;

      // Get pin color for visual feedback
      dragConnectionColor_ = block->pinColor(isOutput ? false : true, pinIndex);

      // If dragging from an input pin that already has a connection, disconnect it
      if (!isOutput) {
        // Find existing connection to this input
        for (ConnectionLine* conn : block->connections()) {
          if (conn->targetBlock() == block && conn->targetPinIndex() == pinIndex) {
            // Store this connection - we'll reconnect it or delete it
            disconnectingLine_ = conn;
            connectionSourceBlock_ = conn->sourceBlock();
            connectionSourcePin_ = conn->sourcePinIndex();
            connectionFromOutput_ = true;
            dragConnectionColor_ = connectionSourceBlock_->pinColor(false, connectionSourcePin_);

            // Hide the original connection while dragging
            conn->setVisible(false);
            break;
          }
        }
      }

      // Create temporary visual connection
      tempConnectionPath_ = new QGraphicsPathItem();
      tempConnectionPath_->setZValue(1000);  // Draw on top
      QPen pen(dragConnectionColor_, 2, Qt::DashLine);
      tempConnectionPath_->setPen(pen);
      scene_->addItem(tempConnectionPath_);

      updateTempConnection(scenePos);

      event->accept();
      return;
    }

    // Check if clicking on empty space - start rubber band selection
    QGraphicsItem* itemAtPos = scene_->itemAt(scenePos, transform());
    if (!itemAtPos || dynamic_cast<QGraphicsPathItem*>(itemAtPos)) {
      // Clicked on empty space or background - start rubber band
      // Clear current selection if not holding Shift or Ctrl
      if (!(event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier))) {
        scene_->clearSelection();
      }

      isRubberBandSelecting_ = true;
      rubberBandOrigin_ = event->pos();

      if (!rubberBand_) {
        rubberBand_ = new QRubberBand(QRubberBand::Rectangle, this);
      }
      rubberBand_->setGeometry(QRect(rubberBandOrigin_, QSize()));
      rubberBand_->show();

      event->accept();
      return;
    }
  }

  QGraphicsView::mousePressEvent(event);
}

void WeaverCanvas::mouseMoveEvent(QMouseEvent* event) {
  if (isPanning_) {
    QPoint delta = event->pos() - lastPanPoint_;
    lastPanPoint_ = event->pos();

    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());

    event->accept();
    return;
  }

  if (isDraggingConnection_) {
    QPointF scenePos = mapToScene(event->pos());
    updateTempConnection(scenePos);
    event->accept();
    return;
  }

  if (isRubberBandSelecting_ && rubberBand_) {
    QRect selectionRect = QRect(rubberBandOrigin_, event->pos()).normalized();
    rubberBand_->setGeometry(selectionRect);
    event->accept();
    return;
  }

  QGraphicsView::mouseMoveEvent(event);
}

void WeaverCanvas::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::MiddleButton && isPanning_) {
    isPanning_ = false;
    setCursor(Qt::ArrowCursor);
    event->accept();
    return;
  }

  if (event->button() == Qt::LeftButton && isDraggingConnection_) {
    QPointF scenePos = mapToScene(event->pos());

    PackageBlock* targetBlock = nullptr;
    int targetPin = -1;
    bool targetIsOutput = false;

    bool connectionMade = false;

    if (findPinAtPos(scenePos, targetBlock, targetPin, targetIsOutput)) {
      // Check if this is a valid connection (output -> input)
      bool validTarget = (connectionFromOutput_ && !targetIsOutput) ||
                         (!connectionFromOutput_ && targetIsOutput);

      if (validTarget && targetBlock != connectionSourceBlock_) {
        PackageBlock* srcBlock = connectionSourceBlock_;
        int srcPin = connectionSourcePin_;
        PackageBlock* dstBlock = targetBlock;
        int dstPin = targetPin;

        // Ensure we're connecting output -> input
        if (!connectionFromOutput_) {
          std::swap(srcBlock, dstBlock);
          std::swap(srcPin, dstPin);
        }

        // Create the connection
        ConnectionLine* newConn = createConnection(srcBlock, srcPin, dstBlock, dstPin);
        if (newConn) {
          connectionMade = true;

          // If we were disconnecting an existing line, delete it
          if (disconnectingLine_) {
            removeConnection(disconnectingLine_);
            disconnectingLine_ = nullptr;
          }
        }
      }
    }

    // If no connection was made and we were disconnecting, restore or delete original
    if (!connectionMade && disconnectingLine_) {
      // Check if we dropped on empty space - delete the connection
      if (!targetBlock) {
        removeConnection(disconnectingLine_);
      } else {
        // Restore the original connection
        disconnectingLine_->setVisible(true);
      }
      disconnectingLine_ = nullptr;
    }

    // Clean up temp visual
    if (tempConnectionPath_) {
      scene_->removeItem(tempConnectionPath_);
      delete tempConnectionPath_;
      tempConnectionPath_ = nullptr;
    }

    isDraggingConnection_ = false;
    connectionSourceBlock_ = nullptr;
    connectionSourcePin_ = -1;

    event->accept();
    return;
  }

  // Complete rubber band selection
  if (event->button() == Qt::LeftButton && isRubberBandSelecting_) {
    if (rubberBand_) {
      // Get the selection rectangle in scene coordinates
      QRect viewRect = rubberBand_->geometry();
      QRectF sceneRect = QRectF(mapToScene(viewRect.topLeft()),
                                 mapToScene(viewRect.bottomRight())).normalized();

      // Select all PackageBlock items within the rectangle
      QPainterPath selectionPath;
      selectionPath.addRect(sceneRect);

      // If Shift or Ctrl held, add to selection; otherwise replace
      bool addToSelection = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier);

      QList<QGraphicsItem*> itemsInRect = scene_->items(sceneRect, Qt::IntersectsItemShape);
      for (QGraphicsItem* item : itemsInRect) {
        if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
          if (addToSelection) {
            // Toggle selection if Ctrl held
            if (event->modifiers() & Qt::ControlModifier) {
              block->setSelected(!block->isSelected());
            } else {
              block->setSelected(true);
            }
          } else {
            block->setSelected(true);
          }
        }
      }

      rubberBand_->hide();
    }

    isRubberBandSelecting_ = false;
    event->accept();
    return;
  }

  QGraphicsView::mouseReleaseEvent(event);
}

void WeaverCanvas::keyPressEvent(QKeyEvent* event) {
  // Cancel connection drag with Escape
  if (event->key() == Qt::Key_Escape && isDraggingConnection_) {
    // Restore disconnected line if any
    if (disconnectingLine_) {
      disconnectingLine_->setVisible(true);
      disconnectingLine_ = nullptr;
    }

    // Clean up temp visual
    if (tempConnectionPath_) {
      scene_->removeItem(tempConnectionPath_);
      delete tempConnectionPath_;
      tempConnectionPath_ = nullptr;
    }

    isDraggingConnection_ = false;
    connectionSourceBlock_ = nullptr;
    connectionSourcePin_ = -1;

    event->accept();
    return;
  }

  // Cancel rubber band selection with Escape
  if (event->key() == Qt::Key_Escape && isRubberBandSelecting_) {
    if (rubberBand_) {
      rubberBand_->hide();
    }
    isRubberBandSelecting_ = false;
    event->accept();
    return;
  }

  // Space key enables pan mode
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    isSpacePressed_ = true;
    setCursor(Qt::OpenHandCursor);
    event->accept();
    return;
  }

  // Delete selected items
  if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
    deleteSelectedItems();
    event->accept();
    return;
  }

  // D - Duplicate selected items
  if (event->key() == Qt::Key_D && !event->modifiers()) {
    duplicateSelectedItems();
    event->accept();
    return;
  }

  // Ctrl+A - Select all items
  if (event->key() == Qt::Key_A && event->modifiers() == Qt::ControlModifier) {
    selectAllItems();
    event->accept();
    return;
  }

  // Ctrl+G - Group selected items
  if (event->key() == Qt::Key_G && event->modifiers() == Qt::ControlModifier) {
    QList<PackageBlock*> selectedBlocks;
    for (QGraphicsItem* item : scene_->selectedItems()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        selectedBlocks.append(block);
      }
    }
    if (selectedBlocks.size() >= 2) {
      createGroupFromSelection(tr("Group"));
    }
    event->accept();
    return;
  }

  // Ctrl+Shift+G - Ungroup selected groups
  if (event->key() == Qt::Key_G &&
      event->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier)) {
    ungroupSelectedGroups();
    event->accept();
    return;
  }

  // Escape to deselect all
  if (event->key() == Qt::Key_Escape) {
    scene_->clearSelection();
    event->accept();
    return;
  }

  QGraphicsView::keyPressEvent(event);
}

void WeaverCanvas::keyReleaseEvent(QKeyEvent* event) {
  // Space key release ends pan mode
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    isSpacePressed_ = false;
    if (!isPanning_) {
      setCursor(Qt::ArrowCursor);
    }
    event->accept();
    return;
  }

  QGraphicsView::keyReleaseEvent(event);
}

void WeaverCanvas::contextMenuEvent(QContextMenuEvent* event) {
  // Don't show context menu while dragging
  if (isDraggingConnection_) {
    event->accept();
    return;
  }

  QMenu menu(this);

  QPointF scenePos = mapToScene(event->pos());

  // Check if we clicked on an item
  QGraphicsItem* item = scene_->itemAt(scenePos, transform());

  if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
    menu.addAction(tr("Edit Package"), [block]() {
      // TODO: Open package editor
    });
    menu.addAction(tr("Expand/Collapse"), [block]() {
      block->setExpanded(!block->isExpanded());
    });

    menu.addSeparator();

    // VS Code integration
    menu.addAction(tr("Open in VS Code"), [this, block]() {
      emit openBlockInVSCodeRequested(block);
    });

    // Show Data Origin
    menu.addAction(tr("Show Data Origin..."), [this, block]() {
      BlockData blockData;
      blockData.id = block->id();
      blockData.name = block->packageName();
      blockData.position = block->pos();
      DataLineage lineage = globalLineageProvider().getBlockLineage(blockData);
      LineageContextMenu::showLineageDialog(lineage, this);
    });

    // Ask AI about this block
    menu.addAction(tr("Ask AI about this..."), [this, block]() {
      emit askAIAboutBlock(block);
    });

    menu.addSeparator();

    // Parameter source submenu
    QMenu* paramSourceMenu = menu.addMenu(tr("Parameter Source"));

    // Auto-detect option
    QAction* autoAction = paramSourceMenu->addAction(tr("Auto-detect"));
    autoAction->setCheckable(true);
    autoAction->setChecked(block->preferredYamlSource().isEmpty());
    connect(autoAction, &QAction::triggered, this, [this, block]() {
      block->setPreferredYamlSource(QString());
      emit blockYamlSourceChanged(block, QString());
    });

    // Block Parameters option
    QAction* blockParamsAction = paramSourceMenu->addAction(tr("Block Parameters"));
    blockParamsAction->setCheckable(true);
    blockParamsAction->setChecked(block->preferredYamlSource() == "block");
    connect(blockParamsAction, &QAction::triggered, this, [this, block]() {
      block->setPreferredYamlSource("block");
      emit blockYamlSourceChanged(block, "block");
    });

    // Add available YAML files
    if (!availableYamlFiles_.isEmpty()) {
      paramSourceMenu->addSeparator();
      for (const QString& yamlFile : availableYamlFiles_) {
        QFileInfo fileInfo(yamlFile);
        QAction* yamlAction = paramSourceMenu->addAction(fileInfo.fileName());
        yamlAction->setCheckable(true);
        yamlAction->setChecked(block->preferredYamlSource() == yamlFile);
        connect(yamlAction, &QAction::triggered, this, [this, block, yamlFile]() {
          block->setPreferredYamlSource(yamlFile);
          emit blockYamlSourceChanged(block, yamlFile);
        });
      }
    }

    menu.addSeparator();

    // Check if there are multiple items selected
    QList<QGraphicsItem*> selected = scene_->selectedItems();
    int selectedBlockCount = 0;
    for (QGraphicsItem* sel : selected) {
      if (dynamic_cast<PackageBlock*>(sel)) selectedBlockCount++;
    }

    // Group creation option when one or more blocks selected
    if (selectedBlockCount >= 1) {
      menu.addAction(tr("Create Group from Selection"), [this]() {
        bool ok;
        QString title = QInputDialog::getText(this, tr("Create Node Group"),
                                               tr("Group Title:"), QLineEdit::Normal,
                                               tr("Group"), &ok);
        if (ok && !title.isEmpty()) {
          createGroupFromSelection(title);
        }
      });
      menu.addSeparator();
    }

    if (selectedBlockCount > 1) {
      menu.addAction(tr("Delete Selected (%1)").arg(selectedBlockCount), [this]() {
        deleteSelectedItems();
      });
    } else {
      menu.addAction(tr("Delete"), [this, block]() {
        removePackageBlock(block);
      });
    }
  } else if (NodeGroup* group = dynamic_cast<NodeGroup*>(item)) {
    menu.addAction(tr("Edit Group Title"), [group]() {
      bool ok;
      QString newTitle = QInputDialog::getText(nullptr, tr("Edit Group Title"),
                                                tr("Title:"), QLineEdit::Normal,
                                                group->title(), &ok);
      if (ok && !newTitle.isEmpty()) {
        group->setTitle(newTitle);
      }
    });

    // Color options
    QMenu* colorMenu = menu.addMenu(tr("Group Color"));
    colorMenu->addAction(tr("Blue"), [group]() {
      group->setColor(QColor(80, 120, 180, 180));
    });
    colorMenu->addAction(tr("Green"), [group]() {
      group->setColor(QColor(80, 160, 100, 180));
    });
    colorMenu->addAction(tr("Red"), [group]() {
      group->setColor(QColor(180, 80, 80, 180));
    });
    colorMenu->addAction(tr("Orange"), [group]() {
      group->setColor(QColor(200, 140, 60, 180));
    });
    colorMenu->addAction(tr("Purple"), [group]() {
      group->setColor(QColor(140, 80, 180, 180));
    });

    menu.addAction(tr("Fit to Contents"), [group]() {
      group->fitToContents(30.0);
    });

    menu.addSeparator();

    // Ask AI about this group
    menu.addAction(tr("Ask AI about this..."), [this, group]() {
      emit askAIAboutGroup(group);
    });

    menu.addSeparator();
    menu.addAction(tr("Delete Group"), [this, group]() {
      removeNodeGroup(group);
    });
  } else if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
    // Show Data Origin for connection
    menu.addAction(tr("Show Data Origin..."), [this, conn]() {
      QString topicName = conn->topicName();
      QString messageType = conn->messageType();
      bool isLive = conn->activityState() != TopicActivityState::Unknown;
      DataLineage lineage = globalLineageProvider().getConnectionLineage(topicName, messageType, isLive);
      LineageContextMenu::showLineageDialog(lineage, this);
    });

    // Ask AI about this connection/topic
    menu.addAction(tr("Ask AI about this..."), [this, conn]() {
      emit askAIAboutConnection(conn);
    });

    menu.addSeparator();
    menu.addAction(tr("Delete Connection"), [this, conn]() {
      removeConnection(conn);
    });
  } else {
    // Canvas context menu - Basic nodes
    QMenu* basicMenu = menu.addMenu(tr("Basic Nodes"));
    basicMenu->addAction(tr("Publisher"), [this, scenePos]() {
      addPackageBlock("publisher_node", scenePos);
    });
    basicMenu->addAction(tr("Subscriber"), [this, scenePos]() {
      addPackageBlock("subscriber_node", scenePos);
    });
    basicMenu->addAction(tr("Relay"), [this, scenePos]() {
      addPackageBlock("relay_node", scenePos);
    });

    // Services
    QMenu* serviceMenu = menu.addMenu(tr("Services"));
    serviceMenu->addAction(tr("Service Server"), [this, scenePos]() {
      addPackageBlock("service_server", scenePos);
    });
    serviceMenu->addAction(tr("Service Client"), [this, scenePos]() {
      addPackageBlock("service_client", scenePos);
    });

    // Actions
    QMenu* actionMenu = menu.addMenu(tr("Actions"));
    actionMenu->addAction(tr("Action Server"), [this, scenePos]() {
      addPackageBlock("action_server", scenePos);
    });
    actionMenu->addAction(tr("Action Client"), [this, scenePos]() {
      addPackageBlock("action_client", scenePos);
    });

    // Example complex nodes
    QMenu* examplesMenu = menu.addMenu(tr("Examples (Multi-Pin)"));
    examplesMenu->addAction(tr("Sensor Fusion (3 in, 1 out)"), [this, scenePos]() {
      addPackageBlock("sensor_fusion", scenePos);
    });
    examplesMenu->addAction(tr("Robot Controller (3 in, 3 out)"), [this, scenePos]() {
      addPackageBlock("robot_controller", scenePos);
    });
    examplesMenu->addAction(tr("Image Processor (2 in, 2 out)"), [this, scenePos]() {
      addPackageBlock("image_processor", scenePos);
    });

    menu.addSeparator();
    menu.addAction(tr("Add Custom Package..."), [this, scenePos]() {
      // TODO: Show package creation dialog
      addPackageBlock("custom_package", scenePos);
    });
  }

  menu.exec(event->globalPos());
}

void WeaverCanvas::dragEnterEvent(QDragEnterEvent* event) {
  if (event->mimeData()->hasFormat("application/x-ros-package")) {
    event->acceptProposedAction();
  } else if (event->mimeData()->hasText()) {
    event->acceptProposedAction();
  }
}

void WeaverCanvas::dragMoveEvent(QDragMoveEvent* event) {
  if (event->mimeData()->hasFormat("application/x-ros-package")) {
    event->acceptProposedAction();
  } else if (event->mimeData()->hasText()) {
    event->acceptProposedAction();
  }
}

void WeaverCanvas::dropEvent(QDropEvent* event) {
  QPointF scenePos = mapToScene(event->pos());

  QString packageName;
  if (event->mimeData()->hasFormat("application/x-ros-package")) {
    packageName = QString::fromUtf8(event->mimeData()->data("application/x-ros-package"));
  } else if (event->mimeData()->hasText()) {
    packageName = event->mimeData()->text();
  }

  if (!packageName.isEmpty()) {
    addPackageBlock(packageName, scenePos);
    event->acceptProposedAction();
  }
}

PackageBlock* WeaverCanvas::addCustomBlock(const QString& name, const QPointF& pos,
                                            const QList<QPair<QString, QString>>& inputPins,
                                            const QList<QPair<QString, QString>>& outputPins) {
  PackageBlock* block = new PackageBlock(name);
  block->setPos(pos);
  scene_->addItem(block);

  // Connect signals for pin hover highlighting
  connectBlockSignals(block);

  // Add custom input pins
  for (const auto& pin : inputPins) {
    // Determine data type from pin type string
    Pin::DataType dataType = Pin::DataType::Topic;
    if (pin.first == "service") {
      dataType = Pin::DataType::Service;
    } else if (pin.first == "action") {
      dataType = Pin::DataType::Action;
    } else if (pin.first == "parameter") {
      dataType = Pin::DataType::Parameter;
    }
    block->addInputPin(pin.second, dataType, "");
  }

  // Add custom output pins
  for (const auto& pin : outputPins) {
    Pin::DataType dataType = Pin::DataType::Topic;
    if (pin.first == "service") {
      dataType = Pin::DataType::Service;
    } else if (pin.first == "action") {
      dataType = Pin::DataType::Action;
    } else if (pin.first == "parameter") {
      dataType = Pin::DataType::Parameter;
    }
    block->addOutputPin(pin.second, dataType, "");
  }

  return block;
}

void WeaverCanvas::exportToProject(Project& project) const {
  // Clear existing project data
  project.blocks().clear();
  project.connections().clear();
  project.nodeGroups().clear();

  // Export all blocks
  for (QGraphicsItem* item : scene_->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      BlockData blockData;
      blockData.id = block->id();
      blockData.name = block->packageName();
      blockData.position = block->pos();

      // Export input pins
      for (const Pin& pin : block->inputPins()) {
        PinData pinData;
        pinData.name = pin.name;
        pinData.type = "input";
        switch (pin.dataType) {
          case Pin::DataType::Topic: pinData.dataType = "topic"; break;
          case Pin::DataType::Service: pinData.dataType = "service"; break;
          case Pin::DataType::Action: pinData.dataType = "action"; break;
          case Pin::DataType::Parameter: pinData.dataType = "parameter"; break;
        }
        pinData.messageType = pin.messageType;
        blockData.inputPins.append(pinData);
      }

      // Export output pins
      for (const Pin& pin : block->outputPins()) {
        PinData pinData;
        pinData.name = pin.name;
        pinData.type = "output";
        switch (pin.dataType) {
          case Pin::DataType::Topic: pinData.dataType = "topic"; break;
          case Pin::DataType::Service: pinData.dataType = "service"; break;
          case Pin::DataType::Action: pinData.dataType = "action"; break;
          case Pin::DataType::Parameter: pinData.dataType = "parameter"; break;
        }
        pinData.messageType = pin.messageType;
        blockData.outputPins.append(pinData);
      }

      // Export parameters
      blockData.parameters = block->parameters();

      // Export preferred YAML source
      blockData.preferredYamlSource = block->preferredYamlSource();

      project.addBlock(blockData);
    }
  }

  // Export all connections (avoiding duplicates)
  QSet<ConnectionLine*> exportedConnections;
  for (QGraphicsItem* item : scene_->items()) {
    if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      if (exportedConnections.contains(conn)) continue;
      exportedConnections.insert(conn);

      if (conn->sourceBlock() && conn->targetBlock()) {
        ConnectionData connData;
        connData.id = QUuid::createUuid();
        connData.sourceBlockId = conn->sourceBlock()->id();
        connData.sourcePinIndex = conn->sourcePinIndex();
        connData.targetBlockId = conn->targetBlock()->id();
        connData.targetPinIndex = conn->targetPinIndex();

        project.addConnection(connData);
      }
    }
  }

  // Export all node groups
  for (NodeGroup* group : nodeGroups_) {
    NodeGroupData groupData;
    groupData.id = group->id();
    groupData.title = group->title();
    groupData.position = group->pos();
    groupData.size = group->size();
    groupData.color = group->color();

    // Get IDs of contained nodes
    for (PackageBlock* node : group->containedNodes()) {
      groupData.containedNodeIds.append(node->id());
    }

    project.addNodeGroup(groupData);
  }
}

void WeaverCanvas::importFromProject(const Project& project) {
  // Clear existing canvas
  clearCanvas();

  // Map from block IDs to created PackageBlock pointers
  QMap<QUuid, PackageBlock*> blockMap;

  // Import blocks
  for (const BlockData& blockData : project.blocks()) {
    // Create list of input pins as type/name pairs
    QList<QPair<QString, QString>> inputPins;
    for (const PinData& pin : blockData.inputPins) {
      inputPins.append(qMakePair(pin.dataType, pin.name));
    }

    // Create list of output pins as type/name pairs
    QList<QPair<QString, QString>> outputPins;
    for (const PinData& pin : blockData.outputPins) {
      outputPins.append(qMakePair(pin.dataType, pin.name));
    }

    // Create block with custom pins
    PackageBlock* block = addCustomBlock(blockData.name, blockData.position,
                                          inputPins, outputPins);

    // Load parameters if present
    if (!blockData.parameters.isEmpty()) {
      RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
                  "importFromProject: Setting %d parameters for block '%s'",
                  static_cast<int>(blockData.parameters.size()),
                  blockData.name.toStdString().c_str());
      block->setParameters(blockData.parameters);
    }

    // Load preferred YAML source
    if (!blockData.preferredYamlSource.isEmpty()) {
      block->setPreferredYamlSource(blockData.preferredYamlSource);
    }

    blockMap[blockData.id] = block;
  }

  // Import connections
  for (const ConnectionData& connData : project.connections()) {
    PackageBlock* sourceBlock = blockMap.value(connData.sourceBlockId);
    PackageBlock* targetBlock = blockMap.value(connData.targetBlockId);

    if (sourceBlock && targetBlock) {
      createConnection(sourceBlock, connData.sourcePinIndex,
                       targetBlock, connData.targetPinIndex);
    }
  }

  // Import node groups
  for (const NodeGroupData& groupData : project.nodeGroups()) {
    NodeGroup* group = new NodeGroup(groupData.title);
    group->setPos(groupData.position);
    group->setSize(groupData.size);
    group->setColor(groupData.color);

    // Add contained nodes
    for (const QUuid& nodeId : groupData.containedNodeIds) {
      PackageBlock* block = blockMap.value(nodeId);
      if (block) {
        group->addNode(block);
      }
    }

    scene_->addItem(group);
    nodeGroups_.append(group);
  }

  // Fit view to show all imported content
  fitToContents();
}

void WeaverCanvas::setAvailableYamlFiles(const QStringList& yamlFiles) {
  availableYamlFiles_ = yamlFiles;
}

void WeaverCanvas::updateZoomIndicator() {
  if (!zoomIndicator_) return;

  int zoomPercent = static_cast<int>(std::round(zoomFactor_ * 100));
  zoomIndicator_->setText(QString("%1%").arg(zoomPercent));
  zoomIndicator_->adjustSize();
  positionZoomIndicator();
}

void WeaverCanvas::positionZoomIndicator() {
  if (!zoomIndicator_) return;

  // Position in the bottom right corner with a small margin
  int margin = 8;
  int x = width() - zoomIndicator_->width() - margin;
  int y = height() - zoomIndicator_->height() - margin;
  zoomIndicator_->move(x, y);
}

void WeaverCanvas::resizeEvent(QResizeEvent* event) {
  QGraphicsView::resizeEvent(event);
  positionZoomIndicator();
}

}  // namespace ros_weaver
