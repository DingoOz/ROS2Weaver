#ifndef ROS_WEAVER_CANVAS_PACKAGE_BLOCK_HPP
#define ROS_WEAVER_CANVAS_PACKAGE_BLOCK_HPP

#include <QGraphicsObject>
#include <QGraphicsSceneHoverEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QString>
#include <QList>
#include <QUuid>
#include <QVariant>
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/widgets/remapping_editor.hpp"

namespace ros_weaver {

class ConnectionLine;

// Runtime status for a block
enum class BlockRuntimeStatus {
  Unknown,      // Not yet scanned
  Running,      // Matched with high/medium confidence
  PartialMatch, // Matched with low confidence
  NotFound      // Not found in running system
};

// Represents a connection pin on a package block
struct Pin {
  enum class Type { Input, Output };
  enum class DataType { Topic, Service, Action, Parameter };

  QString name;
  Type type;
  DataType dataType;
  QString messageType;  // e.g., "std_msgs/String"
  QPointF localPos;     // Position relative to block

  Pin(const QString& n, Type t, DataType dt, const QString& msgType = "")
    : name(n), type(t), dataType(dt), messageType(msgType) {}
};

class PackageBlock : public QGraphicsObject {
  Q_OBJECT

public:
  explicit PackageBlock(const QString& packageName, QGraphicsItem* parent = nullptr);
  ~PackageBlock() override;

  // QGraphicsItem interface
  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  // Package info
  QString packageName() const { return packageName_; }
  void setPackageName(const QString& name);

  QUuid id() const { return id_; }
  void setId(const QUuid& id) { id_ = id; }  // For undo/redo support

  // Position when drag started (for undo/redo)
  QPointF moveStartPos() const { return moveStartPos_; }

  // Pin management
  void addInputPin(const QString& name, Pin::DataType dataType,
                   const QString& messageType = "");
  void addOutputPin(const QString& name, Pin::DataType dataType,
                    const QString& messageType = "");
  const QList<Pin>& inputPins() const { return inputPins_; }
  const QList<Pin>& outputPins() const { return outputPins_; }

  // Get pin position in scene coordinates
  QPointF inputPinScenePos(int index) const;
  QPointF outputPinScenePos(int index) const;

  // Pin hit testing - returns pin index or -1 if no hit
  int inputPinAtPos(const QPointF& localPos) const;
  int outputPinAtPos(const QPointF& localPos) const;

  // Get pin color for a specific pin
  QColor pinColor(bool isInput, int index) const;

  // Check if connection types are compatible
  static bool canConnect(const Pin& output, const Pin& input);

  // Connection management
  void addConnection(ConnectionLine* connection);
  void removeConnection(ConnectionLine* connection);
  const QList<ConnectionLine*>& connections() const { return connections_; }

  // Get connections for a specific pin
  QList<ConnectionLine*> connectionsForPin(int pinIndex, bool isOutput) const;

  // Selection state
  bool isBlockSelected() const { return isSelected_; }
  void setBlockSelected(bool selected);

  // Expanded/collapsed state for showing node details
  bool isExpanded() const { return isExpanded_; }
  void setExpanded(bool expanded);

  // Hover state for pins
  void setHoveredPin(int pinIndex, bool isOutput);
  void clearHoveredPin();
  int hoveredPinIndex() const { return hoveredPinIndex_; }
  bool hoveredPinIsOutput() const { return hoveredPinIsOutput_; }

  // Parameters storage
  void setParameters(const QList<BlockParamData>& params);
  QList<BlockParamData> parameters() const { return parameters_; }
  bool hasParameters() const { return !parameters_.isEmpty(); }

  // Preferred YAML source for parameters
  // Empty = auto-detect, "block" = use block params, path = specific YAML file
  void setPreferredYamlSource(const QString& source);
  QString preferredYamlSource() const { return preferredYamlSource_; }

  // Runtime status (from system discovery)
  void setRuntimeStatus(BlockRuntimeStatus status);
  BlockRuntimeStatus runtimeStatus() const { return runtimeStatus_; }

  // Matched ROS2 node name (from system discovery)
  void setMatchedNodeName(const QString& nodeName);
  QString matchedNodeName() const { return matchedNodeName_; }

  // Match confidence (from system discovery)
  void setMatchConfidence(MatchConfidence confidence);
  MatchConfidence matchConfidence() const { return matchConfidence_; }

  // Update mapping result from system discovery
  void updateMappingResult(const BlockMappingResult& result);

  // Clear runtime status (reset to unknown)
  void clearRuntimeStatus();

  // Show/hide runtime status overlay
  void setShowRuntimeStatus(bool show);
  bool showRuntimeStatus() const { return showRuntimeStatus_; }

  // Health overlay color (for node health dashboard heatmap)
  void setHealthOverlayColor(const QColor& color);
  QColor healthOverlayColor() const { return healthOverlayColor_; }
  bool hasHealthOverlay() const { return healthOverlayColor_.isValid(); }

  // Namespace management
  void setNodeNamespace(const QString& ns);
  QString nodeNamespace() const { return namespace_; }
  bool hasNamespace() const { return !namespace_.isEmpty(); }
  QString fullyQualifiedName() const;  // Returns namespace/nodename

  // Remapping management
  void setRemappings(const QList<Remapping>& remappings);
  QList<Remapping> remappings() const { return remappings_; }
  void addRemapping(const Remapping& remapping);
  void removeRemapping(int index);
  void clearRemappings();
  bool hasRemappings() const { return !remappings_.isEmpty(); }

  // Get remapped name for a topic/service (returns original if not remapped)
  QString getRemappedName(const QString& originalName, const QString& type = "topic") const;
  QString getOriginalName(const QString& remappedName, const QString& type = "topic") const;

signals:
  void pinHovered(PackageBlock* block, int pinIndex, bool isOutput);
  void pinUnhovered(PackageBlock* block);
  void blockMoved(PackageBlock* block);
  void blockMoveFinished(PackageBlock* block);

protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  void updatePinPositions();
  void drawPin(QPainter* painter, const Pin& pin, bool isInput, int pinIndex);
  QColor getDataTypeColor(Pin::DataType dataType) const;

  QUuid id_;
  QString packageName_;
  QList<Pin> inputPins_;
  QList<Pin> outputPins_;
  QList<ConnectionLine*> connections_;

  bool isSelected_;
  bool isExpanded_;

  // Hover state
  int hoveredPinIndex_;
  bool hoveredPinIsOutput_;

  // Drag state
  bool isDragging_;
  QPointF moveStartPos_;  // Position when drag started

  // Parameters
  QList<BlockParamData> parameters_;

  // Preferred YAML source (empty = auto, "block" = block params, path = YAML file)
  QString preferredYamlSource_;

  // Runtime status from system discovery
  BlockRuntimeStatus runtimeStatus_;
  QString matchedNodeName_;
  MatchConfidence matchConfidence_;
  bool showRuntimeStatus_;

  // Health overlay for heatmap visualization
  QColor healthOverlayColor_;

  // Namespace and remappings
  QString namespace_;
  QList<Remapping> remappings_;

  // Visual dimensions
  static constexpr qreal BLOCK_WIDTH = 180.0;
  static constexpr qreal HEADER_HEIGHT = 30.0;
  static constexpr qreal PIN_HEIGHT = 20.0;
  static constexpr qreal PIN_RADIUS = 6.0;
  static constexpr qreal CORNER_RADIUS = 8.0;
  static constexpr qreal MIN_HEIGHT = 60.0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_PACKAGE_BLOCK_HPP
