#ifndef ROS_WEAVER_CANVAS_NODE_GROUP_HPP
#define ROS_WEAVER_CANVAS_NODE_GROUP_HPP

#include <QGraphicsObject>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QString>
#include <QList>
#include <QUuid>
#include <QColor>

namespace ros_weaver {

class PackageBlock;

/**
 * NodeGroup - A visual grouping container for nodes (similar to Unreal Engine's Comment boxes)
 *
 * Features:
 * - Resizable rectangle that visually groups nodes
 * - Editable title/label displayed at the top
 * - Moving the group moves all contained nodes
 * - Customizable color for organization
 * - Renders behind nodes (lower Z-order)
 */
class NodeGroup : public QGraphicsObject {
  Q_OBJECT

public:
  explicit NodeGroup(const QString& title = "Group", QGraphicsItem* parent = nullptr);
  ~NodeGroup() override;

  // QGraphicsItem interface
  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  // Group properties
  QString title() const { return title_; }
  void setTitle(const QString& title);

  QColor color() const { return color_; }
  void setColor(const QColor& color);

  QUuid id() const { return id_; }

  // Size management
  QSizeF size() const { return size_; }
  void setSize(const QSizeF& size);
  void setSize(qreal width, qreal height) { setSize(QSizeF(width, height)); }

  // Resize the group to fit all contained nodes with padding
  void fitToContents(qreal padding = 30.0);

  // Node containment
  void addNode(PackageBlock* node);
  void removeNode(PackageBlock* node);
  void clearNodes();
  QList<PackageBlock*> containedNodes() const { return containedNodes_; }

  // Check if a node is within the group bounds
  bool containsNode(PackageBlock* node) const;

  // Update contained nodes list based on what's inside the bounds
  void updateContainedNodes();

  // Selection state
  bool isGroupSelected() const { return isSelected_; }
  void setGroupSelected(bool selected);

signals:
  void titleChanged(const QString& newTitle);
  void groupMoved(const QPointF& delta);
  void nodesChanged();

protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  enum class ResizeHandle {
    None,
    TopLeft, Top, TopRight,
    Left, Right,
    BottomLeft, Bottom, BottomRight
  };

  ResizeHandle hitTestResizeHandle(const QPointF& pos) const;
  QCursor cursorForHandle(ResizeHandle handle) const;
  void applyResize(const QPointF& delta);

  QUuid id_;
  QString title_;
  QColor color_;
  QSizeF size_;
  bool isSelected_;

  // Contained nodes
  QList<PackageBlock*> containedNodes_;

  // Interaction state
  bool isDragging_;
  bool isResizing_;
  ResizeHandle activeHandle_;
  QPointF lastMousePos_;
  QPointF dragStartPos_;

  // Visual constants
  static constexpr qreal HEADER_HEIGHT = 28.0;
  static constexpr qreal MIN_WIDTH = 150.0;
  static constexpr qreal MIN_HEIGHT = 100.0;
  static constexpr qreal RESIZE_HANDLE_SIZE = 10.0;
  static constexpr qreal CORNER_RADIUS = 8.0;
  static constexpr qreal BORDER_WIDTH = 2.0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_NODE_GROUP_HPP
