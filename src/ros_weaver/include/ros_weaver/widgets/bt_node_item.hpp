#ifndef ROS_WEAVER_BT_NODE_ITEM_HPP
#define ROS_WEAVER_BT_NODE_ITEM_HPP

#include <QGraphicsItem>
#include <QGraphicsTextItem>
#include <memory>
#include "ros_weaver/core/behavior_tree_parser.hpp"

namespace ros_weaver {

/**
 * @brief Execution state for live visualization
 */
enum class BTExecutionState {
  Idle,       // Not running
  Running,    // Currently executing
  Success,    // Completed successfully
  Failure,    // Completed with failure
  Skipped     // Was skipped (e.g., in fallback after success)
};

/**
 * @brief Graphics item representing a behavior tree node
 */
class BTNodeItem : public QGraphicsItem {
public:
  static constexpr qreal NODE_WIDTH = 150.0;
  static constexpr qreal NODE_HEIGHT = 60.0;
  static constexpr qreal CORNER_RADIUS = 8.0;

  explicit BTNodeItem(std::shared_ptr<BTNode> node, QGraphicsItem* parent = nullptr);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  /**
   * @brief Get the associated BT node data
   */
  std::shared_ptr<BTNode> node() const { return node_; }

  /**
   * @brief Set the execution state for live visualization
   */
  void setExecutionState(BTExecutionState state);
  BTExecutionState executionState() const { return executionState_; }

  /**
   * @brief Get connection points for drawing edges
   */
  QPointF topConnectionPoint() const;
  QPointF bottomConnectionPoint() const;

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  QString createTooltip() const;
  std::shared_ptr<BTNode> node_;
  BTExecutionState executionState_ = BTExecutionState::Idle;
  bool isHovered_ = false;

  void drawNodeShape(QPainter* painter);
  void drawNodeContent(QPainter* painter);
  void drawExecutionIndicator(QPainter* painter);
  QColor getBackgroundColor() const;
  QColor getBorderColor() const;
};

/**
 * @brief Graphics item for edges between BT nodes
 */
class BTEdgeItem : public QGraphicsItem {
public:
  BTEdgeItem(BTNodeItem* parent, BTNodeItem* child, QGraphicsItem* parentItem = nullptr);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  void updatePositions();

private:
  BTNodeItem* parentNode_;
  BTNodeItem* childNode_;
  QPainterPath path_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_BT_NODE_ITEM_HPP
