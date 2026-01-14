#ifndef ROS_WEAVER_WAYPOINT_GRAPHICS_ITEM_HPP
#define ROS_WEAVER_WAYPOINT_GRAPHICS_ITEM_HPP

#include <QGraphicsItem>
#include <QGraphicsObject>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneHoverEvent>

#include "ros_weaver/core/mission_data.hpp"

namespace ros_weaver {

/**
 * @brief Visual representation of a waypoint on the map
 *
 * Displays a numbered waypoint with orientation arrow and tolerance circle.
 * Supports dragging to move and orientation handle for rotation.
 */
class WaypointGraphicsItem : public QGraphicsObject {
  Q_OBJECT

public:
  explicit WaypointGraphicsItem(const Waypoint& waypoint, QGraphicsItem* parent = nullptr);

  void setWaypoint(const Waypoint& waypoint);
  Waypoint waypoint() const { return waypoint_; }
  int waypointId() const { return waypoint_.id; }

  void setSelected(bool selected);
  void setSequenceNumber(int number);
  void setShowOrientation(bool show);
  void setShowTolerance(bool show);

  // QGraphicsItem interface
  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

signals:
  void waypointMoved(int waypointId, const QPointF& newPosition);
  void waypointDragStarted(int waypointId, const QPointF& startPosition);
  void waypointDragging(int waypointId, const QPointF& currentPosition);
  void waypointOrientationChanged(int waypointId, double theta);
  void waypointClicked(int waypointId, Qt::KeyboardModifiers modifiers);
  void waypointDoubleClicked(int waypointId);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  void drawOrientationArrow(QPainter* painter);
  void drawToleranceCircle(QPainter* painter);
  bool isOnOrientationHandle(const QPointF& pos) const;
  QPointF orientationHandlePos() const;

  Waypoint waypoint_;
  int sequenceNumber_ = 0;
  bool isSelected_ = false;
  bool isHovered_ = false;
  bool isDragging_ = false;
  bool isDraggingOrientation_ = false;
  bool showOrientation_ = true;
  bool showTolerance_ = true;

  QPointF dragStartPos_;
  QPointF dragStartScenePos_;  // Scene position at drag start for zoom-independent movement
  double dragStartTheta_ = 0.0;

  // Visual constants
  static constexpr double WAYPOINT_RADIUS = 14.0;
  static constexpr double ORIENTATION_ARROW_LENGTH = 28.0;
  static constexpr double ORIENTATION_HANDLE_RADIUS = 7.0;
  static constexpr double TOLERANCE_CIRCLE_OPACITY = 0.2;
};

/**
 * @brief Graphics item for robot start pose
 */
class RobotStartPoseItem : public QGraphicsObject {
  Q_OBJECT

public:
  explicit RobotStartPoseItem(QGraphicsItem* parent = nullptr);

  void setPose(const RobotStartPose& pose);
  RobotStartPose pose() const { return pose_; }

  void setSelected(bool selected);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

signals:
  void poseMoved(const QPointF& newPosition);
  void poseOrientationChanged(double theta);
  void poseClicked();

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  bool isOnOrientationHandle(const QPointF& pos) const;
  QPointF orientationHandlePos() const;

  RobotStartPose pose_;
  bool isSelected_ = false;
  bool isHovered_ = false;
  bool isDragging_ = false;
  bool isDraggingOrientation_ = false;
  QPointF dragStartPos_;
  QPointF dragStartScenePos_;  // Scene position at drag start for zoom-independent movement
  double dragStartTheta_ = 0.0;

  static constexpr double ROBOT_SIZE = 20.0;
  static constexpr double ORIENTATION_ARROW_LENGTH = 30.0;
};

/**
 * @brief Overlay for scale calibration
 */
class ScaleCalibrationOverlay : public QGraphicsObject {
  Q_OBJECT

public:
  explicit ScaleCalibrationOverlay(QGraphicsItem* parent = nullptr);

  void setFirstPoint(const QPointF& point);
  void setSecondPoint(const QPointF& point);
  void setMeasuring(bool measuring);
  void clear();

  QPointF firstPoint() const { return point1_; }
  QPointF secondPoint() const { return point2_; }
  double pixelDistance() const;
  bool isComplete() const { return hasPoint1_ && hasPoint2_; }

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

signals:
  void calibrationComplete(const QPointF& p1, const QPointF& p2);

private:
  QPointF point1_;
  QPointF point2_;
  bool hasPoint1_ = false;
  bool hasPoint2_ = false;
  bool isMeasuring_ = false;
};

/**
 * @brief Path line connecting waypoints
 */
class WaypointPathItem : public QGraphicsItem {
public:
  explicit WaypointPathItem(QGraphicsItem* parent = nullptr);

  void setPath(const QList<QPointF>& points);
  void setShowArrows(bool show);
  void setLooping(bool looping);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

private:
  QList<QPointF> pathPoints_;
  bool showArrows_ = true;
  bool isLooping_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WAYPOINT_GRAPHICS_ITEM_HPP
