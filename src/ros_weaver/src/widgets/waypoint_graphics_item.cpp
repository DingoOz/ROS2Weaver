#include "ros_weaver/widgets/waypoint_graphics_item.hpp"
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QCursor>
#include <cmath>

namespace ros_weaver {

// WaypointGraphicsItem implementation

WaypointGraphicsItem::WaypointGraphicsItem(const Waypoint& waypoint, QGraphicsItem* parent)
    : QGraphicsObject(parent), waypoint_(waypoint) {
  // Don't use Qt's built-in movement - we handle dragging ourselves for proper
  // zoom handling and multi-waypoint support
  setFlag(QGraphicsItem::ItemIsMovable, false);
  setFlag(QGraphicsItem::ItemIsSelectable, false);
  setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
  setAcceptHoverEvents(true);
  setCursor(Qt::PointingHandCursor);
}

void WaypointGraphicsItem::setWaypoint(const Waypoint& waypoint) {
  waypoint_ = waypoint;
  update();
}

void WaypointGraphicsItem::setSelected(bool selected) {
  isSelected_ = selected;
  update();
}

void WaypointGraphicsItem::setSequenceNumber(int number) {
  sequenceNumber_ = number;
  update();
}

void WaypointGraphicsItem::setShowOrientation(bool show) {
  showOrientation_ = show;
  update();
}

void WaypointGraphicsItem::setShowTolerance(bool show) {
  showTolerance_ = show;
  update();
}

QRectF WaypointGraphicsItem::boundingRect() const {
  double extent = WAYPOINT_RADIUS + ORIENTATION_ARROW_LENGTH + 5.0;
  return QRectF(-extent, -extent, extent * 2, extent * 2);
}

void WaypointGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
  painter->setRenderHint(QPainter::Antialiasing);

  // Draw tolerance circle if enabled
  if (showTolerance_) {
    drawToleranceCircle(painter);
  }

  // Draw orientation arrow if enabled
  if (showOrientation_) {
    drawOrientationArrow(painter);
  }

  // Draw waypoint circle with visual feedback for different states
  QColor baseColor = waypoint_.color;

  // Visual feedback: dragging state takes priority
  if (isDragging_) {
    // Bright orange/yellow color when dragging
    baseColor = QColor(255, 165, 0);  // Orange
    painter->setPen(QPen(QColor(255, 100, 0), 4));  // Thick orange-red border
  } else if (isSelected_) {
    painter->setPen(QPen(Qt::yellow, 3));
  } else if (isHovered_) {
    baseColor = baseColor.lighter(120);
    painter->setPen(QPen(baseColor.darker(120), 2));
  } else {
    painter->setPen(QPen(baseColor.darker(120), 2));
  }

  painter->setBrush(baseColor);

  // Draw slightly larger circle when dragging for emphasis
  double radius = isDragging_ ? WAYPOINT_RADIUS * 1.15 : WAYPOINT_RADIUS;
  painter->drawEllipse(QPointF(0, 0), radius, radius);

  // Draw sequence number
  painter->setPen(Qt::white);
  QFont font = painter->font();
  font.setBold(true);
  font.setPointSize(10);
  painter->setFont(font);

  QString label = sequenceNumber_ > 0 ? QString::number(sequenceNumber_) : waypoint_.name;
  if (label.length() > 3) {
    label = label.left(3);
  }
  QRectF textRect(-WAYPOINT_RADIUS, -WAYPOINT_RADIUS, WAYPOINT_RADIUS * 2, WAYPOINT_RADIUS * 2);
  painter->drawText(textRect, Qt::AlignCenter, label);

  // Draw name label below
  if (!waypoint_.name.isEmpty() && sequenceNumber_ > 0) {
    painter->setPen(Qt::black);
    font.setPointSize(8);
    font.setBold(false);
    painter->setFont(font);
    QRectF nameRect(-50, WAYPOINT_RADIUS + 2, 100, 16);
    painter->drawText(nameRect, Qt::AlignCenter, waypoint_.name);
  }
}

void WaypointGraphicsItem::drawOrientationArrow(QPainter* painter) {
  double endX = ORIENTATION_ARROW_LENGTH * std::cos(waypoint_.theta);
  double endY = -ORIENTATION_ARROW_LENGTH * std::sin(waypoint_.theta);  // Qt Y is inverted

  // Draw arrow line
  painter->setPen(QPen(waypoint_.color.darker(110), 2));
  painter->drawLine(QPointF(0, 0), QPointF(endX, endY));

  // Draw arrowhead
  double arrowSize = 8.0;
  double angle = std::atan2(-endY, endX);
  QPointF arrowP1 = QPointF(endX, endY) -
      QPointF(arrowSize * std::cos(angle - M_PI / 6),
              -arrowSize * std::sin(angle - M_PI / 6));
  QPointF arrowP2 = QPointF(endX, endY) -
      QPointF(arrowSize * std::cos(angle + M_PI / 6),
              -arrowSize * std::sin(angle + M_PI / 6));

  painter->setBrush(waypoint_.color.darker(110));
  QPolygonF arrowHead;
  arrowHead << QPointF(endX, endY) << arrowP1 << arrowP2;
  painter->drawPolygon(arrowHead);

  // Draw orientation handle
  QPointF handlePos = orientationHandlePos();
  QColor handleColor = isHovered_ || isDraggingOrientation_ ? Qt::yellow : Qt::white;
  painter->setPen(QPen(Qt::black, 1));
  painter->setBrush(handleColor);
  painter->drawEllipse(handlePos, ORIENTATION_HANDLE_RADIUS, ORIENTATION_HANDLE_RADIUS);
}

void WaypointGraphicsItem::drawToleranceCircle(QPainter* painter) {
  // Scale tolerance to pixel coordinates (assuming 1 pixel = metersPerPixel in scene)
  // For now, use a fixed visual radius; actual scaling happens in MissionMapView
  double visualRadius = waypoint_.tolerance * 100;  // Placeholder scaling
  if (visualRadius < WAYPOINT_RADIUS + 5) {
    visualRadius = WAYPOINT_RADIUS + 5;
  }

  QColor toleranceColor = waypoint_.color;
  toleranceColor.setAlphaF(TOLERANCE_CIRCLE_OPACITY);
  painter->setPen(QPen(waypoint_.color.darker(110), 1, Qt::DashLine));
  painter->setBrush(toleranceColor);
  painter->drawEllipse(QPointF(0, 0), visualRadius, visualRadius);
}

QPointF WaypointGraphicsItem::orientationHandlePos() const {
  double handleDist = ORIENTATION_ARROW_LENGTH * 0.8;
  return QPointF(handleDist * std::cos(waypoint_.theta),
                 -handleDist * std::sin(waypoint_.theta));
}

bool WaypointGraphicsItem::isOnOrientationHandle(const QPointF& pos) const {
  QPointF handlePos = orientationHandlePos();
  return QLineF(pos, handlePos).length() <= ORIENTATION_HANDLE_RADIUS + 2;
}

void WaypointGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    if (isOnOrientationHandle(event->pos())) {
      isDraggingOrientation_ = true;
      dragStartTheta_ = waypoint_.theta;
      event->accept();
      return;
    }
    isDragging_ = true;
    dragStartPos_ = pos();
    dragStartScenePos_ = event->scenePos();  // Store scene position for delta calculation
    setCursor(Qt::ClosedHandCursor);
    update();
    emit waypointDragStarted(waypoint_.id, pos());
    event->accept();
  }
  emit waypointClicked(waypoint_.id, event->modifiers());
}

void WaypointGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (isDraggingOrientation_) {
    QPointF delta = event->pos();
    double newTheta = std::atan2(-delta.y(), delta.x());  // Qt Y is inverted
    waypoint_.theta = newTheta;
    update();
    emit waypointOrientationChanged(waypoint_.id, newTheta);
    event->accept();
    return;
  }

  if (isDragging_) {
    // Calculate movement delta in scene coordinates (zoom-independent)
    QPointF scenePos = event->scenePos();
    QPointF delta = scenePos - dragStartScenePos_;

    // Minimum drag threshold to avoid spurious movements from click jitter
    // or coordinate rounding issues at different zoom levels
    if (delta.manhattanLength() < 2.0) {
      event->accept();
      return;  // Don't move for tiny movements
    }

    // Move to new position
    QPointF newPos = dragStartPos_ + delta;
    setPos(newPos);

    // Emit dragging signal for multi-waypoint support
    emit waypointDragging(waypoint_.id, newPos);
    event->accept();
  }
}

void WaypointGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (isDraggingOrientation_) {
    isDraggingOrientation_ = false;
    update();
    event->accept();
    return;
  }

  if (isDragging_) {
    isDragging_ = false;
    setCursor(Qt::PointingHandCursor);
    update();
    if (pos() != dragStartPos_) {
      emit waypointMoved(waypoint_.id, pos());
    }
    event->accept();
  }
}

void WaypointGraphicsItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
  emit waypointDoubleClicked(waypoint_.id);
  QGraphicsObject::mouseDoubleClickEvent(event);
}

void WaypointGraphicsItem::hoverEnterEvent(QGraphicsSceneHoverEvent*) {
  isHovered_ = true;
  update();
}

void WaypointGraphicsItem::hoverLeaveEvent(QGraphicsSceneHoverEvent*) {
  isHovered_ = false;
  update();
}

void WaypointGraphicsItem::hoverMoveEvent(QGraphicsSceneHoverEvent* event) {
  if (isOnOrientationHandle(event->pos())) {
    setCursor(Qt::CrossCursor);
  } else {
    setCursor(Qt::PointingHandCursor);
  }
}

// RobotStartPoseItem implementation

RobotStartPoseItem::RobotStartPoseItem(QGraphicsItem* parent)
    : QGraphicsObject(parent) {
  // Don't use Qt's built-in movement - we handle dragging ourselves for proper zoom handling
  setFlag(QGraphicsItem::ItemIsMovable, false);
  // Don't use Qt's built-in selection - we manage selection ourselves
  setFlag(QGraphicsItem::ItemIsSelectable, false);
  setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
  setAcceptHoverEvents(true);
  setCursor(Qt::PointingHandCursor);
}

void RobotStartPoseItem::setPose(const RobotStartPose& pose) {
  pose_ = pose;
  update();
}

void RobotStartPoseItem::setSelected(bool selected) {
  isSelected_ = selected;
  update();
}

QRectF RobotStartPoseItem::boundingRect() const {
  double extent = ROBOT_SIZE + ORIENTATION_ARROW_LENGTH + 5.0;
  return QRectF(-extent, -extent, extent * 2, extent * 2);
}

void RobotStartPoseItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
  painter->setRenderHint(QPainter::Antialiasing);

  // Draw orientation arrow
  double endX = ORIENTATION_ARROW_LENGTH * std::cos(pose_.theta);
  double endY = -ORIENTATION_ARROW_LENGTH * std::sin(pose_.theta);

  painter->setPen(QPen(QColor(0, 150, 0), 2));
  painter->drawLine(QPointF(0, 0), QPointF(endX, endY));

  // Arrowhead
  double arrowSize = 10.0;
  double angle = std::atan2(-endY, endX);
  QPointF arrowP1 = QPointF(endX, endY) -
      QPointF(arrowSize * std::cos(angle - M_PI / 6),
              -arrowSize * std::sin(angle - M_PI / 6));
  QPointF arrowP2 = QPointF(endX, endY) -
      QPointF(arrowSize * std::cos(angle + M_PI / 6),
              -arrowSize * std::sin(angle + M_PI / 6));

  painter->setBrush(QColor(0, 150, 0));
  QPolygonF arrowHead;
  arrowHead << QPointF(endX, endY) << arrowP1 << arrowP2;
  painter->drawPolygon(arrowHead);

  // Draw robot shape with visual feedback for drag state
  QColor robotColor;
  if (isDragging_) {
    // Bright orange when dragging
    robotColor = QColor(255, 165, 0);
    painter->setPen(QPen(QColor(255, 100, 0), 4));
  } else if (isSelected_) {
    robotColor = QColor(0, 150, 0);
    painter->setPen(QPen(Qt::yellow, 3));
  } else if (isHovered_) {
    robotColor = QColor(0, 200, 0);
    painter->setPen(QPen(robotColor.darker(120), 2));
  } else {
    robotColor = QColor(0, 150, 0);
    painter->setPen(QPen(robotColor.darker(120), 2));
  }
  painter->setBrush(robotColor);

  // Draw as a circle with robot icon (slightly larger when dragging)
  double size = isDragging_ ? ROBOT_SIZE * 1.15 : ROBOT_SIZE;
  painter->drawEllipse(QPointF(0, 0), size, size);

  // Draw "R" for robot
  painter->setPen(Qt::white);
  QFont font = painter->font();
  font.setBold(true);
  font.setPointSize(12);
  painter->setFont(font);
  painter->drawText(QRectF(-ROBOT_SIZE, -ROBOT_SIZE, ROBOT_SIZE * 2, ROBOT_SIZE * 2),
                    Qt::AlignCenter, "R");

  // Label
  painter->setPen(Qt::black);
  font.setPointSize(9);
  font.setBold(false);
  painter->setFont(font);
  painter->drawText(QRectF(-40, ROBOT_SIZE + 2, 80, 16), Qt::AlignCenter, "Start");
}

QPointF RobotStartPoseItem::orientationHandlePos() const {
  double handleDist = ORIENTATION_ARROW_LENGTH * 0.8;
  return QPointF(handleDist * std::cos(pose_.theta),
                 -handleDist * std::sin(pose_.theta));
}

bool RobotStartPoseItem::isOnOrientationHandle(const QPointF& pos) const {
  QPointF handlePos = orientationHandlePos();
  return QLineF(pos, handlePos).length() <= 8.0;
}

void RobotStartPoseItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    if (isOnOrientationHandle(event->pos())) {
      isDraggingOrientation_ = true;
      dragStartTheta_ = pose_.theta;
      event->accept();
      return;
    }
    isDragging_ = true;
    dragStartPos_ = pos();
    dragStartScenePos_ = event->scenePos();  // Store scene position for zoom-independent movement
    setCursor(Qt::ClosedHandCursor);  // Change cursor to show grabbing
    update();  // Update visual to show drag state
    event->accept();
  }
  emit poseClicked();
}

void RobotStartPoseItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (isDraggingOrientation_) {
    QPointF delta = event->pos();
    double newTheta = std::atan2(-delta.y(), delta.x());
    pose_.theta = newTheta;
    update();
    emit poseOrientationChanged(newTheta);
    event->accept();
    return;
  }

  if (isDragging_) {
    // Calculate movement delta in scene coordinates (zoom-independent)
    QPointF scenePos = event->scenePos();
    QPointF delta = scenePos - dragStartScenePos_;

    // Minimum drag threshold to avoid spurious movements
    if (delta.manhattanLength() < 2.0) {
      event->accept();
      return;
    }

    QPointF newPos = dragStartPos_ + delta;
    setPos(newPos);
    event->accept();
  }
}

void RobotStartPoseItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (isDraggingOrientation_) {
    isDraggingOrientation_ = false;
    update();
    event->accept();
    return;
  }

  if (isDragging_) {
    isDragging_ = false;
    setCursor(Qt::PointingHandCursor);  // Restore cursor
    update();  // Update visual to clear drag state
    if (pos() != dragStartPos_) {
      emit poseMoved(pos());
    }
    event->accept();
  }
}

void RobotStartPoseItem::hoverEnterEvent(QGraphicsSceneHoverEvent*) {
  isHovered_ = true;
  update();
}

void RobotStartPoseItem::hoverLeaveEvent(QGraphicsSceneHoverEvent*) {
  isHovered_ = false;
  update();
}

void RobotStartPoseItem::hoverMoveEvent(QGraphicsSceneHoverEvent* event) {
  if (isOnOrientationHandle(event->pos())) {
    setCursor(Qt::CrossCursor);
  } else {
    setCursor(Qt::PointingHandCursor);
  }
}

// ScaleCalibrationOverlay implementation

ScaleCalibrationOverlay::ScaleCalibrationOverlay(QGraphicsItem* parent)
    : QGraphicsObject(parent) {
  setFlag(QGraphicsItem::ItemHasNoContents, false);
}

void ScaleCalibrationOverlay::setFirstPoint(const QPointF& point) {
  point1_ = point;
  hasPoint1_ = true;
  update();
}

void ScaleCalibrationOverlay::setSecondPoint(const QPointF& point) {
  point2_ = point;
  hasPoint2_ = true;
  update();
  if (hasPoint1_ && hasPoint2_) {
    emit calibrationComplete(point1_, point2_);
  }
}

void ScaleCalibrationOverlay::setMeasuring(bool measuring) {
  isMeasuring_ = measuring;
  update();
}

void ScaleCalibrationOverlay::clear() {
  hasPoint1_ = false;
  hasPoint2_ = false;
  isMeasuring_ = false;
  update();
}

double ScaleCalibrationOverlay::pixelDistance() const {
  if (hasPoint1_ && hasPoint2_) {
    return QLineF(point1_, point2_).length();
  }
  return 0.0;
}

QRectF ScaleCalibrationOverlay::boundingRect() const {
  return QRectF(-10000, -10000, 20000, 20000);  // Full scene
}

void ScaleCalibrationOverlay::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
  if (!hasPoint1_ && !isMeasuring_) {
    return;
  }

  painter->setRenderHint(QPainter::Antialiasing);

  // Draw first point
  if (hasPoint1_) {
    painter->setPen(QPen(Qt::red, 2));
    painter->setBrush(Qt::red);
    painter->drawEllipse(point1_, 6, 6);
  }

  // Draw line and second point
  if (hasPoint1_ && hasPoint2_) {
    painter->setPen(QPen(Qt::red, 2, Qt::DashLine));
    painter->drawLine(point1_, point2_);

    painter->setBrush(Qt::red);
    painter->drawEllipse(point2_, 6, 6);

    // Draw distance label
    QPointF midPoint = (point1_ + point2_) / 2.0;
    double dist = pixelDistance();
    QString label = QString("%1 px").arg(dist, 0, 'f', 1);

    painter->setPen(Qt::black);
    QFont font = painter->font();
    font.setBold(true);
    painter->setFont(font);

    QRectF labelRect(midPoint.x() - 40, midPoint.y() - 20, 80, 20);
    painter->fillRect(labelRect, QColor(255, 255, 200, 200));
    painter->drawText(labelRect, Qt::AlignCenter, label);
  }
}

// WaypointPathItem implementation

WaypointPathItem::WaypointPathItem(QGraphicsItem* parent)
    : QGraphicsItem(parent) {
  setZValue(-1);  // Draw behind waypoints
}

void WaypointPathItem::setPath(const QList<QPointF>& points) {
  pathPoints_ = points;
  update();
}

void WaypointPathItem::setShowArrows(bool show) {
  showArrows_ = show;
  update();
}

void WaypointPathItem::setLooping(bool looping) {
  isLooping_ = looping;
  update();
}

QRectF WaypointPathItem::boundingRect() const {
  if (pathPoints_.isEmpty()) {
    return QRectF();
  }

  double minX = pathPoints_[0].x(), maxX = minX;
  double minY = pathPoints_[0].y(), maxY = minY;

  for (const auto& pt : pathPoints_) {
    minX = std::min(minX, pt.x());
    maxX = std::max(maxX, pt.x());
    minY = std::min(minY, pt.y());
    maxY = std::max(maxY, pt.y());
  }

  return QRectF(minX - 10, minY - 10, maxX - minX + 20, maxY - minY + 20);
}

void WaypointPathItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
  if (pathPoints_.size() < 2) {
    return;
  }

  painter->setRenderHint(QPainter::Antialiasing);

  // Determine how many segments are "normal" path vs "return" path
  int normalSegments = pathPoints_.size() - 1;
  if (isLooping_ && pathPoints_.size() >= 3) {
    normalSegments = pathPoints_.size() - 2;  // Last segment is return to start
  }

  // Draw normal path segments
  painter->setPen(QPen(QColor(100, 100, 100), 2, Qt::DashDotLine));
  QPainterPath path;
  path.moveTo(pathPoints_[0]);
  for (int i = 1; i <= normalSegments; ++i) {
    path.lineTo(pathPoints_[i]);
  }
  painter->drawPath(path);

  // Draw return path segment with different style (green, dotted)
  if (isLooping_ && pathPoints_.size() >= 3) {
    painter->setPen(QPen(QColor(0, 150, 0), 2, Qt::DotLine));
    painter->drawLine(pathPoints_[normalSegments], pathPoints_.last());
  }

  // Draw arrows at midpoints if enabled
  if (showArrows_) {
    for (int i = 0; i < pathPoints_.size() - 1; ++i) {
      QPointF p1 = pathPoints_[i];
      QPointF p2 = pathPoints_[i + 1];
      QPointF mid = (p1 + p2) / 2.0;

      double angle = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
      double arrowSize = 8.0;

      QPointF arrowP1 = mid - QPointF(arrowSize * std::cos(angle - M_PI / 6),
                                       arrowSize * std::sin(angle - M_PI / 6));
      QPointF arrowP2 = mid - QPointF(arrowSize * std::cos(angle + M_PI / 6),
                                       arrowSize * std::sin(angle + M_PI / 6));

      // Use green for return arrow
      bool isReturnSegment = isLooping_ && (i == pathPoints_.size() - 2);
      QColor arrowColor = isReturnSegment ? QColor(0, 150, 0) : QColor(100, 100, 100);
      painter->setPen(arrowColor);
      painter->setBrush(arrowColor);

      QPolygonF arrow;
      arrow << mid << arrowP1 << arrowP2;
      painter->drawPolygon(arrow);
    }
  }
}

}  // namespace ros_weaver
