#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QGraphicsSceneHoverEvent>
#include <QPainterPathStroker>
#include <cmath>

namespace ros_weaver {

ConnectionLine::ConnectionLine(PackageBlock* sourceBlock, int sourcePin,
                               PackageBlock* targetBlock, int targetPin,
                               QGraphicsItem* parent)
  : QGraphicsObject(parent)
  , id_(QUuid::createUuid())
  , sourceBlock_(sourceBlock)
  , targetBlock_(targetBlock)
  , sourcePinIndex_(sourcePin)
  , targetPinIndex_(targetPin)
  , isHighlighted_(false)
  , isHovered_(false)
  , isPinHighlighted_(false)
  , connectionColor_(QColor(100, 200, 100))
  , pulseAnimation_(nullptr)
  , pulsePhase_(0.0)
{
  setFlag(QGraphicsItem::ItemIsSelectable, true);
  setAcceptHoverEvents(true);
  setZValue(-1);  // Draw behind blocks

  updatePath();
}

ConnectionLine::~ConnectionLine() {
  // Stop animation before destruction
  if (pulseAnimation_) {
    pulseAnimation_->stop();
    delete pulseAnimation_;
  }

  // Remove this connection from source and target blocks
  if (sourceBlock_) {
    sourceBlock_->removeConnection(this);
  }
  if (targetBlock_) {
    targetBlock_->removeConnection(this);
  }
}

QRectF ConnectionLine::boundingRect() const {
  if (path_.isEmpty()) {
    return QRectF();
  }
  // Add margin for the glow effect and line width
  qreal margin = HIGHLIGHT_WIDTH + 10;
  return path_.boundingRect().adjusted(-margin, -margin, margin, margin);
}

QPainterPath ConnectionLine::shape() const {
  // Create a wider path for easier mouse interaction
  QPainterPathStroker stroker;
  stroker.setWidth(HIT_TOLERANCE * 2);
  return stroker.createStroke(path_);
}

void ConnectionLine::updatePath() {
  prepareGeometryChange();
  path_ = calculatePath();
  update();
}

QPainterPath ConnectionLine::calculatePath() const {
  if (!sourceBlock_ || !targetBlock_) {
    return QPainterPath();
  }

  QPointF startPos = sourceBlock_->outputPinScenePos(sourcePinIndex_);
  QPointF endPos = targetBlock_->inputPinScenePos(targetPinIndex_);

  // Calculate control points for a smooth bezier curve
  qreal dx = std::abs(endPos.x() - startPos.x());
  qreal offset = std::max(CURVE_OFFSET, dx * 0.5);

  QPointF ctrl1(startPos.x() + offset, startPos.y());
  QPointF ctrl2(endPos.x() - offset, endPos.y());

  QPainterPath path;
  path.moveTo(startPos);
  path.cubicTo(ctrl1, ctrl2, endPos);

  return path;
}

void ConnectionLine::setHighlighted(bool highlighted) {
  if (isHighlighted_ != highlighted) {
    isHighlighted_ = highlighted;
    update();
  }
}

void ConnectionLine::setConnectionColor(const QColor& color) {
  if (connectionColor_ != color) {
    connectionColor_ = color;
    update();
  }
}

void ConnectionLine::setPulsePhase(qreal phase) {
  if (pulsePhase_ != phase) {
    pulsePhase_ = phase;
    update();
  }
}

void ConnectionLine::setPinHighlighted(bool highlighted) {
  if (isPinHighlighted_ != highlighted) {
    isPinHighlighted_ = highlighted;
    if (highlighted) {
      startPulseAnimation();
    } else if (!isHovered_) {
      // Only stop animation if not also being hovered
      stopPulseAnimation();
    }
    update();
  }
}

void ConnectionLine::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
  Q_UNUSED(event)
  isHovered_ = true;
  startPulseAnimation();
  update();
}

void ConnectionLine::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
  Q_UNUSED(event)
  isHovered_ = false;
  if (!isPinHighlighted_) {
    // Only stop animation if not also pin-highlighted
    stopPulseAnimation();
  }
  update();
}

void ConnectionLine::startPulseAnimation() {
  if (!pulseAnimation_) {
    pulseAnimation_ = new QPropertyAnimation(this, "pulsePhase");
    pulseAnimation_->setDuration(1000);  // 1 second per cycle
    pulseAnimation_->setStartValue(0.0);
    pulseAnimation_->setEndValue(1.0);
    pulseAnimation_->setLoopCount(-1);  // Infinite loop
  }
  pulseAnimation_->start();
}

void ConnectionLine::stopPulseAnimation() {
  if (pulseAnimation_) {
    pulseAnimation_->stop();
  }
  pulsePhase_ = 0.0;
  update();
}

void ConnectionLine::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                           QWidget* widget) {
  Q_UNUSED(option)
  Q_UNUSED(widget)

  if (path_.isEmpty()) return;

  painter->setRenderHint(QPainter::Antialiasing, true);

  // Determine pen width and color based on state
  qreal penWidth = LINE_WIDTH;
  QColor color = connectionColor_;

  bool shouldAnimate = isHovered_ || isPinHighlighted_;

  if (isSelected()) {
    color = QColor(42, 130, 218);
    penWidth = HIGHLIGHT_WIDTH;
  } else if (isHighlighted_ || isPinHighlighted_) {
    penWidth = HIGHLIGHT_WIDTH;
  } else if (isHovered_) {
    penWidth = HOVER_WIDTH;
  }

  // Draw glow/pulse effect for hovered or highlighted connections
  if (shouldAnimate || isHighlighted_ || isSelected()) {
    // Calculate pulse intensity (sine wave for smooth pulsing)
    qreal pulseIntensity = 0.5;
    if (shouldAnimate && pulseAnimation_ && pulseAnimation_->state() == QAbstractAnimation::Running) {
      pulseIntensity = 0.3 + 0.7 * (0.5 + 0.5 * std::sin(pulsePhase_ * 2 * M_PI));
    }

    // Outer glow
    QColor glowColor = color.lighter(150);
    glowColor.setAlpha(static_cast<int>(100 * pulseIntensity));
    QPen glowPen(glowColor);
    glowPen.setWidthF(penWidth + 8);
    glowPen.setCapStyle(Qt::RoundCap);
    painter->setPen(glowPen);
    painter->drawPath(path_);

    // Middle glow
    glowColor.setAlpha(static_cast<int>(150 * pulseIntensity));
    glowPen.setColor(glowColor);
    glowPen.setWidthF(penWidth + 4);
    painter->setPen(glowPen);
    painter->drawPath(path_);
  }

  // Draw main connection line
  QPen pen(color);
  pen.setWidthF(penWidth);
  pen.setCapStyle(Qt::RoundCap);
  painter->setPen(pen);
  painter->drawPath(path_);

  // Draw flowing particles effect when hovered or pin-highlighted
  if (shouldAnimate && pulseAnimation_ && pulseAnimation_->state() == QAbstractAnimation::Running) {
    // Draw animated dots along the path
    qreal pathLength = path_.length();
    int numDots = 3;
    qreal dotSpacing = 1.0 / numDots;

    Q_UNUSED(pathLength)
    for (int i = 0; i < numDots; ++i) {
      qreal t = std::fmod(pulsePhase_ + i * dotSpacing, 1.0);
      QPointF dotPos = path_.pointAtPercent(t);

      // Fade in and out at the ends
      qreal alpha = 1.0;
      if (t < 0.1) alpha = t / 0.1;
      else if (t > 0.9) alpha = (1.0 - t) / 0.1;

      QColor dotColor = color.lighter(180);
      dotColor.setAlpha(static_cast<int>(255 * alpha));

      painter->setBrush(dotColor);
      painter->setPen(Qt::NoPen);
      painter->drawEllipse(dotPos, 4, 4);
    }
  }

  // Draw arrow at end point
  if (targetBlock_) {
    QPointF endPos = targetBlock_->inputPinScenePos(targetPinIndex_);
    qreal arrowSize = 8.0;

    // Get the tangent direction at the end of the path
    qreal t = 0.99;
    QPointF tangentEnd = path_.pointAtPercent(1.0);
    QPointF tangentStart = path_.pointAtPercent(t);
    QPointF direction = tangentEnd - tangentStart;
    qreal length = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    if (length > 0) {
      direction /= length;
    } else {
      direction = QPointF(-1, 0);  // Default direction
    }

    // Calculate arrow points perpendicular to direction
    QPointF perpendicular(-direction.y(), direction.x());
    QPointF arrowBase = endPos - direction * arrowSize;
    QPointF arrowP1 = arrowBase + perpendicular * (arrowSize / 2);
    QPointF arrowP2 = arrowBase - perpendicular * (arrowSize / 2);

    QPainterPath arrowPath;
    arrowPath.moveTo(endPos);
    arrowPath.lineTo(arrowP1);
    arrowPath.lineTo(arrowP2);
    arrowPath.closeSubpath();

    painter->setBrush(color);
    painter->setPen(Qt::NoPen);
    painter->drawPath(arrowPath);
  }
}

}  // namespace ros_weaver
