#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/core/tooltip_provider.hpp"

#include <QGraphicsSceneHoverEvent>
#include <QGraphicsSceneMouseEvent>
#include <QPainterPathStroker>
#include <QFontMetrics>
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
  , dataFlowAnimation_(nullptr)
  , activityGlowAnimation_(nullptr)
  , pulsePhase_(0.0)
  , dataFlowPhase_(0.0)
  , activityGlow_(0.0)
  , activityState_(TopicActivityState::Unknown)
  , messageRate_(0.0)
  , showRateLabel_(true)
  , liveMonitoringEnabled_(false)
{
  setFlag(QGraphicsItem::ItemIsSelectable, true);
  setAcceptHoverEvents(true);
  setZValue(-1);  // Draw behind blocks

  updatePath();
}

ConnectionLine::~ConnectionLine() {
  // Stop animations before destruction
  if (pulseAnimation_) {
    pulseAnimation_->stop();
    delete pulseAnimation_;
  }
  if (dataFlowAnimation_) {
    dataFlowAnimation_->stop();
    delete dataFlowAnimation_;
  }
  if (activityGlowAnimation_) {
    activityGlowAnimation_->stop();
    delete activityGlowAnimation_;
  }

  // Remove this connection from source and target blocks
  // Note: If detach() was called, these pointers are already null
  if (sourceBlock_) {
    sourceBlock_->removeConnection(this);
  }
  if (targetBlock_) {
    targetBlock_->removeConnection(this);
  }
}

void ConnectionLine::detach() {
  // Safely remove this connection from blocks BEFORE scene clearing
  // This must be called before scene_->clear() to avoid use-after-free
  // when blocks are deleted before connections in undefined order
  if (sourceBlock_) {
    sourceBlock_->removeConnection(this);
    sourceBlock_ = nullptr;
  }
  if (targetBlock_) {
    targetBlock_->removeConnection(this);
    targetBlock_ = nullptr;
  }
}

QRectF ConnectionLine::boundingRect() const {
  if (path_.isEmpty()) {
    return QRectF();
  }
  // Add margin for the glow effect, line width, and animation effects
  // Animation can add up to 6 pixels of glow + activity glow of 8 pixels
  qreal margin = HIGHLIGHT_WIDTH + 20;
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

  // Set rich tooltip for the connection
  QString tooltip = TooltipProvider::instance().tooltipForConnection(this);
  setToolTip(tooltip);

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

  // Check if this connection is actively receiving data
  bool isActiveDataFlow = liveMonitoringEnabled_ &&
    (activityState_ == TopicActivityState::Active ||
     activityState_ == TopicActivityState::HighRate);

  if (isSelected()) {
    color = QColor(42, 130, 218);
    penWidth = HIGHLIGHT_WIDTH;
  } else if (isHighlighted_ || isPinHighlighted_) {
    penWidth = HIGHLIGHT_WIDTH;
  } else if (isHovered_) {
    penWidth = HOVER_WIDTH;
  } else if (isActiveDataFlow) {
    // Throbbing width effect for active data flow
    // Scale throb intensity with message rate for visual feedback
    qreal rateIntensity = 0.5;  // Base intensity for low rates
    if (messageRate_ > 0) {
      // Scale from 0.5 at 1Hz to 1.0 at 10kHz
      rateIntensity = std::min(1.0, 0.5 + 0.125 * std::log10(messageRate_));
    }
    qreal throb = 0.5 + 0.5 * std::sin(dataFlowPhase_ * 2 * M_PI);
    // Higher rates = thicker line (2.0-3.0 base + 0-2 rate bonus)
    qreal rateBonus = rateIntensity * 2.0;
    penWidth = LINE_WIDTH + rateBonus + (1.5 * rateIntensity) * throb;
    color = getActivityColor();
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

  // Draw throbbing glow for active data flow connections
  if (isActiveDataFlow) {
    qreal throb = 0.5 + 0.5 * std::sin(dataFlowPhase_ * 2 * M_PI);

    // Scale glow intensity with rate
    qreal rateIntensity = 0.5;
    if (messageRate_ > 0) {
      rateIntensity = std::min(1.0, 0.5 + 0.125 * std::log10(messageRate_));
    }

    // Outer glow that pulses - brighter and larger for higher rates
    QColor glowColor = color.lighter(130);
    glowColor.setAlpha(static_cast<int>((60 + 80 * rateIntensity) * throb));
    QPen glowPen(glowColor);
    glowPen.setWidthF(penWidth + (4 + 8 * rateIntensity) * throb);
    glowPen.setCapStyle(Qt::RoundCap);
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

  // Draw live monitoring elements if enabled
  if (liveMonitoringEnabled_) {
    drawActivityIndicator(painter);
    if (showRateLabel_ && messageRate_ > 0) {
      drawRateLabel(painter);
    }
  }
}

void ConnectionLine::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    emit clicked(this);
  }
  QGraphicsObject::mousePressEvent(event);
}

void ConnectionLine::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    emit doubleClicked(this);
  }
  QGraphicsObject::mouseDoubleClickEvent(event);
}

// Live topic monitoring methods

void ConnectionLine::setTopicName(const QString& topicName) {
  topicName_ = topicName;
}

void ConnectionLine::setMessageType(const QString& messageType) {
  messageType_ = messageType;
}

void ConnectionLine::setActivityState(TopicActivityState state) {
  if (activityState_ != state) {
    activityState_ = state;

    // Update connection color based on state
    if (liveMonitoringEnabled_) {
      switch (state) {
        case TopicActivityState::Active:
          startDataFlowAnimation();
          break;
        case TopicActivityState::HighRate:
          startDataFlowAnimation();
          break;
        case TopicActivityState::Inactive:
        case TopicActivityState::Unknown:
          stopDataFlowAnimation();
          break;
      }
    }
    update();
  }
}

void ConnectionLine::setMessageRate(double rateHz) {
  if (messageRate_ != rateHz) {
    messageRate_ = rateHz;

    // Adjust animation speed based on rate
    if (dataFlowAnimation_ && dataFlowAnimation_->state() == QAbstractAnimation::Running) {
      int duration = 2500;
      if (rateHz > 0) {
        // Match the formula in startDataFlowAnimation
        duration = static_cast<int>(std::max(500.0, 2500.0 / (1.0 + std::log10(rateHz))));
      }
      dataFlowAnimation_->setDuration(duration);
    }

    update();
  }
}

void ConnectionLine::setShowRateLabel(bool show) {
  if (showRateLabel_ != show) {
    showRateLabel_ = show;
    update();
  }
}

void ConnectionLine::setLiveMonitoringEnabled(bool enabled) {
  if (liveMonitoringEnabled_ != enabled) {
    liveMonitoringEnabled_ = enabled;

    if (enabled && (activityState_ == TopicActivityState::Active ||
                    activityState_ == TopicActivityState::HighRate)) {
      startDataFlowAnimation();
    } else {
      stopDataFlowAnimation();
    }
    update();
  }
}

void ConnectionLine::setDataFlowPhase(qreal phase) {
  if (dataFlowPhase_ != phase) {
    dataFlowPhase_ = phase;
    update();
  }
}

void ConnectionLine::setActivityGlow(qreal glow) {
  if (activityGlow_ != glow) {
    activityGlow_ = glow;
    update();
  }
}

void ConnectionLine::pulseActivity() {
  // Create a quick glow animation for message receipt
  if (!activityGlowAnimation_) {
    activityGlowAnimation_ = new QPropertyAnimation(this, "activityGlow");
  }

  activityGlowAnimation_->stop();
  activityGlowAnimation_->setDuration(300);
  activityGlowAnimation_->setStartValue(1.0);
  activityGlowAnimation_->setEndValue(0.0);
  activityGlowAnimation_->setEasingCurve(QEasingCurve::OutQuad);
  activityGlowAnimation_->start();
}

void ConnectionLine::startDataFlowAnimation() {
  if (!dataFlowAnimation_) {
    dataFlowAnimation_ = new QPropertyAnimation(this, "dataFlowPhase");
    dataFlowAnimation_->setLoopCount(-1);  // Infinite loop
  }

  // Duration based on message rate - faster rate = faster animation
  // The animation speed directly conveys the data rate visually
  // At 1Hz: ~2500ms, At 10Hz: ~1250ms, At 100Hz: ~833ms, At 1kHz: ~625ms, At 10kHz: ~500ms
  int duration = 2500;  // Default for very low rates
  if (messageRate_ > 0) {
    // Logarithmic scaling so animation speed increases with rate
    // but remains smooth even at very high rates (min 500ms)
    duration = static_cast<int>(std::max(500.0, 2500.0 / (1.0 + std::log10(messageRate_))));
  }

  dataFlowAnimation_->setDuration(duration);
  dataFlowAnimation_->setStartValue(0.0);
  dataFlowAnimation_->setEndValue(1.0);
  dataFlowAnimation_->start();
}

void ConnectionLine::stopDataFlowAnimation() {
  if (dataFlowAnimation_ && dataFlowAnimation_->state() == QAbstractAnimation::Running) {
    // Quick fade-out over 250ms instead of abrupt stop
    // Stop the looping animation and animate to 0
    dataFlowAnimation_->stop();

    // Create a one-shot fade-out animation
    QPropertyAnimation* fadeOut = new QPropertyAnimation(this, "dataFlowPhase");
    fadeOut->setDuration(250);
    fadeOut->setStartValue(dataFlowPhase_);
    fadeOut->setEndValue(0.0);
    fadeOut->setEasingCurve(QEasingCurve::OutQuad);

    connect(fadeOut, &QPropertyAnimation::finished, fadeOut, &QObject::deleteLater);
    fadeOut->start();
  } else {
    dataFlowPhase_ = 0.0;
    update();
  }
}

QColor ConnectionLine::getActivityColor() const {
  switch (activityState_) {
    case TopicActivityState::HighRate:
      return QColor(50, 255, 50);   // Bright green
    case TopicActivityState::Active:
      return QColor(100, 200, 100); // Green
    case TopicActivityState::Inactive:
      return QColor(150, 150, 150); // Gray
    case TopicActivityState::Unknown:
    default:
      return connectionColor_;
  }
}

void ConnectionLine::drawRateLabel(QPainter* painter) {
  if (path_.isEmpty()) return;

  // Position label at center of path
  QPointF midPoint = path_.pointAtPercent(0.5);

  // Format rate string
  QString rateStr;
  if (messageRate_ >= 1000) {
    rateStr = QString("%1 kHz").arg(messageRate_ / 1000.0, 0, 'f', 1);
  } else if (messageRate_ >= 1) {
    rateStr = QString("%1 Hz").arg(messageRate_, 0, 'f', 1);
  } else if (messageRate_ > 0) {
    rateStr = QString("%1 Hz").arg(messageRate_, 0, 'f', 2);
  } else {
    return;  // Don't draw if rate is 0
  }

  // Set up font
  QFont font("Sans", 8);
  font.setBold(true);
  painter->setFont(font);

  QFontMetrics fm(font);
  QRect textRect = fm.boundingRect(rateStr);
  textRect.moveCenter(midPoint.toPoint());

  // Draw background
  QRect bgRect = textRect.adjusted(-4, -2, 4, 2);
  QColor bgColor(30, 30, 30, 200);
  painter->setBrush(bgColor);
  painter->setPen(Qt::NoPen);
  painter->drawRoundedRect(bgRect, 3, 3);

  // Draw text
  QColor textColor = getActivityColor();
  if (activityState_ == TopicActivityState::Unknown ||
      activityState_ == TopicActivityState::Inactive) {
    textColor = QColor(200, 200, 200);
  }
  painter->setPen(textColor);
  painter->drawText(textRect, Qt::AlignCenter, rateStr);
}

void ConnectionLine::drawActivityIndicator(QPainter* painter) {
  if (path_.isEmpty() || !liveMonitoringEnabled_) return;

  QColor activityColor = getActivityColor();

  // Draw marching ants / data flow particles for active connections
  if ((activityState_ == TopicActivityState::Active ||
       activityState_ == TopicActivityState::HighRate) &&
      dataFlowAnimation_ && dataFlowAnimation_->state() == QAbstractAnimation::Running) {

    // Scale particle count with rate to visually convey data throughput
    // 1Hz: 2 particles, 10Hz: 3, 100Hz: 5, 1kHz: 7, 10kHz: 9
    int numParticles = 2;
    if (messageRate_ > 0) {
      numParticles = std::min(10, std::max(2, static_cast<int>(2 + 2 * std::log10(messageRate_))));
    }
    qreal spacing = 1.0 / numParticles;

    // Scale particle size with rate
    qreal rateIntensity = 0.5;
    if (messageRate_ > 0) {
      rateIntensity = std::min(1.0, 0.5 + 0.125 * std::log10(messageRate_));
    }

    for (int i = 0; i < numParticles; ++i) {
      qreal t = std::fmod(dataFlowPhase_ + i * spacing, 1.0);
      QPointF particlePos = path_.pointAtPercent(t);

      // Fade at ends
      qreal alpha = 1.0;
      if (t < 0.1) alpha = t / 0.1;
      else if (t > 0.9) alpha = (1.0 - t) / 0.1;

      QColor particleColor = activityColor.lighter(150);
      particleColor.setAlpha(static_cast<int>(255 * alpha));

      // Particle size scales with rate (3-6 pixels)
      qreal particleSize = 3 + 3 * rateIntensity;

      // Outer glow - larger for higher rates
      QColor glowColor = particleColor;
      glowColor.setAlpha(static_cast<int>((80 + 60 * rateIntensity) * alpha));
      painter->setBrush(glowColor);
      painter->setPen(Qt::NoPen);
      painter->drawEllipse(particlePos, particleSize + 2 + 2 * rateIntensity, particleSize + 2 + 2 * rateIntensity);

      // Inner particle
      painter->setBrush(particleColor);
      painter->drawEllipse(particlePos, particleSize, particleSize);
    }
  }

  // Draw activity pulse glow when message received
  if (activityGlow_ > 0) {
    QColor pulseColor = activityColor;
    pulseColor.setAlpha(static_cast<int>(150 * activityGlow_));

    QPen glowPen(pulseColor);
    glowPen.setWidthF(LINE_WIDTH + 8 * activityGlow_);
    glowPen.setCapStyle(Qt::RoundCap);
    painter->setPen(glowPen);
    painter->drawPath(path_);
  }
}

}  // namespace ros_weaver
