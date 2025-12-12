#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/core/project.hpp"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneHoverEvent>
#include <QCursor>
#include <cmath>

namespace ros_weaver {

PackageBlock::PackageBlock(const QString& packageName, QGraphicsItem* parent)
  : QGraphicsObject(parent)
  , id_(QUuid::createUuid())
  , packageName_(packageName)
  , isSelected_(false)
  , isExpanded_(false)
  , hoveredPinIndex_(-1)
  , hoveredPinIsOutput_(false)
  , isDragging_(false)
  , runtimeStatus_(BlockRuntimeStatus::Unknown)
  , matchConfidence_(MatchConfidence::None)
  , showRuntimeStatus_(true)
{
  setFlag(QGraphicsItem::ItemIsMovable, true);
  setFlag(QGraphicsItem::ItemIsSelectable, true);
  setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
  setCursor(Qt::OpenHandCursor);
  setAcceptHoverEvents(true);
}

PackageBlock::~PackageBlock() = default;

QRectF PackageBlock::boundingRect() const {
  qreal pinCount = std::max(inputPins_.size(), outputPins_.size());
  qreal height = std::max(MIN_HEIGHT, HEADER_HEIGHT + pinCount * PIN_HEIGHT + 10);
  // Expand bounding rect to include pin hover glow (PIN_RADIUS + 3 for glow + 2 for pen width)
  qreal pinMargin = PIN_RADIUS + 5;
  return QRectF(-pinMargin, 0, BLOCK_WIDTH + pinMargin * 2, height);
}

void PackageBlock::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                         QWidget* widget) {
  Q_UNUSED(option)
  Q_UNUSED(widget)

  // Use visual rect for the block body (not the expanded bounding rect)
  qreal pinCount = std::max(inputPins_.size(), outputPins_.size());
  qreal height = std::max(MIN_HEIGHT, HEADER_HEIGHT + pinCount * PIN_HEIGHT + 10);
  QRectF rect(0, 0, BLOCK_WIDTH, height);

  // Draw shadow
  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(0, 0, 0, 50));
  painter->drawRoundedRect(rect.translated(3, 3), CORNER_RADIUS, CORNER_RADIUS);

  // Draw main body
  QLinearGradient bodyGradient(0, 0, 0, rect.height());
  bodyGradient.setColorAt(0, QColor(70, 70, 70));
  bodyGradient.setColorAt(1, QColor(50, 50, 50));

  painter->setBrush(bodyGradient);
  painter->setPen(QPen(isSelected_ ? QColor(42, 130, 218) : QColor(80, 80, 80), 2));
  painter->drawRoundedRect(rect, CORNER_RADIUS, CORNER_RADIUS);

  // Draw header background
  QLinearGradient headerGradient(0, 0, 0, HEADER_HEIGHT);
  headerGradient.setColorAt(0, QColor(42, 130, 218));
  headerGradient.setColorAt(1, QColor(30, 100, 180));

  QPainterPath headerPath;
  headerPath.addRoundedRect(QRectF(0, 0, BLOCK_WIDTH, HEADER_HEIGHT), CORNER_RADIUS, CORNER_RADIUS);

  // Clip to only round top corners
  QPainterPath clipPath;
  clipPath.addRect(QRectF(0, CORNER_RADIUS, BLOCK_WIDTH, HEADER_HEIGHT - CORNER_RADIUS));
  headerPath = headerPath.united(clipPath);
  QPainterPath boundingPath;
  boundingPath.addRect(QRectF(0, 0, BLOCK_WIDTH, HEADER_HEIGHT));
  headerPath = headerPath.intersected(boundingPath);

  painter->setBrush(headerGradient);
  painter->setPen(Qt::NoPen);
  painter->drawPath(headerPath);

  // Draw runtime status indicator
  if (showRuntimeStatus_ && runtimeStatus_ != BlockRuntimeStatus::Unknown) {
    QColor statusColor;
    switch (runtimeStatus_) {
      case BlockRuntimeStatus::Running:
        statusColor = QColor(76, 175, 80);   // Green
        break;
      case BlockRuntimeStatus::PartialMatch:
        statusColor = QColor(255, 152, 0);   // Orange
        break;
      case BlockRuntimeStatus::NotFound:
        statusColor = QColor(158, 158, 158); // Gray
        break;
      default:
        statusColor = QColor(158, 158, 158);
        break;
    }

    // Draw status dot in header
    qreal dotRadius = 5.0;
    qreal dotX = BLOCK_WIDTH - 15;
    qreal dotY = HEADER_HEIGHT / 2;

    painter->setBrush(statusColor);
    painter->setPen(QPen(statusColor.darker(120), 1));
    painter->drawEllipse(QPointF(dotX, dotY), dotRadius, dotRadius);

    // Add pulsing glow for running status
    if (runtimeStatus_ == BlockRuntimeStatus::Running) {
      painter->setBrush(Qt::NoBrush);
      painter->setPen(QPen(statusColor.lighter(130), 1));
      painter->drawEllipse(QPointF(dotX, dotY), dotRadius + 2, dotRadius + 2);
    }
  }

  // Draw package name (adjust width if status indicator is shown)
  qreal nameWidth = showRuntimeStatus_ && runtimeStatus_ != BlockRuntimeStatus::Unknown
                    ? BLOCK_WIDTH - 40 : BLOCK_WIDTH - 20;
  painter->setPen(Qt::white);
  painter->setFont(QFont("Sans", 10, QFont::Bold));
  painter->drawText(QRectF(10, 0, nameWidth, HEADER_HEIGHT),
                    Qt::AlignVCenter | Qt::AlignLeft, packageName_);

  // Draw pins
  for (int i = 0; i < inputPins_.size(); ++i) {
    drawPin(painter, inputPins_[i], true, i);
  }

  for (int i = 0; i < outputPins_.size(); ++i) {
    drawPin(painter, outputPins_[i], false, i);
  }
}

void PackageBlock::drawPin(QPainter* painter, const Pin& pin, bool isInput, int pinIndex) {
  QColor pinColor = getDataTypeColor(pin.dataType);

  qreal y = pin.localPos.y();
  qreal x = isInput ? 0 : BLOCK_WIDTH;

  // Check if this pin is hovered
  bool isHovered = (hoveredPinIndex_ == pinIndex) &&
                   (hoveredPinIsOutput_ == !isInput);

  // Draw hover glow effect
  if (isHovered) {
    painter->setBrush(Qt::NoBrush);
    QPen glowPen(pinColor.lighter(150), 2);
    painter->setPen(glowPen);
    painter->drawEllipse(QPointF(x, y), PIN_RADIUS + 3, PIN_RADIUS + 3);
  }

  // Draw pin circle
  QColor fillColor = isHovered ? pinColor.lighter(130) : pinColor;
  painter->setBrush(fillColor);
  painter->setPen(QPen(pinColor.darker(120), 1.5));
  qreal radius = isHovered ? PIN_RADIUS + 1 : PIN_RADIUS;
  painter->drawEllipse(QPointF(x, y), radius, radius);

  // Draw pin name
  painter->setPen(Qt::white);
  painter->setFont(QFont("Sans", 8));

  QRectF textRect;
  Qt::Alignment alignment;
  if (isInput) {
    textRect = QRectF(PIN_RADIUS + 5, y - 8, BLOCK_WIDTH / 2 - PIN_RADIUS - 10, 16);
    alignment = Qt::AlignVCenter | Qt::AlignLeft;
  } else {
    textRect = QRectF(BLOCK_WIDTH / 2, y - 8, BLOCK_WIDTH / 2 - PIN_RADIUS - 5, 16);
    alignment = Qt::AlignVCenter | Qt::AlignRight;
  }

  painter->drawText(textRect, alignment, pin.name);
}

QColor PackageBlock::getDataTypeColor(Pin::DataType dataType) const {
  switch (dataType) {
    case Pin::DataType::Topic:
      return QColor(100, 200, 100);  // Green for topics
    case Pin::DataType::Service:
      return QColor(100, 150, 255);  // Blue for services
    case Pin::DataType::Action:
      return QColor(255, 180, 100);  // Orange for actions
    case Pin::DataType::Parameter:
      return QColor(200, 100, 200);  // Purple for parameters
    default:
      return QColor(180, 180, 180);  // Gray default
  }
}

void PackageBlock::setPackageName(const QString& name) {
  packageName_ = name;
  update();
}

void PackageBlock::addInputPin(const QString& name, Pin::DataType dataType,
                               const QString& messageType) {
  Pin pin(name, Pin::Type::Input, dataType, messageType);
  inputPins_.append(pin);
  updatePinPositions();
  prepareGeometryChange();
}

void PackageBlock::addOutputPin(const QString& name, Pin::DataType dataType,
                                const QString& messageType) {
  Pin pin(name, Pin::Type::Output, dataType, messageType);
  outputPins_.append(pin);
  updatePinPositions();
  prepareGeometryChange();
}

void PackageBlock::updatePinPositions() {
  qreal startY = HEADER_HEIGHT + PIN_HEIGHT / 2 + 5;

  for (int i = 0; i < inputPins_.size(); ++i) {
    inputPins_[i].localPos = QPointF(0, startY + i * PIN_HEIGHT);
  }

  for (int i = 0; i < outputPins_.size(); ++i) {
    outputPins_[i].localPos = QPointF(BLOCK_WIDTH, startY + i * PIN_HEIGHT);
  }
}

QPointF PackageBlock::inputPinScenePos(int index) const {
  if (index < 0 || index >= inputPins_.size()) {
    return scenePos();
  }
  return scenePos() + inputPins_[index].localPos;
}

QPointF PackageBlock::outputPinScenePos(int index) const {
  if (index < 0 || index >= outputPins_.size()) {
    return scenePos() + QPointF(BLOCK_WIDTH, HEADER_HEIGHT / 2);
  }
  return scenePos() + outputPins_[index].localPos;
}

int PackageBlock::inputPinAtPos(const QPointF& localPos) const {
  for (int i = 0; i < inputPins_.size(); ++i) {
    QPointF pinPos = inputPins_[i].localPos;
    qreal dist = std::sqrt(std::pow(localPos.x() - pinPos.x(), 2) +
                           std::pow(localPos.y() - pinPos.y(), 2));
    if (dist <= PIN_RADIUS * 2.0) {  // Slightly larger hit area
      return i;
    }
  }
  return -1;
}

int PackageBlock::outputPinAtPos(const QPointF& localPos) const {
  for (int i = 0; i < outputPins_.size(); ++i) {
    QPointF pinPos = outputPins_[i].localPos;
    qreal dist = std::sqrt(std::pow(localPos.x() - pinPos.x(), 2) +
                           std::pow(localPos.y() - pinPos.y(), 2));
    if (dist <= PIN_RADIUS * 2.0) {  // Slightly larger hit area
      return i;
    }
  }
  return -1;
}

QColor PackageBlock::pinColor(bool isInput, int index) const {
  const QList<Pin>& pins = isInput ? inputPins_ : outputPins_;
  if (index < 0 || index >= pins.size()) {
    return QColor(180, 180, 180);
  }
  return getDataTypeColor(pins[index].dataType);
}

bool PackageBlock::canConnect(const Pin& output, const Pin& input) {
  // Same data type required for connection
  if (output.dataType != input.dataType) {
    return false;
  }
  // If message types are specified, they must match
  if (!output.messageType.isEmpty() && !input.messageType.isEmpty()) {
    return output.messageType == input.messageType;
  }
  return true;
}

void PackageBlock::addConnection(ConnectionLine* connection) {
  if (connection && !connections_.contains(connection)) {
    connections_.append(connection);
  }
}

void PackageBlock::removeConnection(ConnectionLine* connection) {
  connections_.removeAll(connection);
}

QList<ConnectionLine*> PackageBlock::connectionsForPin(int pinIndex, bool isOutput) const {
  QList<ConnectionLine*> result;
  for (ConnectionLine* conn : connections_) {
    if (isOutput) {
      // Check if this connection originates from this pin
      if (conn->sourceBlock() == this && conn->sourcePinIndex() == pinIndex) {
        result.append(conn);
      }
    } else {
      // Check if this connection terminates at this pin
      if (conn->targetBlock() == this && conn->targetPinIndex() == pinIndex) {
        result.append(conn);
      }
    }
  }
  return result;
}

void PackageBlock::setBlockSelected(bool selected) {
  isSelected_ = selected;
  update();
}

void PackageBlock::setExpanded(bool expanded) {
  isExpanded_ = expanded;
  prepareGeometryChange();
  update();
}

QVariant PackageBlock::itemChange(GraphicsItemChange change, const QVariant& value) {
  if (change == ItemPositionChange && scene()) {
    // Update all connections when block moves
    for (ConnectionLine* connection : connections_) {
      connection->updatePath();
    }
    // Emit signal that block is moving (for group membership updates)
    if (isDragging_) {
      emit blockMoved(this);
    }
  } else if (change == ItemSelectedChange) {
    isSelected_ = value.toBool();
  }

  return QGraphicsItem::itemChange(change, value);
}

void PackageBlock::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    isDragging_ = true;
  }
  QGraphicsObject::mousePressEvent(event);
}

void PackageBlock::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton && isDragging_) {
    isDragging_ = false;
    emit blockMoveFinished(this);
  }
  QGraphicsObject::mouseReleaseEvent(event);
}

void PackageBlock::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
  Q_UNUSED(event)
  setExpanded(!isExpanded_);
}

void PackageBlock::hoverMoveEvent(QGraphicsSceneHoverEvent* event) {
  QPointF localPos = event->pos();

  // Check output pins first
  int outputIdx = outputPinAtPos(localPos);
  if (outputIdx >= 0) {
    if (hoveredPinIndex_ != outputIdx || !hoveredPinIsOutput_) {
      hoveredPinIndex_ = outputIdx;
      hoveredPinIsOutput_ = true;
      setCursor(Qt::CrossCursor);
      emit pinHovered(this, outputIdx, true);
      update();
    }
    return;
  }

  // Check input pins
  int inputIdx = inputPinAtPos(localPos);
  if (inputIdx >= 0) {
    if (hoveredPinIndex_ != inputIdx || hoveredPinIsOutput_) {
      hoveredPinIndex_ = inputIdx;
      hoveredPinIsOutput_ = false;
      setCursor(Qt::CrossCursor);
      emit pinHovered(this, inputIdx, false);
      update();
    }
    return;
  }

  // Not hovering over any pin
  if (hoveredPinIndex_ >= 0) {
    clearHoveredPin();
    emit pinUnhovered(this);
  }
  setCursor(Qt::OpenHandCursor);
}

void PackageBlock::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
  Q_UNUSED(event)
  if (hoveredPinIndex_ >= 0) {
    emit pinUnhovered(this);
  }
  clearHoveredPin();
  setCursor(Qt::OpenHandCursor);
}

void PackageBlock::setHoveredPin(int pinIndex, bool isOutput) {
  if (hoveredPinIndex_ != pinIndex || hoveredPinIsOutput_ != isOutput) {
    hoveredPinIndex_ = pinIndex;
    hoveredPinIsOutput_ = isOutput;
    update();
  }
}

void PackageBlock::clearHoveredPin() {
  if (hoveredPinIndex_ >= 0) {
    hoveredPinIndex_ = -1;
    update();
  }
}

void PackageBlock::setParameters(const QList<BlockParamData>& params) {
  parameters_ = params;
}

void PackageBlock::setPreferredYamlSource(const QString& source) {
  preferredYamlSource_ = source;
}

void PackageBlock::setRuntimeStatus(BlockRuntimeStatus status) {
  if (runtimeStatus_ != status) {
    runtimeStatus_ = status;
    update();
  }
}

void PackageBlock::setMatchedNodeName(const QString& nodeName) {
  matchedNodeName_ = nodeName;
}

void PackageBlock::setMatchConfidence(MatchConfidence confidence) {
  matchConfidence_ = confidence;
}

void PackageBlock::updateMappingResult(const BlockMappingResult& result) {
  matchedNodeName_ = result.ros2NodeName;
  matchConfidence_ = result.confidence;

  switch (result.confidence) {
    case MatchConfidence::High:
    case MatchConfidence::Medium:
      runtimeStatus_ = BlockRuntimeStatus::Running;
      break;
    case MatchConfidence::Low:
      runtimeStatus_ = BlockRuntimeStatus::PartialMatch;
      break;
    case MatchConfidence::None:
      runtimeStatus_ = BlockRuntimeStatus::NotFound;
      break;
  }

  update();
}

void PackageBlock::clearRuntimeStatus() {
  runtimeStatus_ = BlockRuntimeStatus::Unknown;
  matchedNodeName_.clear();
  matchConfidence_ = MatchConfidence::None;
  update();
}

void PackageBlock::setShowRuntimeStatus(bool show) {
  if (showRuntimeStatus_ != show) {
    showRuntimeStatus_ = show;
    update();
  }
}

}  // namespace ros_weaver
