#include "ros_weaver/widgets/bt_node_item.hpp"
#include <QPainter>
#include <QGraphicsSceneHoverEvent>
#include <QToolTip>
#include <QCursor>

namespace ros_weaver {

// ============================================================================
// BTNodeItem
// ============================================================================

BTNodeItem::BTNodeItem(std::shared_ptr<BTNode> node, QGraphicsItem* parent)
    : QGraphicsItem(parent), node_(node) {
  setAcceptHoverEvents(true);
  setFlag(QGraphicsItem::ItemIsSelectable, true);
  setToolTip(createTooltip());
}

QString BTNodeItem::createTooltip() const {
  QString tooltip = QString("<b>%1</b><br>Type: %2")
                        .arg(node_->displayName)
                        .arg(BehaviorTreeParser::getNodeTypeName(node_->type));

  if (!node_->attributes.isEmpty()) {
    tooltip += "<br><br><b>Attributes:</b>";
    for (auto it = node_->attributes.begin(); it != node_->attributes.end(); ++it) {
      if (it.key() != "ID" && it.key() != "name") {
        tooltip += QString("<br>%1: %2").arg(it.key(), it.value());
      }
    }
  }

  return tooltip;
}

QRectF BTNodeItem::boundingRect() const {
  return QRectF(-2, -2, NODE_WIDTH + 4, NODE_HEIGHT + 4);
}

void BTNodeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                       QWidget* widget) {
  Q_UNUSED(option);
  Q_UNUSED(widget);

  painter->setRenderHint(QPainter::Antialiasing);

  drawNodeShape(painter);
  drawNodeContent(painter);

  if (executionState_ != BTExecutionState::Idle) {
    drawExecutionIndicator(painter);
  }
}

void BTNodeItem::drawNodeShape(QPainter* painter) {
  QColor bgColor = getBackgroundColor();
  QColor borderColor = getBorderColor();

  // Draw shadow
  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(0, 0, 0, 40));
  painter->drawRoundedRect(QRectF(3, 3, NODE_WIDTH, NODE_HEIGHT), CORNER_RADIUS, CORNER_RADIUS);

  // Draw main shape
  painter->setBrush(bgColor);
  painter->setPen(QPen(borderColor, isSelected() ? 3 : 2));
  painter->drawRoundedRect(QRectF(0, 0, NODE_WIDTH, NODE_HEIGHT), CORNER_RADIUS, CORNER_RADIUS);

  // Draw type indicator bar at top
  QColor typeColor = BehaviorTreeParser::getNodeColor(node_->type);
  painter->setBrush(typeColor);
  painter->setPen(Qt::NoPen);

  QPainterPath topBar;
  topBar.moveTo(CORNER_RADIUS, 0);
  topBar.lineTo(NODE_WIDTH - CORNER_RADIUS, 0);
  topBar.arcTo(NODE_WIDTH - 2 * CORNER_RADIUS, 0, 2 * CORNER_RADIUS, 2 * CORNER_RADIUS, 90, -90);
  topBar.lineTo(NODE_WIDTH, 8);
  topBar.lineTo(0, 8);
  topBar.lineTo(0, CORNER_RADIUS);
  topBar.arcTo(0, 0, 2 * CORNER_RADIUS, 2 * CORNER_RADIUS, 180, -90);
  topBar.closeSubpath();
  painter->drawPath(topBar);
}

void BTNodeItem::drawNodeContent(QPainter* painter) {
  // Draw symbol
  QString symbol = BehaviorTreeParser::getNodeSymbol(node_->type);
  if (!symbol.isEmpty()) {
    painter->setPen(Qt::white);
    painter->setFont(QFont("Sans", 12, QFont::Bold));
    painter->drawText(QRectF(5, 0, 20, 20), Qt::AlignCenter, symbol);
  }

  // Draw node name
  painter->setPen(Qt::black);
  painter->setFont(QFont("Sans", 10, QFont::Bold));

  QString displayText = node_->displayName;
  QFontMetrics fm(painter->font());
  if (fm.horizontalAdvance(displayText) > NODE_WIDTH - 16) {
    displayText = fm.elidedText(displayText, Qt::ElideRight, static_cast<int>(NODE_WIDTH - 16));
  }

  painter->drawText(QRectF(8, 12, NODE_WIDTH - 16, 24), Qt::AlignCenter, displayText);

  // Draw node type (smaller, below name)
  painter->setFont(QFont("Sans", 8));
  painter->setPen(QColor(80, 80, 80));
  QString typeName = BehaviorTreeParser::getNodeTypeName(node_->type);
  if (node_->name != typeName && node_->name != node_->displayName) {
    typeName = node_->name;
  }
  painter->drawText(QRectF(8, 36, NODE_WIDTH - 16, 20), Qt::AlignCenter, typeName);
}

void BTNodeItem::drawExecutionIndicator(QPainter* painter) {
  QColor indicatorColor;
  switch (executionState_) {
    case BTExecutionState::Running:
      indicatorColor = QColor(33, 150, 243);  // Blue
      break;
    case BTExecutionState::Success:
      indicatorColor = QColor(76, 175, 80);   // Green
      break;
    case BTExecutionState::Failure:
      indicatorColor = QColor(244, 67, 54);   // Red
      break;
    case BTExecutionState::Skipped:
      indicatorColor = QColor(158, 158, 158); // Gray
      break;
    default:
      return;
  }

  // Draw glowing border
  painter->setPen(QPen(indicatorColor, 3));
  painter->setBrush(Qt::NoBrush);
  painter->drawRoundedRect(QRectF(-2, -2, NODE_WIDTH + 4, NODE_HEIGHT + 4),
                           CORNER_RADIUS + 2, CORNER_RADIUS + 2);

  // Draw small status circle in top-right
  painter->setBrush(indicatorColor);
  painter->setPen(QPen(Qt::white, 2));
  painter->drawEllipse(QPointF(NODE_WIDTH - 8, 8), 6, 6);
}

QColor BTNodeItem::getBackgroundColor() const {
  if (isHovered_) {
    return QColor(255, 255, 255);
  }
  return QColor(250, 250, 250);
}

QColor BTNodeItem::getBorderColor() const {
  if (isSelected()) {
    return QColor(33, 150, 243);  // Blue selection
  }
  if (isHovered_) {
    return QColor(100, 100, 100);
  }
  return QColor(180, 180, 180);
}

void BTNodeItem::setExecutionState(BTExecutionState state) {
  if (executionState_ != state) {
    executionState_ = state;
    update();
  }
}

QPointF BTNodeItem::topConnectionPoint() const {
  return mapToScene(QPointF(NODE_WIDTH / 2, 0));
}

QPointF BTNodeItem::bottomConnectionPoint() const {
  return mapToScene(QPointF(NODE_WIDTH / 2, NODE_HEIGHT));
}

void BTNodeItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
  Q_UNUSED(event);
  isHovered_ = true;
  setCursor(Qt::PointingHandCursor);
  update();
}

void BTNodeItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
  Q_UNUSED(event);
  isHovered_ = false;
  unsetCursor();
  update();
}

// ============================================================================
// BTEdgeItem
// ============================================================================

BTEdgeItem::BTEdgeItem(BTNodeItem* parent, BTNodeItem* child, QGraphicsItem* parentItem)
    : QGraphicsItem(parentItem), parentNode_(parent), childNode_(child) {
  setZValue(-1);  // Draw behind nodes
  updatePositions();
}

QRectF BTEdgeItem::boundingRect() const {
  return path_.boundingRect().adjusted(-5, -5, 5, 5);
}

void BTEdgeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                       QWidget* widget) {
  Q_UNUSED(option);
  Q_UNUSED(widget);

  painter->setRenderHint(QPainter::Antialiasing);
  painter->setPen(QPen(QColor(150, 150, 150), 2));
  painter->setBrush(Qt::NoBrush);
  painter->drawPath(path_);

  // Draw arrow at end
  QPointF endPoint = childNode_->topConnectionPoint();
  painter->setBrush(QColor(150, 150, 150));

  QPolygonF arrow;
  arrow << endPoint
        << QPointF(endPoint.x() - 6, endPoint.y() - 10)
        << QPointF(endPoint.x() + 6, endPoint.y() - 10);
  painter->drawPolygon(arrow);
}

void BTEdgeItem::updatePositions() {
  prepareGeometryChange();

  QPointF start = parentNode_->bottomConnectionPoint();
  QPointF end = childNode_->topConnectionPoint();

  // Create a smooth curve
  path_ = QPainterPath();
  path_.moveTo(start);

  qreal midY = (start.y() + end.y()) / 2;
  path_.cubicTo(QPointF(start.x(), midY),
                QPointF(end.x(), midY),
                QPointF(end.x(), end.y() - 10));  // Stop before arrow

  update();
}

}  // namespace ros_weaver
