#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QGraphicsScene>
#include <QInputDialog>
#include <QApplication>

namespace ros_weaver {

NodeGroup::NodeGroup(const QString& title, QGraphicsItem* parent)
    : QGraphicsObject(parent),
      id_(QUuid::createUuid()),
      title_(title),
      color_(QColor(80, 120, 180, 180)),  // Semi-transparent blue
      size_(300.0, 200.0),
      isSelected_(false),
      isDragging_(false),
      isResizing_(false),
      activeHandle_(ResizeHandle::None) {
  setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable |
           QGraphicsItem::ItemSendsGeometryChanges);
  setAcceptHoverEvents(true);
  setZValue(-100);  // Render behind nodes
}

NodeGroup::~NodeGroup() = default;

QRectF NodeGroup::boundingRect() const {
  // Include resize handles in bounding rect
  return QRectF(-RESIZE_HANDLE_SIZE / 2, -RESIZE_HANDLE_SIZE / 2,
                size_.width() + RESIZE_HANDLE_SIZE,
                size_.height() + RESIZE_HANDLE_SIZE);
}

void NodeGroup::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                      QWidget* widget) {
  Q_UNUSED(option);
  Q_UNUSED(widget);

  painter->setRenderHint(QPainter::Antialiasing);

  // Main background
  QRectF mainRect(0, 0, size_.width(), size_.height());

  // Fill with semi-transparent color
  QColor fillColor = color_;
  fillColor.setAlpha(60);
  painter->setBrush(fillColor);

  // Border - thicker if selected
  QPen borderPen(color_.darker(120));
  borderPen.setWidth(isSelected_ ? 3 : 2);
  painter->setPen(borderPen);

  painter->drawRoundedRect(mainRect, CORNER_RADIUS, CORNER_RADIUS);

  // Header area
  QRectF headerRect(0, 0, size_.width(), HEADER_HEIGHT);
  QColor headerColor = color_;
  headerColor.setAlpha(150);
  painter->setBrush(headerColor);
  painter->setPen(Qt::NoPen);

  // Draw header with rounded top corners only
  QPainterPath headerPath;
  headerPath.moveTo(CORNER_RADIUS, HEADER_HEIGHT);
  headerPath.lineTo(0, HEADER_HEIGHT);
  headerPath.lineTo(0, CORNER_RADIUS);
  headerPath.arcTo(0, 0, CORNER_RADIUS * 2, CORNER_RADIUS * 2, 180, -90);
  headerPath.lineTo(size_.width() - CORNER_RADIUS, 0);
  headerPath.arcTo(size_.width() - CORNER_RADIUS * 2, 0, CORNER_RADIUS * 2, CORNER_RADIUS * 2, 90, -90);
  headerPath.lineTo(size_.width(), HEADER_HEIGHT);
  headerPath.closeSubpath();
  painter->drawPath(headerPath);

  // Title text
  painter->setPen(Qt::white);
  QFont titleFont = painter->font();
  titleFont.setBold(true);
  titleFont.setPointSize(11);
  painter->setFont(titleFont);

  QRectF titleRect(10, 0, size_.width() - 20, HEADER_HEIGHT);
  painter->drawText(titleRect, Qt::AlignVCenter | Qt::AlignLeft, title_);

  // Draw resize handles when selected
  if (isSelected_) {
    painter->setBrush(Qt::white);
    painter->setPen(QPen(color_.darker(150), 1));

    qreal hs = RESIZE_HANDLE_SIZE;
    qreal hw = hs / 2;

    // Corner handles
    painter->drawRect(QRectF(-hw, -hw, hs, hs));  // Top-left
    painter->drawRect(QRectF(size_.width() - hw, -hw, hs, hs));  // Top-right
    painter->drawRect(QRectF(-hw, size_.height() - hw, hs, hs));  // Bottom-left
    painter->drawRect(QRectF(size_.width() - hw, size_.height() - hw, hs, hs));  // Bottom-right

    // Edge handles
    painter->drawRect(QRectF(size_.width() / 2 - hw, -hw, hs, hs));  // Top
    painter->drawRect(QRectF(size_.width() / 2 - hw, size_.height() - hw, hs, hs));  // Bottom
    painter->drawRect(QRectF(-hw, size_.height() / 2 - hw, hs, hs));  // Left
    painter->drawRect(QRectF(size_.width() - hw, size_.height() / 2 - hw, hs, hs));  // Right
  }
}

void NodeGroup::setTitle(const QString& title) {
  if (title_ != title) {
    title_ = title;
    update();
    emit titleChanged(title);
  }
}

void NodeGroup::setColor(const QColor& color) {
  if (color_ != color) {
    color_ = color;
    update();
  }
}

void NodeGroup::setSize(const QSizeF& size) {
  prepareGeometryChange();
  size_.setWidth(qMax(MIN_WIDTH, size.width()));
  size_.setHeight(qMax(MIN_HEIGHT, size.height()));
  update();
}

void NodeGroup::fitToContents(qreal padding) {
  if (containedNodes_.isEmpty()) {
    return;
  }

  // Find bounding rect of all contained nodes
  QRectF bounds;
  bool first = true;
  for (PackageBlock* node : containedNodes_) {
    QRectF nodeRect = node->sceneBoundingRect();
    if (first) {
      bounds = nodeRect;
      first = false;
    } else {
      bounds = bounds.united(nodeRect);
    }
  }

  // Add padding and header height
  bounds.adjust(-padding, -padding - HEADER_HEIGHT, padding, padding);

  // Set position and size
  setPos(bounds.topLeft());
  setSize(bounds.size());
}

void NodeGroup::addNode(PackageBlock* node) {
  if (node && !containedNodes_.contains(node)) {
    containedNodes_.append(node);
    emit nodesChanged();
  }
}

void NodeGroup::removeNode(PackageBlock* node) {
  if (containedNodes_.removeOne(node)) {
    emit nodesChanged();
  }
}

void NodeGroup::clearNodes() {
  if (!containedNodes_.isEmpty()) {
    containedNodes_.clear();
    emit nodesChanged();
  }
}

bool NodeGroup::containsNode(PackageBlock* node) const {
  if (!node) return false;

  // Check if node center is within group bounds
  QRectF groupBounds(scenePos(), size_);
  groupBounds.setTop(groupBounds.top() + HEADER_HEIGHT);  // Exclude header
  QPointF nodeCenter = node->sceneBoundingRect().center();
  return groupBounds.contains(nodeCenter);
}

void NodeGroup::updateContainedNodes() {
  if (!scene()) return;

  QRectF groupBounds(scenePos(), size_);
  groupBounds.setTop(groupBounds.top() + HEADER_HEIGHT);

  QList<PackageBlock*> newContained;

  // Find all PackageBlocks that are within bounds
  QList<QGraphicsItem*> items = scene()->items(groupBounds, Qt::IntersectsItemBoundingRect);
  for (QGraphicsItem* item : items) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      // Check if center is inside
      if (groupBounds.contains(block->sceneBoundingRect().center())) {
        newContained.append(block);
      }
    }
  }

  if (newContained != containedNodes_) {
    containedNodes_ = newContained;
    emit nodesChanged();
  }
}

void NodeGroup::setGroupSelected(bool selected) {
  if (isSelected_ != selected) {
    isSelected_ = selected;
    update();
  }
}

QVariant NodeGroup::itemChange(GraphicsItemChange change, const QVariant& value) {
  if (change == ItemPositionChange && !isResizing_) {
    // Calculate the movement delta
    QPointF newPos = value.toPointF();
    QPointF delta = newPos - pos();

    // Move all contained nodes by the same delta
    if (!delta.isNull() && isDragging_) {
      for (PackageBlock* node : containedNodes_) {
        node->moveBy(delta.x(), delta.y());
      }
      emit groupMoved(delta);
    }
  } else if (change == ItemSelectedChange) {
    setGroupSelected(value.toBool());
  }

  return QGraphicsObject::itemChange(change, value);
}

void NodeGroup::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    ResizeHandle handle = hitTestResizeHandle(event->pos());
    if (handle != ResizeHandle::None && isSelected_) {
      isResizing_ = true;
      activeHandle_ = handle;
      lastMousePos_ = event->pos();
      event->accept();
      return;
    }

    isDragging_ = true;
    lastMousePos_ = event->pos();
  }

  QGraphicsObject::mousePressEvent(event);
}

void NodeGroup::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (isResizing_) {
    QPointF delta = event->pos() - lastMousePos_;
    applyResize(delta);
    lastMousePos_ = event->pos();
    event->accept();
    return;
  }

  QGraphicsObject::mouseMoveEvent(event);
}

void NodeGroup::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    isDragging_ = false;
    isResizing_ = false;
    activeHandle_ = ResizeHandle::None;
  }

  QGraphicsObject::mouseReleaseEvent(event);
}

void NodeGroup::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
  // Check if double-click is in header area
  if (event->pos().y() <= HEADER_HEIGHT) {
    bool ok;
    QString newTitle = QInputDialog::getText(
        nullptr, tr("Edit Group Title"),
        tr("Title:"), QLineEdit::Normal, title_, &ok);

    if (ok && !newTitle.isEmpty()) {
      setTitle(newTitle);
    }
    event->accept();
    return;
  }

  QGraphicsObject::mouseDoubleClickEvent(event);
}

void NodeGroup::hoverMoveEvent(QGraphicsSceneHoverEvent* event) {
  ResizeHandle handle = hitTestResizeHandle(event->pos());
  if (handle != ResizeHandle::None && isSelected_) {
    setCursor(cursorForHandle(handle));
  } else if (event->pos().y() <= HEADER_HEIGHT) {
    setCursor(Qt::SizeAllCursor);
  } else {
    setCursor(Qt::ArrowCursor);
  }

  QGraphicsObject::hoverMoveEvent(event);
}

void NodeGroup::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
  setCursor(Qt::ArrowCursor);
  QGraphicsObject::hoverLeaveEvent(event);
}

NodeGroup::ResizeHandle NodeGroup::hitTestResizeHandle(const QPointF& pos) const {
  if (!isSelected_) return ResizeHandle::None;

  qreal hs = RESIZE_HANDLE_SIZE;
  qreal hw = hs / 2;

  // Check corners first (they have priority)
  if (QRectF(-hw, -hw, hs, hs).contains(pos)) return ResizeHandle::TopLeft;
  if (QRectF(size_.width() - hw, -hw, hs, hs).contains(pos)) return ResizeHandle::TopRight;
  if (QRectF(-hw, size_.height() - hw, hs, hs).contains(pos)) return ResizeHandle::BottomLeft;
  if (QRectF(size_.width() - hw, size_.height() - hw, hs, hs).contains(pos)) return ResizeHandle::BottomRight;

  // Check edges
  if (QRectF(size_.width() / 2 - hw, -hw, hs, hs).contains(pos)) return ResizeHandle::Top;
  if (QRectF(size_.width() / 2 - hw, size_.height() - hw, hs, hs).contains(pos)) return ResizeHandle::Bottom;
  if (QRectF(-hw, size_.height() / 2 - hw, hs, hs).contains(pos)) return ResizeHandle::Left;
  if (QRectF(size_.width() - hw, size_.height() / 2 - hw, hs, hs).contains(pos)) return ResizeHandle::Right;

  return ResizeHandle::None;
}

QCursor NodeGroup::cursorForHandle(ResizeHandle handle) const {
  switch (handle) {
    case ResizeHandle::TopLeft:
    case ResizeHandle::BottomRight:
      return Qt::SizeFDiagCursor;
    case ResizeHandle::TopRight:
    case ResizeHandle::BottomLeft:
      return Qt::SizeBDiagCursor;
    case ResizeHandle::Top:
    case ResizeHandle::Bottom:
      return Qt::SizeVerCursor;
    case ResizeHandle::Left:
    case ResizeHandle::Right:
      return Qt::SizeHorCursor;
    default:
      return Qt::ArrowCursor;
  }
}

void NodeGroup::applyResize(const QPointF& delta) {
  prepareGeometryChange();

  QPointF newPos = pos();
  QSizeF newSize = size_;

  switch (activeHandle_) {
    case ResizeHandle::TopLeft:
      newPos += QPointF(delta.x(), delta.y());
      newSize += QSizeF(-delta.x(), -delta.y());
      break;
    case ResizeHandle::Top:
      newPos += QPointF(0, delta.y());
      newSize += QSizeF(0, -delta.y());
      break;
    case ResizeHandle::TopRight:
      newPos += QPointF(0, delta.y());
      newSize += QSizeF(delta.x(), -delta.y());
      break;
    case ResizeHandle::Left:
      newPos += QPointF(delta.x(), 0);
      newSize += QSizeF(-delta.x(), 0);
      break;
    case ResizeHandle::Right:
      newSize += QSizeF(delta.x(), 0);
      break;
    case ResizeHandle::BottomLeft:
      newPos += QPointF(delta.x(), 0);
      newSize += QSizeF(-delta.x(), delta.y());
      break;
    case ResizeHandle::Bottom:
      newSize += QSizeF(0, delta.y());
      break;
    case ResizeHandle::BottomRight:
      newSize += QSizeF(delta.x(), delta.y());
      break;
    default:
      break;
  }

  // Enforce minimum size
  if (newSize.width() >= MIN_WIDTH && newSize.height() >= MIN_HEIGHT) {
    setPos(newPos);
    size_ = newSize;
    update();
  }
}

}  // namespace ros_weaver
