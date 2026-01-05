#include "ros_weaver/widgets/minimap_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"

#include <QPainter>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QScrollBar>
#include <QGraphicsScene>
#include <cmath>

namespace ros_weaver {

MinimapPanel::MinimapPanel(QWidget* parent)
  : QWidget(parent)
  , canvas_(nullptr)
  , scale_(1.0)
  , isDragging_(false)
  , refreshTimer_(new QTimer(this))
{
  setMinimumSize(MIN_SIZE, MIN_SIZE);
  setFixedSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

  // Dark background
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(25, 25, 25));
  setPalette(pal);

  // Cursor for indicating clickable area
  setCursor(Qt::CrossCursor);

  // Periodic refresh for live updates
  connect(refreshTimer_, &QTimer::timeout, this, &MinimapPanel::onRefreshTimer);
  refreshTimer_->start(100);  // 10 FPS refresh
}

MinimapPanel::~MinimapPanel() = default;

void MinimapPanel::setCanvas(WeaverCanvas* canvas) {
  if (canvas_ == canvas) return;

  // Disconnect from old canvas
  if (canvas_) {
    disconnect(canvas_->scene(), nullptr, this, nullptr);
    disconnect(canvas_, nullptr, this, nullptr);
  }

  canvas_ = canvas;

  if (canvas_) {
    // Connect to scene changes
    connect(canvas_->scene(), &QGraphicsScene::changed,
            this, &MinimapPanel::onCanvasChanged);
    connect(canvas_->scene(), &QGraphicsScene::sceneRectChanged,
            this, &MinimapPanel::onCanvasChanged);

    updateTransform();
  }

  update();
}

void MinimapPanel::refresh() {
  if (canvas_) {
    updateTransform();
    update();
  }
}

void MinimapPanel::onCanvasChanged() {
  updateTransform();
  update();
}

void MinimapPanel::onRefreshTimer() {
  // Lightweight refresh - just redraw to update viewport position
  if (canvas_) {
    update();
  }
}

void MinimapPanel::updateTransform() {
  if (!canvas_ || !canvas_->scene()) return;

  // Calculate content bounds from all items
  QRectF itemsBounds;
  bool hasItems = false;

  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (dynamic_cast<PackageBlock*>(item) ||
        dynamic_cast<NodeGroup*>(item)) {
      if (!hasItems) {
        itemsBounds = item->sceneBoundingRect();
        hasItems = true;
      } else {
        itemsBounds = itemsBounds.united(item->sceneBoundingRect());
      }
    }
  }

  if (!hasItems) {
    // Default bounds if no items
    itemsBounds = QRectF(-500, -500, 1000, 1000);
  }

  // Add padding
  qreal padding = 100;
  itemsBounds.adjust(-padding, -padding, padding, padding);
  contentBounds_ = itemsBounds;

  // Calculate scale to fit content in minimap
  QRectF availableRect(MARGIN, MARGIN,
                       width() - 2 * MARGIN,
                       height() - 2 * MARGIN);

  qreal scaleX = availableRect.width() / contentBounds_.width();
  qreal scaleY = availableRect.height() / contentBounds_.height();
  scale_ = std::min(scaleX, scaleY);

  // Calculate offset to center content
  qreal scaledWidth = contentBounds_.width() * scale_;
  qreal scaledHeight = contentBounds_.height() * scale_;

  offset_ = QPointF(
    (width() - scaledWidth) / 2.0 - contentBounds_.left() * scale_,
    (height() - scaledHeight) / 2.0 - contentBounds_.top() * scale_
  );
}

QPointF MinimapPanel::mapToCanvas(const QPointF& widgetPos) const {
  if (qFuzzyIsNull(scale_)) return QPointF();
  return QPointF(
    (widgetPos.x() - offset_.x()) / scale_,
    (widgetPos.y() - offset_.y()) / scale_
  );
}

QPointF MinimapPanel::mapFromCanvas(const QPointF& canvasPos) const {
  return QPointF(
    canvasPos.x() * scale_ + offset_.x(),
    canvasPos.y() * scale_ + offset_.y()
  );
}

QRectF MinimapPanel::getViewportRect() const {
  if (!canvas_) return QRectF();

  // Get the visible area in scene coordinates
  QRectF viewRect = canvas_->mapToScene(canvas_->viewport()->rect()).boundingRect();
  return viewRect;
}

void MinimapPanel::centerCanvasOn(const QPointF& canvasPos) {
  if (!canvas_) return;
  canvas_->centerOn(canvasPos);
}

void MinimapPanel::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event)

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw border
  painter.setPen(QPen(QColor(60, 60, 60), 1));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(rect().adjusted(0, 0, -1, -1));

  if (!canvas_ || !canvas_->scene()) return;

  // Draw node groups (behind blocks)
  painter.setPen(Qt::NoPen);
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (NodeGroup* group = dynamic_cast<NodeGroup*>(item)) {
      QRectF groupRect = group->sceneBoundingRect();
      QRectF minimapRect(mapFromCanvas(groupRect.topLeft()),
                         mapFromCanvas(groupRect.bottomRight()));

      QColor groupColor = group->color();
      groupColor.setAlpha(80);
      painter.setBrush(groupColor);
      painter.drawRect(minimapRect);
    }
  }

  // Draw connections
  painter.setPen(QPen(QColor(80, 150, 80, 150), 1));
  painter.setBrush(Qt::NoBrush);
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      if (conn->sourceBlock() && conn->targetBlock()) {
        QPointF start = mapFromCanvas(conn->sourceBlock()->scenePos() +
                                       QPointF(90, 30));  // Approximate center-right
        QPointF end = mapFromCanvas(conn->targetBlock()->scenePos() +
                                     QPointF(0, 30));     // Approximate center-left
        painter.drawLine(start, end);
      }
    }
  }

  // Draw package blocks
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      QRectF blockRect = block->sceneBoundingRect();
      QRectF minimapRect(mapFromCanvas(blockRect.topLeft()),
                         mapFromCanvas(blockRect.bottomRight()));

      // Minimum size for visibility
      if (minimapRect.width() < 4) {
        qreal center = minimapRect.center().x();
        minimapRect.setLeft(center - 2);
        minimapRect.setRight(center + 2);
      }
      if (minimapRect.height() < 4) {
        qreal center = minimapRect.center().y();
        minimapRect.setTop(center - 2);
        minimapRect.setBottom(center + 2);
      }

      // Draw block
      QColor blockColor(42, 130, 218);
      if (block->isSelected()) {
        blockColor = blockColor.lighter(130);
      }
      painter.setBrush(blockColor);
      painter.setPen(QPen(blockColor.darker(120), 1));
      painter.drawRect(minimapRect);
    }
  }

  // Draw viewport rectangle
  QRectF viewportRect = getViewportRect();
  if (!viewportRect.isEmpty()) {
    QRectF minimapViewport(mapFromCanvas(viewportRect.topLeft()),
                           mapFromCanvas(viewportRect.bottomRight()));

    // Viewport fill (semi-transparent)
    painter.setBrush(QColor(255, 255, 255, 30));
    painter.setPen(QPen(QColor(255, 255, 255, 200), 2));
    painter.drawRect(minimapViewport);

    // Draw corner handles
    qreal handleSize = 4;
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.setPen(Qt::NoPen);

    // Top-left
    painter.drawRect(QRectF(minimapViewport.topLeft(),
                            QSizeF(handleSize, handleSize)));
    // Top-right
    painter.drawRect(QRectF(minimapViewport.topRight() - QPointF(handleSize, 0),
                            QSizeF(handleSize, handleSize)));
    // Bottom-left
    painter.drawRect(QRectF(minimapViewport.bottomLeft() - QPointF(0, handleSize),
                            QSizeF(handleSize, handleSize)));
    // Bottom-right
    painter.drawRect(QRectF(minimapViewport.bottomRight() - QPointF(handleSize, handleSize),
                            QSizeF(handleSize, handleSize)));
  }

  // Draw "MINIMAP" label
  painter.setPen(QColor(100, 100, 100));
  painter.setFont(QFont("Sans", 7));
  painter.drawText(QRectF(0, height() - 15, width(), 15),
                   Qt::AlignCenter, tr("MINIMAP"));
}

void MinimapPanel::mousePressEvent(QMouseEvent* event) {
  if (!canvas_) return;

  if (event->button() == Qt::LeftButton) {
    QPointF canvasPos = mapToCanvas(event->pos());

    // Check if clicking inside viewport rect
    QRectF viewportRect = getViewportRect();
    if (viewportRect.contains(canvasPos)) {
      // Start dragging the viewport
      isDragging_ = true;
      dragStartPos_ = event->pos();
      dragStartViewport_ = viewportRect.center();
      setCursor(Qt::ClosedHandCursor);
    } else {
      // Click to navigate - center on clicked position
      centerCanvasOn(canvasPos);
    }

    event->accept();
  }
}

void MinimapPanel::mouseMoveEvent(QMouseEvent* event) {
  if (!canvas_) return;

  if (isDragging_) {
    // Calculate delta in canvas coordinates
    QPointF currentCanvasPos = mapToCanvas(event->pos());
    QPointF startCanvasPos = mapToCanvas(dragStartPos_);
    QPointF delta = currentCanvasPos - startCanvasPos;

    // Move viewport
    centerCanvasOn(dragStartViewport_ + delta);

    event->accept();
  }
}

void MinimapPanel::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton && isDragging_) {
    isDragging_ = false;
    setCursor(Qt::CrossCursor);
    event->accept();
  }
}

void MinimapPanel::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateTransform();
}

}  // namespace ros_weaver
