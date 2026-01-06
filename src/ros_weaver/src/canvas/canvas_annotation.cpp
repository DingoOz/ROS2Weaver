#include "ros_weaver/canvas/canvas_annotation.hpp"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QAction>
#include <QInputDialog>
#include <QCursor>
#include <cmath>

namespace ros_weaver {

CanvasAnnotation::CanvasAnnotation(QGraphicsItem* parent)
  : QGraphicsObject(parent)
  , id_(QUuid::createUuid())
  , createdAt_(QDateTime::currentDateTime())
  , modifiedAt_(QDateTime::currentDateTime())
{
  setFlags(QGraphicsItem::ItemIsMovable |
           QGraphicsItem::ItemIsSelectable |
           QGraphicsItem::ItemSendsGeometryChanges);
  setAcceptHoverEvents(true);
  setCursor(Qt::OpenHandCursor);

  font_.setFamily("Sans Serif");
  font_.setPixelSize(11);

  updateAppearance();
}

CanvasAnnotation::~CanvasAnnotation() = default;

QRectF CanvasAnnotation::boundingRect() const {
  return QRectF(0, 0, size_.width(), size_.height());
}

void CanvasAnnotation::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget*) {
  painter->setRenderHint(QPainter::Antialiasing, true);

  QRectF rect = boundingRect();
  bool selected = option->state & QStyle::State_Selected;

  // Shadow
  if (hovered_ || selected) {
    QRectF shadowRect = rect.translated(3, 3);
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(0, 0, 0, 40));
    painter->drawRoundedRect(shadowRect, CORNER_RADIUS, CORNER_RADIUS);
  }

  // Background
  painter->setPen(QPen(backgroundColor_.darker(120), selected ? 2.0 : 1.0));
  painter->setBrush(backgroundColor_);
  painter->drawRoundedRect(rect, CORNER_RADIUS, CORNER_RADIUS);

  // Header
  drawHeader(painter, rect);

  // Content
  QRectF contentRect(PADDING, HEADER_HEIGHT + PADDING,
                     rect.width() - 2 * PADDING,
                     rect.height() - HEADER_HEIGHT - 2 * PADDING);
  drawContent(painter, contentRect);

  // Resize handle
  if (hovered_ && !pinned_) {
    drawResizeHandle(painter);
  }

  // Pinned indicator
  if (pinned_) {
    painter->setPen(QPen(QColor(100, 100, 100), 1.5));
    QPointF pinPos(rect.width() - 18, 6);
    painter->drawLine(pinPos, pinPos + QPointF(0, 12));
    painter->drawEllipse(pinPos + QPointF(0, -3), 3, 3);
  }
}

void CanvasAnnotation::drawHeader(QPainter* painter, const QRectF& rect) {
  // Header background
  QRectF headerRect(0, 0, rect.width(), HEADER_HEIGHT);
  QPainterPath headerPath;
  headerPath.addRoundedRect(headerRect, CORNER_RADIUS, CORNER_RADIUS);

  // Clip to top corners only
  QPainterPath clipPath;
  clipPath.addRect(QRectF(0, CORNER_RADIUS, rect.width(), HEADER_HEIGHT));
  headerPath = headerPath.united(clipPath);

  painter->setPen(Qt::NoPen);
  painter->setBrush(backgroundColor_.darker(110));
  painter->drawPath(headerPath);

  // Type icon
  QString icon = typeToIcon(type_);
  painter->setFont(QFont("Sans Serif", 12));
  painter->setPen(backgroundColor_.darker(150));
  painter->drawText(QRectF(6, 2, 20, HEADER_HEIGHT - 4), Qt::AlignCenter, icon);

  // Type label
  QString typeLabel = typeToString(type_);
  painter->setFont(QFont("Sans Serif", 9, QFont::Bold));
  painter->drawText(QRectF(26, 0, rect.width() - 50, HEADER_HEIGHT),
                    Qt::AlignLeft | Qt::AlignVCenter, typeLabel);
}

void CanvasAnnotation::drawContent(QPainter* painter, const QRectF& rect) {
  painter->setFont(font_);
  painter->setPen(QColor(40, 40, 40));

  QTextOption textOption;
  textOption.setWrapMode(QTextOption::WordWrap);
  textOption.setAlignment(Qt::AlignTop | Qt::AlignLeft);

  QString displayText = text_;
  if (displayText.isEmpty()) {
    displayText = tr("Double-click to edit...");
    painter->setPen(QColor(120, 120, 120));
  }

  painter->drawText(rect, displayText, textOption);

  // Author and timestamp at bottom if there's room
  if (rect.height() > 60 && !author_.isEmpty()) {
    painter->setFont(QFont("Sans Serif", 8));
    painter->setPen(QColor(100, 100, 100));
    QString info = QString("%1 - %2").arg(author_).arg(modifiedAt_.toString("MMM d"));
    painter->drawText(QRectF(rect.left(), rect.bottom() - 14, rect.width(), 14),
                      Qt::AlignRight | Qt::AlignBottom, info);
  }
}

void CanvasAnnotation::drawResizeHandle(QPainter* painter) {
  QRectF rect = boundingRect();
  QPointF corner(rect.right() - RESIZE_HANDLE_SIZE, rect.bottom() - RESIZE_HANDLE_SIZE);

  painter->setPen(QPen(backgroundColor_.darker(140), 1.5));
  for (int i = 0; i < 3; ++i) {
    qreal offset = i * 4;
    painter->drawLine(corner + QPointF(RESIZE_HANDLE_SIZE - offset, RESIZE_HANDLE_SIZE),
                      corner + QPointF(RESIZE_HANDLE_SIZE, RESIZE_HANDLE_SIZE - offset));
  }
}

bool CanvasAnnotation::isOverResizeHandle(const QPointF& pos) const {
  if (pinned_) return false;
  QRectF rect = boundingRect();
  QRectF handleRect(rect.right() - RESIZE_HANDLE_SIZE, rect.bottom() - RESIZE_HANDLE_SIZE,
                    RESIZE_HANDLE_SIZE, RESIZE_HANDLE_SIZE);
  return handleRect.contains(pos);
}

void CanvasAnnotation::setText(const QString& text) {
  if (text_ != text) {
    text_ = text;
    modifiedAt_ = QDateTime::currentDateTime();
    update();
    emit textChanged(text);
    emit annotationModified();
  }
}

void CanvasAnnotation::setAnnotationType(AnnotationType type) {
  if (type_ != type) {
    type_ = type;
    updateAppearance();
    emit typeChanged(type);
    emit annotationModified();
  }
}

void CanvasAnnotation::setBackgroundColor(const QColor& color) {
  if (backgroundColor_ != color) {
    backgroundColor_ = color;
    update();
    emit annotationModified();
  }
}

void CanvasAnnotation::setPinned(bool pinned) {
  if (pinned_ != pinned) {
    pinned_ = pinned;
    setFlag(QGraphicsItem::ItemIsMovable, !pinned);
    update();
    emit pinnedChanged(pinned);
    emit annotationModified();
  }
}

void CanvasAnnotation::setSize(const QSizeF& size) {
  QSizeF newSize(std::max(size.width(), MIN_WIDTH),
                 std::max(size.height(), MIN_HEIGHT));
  if (size_ != newSize) {
    prepareGeometryChange();
    size_ = newSize;
    update();
    emit annotationModified();
  }
}

void CanvasAnnotation::setAuthor(const QString& author) {
  if (author_ != author) {
    author_ = author;
    update();
  }
}

void CanvasAnnotation::updateAppearance() {
  backgroundColor_ = typeToColor(type_);
  update();
}

void CanvasAnnotation::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    if (isOverResizeHandle(event->pos())) {
      resizing_ = true;
      resizeStartPos_ = event->pos();
      resizeStartSize_ = size_;
      event->accept();
      return;
    }
    setCursor(Qt::ClosedHandCursor);
  }
  QGraphicsObject::mousePressEvent(event);
}

void CanvasAnnotation::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (resizing_) {
    QPointF delta = event->pos() - resizeStartPos_;
    setSize(QSizeF(resizeStartSize_.width() + delta.x(),
                   resizeStartSize_.height() + delta.y()));
    event->accept();
    return;
  }
  QGraphicsObject::mouseMoveEvent(event);
}

void CanvasAnnotation::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (resizing_) {
    resizing_ = false;
    event->accept();
    return;
  }
  setCursor(pinned_ ? Qt::ArrowCursor : Qt::OpenHandCursor);
  QGraphicsObject::mouseReleaseEvent(event);
}

void CanvasAnnotation::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    bool ok;
    QString newText = QInputDialog::getMultiLineText(
      nullptr,
      tr("Edit Annotation"),
      tr("Enter annotation text:"),
      text_,
      &ok
    );
    if (ok) {
      setText(newText);
    }
    event->accept();
    return;
  }
  QGraphicsObject::mouseDoubleClickEvent(event);
}

void CanvasAnnotation::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
  hovered_ = true;
  update();
  QGraphicsObject::hoverEnterEvent(event);
}

void CanvasAnnotation::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
  hovered_ = false;
  update();
  QGraphicsObject::hoverLeaveEvent(event);
}

void CanvasAnnotation::contextMenuEvent(QGraphicsSceneContextMenuEvent* event) {
  QMenu menu;

  // Edit action
  QAction* editAction = menu.addAction(tr("Edit..."));
  connect(editAction, &QAction::triggered, this, [this]() {
    bool ok;
    QString newText = QInputDialog::getMultiLineText(
      nullptr, tr("Edit Annotation"), tr("Enter annotation text:"), text_, &ok);
    if (ok) setText(newText);
  });

  menu.addSeparator();

  // Type submenu
  QMenu* typeMenu = menu.addMenu(tr("Type"));
  QAction* noteAction = typeMenu->addAction(tr("Note"));
  QAction* infoAction = typeMenu->addAction(tr("Info"));
  QAction* warningAction = typeMenu->addAction(tr("Warning"));
  QAction* todoAction = typeMenu->addAction(tr("Todo"));
  QAction* questionAction = typeMenu->addAction(tr("Question"));
  QAction* importantAction = typeMenu->addAction(tr("Important"));

  connect(noteAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Note); });
  connect(infoAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Info); });
  connect(warningAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Warning); });
  connect(todoAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Todo); });
  connect(questionAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Question); });
  connect(importantAction, &QAction::triggered, this, [this]() { setAnnotationType(AnnotationType::Important); });

  menu.addSeparator();

  // Pin/Unpin
  QAction* pinAction = menu.addAction(pinned_ ? tr("Unpin") : tr("Pin"));
  connect(pinAction, &QAction::triggered, this, [this]() { setPinned(!pinned_); });

  menu.addSeparator();

  // Delete
  QAction* deleteAction = menu.addAction(tr("Delete"));
  connect(deleteAction, &QAction::triggered, this, &CanvasAnnotation::deleteRequested);

  menu.exec(event->screenPos());
  event->accept();
}

QVariant CanvasAnnotation::itemChange(GraphicsItemChange change, const QVariant& value) {
  if (change == ItemPositionHasChanged) {
    emit annotationModified();
  }
  return QGraphicsObject::itemChange(change, value);
}

QVariantMap CanvasAnnotation::toVariantMap() const {
  QVariantMap map;
  map["id"] = id_.toString();
  map["text"] = text_;
  map["type"] = typeToString(type_);
  map["pinned"] = pinned_;
  map["x"] = pos().x();
  map["y"] = pos().y();
  map["width"] = size_.width();
  map["height"] = size_.height();
  map["author"] = author_;
  map["createdAt"] = createdAt_.toString(Qt::ISODate);
  map["modifiedAt"] = modifiedAt_.toString(Qt::ISODate);
  return map;
}

CanvasAnnotation* CanvasAnnotation::fromVariantMap(const QVariantMap& map, QGraphicsItem* parent) {
  CanvasAnnotation* annotation = new CanvasAnnotation(parent);

  if (map.contains("id")) {
    annotation->id_ = QUuid(map["id"].toString());
  }
  if (map.contains("text")) {
    annotation->text_ = map["text"].toString();
  }
  if (map.contains("type")) {
    annotation->type_ = stringToType(map["type"].toString());
    annotation->updateAppearance();
  }
  if (map.contains("pinned")) {
    annotation->setPinned(map["pinned"].toBool());
  }
  if (map.contains("x") && map.contains("y")) {
    annotation->setPos(map["x"].toDouble(), map["y"].toDouble());
  }
  if (map.contains("width") && map.contains("height")) {
    annotation->size_ = QSizeF(map["width"].toDouble(), map["height"].toDouble());
  }
  if (map.contains("author")) {
    annotation->author_ = map["author"].toString();
  }
  if (map.contains("createdAt")) {
    annotation->createdAt_ = QDateTime::fromString(map["createdAt"].toString(), Qt::ISODate);
  }
  if (map.contains("modifiedAt")) {
    annotation->modifiedAt_ = QDateTime::fromString(map["modifiedAt"].toString(), Qt::ISODate);
  }

  return annotation;
}

QString CanvasAnnotation::typeToString(AnnotationType type) {
  switch (type) {
    case AnnotationType::Note: return "Note";
    case AnnotationType::Info: return "Info";
    case AnnotationType::Warning: return "Warning";
    case AnnotationType::Todo: return "Todo";
    case AnnotationType::Question: return "Question";
    case AnnotationType::Important: return "Important";
  }
  return "Note";
}

AnnotationType CanvasAnnotation::stringToType(const QString& str) {
  if (str == "Info") return AnnotationType::Info;
  if (str == "Warning") return AnnotationType::Warning;
  if (str == "Todo") return AnnotationType::Todo;
  if (str == "Question") return AnnotationType::Question;
  if (str == "Important") return AnnotationType::Important;
  return AnnotationType::Note;
}

QColor CanvasAnnotation::typeToColor(AnnotationType type) {
  switch (type) {
    case AnnotationType::Note: return QColor(255, 248, 187);      // Yellow
    case AnnotationType::Info: return QColor(187, 222, 251);      // Blue
    case AnnotationType::Warning: return QColor(255, 224, 178);   // Orange
    case AnnotationType::Todo: return QColor(200, 230, 201);      // Green
    case AnnotationType::Question: return QColor(225, 190, 231);  // Purple
    case AnnotationType::Important: return QColor(255, 205, 210); // Red
  }
  return QColor(255, 248, 187);
}

QString CanvasAnnotation::typeToIcon(AnnotationType type) {
  switch (type) {
    case AnnotationType::Note: return QString::fromUtf8("\xF0\x9F\x93\x9D");      // Memo
    case AnnotationType::Info: return QString::fromUtf8("\xE2\x84\xB9");          // Info
    case AnnotationType::Warning: return QString::fromUtf8("\xE2\x9A\xA0");       // Warning
    case AnnotationType::Todo: return QString::fromUtf8("\xE2\x9C\x93");          // Checkmark
    case AnnotationType::Question: return QString::fromUtf8("\xE2\x9D\x93");      // Question
    case AnnotationType::Important: return QString::fromUtf8("\xE2\x9D\x97");     // Exclamation
  }
  return QString::fromUtf8("\xF0\x9F\x93\x9D");
}

}  // namespace ros_weaver
