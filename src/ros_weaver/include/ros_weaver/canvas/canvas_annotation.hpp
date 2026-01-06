#ifndef ROS_WEAVER_CANVAS_ANNOTATION_HPP
#define ROS_WEAVER_CANVAS_ANNOTATION_HPP

#include <QGraphicsObject>
#include <QString>
#include <QColor>
#include <QFont>
#include <QUuid>
#include <QDateTime>

namespace ros_weaver {

/**
 * @brief Type of annotation/sticky note
 */
enum class AnnotationType {
  Note,       // General note (yellow)
  Info,       // Information (blue)
  Warning,    // Warning/caution (orange)
  Todo,       // Todo item (green)
  Question,   // Question/unclear (purple)
  Important   // Important/critical (red)
};

/**
 * @brief A canvas annotation/sticky note for documentation
 *
 * Features:
 * - Rich text content support
 * - Multiple visual styles (Note, Info, Warning, Todo, etc.)
 * - Resizable and movable
 * - Can be pinned (locked position)
 * - Author and timestamp tracking
 */
class CanvasAnnotation : public QGraphicsObject {
  Q_OBJECT
  Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)
  Q_PROPERTY(AnnotationType annotationType READ annotationType WRITE setAnnotationType NOTIFY typeChanged)
  Q_PROPERTY(bool pinned READ isPinned WRITE setPinned NOTIFY pinnedChanged)

public:
  explicit CanvasAnnotation(QGraphicsItem* parent = nullptr);
  ~CanvasAnnotation() override;

  // QGraphicsItem interface
  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  // Properties
  QUuid id() const { return id_; }
  QString text() const { return text_; }
  void setText(const QString& text);

  AnnotationType annotationType() const { return type_; }
  void setAnnotationType(AnnotationType type);

  QColor backgroundColor() const { return backgroundColor_; }
  void setBackgroundColor(const QColor& color);

  bool isPinned() const { return pinned_; }
  void setPinned(bool pinned);

  QSizeF size() const { return size_; }
  void setSize(const QSizeF& size);

  QString author() const { return author_; }
  void setAuthor(const QString& author);

  QDateTime createdAt() const { return createdAt_; }
  QDateTime modifiedAt() const { return modifiedAt_; }

  // Serialization
  QVariantMap toVariantMap() const;
  static CanvasAnnotation* fromVariantMap(const QVariantMap& map, QGraphicsItem* parent = nullptr);

  // Type utilities
  static QString typeToString(AnnotationType type);
  static AnnotationType stringToType(const QString& str);
  static QColor typeToColor(AnnotationType type);
  static QString typeToIcon(AnnotationType type);

signals:
  void textChanged(const QString& text);
  void typeChanged(AnnotationType type);
  void pinnedChanged(bool pinned);
  void annotationModified();
  void deleteRequested();

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
  void contextMenuEvent(QGraphicsSceneContextMenuEvent* event) override;

  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

private:
  void updateAppearance();
  void drawHeader(QPainter* painter, const QRectF& rect);
  void drawContent(QPainter* painter, const QRectF& rect);
  void drawResizeHandle(QPainter* painter);
  bool isOverResizeHandle(const QPointF& pos) const;

  QUuid id_;
  QString text_;
  AnnotationType type_ = AnnotationType::Note;
  QColor backgroundColor_;
  bool pinned_ = false;
  QSizeF size_{200, 150};
  QString author_;
  QDateTime createdAt_;
  QDateTime modifiedAt_;
  QFont font_;

  // Interaction state
  bool hovered_ = false;
  bool resizing_ = false;
  QPointF resizeStartPos_;
  QSizeF resizeStartSize_;

  // Layout constants
  static constexpr qreal HEADER_HEIGHT = 24.0;
  static constexpr qreal CORNER_RADIUS = 6.0;
  static constexpr qreal PADDING = 8.0;
  static constexpr qreal RESIZE_HANDLE_SIZE = 12.0;
  static constexpr qreal MIN_WIDTH = 100.0;
  static constexpr qreal MIN_HEIGHT = 80.0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_ANNOTATION_HPP
