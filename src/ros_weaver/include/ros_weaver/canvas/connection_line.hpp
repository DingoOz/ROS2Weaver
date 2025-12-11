#ifndef ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
#define ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP

#include <QGraphicsObject>
#include <QPainter>
#include <QPainterPath>
#include <QUuid>
#include <QTimer>
#include <QPropertyAnimation>

namespace ros_weaver {

class PackageBlock;

class ConnectionLine : public QGraphicsObject {
  Q_OBJECT
  Q_PROPERTY(qreal pulsePhase READ pulsePhase WRITE setPulsePhase)

public:
  ConnectionLine(PackageBlock* sourceBlock, int sourcePin,
                 PackageBlock* targetBlock, int targetPin,
                 QGraphicsItem* parent = nullptr);
  ~ConnectionLine() override;

  // QGraphicsItem interface
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  // Source and target blocks
  PackageBlock* sourceBlock() const { return sourceBlock_; }
  PackageBlock* targetBlock() const { return targetBlock_; }

  int sourcePinIndex() const { return sourcePinIndex_; }
  int targetPinIndex() const { return targetPinIndex_; }

  QUuid id() const { return id_; }

  // Update the path when blocks move
  void updatePath();

  // Visual customization
  void setHighlighted(bool highlighted);
  bool isHighlighted() const { return isHighlighted_; }

  void setConnectionColor(const QColor& color);
  QColor connectionColor() const { return connectionColor_; }

  // Pulse animation
  qreal pulsePhase() const { return pulsePhase_; }
  void setPulsePhase(qreal phase);

  // External animation control (for pin hover highlighting)
  void setPinHighlighted(bool highlighted);
  bool isPinHighlighted() const { return isPinHighlighted_; }

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
  void startPulseAnimation();
  void stopPulseAnimation();
  QPainterPath calculatePath() const;

  QUuid id_;
  PackageBlock* sourceBlock_;
  PackageBlock* targetBlock_;
  int sourcePinIndex_;
  int targetPinIndex_;

  bool isHighlighted_;
  bool isHovered_;
  bool isPinHighlighted_;  // Highlighted due to pin hover
  QColor connectionColor_;
  QPainterPath path_;

  // Animation
  QPropertyAnimation* pulseAnimation_;
  qreal pulsePhase_;

  static constexpr qreal LINE_WIDTH = 2.0;
  static constexpr qreal HIGHLIGHT_WIDTH = 4.0;
  static constexpr qreal HOVER_WIDTH = 3.0;
  static constexpr qreal CURVE_OFFSET = 50.0;
  static constexpr qreal HIT_TOLERANCE = 10.0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
