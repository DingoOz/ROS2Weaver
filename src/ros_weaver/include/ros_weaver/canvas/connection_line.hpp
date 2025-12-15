#ifndef ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
#define ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP

#include <QGraphicsObject>
#include <QPainter>
#include <QPainterPath>
#include <QUuid>
#include <QTimer>
#include <QPropertyAnimation>
#include <QFont>

namespace ros_weaver {

class PackageBlock;

/**
 * @brief Activity state for live topic visualization
 */
enum class TopicActivityState {
  Unknown,    // Not monitoring
  Inactive,   // Monitoring but no messages
  Active,     // Receiving messages
  HighRate    // Receiving at high rate (> 50 Hz)
};

class ConnectionLine : public QGraphicsObject {
  Q_OBJECT
  Q_PROPERTY(qreal pulsePhase READ pulsePhase WRITE setPulsePhase)
  Q_PROPERTY(qreal dataFlowPhase READ dataFlowPhase WRITE setDataFlowPhase)
  Q_PROPERTY(qreal activityGlow READ activityGlow WRITE setActivityGlow)

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
  void setId(const QUuid& id) { id_ = id; }  // For undo/redo support

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

  // Live topic monitoring
  void setTopicName(const QString& topicName);
  QString topicName() const { return topicName_; }

  void setMessageType(const QString& messageType);
  QString messageType() const { return messageType_; }

  void setActivityState(TopicActivityState state);
  TopicActivityState activityState() const { return activityState_; }

  void setMessageRate(double rateHz);
  double messageRate() const { return messageRate_; }

  void setShowRateLabel(bool show);
  bool showRateLabel() const { return showRateLabel_; }

  void setLiveMonitoringEnabled(bool enabled);
  bool isLiveMonitoringEnabled() const { return liveMonitoringEnabled_; }

  // Data flow animation property
  qreal dataFlowPhase() const { return dataFlowPhase_; }
  void setDataFlowPhase(qreal phase);

  // Activity glow property
  qreal activityGlow() const { return activityGlow_; }
  void setActivityGlow(qreal glow);

  // Trigger activity pulse (called when message received)
  void pulseActivity();

signals:
  void clicked(ConnectionLine* connection);
  void doubleClicked(ConnectionLine* connection);

protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;

private:
  void startPulseAnimation();
  void stopPulseAnimation();
  void startDataFlowAnimation();
  void stopDataFlowAnimation();
  void drawRateLabel(QPainter* painter);
  void drawActivityIndicator(QPainter* painter);
  QColor getActivityColor() const;
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
  QPropertyAnimation* dataFlowAnimation_;
  QPropertyAnimation* activityGlowAnimation_;
  qreal pulsePhase_;
  qreal dataFlowPhase_;
  qreal activityGlow_;

  // Live topic monitoring
  QString topicName_;
  QString messageType_;
  TopicActivityState activityState_;
  double messageRate_;
  bool showRateLabel_;
  bool liveMonitoringEnabled_;

  static constexpr qreal LINE_WIDTH = 2.0;
  static constexpr qreal HIGHLIGHT_WIDTH = 4.0;
  static constexpr qreal HOVER_WIDTH = 3.0;
  static constexpr qreal CURVE_OFFSET = 50.0;
  static constexpr qreal HIT_TOLERANCE = 10.0;

public:
  // Rate thresholds for visual feedback
  static constexpr double HIGH_RATE_THRESHOLD = 50.0;  // Hz
  static constexpr double LOW_RATE_THRESHOLD = 1.0;    // Hz
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
