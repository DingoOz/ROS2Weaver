#ifndef ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
#define ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP

#include <QGraphicsObject>
#include <QPainter>
#include <QPainterPath>
#include <QUuid>
#include <QTimer>
#include <QPropertyAnimation>
#include <QFont>
#include <QDateTime>

namespace ros_weaver {

class PackageBlock;

/**
 * @brief Activity state for live topic visualization
 */
enum class TopicActivityState {
  Unknown,    // Not monitoring
  Inactive,   // Monitoring but no messages
  Active,     // Receiving messages at normal rate
  HighRate,   // Receiving at high rate (> 50 Hz)
  Degraded,   // Receiving below expected rate
  Stale       // No messages received recently (timeout)
};

/**
 * @brief Statistics for a connection's data flow
 */
struct ConnectionStats {
  double messageRate = 0.0;       // Messages per second (Hz)
  double bandwidthBps = 0.0;      // Bytes per second
  double avgLatencyMs = 0.0;      // Average latency in milliseconds
  double jitterMs = 0.0;          // Latency jitter (std dev)
  int queueDepth = 0;             // Current queue depth
  int droppedCount = 0;           // Number of dropped messages
  qint64 totalMessages = 0;       // Total messages received
  qint64 totalBytes = 0;          // Total bytes received
  double expectedRate = 0.0;      // Expected rate for health calculation
  QDateTime lastMessageTime;      // Time of last message
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

  // Connection statistics for detailed monitoring
  void setConnectionStats(const ConnectionStats& stats);
  ConnectionStats connectionStats() const { return stats_; }

  // Bandwidth display
  void setShowBandwidth(bool show);
  bool showBandwidth() const { return showBandwidth_; }

  // Expected rate for health calculation
  void setExpectedRate(double rateHz);
  double expectedRate() const { return expectedRate_; }

  // Remapping visualization
  void setIsRemapped(bool remapped);
  bool isRemapped() const { return isRemapped_; }

  void setOriginalTopicName(const QString& name);
  QString originalTopicName() const { return originalTopicName_; }

  // Check if this connection has a remapping from source or target block
  void updateRemappingStatus();

  // Safely detach from blocks before canvas clearing
  // This prevents use-after-free when scene_->clear() deletes items in undefined order
  void detach();

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
  void drawBandwidthLabel(QPainter* painter);
  void drawActivityIndicator(QPainter* painter);
  void drawHealthIndicator(QPainter* painter);
  QColor getActivityColor() const;
  QString formatBandwidth(double bytesPerSec) const;
  QPainterPath calculatePath() const;
  void updateHealthState();

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

  // Bandwidth visualization
  ConnectionStats stats_;
  bool showBandwidth_;
  double expectedRate_;

  // Remapping visualization
  bool isRemapped_;
  QString originalTopicName_;

  static constexpr qreal LINE_WIDTH = 2.0;
  static constexpr qreal HIGHLIGHT_WIDTH = 4.0;
  static constexpr qreal HOVER_WIDTH = 3.0;
  static constexpr qreal CURVE_OFFSET = 50.0;
  static constexpr qreal HIT_TOLERANCE = 10.0;

public:
  // Rate thresholds for visual feedback
  static constexpr double HIGH_RATE_THRESHOLD = 50.0;  // Hz
  static constexpr double LOW_RATE_THRESHOLD = 1.0;    // Hz
  static constexpr double DEGRADED_RATE_RATIO = 0.5;   // Below 50% of expected = degraded
  static constexpr int STALE_TIMEOUT_MS = 5000;        // 5 seconds without message = stale
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_CONNECTION_LINE_HPP
