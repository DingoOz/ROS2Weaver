#ifndef ROS_WEAVER_CORE_LATENCY_TRACKER_HPP
#define ROS_WEAVER_CORE_LATENCY_TRACKER_HPP

#include <QObject>
#include <QString>
#include <QMap>
#include <QList>
#include <QTimer>
#include <QMutex>
#include <QDateTime>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

/**
 * @brief A single latency measurement between topics
 */
struct LatencyMeasurement {
  QString connectionId;     // Unique identifier for the connection
  QString sourceTopic;      // Input topic
  QString targetTopic;      // Output topic
  double latencyMs;         // Measured latency in milliseconds
  qint64 timestamp;         // When measurement was taken
};

/**
 * @brief Statistics for a connection's latency
 */
struct LatencyStats {
  double averageMs = 0.0;     // Average latency
  double minMs = 0.0;         // Minimum latency
  double maxMs = 0.0;         // Maximum latency
  double p50Ms = 0.0;         // 50th percentile (median)
  double p95Ms = 0.0;         // 95th percentile
  double p99Ms = 0.0;         // 99th percentile
  double jitterMs = 0.0;      // Standard deviation
  int sampleCount = 0;        // Number of samples
  qint64 lastUpdateTime = 0;  // Last update timestamp
};

/**
 * @brief Tracks end-to-end latency through node chains
 *
 * This class monitors message latency by:
 * - Tracking message timestamps at subscription points
 * - Correlating messages across topics using header timestamps or sequence IDs
 * - Calculating latency statistics for visualization
 */
class LatencyTracker : public QObject {
  Q_OBJECT

public:
  explicit LatencyTracker(QObject* parent = nullptr);
  ~LatencyTracker() override;

  /**
   * @brief Initialize with a ROS node
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

  /**
   * @brief Start tracking latency for a connection
   * @param connectionId Unique ID for this connection
   * @param inputTopic The input topic to monitor
   * @param outputTopic The output topic to monitor
   * @param correlationField Field path for message correlation (e.g., "header.stamp")
   */
  void trackConnection(const QString& connectionId,
                       const QString& inputTopic,
                       const QString& outputTopic,
                       const QString& correlationField = "header.stamp");

  /**
   * @brief Stop tracking a specific connection
   */
  void stopTracking(const QString& connectionId);

  /**
   * @brief Stop all latency tracking
   */
  void stopAllTracking();

  /**
   * @brief Check if tracking is active for a connection
   */
  bool isTracking(const QString& connectionId) const;

  /**
   * @brief Get current latency for a connection
   * @return Latest latency in milliseconds, or -1 if unavailable
   */
  double getCurrentLatency(const QString& connectionId) const;

  /**
   * @brief Get latency statistics for a connection
   */
  LatencyStats getStats(const QString& connectionId) const;

  /**
   * @brief Get recent latency history for a connection
   */
  QList<LatencyMeasurement> getHistory(const QString& connectionId, int maxItems = 100) const;

  /**
   * @brief Set latency alert threshold
   * @param thresholdMs Latency threshold in milliseconds
   */
  void setAlertThreshold(double thresholdMs);

  /**
   * @brief Get alert threshold
   */
  double alertThreshold() const { return alertThresholdMs_; }

  /**
   * @brief Enable/disable latency tracking globally
   */
  void setEnabled(bool enabled);
  bool isEnabled() const { return enabled_; }

  /**
   * @brief Set the maximum history size per connection
   */
  void setMaxHistorySize(int size);

  /**
   * @brief Get list of tracked connections
   */
  QStringList trackedConnections() const;

signals:
  /**
   * @brief Emitted when latency is measured for a connection
   */
  void latencyUpdated(const QString& connectionId, double latencyMs);

  /**
   * @brief Emitted when latency exceeds threshold
   */
  void latencyAlert(const QString& connectionId, double latencyMs, double thresholdMs);

  /**
   * @brief Emitted when tracking starts for a connection
   */
  void trackingStarted(const QString& connectionId);

  /**
   * @brief Emitted when tracking stops for a connection
   */
  void trackingStopped(const QString& connectionId);

  /**
   * @brief Emitted on tracking error
   */
  void trackingError(const QString& connectionId, const QString& error);

private slots:
  void onStatsUpdateTimer();

private:
  /**
   * @brief Internal tracking state for a connection
   */
  struct TrackingState {
    QString inputTopic;
    QString outputTopic;
    QString correlationField;
    rclcpp::GenericSubscription::SharedPtr inputSubscription;
    rclcpp::GenericSubscription::SharedPtr outputSubscription;
    QList<LatencyMeasurement> history;
    LatencyStats stats;
    double currentLatency = -1.0;

    // Pending messages waiting for correlation
    // Key: correlation value (e.g., timestamp), Value: receive time
    QMap<QString, qint64> pendingInputMessages;
  };

  /**
   * @brief Extract correlation value from a message
   */
  QString extractCorrelationValue(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                                   const QString& messageType,
                                   const QString& fieldPath);

  /**
   * @brief Calculate statistics from history
   */
  void updateStats(TrackingState& state);

  /**
   * @brief Handle incoming message on input topic
   */
  void handleInputMessage(const QString& connectionId,
                          const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                          const QString& messageType);

  /**
   * @brief Handle incoming message on output topic
   */
  void handleOutputMessage(const QString& connectionId,
                           const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                           const QString& messageType);

  /**
   * @brief Clean up old pending messages
   */
  void cleanupPendingMessages(TrackingState& state);

  rclcpp::Node::SharedPtr node_;
  QMap<QString, TrackingState> trackingStates_;
  mutable QMutex mutex_;

  QTimer* statsUpdateTimer_;
  double alertThresholdMs_ = 100.0;  // Default 100ms threshold
  bool enabled_ = true;
  int maxHistorySize_ = 1000;

  static constexpr int STATS_UPDATE_INTERVAL_MS = 1000;
  static constexpr int PENDING_MESSAGE_TIMEOUT_MS = 5000;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LATENCY_TRACKER_HPP
