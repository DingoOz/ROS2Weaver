#ifndef ROS_WEAVER_CORE_NODE_HEALTH_MONITOR_HPP
#define ROS_WEAVER_CORE_NODE_HEALTH_MONITOR_HPP

#include <QObject>
#include <QTimer>
#include <QMap>
#include <QColor>
#include <QDateTime>
#include <QList>

namespace ros_weaver {

/**
 * @brief Health status severity levels
 */
enum class HealthStatus {
  Unknown,    // No data yet
  Healthy,    // All metrics within normal range
  Warning,    // Some metrics elevated
  Critical    // One or more metrics at critical level
};

/**
 * @brief Health metric for a single data point
 */
struct HealthMetricPoint {
  QDateTime timestamp;
  double value;
};

/**
 * @brief Health data for a single ROS2 node
 */
struct NodeHealthData {
  QString nodeName;
  QString namespace_;

  // Resource usage
  double cpuPercent = 0.0;           // 0-100%
  double memoryMB = 0.0;             // Memory usage in MB
  double memoryPercent = 0.0;        // 0-100%

  // Message statistics
  double callbackLatencyMs = 0.0;    // Average callback execution time
  double maxCallbackLatencyMs = 0.0; // Maximum callback latency observed
  int droppedMessages = 0;           // Count of dropped messages
  double messageRate = 0.0;          // Messages per second

  // Status
  HealthStatus status = HealthStatus::Unknown;
  bool isAlive = false;
  QDateTime lastHeartbeat;

  // Historical data (last N samples)
  QList<HealthMetricPoint> cpuHistory;
  QList<HealthMetricPoint> memoryHistory;
  QList<HealthMetricPoint> latencyHistory;

  /**
   * @brief Get color representing the current health status
   */
  QColor getStatusColor() const {
    switch (status) {
      case HealthStatus::Healthy:  return QColor(76, 175, 80);   // Green
      case HealthStatus::Warning:  return QColor(255, 152, 0);   // Orange
      case HealthStatus::Critical: return QColor(244, 67, 54);   // Red
      default:                     return QColor(128, 128, 128); // Gray
    }
  }

  /**
   * @brief Get string representation of health status
   */
  QString getStatusString() const {
    switch (status) {
      case HealthStatus::Healthy:  return "Healthy";
      case HealthStatus::Warning:  return "Warning";
      case HealthStatus::Critical: return "Critical";
      default:                     return "Unknown";
    }
  }

  /**
   * @brief Get heatmap color based on CPU usage (gradient from green to red)
   */
  QColor getCpuHeatmapColor() const {
    if (cpuPercent < 25.0) return QColor(76, 175, 80);   // Green
    if (cpuPercent < 50.0) return QColor(139, 195, 74);  // Light green
    if (cpuPercent < 75.0) return QColor(255, 193, 7);   // Yellow
    if (cpuPercent < 90.0) return QColor(255, 152, 0);   // Orange
    return QColor(244, 67, 54);  // Red
  }

  /**
   * @brief Get heatmap color based on memory usage
   */
  QColor getMemoryHeatmapColor() const {
    if (memoryPercent < 25.0) return QColor(76, 175, 80);
    if (memoryPercent < 50.0) return QColor(139, 195, 74);
    if (memoryPercent < 75.0) return QColor(255, 193, 7);
    if (memoryPercent < 90.0) return QColor(255, 152, 0);
    return QColor(244, 67, 54);
  }
};

/**
 * @brief Alert thresholds for node health monitoring
 */
struct HealthThresholds {
  double cpuWarning = 70.0;      // CPU % warning threshold
  double cpuCritical = 90.0;     // CPU % critical threshold
  double memoryWarning = 70.0;   // Memory % warning threshold
  double memoryCritical = 90.0;  // Memory % critical threshold
  double latencyWarning = 50.0;  // Callback latency (ms) warning threshold
  double latencyCritical = 100.0;// Callback latency (ms) critical threshold
  int droppedMessagesWarning = 10;    // Dropped messages warning threshold
  int droppedMessagesCritical = 100;  // Dropped messages critical threshold
};

/**
 * @brief Monitors health metrics for ROS2 nodes
 *
 * Features:
 * - Real-time CPU and memory usage tracking
 * - Callback latency monitoring
 * - Message drop detection
 * - Historical data storage
 * - Configurable alert thresholds
 */
class NodeHealthMonitor : public QObject {
  Q_OBJECT

public:
  explicit NodeHealthMonitor(QObject* parent = nullptr);
  ~NodeHealthMonitor() override = default;

  /**
   * @brief Start monitoring
   */
  void start();

  /**
   * @brief Stop monitoring
   */
  void stop();

  /**
   * @brief Check if monitoring is active
   */
  bool isMonitoring() const { return isMonitoring_; }

  /**
   * @brief Set update interval in milliseconds
   */
  void setUpdateInterval(int ms);
  int updateInterval() const { return updateIntervalMs_; }

  /**
   * @brief Set alert thresholds
   */
  void setThresholds(const HealthThresholds& thresholds);
  HealthThresholds thresholds() const { return thresholds_; }

  /**
   * @brief Get health data for a specific node
   */
  NodeHealthData getNodeHealth(const QString& nodeName) const;

  /**
   * @brief Get health data for all monitored nodes
   */
  QList<NodeHealthData> getAllNodeHealth() const;

  /**
   * @brief Get list of node names being monitored
   */
  QStringList monitoredNodes() const;

  /**
   * @brief Manually add a node to monitor (for canvas integration)
   */
  void addNode(const QString& nodeName);

  /**
   * @brief Remove a node from monitoring
   */
  void removeNode(const QString& nodeName);

  /**
   * @brief Clear all monitored nodes
   */
  void clearNodes();

  /**
   * @brief Set maximum history size (number of samples to keep)
   */
  void setHistorySize(int size);
  int historySize() const { return historySize_; }

signals:
  /**
   * @brief Emitted when health data is updated
   */
  void healthUpdated(const QString& nodeName, const NodeHealthData& health);

  /**
   * @brief Emitted when a node enters warning state
   */
  void warningTriggered(const QString& nodeName, const QString& reason);

  /**
   * @brief Emitted when a node enters critical state
   */
  void criticalTriggered(const QString& nodeName, const QString& reason);

  /**
   * @brief Emitted when a node recovers to healthy state
   */
  void healthRecovered(const QString& nodeName);

  /**
   * @brief Emitted when monitoring starts/stops
   */
  void monitoringStateChanged(bool isMonitoring);

private slots:
  void onUpdateTimerTick();

private:
  void updateNodeHealth(const QString& nodeName);
  void evaluateHealthStatus(NodeHealthData& health);
  void addHistoryPoint(QList<HealthMetricPoint>& history, double value);
  double simulateMetric(double base, double variance);  // For demo purposes

  QTimer* updateTimer_ = nullptr;
  int updateIntervalMs_ = 1000;
  bool isMonitoring_ = false;

  QMap<QString, NodeHealthData> nodeHealthData_;
  HealthThresholds thresholds_;
  int historySize_ = 60;  // Keep 60 samples (1 minute at 1s interval)
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::HealthStatus)
Q_DECLARE_METATYPE(ros_weaver::NodeHealthData)
Q_DECLARE_METATYPE(ros_weaver::HealthThresholds)

#endif  // ROS_WEAVER_CORE_NODE_HEALTH_MONITOR_HPP
