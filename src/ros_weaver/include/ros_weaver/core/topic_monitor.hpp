#ifndef ROS_WEAVER_TOPIC_MONITOR_HPP
#define ROS_WEAVER_TOPIC_MONITOR_HPP

#include <QObject>
#include <QString>
#include <QMap>
#include <QTimer>
#include <QMutex>
#include <QDateTime>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <deque>

namespace ros_weaver {

/**
 * @brief Information about a discovered ROS2 topic
 */
struct TopicInfo {
  QString name;
  QString type;
  QStringList publishers;
  QStringList subscribers;
  bool isActive = false;
  double messageRate = 0.0;  // Hz
  QDateTime lastMessageTime;
  int messageCount = 0;
};

/**
 * @brief Statistics for a monitored topic
 */
struct TopicStats {
  QString topicName;
  double currentRate = 0.0;      // Current message rate in Hz
  double averageRate = 0.0;      // Average rate over monitoring period
  double peakRate = 0.0;         // Peak rate observed
  int totalMessages = 0;         // Total messages received
  qint64 lastMessageTimestamp = 0;  // Timestamp of last message (ms since epoch)
  std::deque<qint64> recentTimestamps;  // Recent message timestamps for rate calculation

  static constexpr int MAX_TIMESTAMPS = 100;  // Keep last 100 timestamps
  static constexpr double RATE_WINDOW_SEC = 2.0;  // Calculate rate over 2 seconds
};

/**
 * @brief Monitors ROS2 topics for activity and message rates
 *
 * This class discovers topics in the ROS2 graph and monitors their
 * activity levels. It provides signals when topic activity changes
 * so the canvas can update visualizations accordingly.
 */
class TopicMonitor : public QObject {
  Q_OBJECT

public:
  explicit TopicMonitor(QObject* parent = nullptr);
  ~TopicMonitor() override;

  /**
   * @brief Start monitoring topics
   */
  void startMonitoring();

  /**
   * @brief Stop monitoring topics
   */
  void stopMonitoring();

  /**
   * @brief Check if monitoring is active
   */
  bool isMonitoring() const { return monitoring_.load(); }

  /**
   * @brief Get list of all discovered topics
   */
  QList<TopicInfo> getTopics() const;

  /**
   * @brief Get info for a specific topic
   */
  TopicInfo getTopicInfo(const QString& topicName) const;

  /**
   * @brief Get statistics for a monitored topic
   */
  TopicStats getTopicStats(const QString& topicName) const;

  /**
   * @brief Subscribe to monitor a specific topic
   * @param topicName The topic to monitor
   * @param messageType The message type (e.g., "std_msgs/msg/String")
   */
  void monitorTopic(const QString& topicName, const QString& messageType);

  /**
   * @brief Stop monitoring a specific topic
   */
  void unmonitorTopic(const QString& topicName);

  /**
   * @brief Check if a topic is being monitored
   */
  bool isTopicMonitored(const QString& topicName) const;

  /**
   * @brief Get list of monitored topics
   */
  QStringList getMonitoredTopics() const;

  /**
   * @brief Refresh the topic list from ROS2 graph
   */
  void refreshTopics();

signals:
  /**
   * @brief Emitted when topic list changes
   */
  void topicsDiscovered(const QList<TopicInfo>& topics);

  /**
   * @brief Emitted when a topic receives a message
   */
  void topicActivity(const QString& topicName, double rate);

  /**
   * @brief Emitted when topic stats are updated
   */
  void topicStatsUpdated(const QString& topicName, const TopicStats& stats);

  /**
   * @brief Emitted when a topic becomes active (starts receiving messages)
   */
  void topicBecameActive(const QString& topicName);

  /**
   * @brief Emitted when a topic becomes inactive (stopped receiving messages)
   */
  void topicBecameInactive(const QString& topicName);

  /**
   * @brief Emitted when monitoring starts/stops
   */
  void monitoringStateChanged(bool active);

private slots:
  void processUpdates();
  void checkTopicActivity();

private:
  void spinThread();
  void discoverTopics();
  void createGenericSubscription(const QString& topicName, const QString& messageType);
  void updateTopicRate(const QString& topicName);

  // ROS2 node and subscriptions
  std::shared_ptr<rclcpp::Node> node_;
  std::thread spinThread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> monitoring_{false};

  // Topic tracking
  mutable QMutex mutex_;
  QMap<QString, TopicInfo> topics_;
  QMap<QString, TopicStats> topicStats_;
  QMap<QString, rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  // Update timers
  QTimer* updateTimer_;
  QTimer* activityCheckTimer_;

  // Pending updates from ROS thread
  struct PendingUpdate {
    QString topicName;
    qint64 timestamp;
  };
  QList<PendingUpdate> pendingUpdates_;
  QMutex updateMutex_;

  // Activity timeout (ms) - topic considered inactive after this
  static constexpr int ACTIVITY_TIMEOUT_MS = 3000;
  // Update interval for processing (ms)
  static constexpr int UPDATE_INTERVAL_MS = 100;
  // Topic discovery interval (ms)
  static constexpr int DISCOVERY_INTERVAL_MS = 2000;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_TOPIC_MONITOR_HPP
