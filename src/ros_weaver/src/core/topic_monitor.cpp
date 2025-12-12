#include "ros_weaver/core/topic_monitor.hpp"

#include <QDebug>

namespace ros_weaver {

TopicMonitor::TopicMonitor(QObject* parent)
    : QObject(parent)
    , updateTimer_(new QTimer(this))
    , activityCheckTimer_(new QTimer(this))
{
  connect(updateTimer_, &QTimer::timeout, this, &TopicMonitor::processUpdates);
  connect(activityCheckTimer_, &QTimer::timeout, this, &TopicMonitor::checkTopicActivity);
}

TopicMonitor::~TopicMonitor()
{
  stopMonitoring();
}

void TopicMonitor::startMonitoring()
{
  if (monitoring_.load()) {
    return;
  }

  try {
    // Initialize ROS2 if not already done
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create node
    node_ = std::make_shared<rclcpp::Node>("ros_weaver_topic_monitor");

    // Start spin thread
    running_.store(true);
    spinThread_ = std::thread(&TopicMonitor::spinThread, this);

    // Start update timers
    updateTimer_->start(UPDATE_INTERVAL_MS);
    activityCheckTimer_->start(DISCOVERY_INTERVAL_MS);

    monitoring_.store(true);
    emit monitoringStateChanged(true);

    // Initial topic discovery
    discoverTopics();

    qDebug() << "TopicMonitor: Started monitoring";

  } catch (const std::exception& e) {
    qWarning() << "TopicMonitor: Failed to start monitoring:" << e.what();
  }
}

void TopicMonitor::stopMonitoring()
{
  if (!monitoring_.load()) {
    return;
  }

  monitoring_.store(false);

  // Stop timers
  updateTimer_->stop();
  activityCheckTimer_->stop();

  // Stop spin thread
  running_.store(false);
  if (spinThread_.joinable()) {
    spinThread_.join();
  }

  // Clear subscriptions
  {
    QMutexLocker locker(&mutex_);
    subscriptions_.clear();
  }

  // Reset node
  node_.reset();

  emit monitoringStateChanged(false);

  qDebug() << "TopicMonitor: Stopped monitoring";
}

void TopicMonitor::spinThread()
{
  rclcpp::WallRate rate(100);  // 100 Hz spin rate

  while (running_.load() && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void TopicMonitor::discoverTopics()
{
  if (!node_) {
    return;
  }

  try {
    auto topicNamesAndTypes = node_->get_topic_names_and_types();

    QList<TopicInfo> discoveredTopics;

    {
      QMutexLocker locker(&mutex_);

      for (const auto& [name, types] : topicNamesAndTypes) {
        QString topicName = QString::fromStdString(name);

        // Skip internal ROS topics
        if (topicName.startsWith("/rosout") ||
            topicName.startsWith("/parameter_events") ||
            topicName.contains("/_")) {
          continue;
        }

        TopicInfo info;
        info.name = topicName;
        info.type = types.empty() ? "unknown" : QString::fromStdString(types[0]);

        // Get publishers and subscribers
        auto pubs = node_->get_publishers_info_by_topic(name);
        auto subs = node_->get_subscriptions_info_by_topic(name);

        for (const auto& pub : pubs) {
          info.publishers.append(QString::fromStdString(pub.node_name()));
        }
        for (const auto& sub : subs) {
          info.subscribers.append(QString::fromStdString(sub.node_name()));
        }

        // Preserve existing activity state if topic was already known
        if (topics_.contains(topicName)) {
          info.isActive = topics_[topicName].isActive;
          info.messageRate = topics_[topicName].messageRate;
          info.lastMessageTime = topics_[topicName].lastMessageTime;
          info.messageCount = topics_[topicName].messageCount;
        }

        topics_[topicName] = info;
        discoveredTopics.append(info);
      }
    }

    emit topicsDiscovered(discoveredTopics);

  } catch (const std::exception& e) {
    qWarning() << "TopicMonitor: Failed to discover topics:" << e.what();
  }
}

QList<TopicInfo> TopicMonitor::getTopics() const
{
  QMutexLocker locker(&mutex_);
  return topics_.values();
}

TopicInfo TopicMonitor::getTopicInfo(const QString& topicName) const
{
  QMutexLocker locker(&mutex_);
  return topics_.value(topicName);
}

TopicStats TopicMonitor::getTopicStats(const QString& topicName) const
{
  QMutexLocker locker(&mutex_);
  return topicStats_.value(topicName);
}

void TopicMonitor::monitorTopic(const QString& topicName, const QString& messageType)
{
  QMutexLocker locker(&mutex_);

  if (subscriptions_.contains(topicName)) {
    return;  // Already monitoring
  }

  // Initialize stats
  TopicStats stats;
  stats.topicName = topicName;
  topicStats_[topicName] = stats;

  locker.unlock();

  createGenericSubscription(topicName, messageType);

  qDebug() << "TopicMonitor: Now monitoring" << topicName;
}

void TopicMonitor::unmonitorTopic(const QString& topicName)
{
  QMutexLocker locker(&mutex_);

  subscriptions_.remove(topicName);
  topicStats_.remove(topicName);

  qDebug() << "TopicMonitor: Stopped monitoring" << topicName;
}

bool TopicMonitor::isTopicMonitored(const QString& topicName) const
{
  QMutexLocker locker(&mutex_);
  return subscriptions_.contains(topicName);
}

QStringList TopicMonitor::getMonitoredTopics() const
{
  QMutexLocker locker(&mutex_);
  return subscriptions_.keys();
}

void TopicMonitor::refreshTopics()
{
  discoverTopics();
}

void TopicMonitor::createGenericSubscription(const QString& topicName, const QString& messageType)
{
  if (!node_) {
    return;
  }

  try {
    auto callback = [this, topicName](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      (void)msg;  // We don't need the message content, just the timing

      qint64 timestamp = QDateTime::currentMSecsSinceEpoch();

      // Queue update for main thread processing
      {
        QMutexLocker locker(&updateMutex_);
        pendingUpdates_.append({topicName, timestamp});
      }
    };

    auto subscription = node_->create_generic_subscription(
        topicName.toStdString(),
        messageType.toStdString(),
        rclcpp::QoS(10).best_effort(),
        callback);

    {
      QMutexLocker locker(&mutex_);
      subscriptions_[topicName] = subscription;
    }

  } catch (const std::exception& e) {
    qWarning() << "TopicMonitor: Failed to create subscription for" << topicName << ":" << e.what();
  }
}

void TopicMonitor::processUpdates()
{
  QList<PendingUpdate> updates;

  {
    QMutexLocker locker(&updateMutex_);
    updates = pendingUpdates_;
    pendingUpdates_.clear();
  }

  for (const auto& update : updates) {
    QMutexLocker locker(&mutex_);

    // Update topic info
    if (topics_.contains(update.topicName)) {
      auto& info = topics_[update.topicName];
      bool wasActive = info.isActive;
      info.isActive = true;
      info.lastMessageTime = QDateTime::fromMSecsSinceEpoch(update.timestamp);
      info.messageCount++;

      if (!wasActive) {
        locker.unlock();
        emit topicBecameActive(update.topicName);
        locker.relock();
      }
    }

    // Update stats
    if (topicStats_.contains(update.topicName)) {
      auto& stats = topicStats_[update.topicName];
      stats.totalMessages++;
      stats.lastMessageTimestamp = update.timestamp;

      // Add timestamp to recent list
      stats.recentTimestamps.push_back(update.timestamp);
      while (stats.recentTimestamps.size() > TopicStats::MAX_TIMESTAMPS) {
        stats.recentTimestamps.pop_front();
      }

      // Calculate current rate
      updateTopicRate(update.topicName);

      TopicStats statsCopy = stats;
      locker.unlock();

      emit topicActivity(update.topicName, statsCopy.currentRate);
      emit topicStatsUpdated(update.topicName, statsCopy);
    }
  }
}

void TopicMonitor::updateTopicRate(const QString& topicName)
{
  // Must be called with mutex locked

  if (!topicStats_.contains(topicName)) {
    return;
  }

  auto& stats = topicStats_[topicName];

  if (stats.recentTimestamps.size() < 2) {
    stats.currentRate = 0.0;
    return;
  }

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 windowStart = now - static_cast<qint64>(TopicStats::RATE_WINDOW_SEC * 1000);

  // Count messages within the rate window
  int messagesInWindow = 0;
  qint64 firstTimestamp = now;
  qint64 lastTimestamp = 0;

  for (const auto& ts : stats.recentTimestamps) {
    if (ts >= windowStart) {
      messagesInWindow++;
      if (ts < firstTimestamp) firstTimestamp = ts;
      if (ts > lastTimestamp) lastTimestamp = ts;
    }
  }

  if (messagesInWindow >= 2 && lastTimestamp > firstTimestamp) {
    double durationSec = (lastTimestamp - firstTimestamp) / 1000.0;
    stats.currentRate = (messagesInWindow - 1) / durationSec;
  } else if (messagesInWindow == 1) {
    // Single message in window, estimate from previous rate or set to 0
    stats.currentRate = 0.5;  // Assume at least some activity
  } else {
    stats.currentRate = 0.0;
  }

  // Update peak rate
  if (stats.currentRate > stats.peakRate) {
    stats.peakRate = stats.currentRate;
  }

  // Update average (simple exponential moving average)
  if (stats.averageRate == 0.0) {
    stats.averageRate = stats.currentRate;
  } else {
    stats.averageRate = 0.9 * stats.averageRate + 0.1 * stats.currentRate;
  }
}

void TopicMonitor::checkTopicActivity()
{
  // Refresh topic list periodically
  discoverTopics();

  qint64 now = QDateTime::currentMSecsSinceEpoch();

  QMutexLocker locker(&mutex_);

  for (auto it = topics_.begin(); it != topics_.end(); ++it) {
    auto& info = it.value();

    if (info.isActive) {
      qint64 lastMsg = info.lastMessageTime.toMSecsSinceEpoch();
      if (now - lastMsg > ACTIVITY_TIMEOUT_MS) {
        info.isActive = false;
        info.messageRate = 0.0;

        QString topicName = it.key();
        locker.unlock();
        emit topicBecameInactive(topicName);
        locker.relock();
      }
    }
  }
}

}  // namespace ros_weaver
