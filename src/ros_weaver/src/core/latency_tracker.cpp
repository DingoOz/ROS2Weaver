#include "ros_weaver/core/latency_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ros_weaver {

LatencyTracker::LatencyTracker(QObject* parent)
    : QObject(parent)
    , statsUpdateTimer_(new QTimer(this)) {
  connect(statsUpdateTimer_, &QTimer::timeout,
          this, &LatencyTracker::onStatsUpdateTimer);
  statsUpdateTimer_->start(STATS_UPDATE_INTERVAL_MS);
}

LatencyTracker::~LatencyTracker() {
  stopAllTracking();
}

void LatencyTracker::setRosNode(rclcpp::Node::SharedPtr node) {
  QMutexLocker locker(&mutex_);
  node_ = node;
}

void LatencyTracker::trackConnection(const QString& connectionId,
                                      const QString& inputTopic,
                                      const QString& outputTopic,
                                      const QString& correlationField) {
  if (!node_) {
    emit trackingError(connectionId, "ROS node not initialized");
    return;
  }

  QMutexLocker locker(&mutex_);

  // Check if already tracking
  if (trackingStates_.contains(connectionId)) {
    return;
  }

  TrackingState state;
  state.inputTopic = inputTopic;
  state.outputTopic = outputTopic;
  state.correlationField = correlationField;

  // Get topic types
  auto topicsAndTypes = node_->get_topic_names_and_types();

  QString inputType;
  QString outputType;

  std::string inputTopicStd = inputTopic.toStdString();
  std::string outputTopicStd = outputTopic.toStdString();

  if (topicsAndTypes.count(inputTopicStd) > 0 && !topicsAndTypes[inputTopicStd].empty()) {
    inputType = QString::fromStdString(topicsAndTypes[inputTopicStd][0]);
  }

  if (topicsAndTypes.count(outputTopicStd) > 0 && !topicsAndTypes[outputTopicStd].empty()) {
    outputType = QString::fromStdString(topicsAndTypes[outputTopicStd][0]);
  }

  if (inputType.isEmpty() || outputType.isEmpty()) {
    emit trackingError(connectionId, "Could not determine topic types");
    return;
  }

  try {
    // Create input subscription
    auto inputCallback = [this, connectionId, inputType](
        std::shared_ptr<rclcpp::SerializedMessage> msg) {
      handleInputMessage(connectionId, msg, inputType);
    };

    state.inputSubscription = node_->create_generic_subscription(
        inputTopicStd, inputType.toStdString(),
        rclcpp::QoS(10).best_effort(),
        inputCallback);

    // Create output subscription
    auto outputCallback = [this, connectionId, outputType](
        std::shared_ptr<rclcpp::SerializedMessage> msg) {
      handleOutputMessage(connectionId, msg, outputType);
    };

    state.outputSubscription = node_->create_generic_subscription(
        outputTopicStd, outputType.toStdString(),
        rclcpp::QoS(10).best_effort(),
        outputCallback);

    trackingStates_[connectionId] = std::move(state);

    locker.unlock();
    emit trackingStarted(connectionId);

  } catch (const std::exception& e) {
    emit trackingError(connectionId, QString("Failed to create subscriptions: %1").arg(e.what()));
  }
}

void LatencyTracker::stopTracking(const QString& connectionId) {
  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return;
  }

  // Reset subscriptions (shared_ptr will clean up)
  trackingStates_[connectionId].inputSubscription.reset();
  trackingStates_[connectionId].outputSubscription.reset();
  trackingStates_.remove(connectionId);

  locker.unlock();
  emit trackingStopped(connectionId);
}

void LatencyTracker::stopAllTracking() {
  QMutexLocker locker(&mutex_);

  QStringList connections = trackingStates_.keys();

  for (const QString& connId : connections) {
    trackingStates_[connId].inputSubscription.reset();
    trackingStates_[connId].outputSubscription.reset();
  }
  trackingStates_.clear();

  locker.unlock();

  for (const QString& connId : connections) {
    emit trackingStopped(connId);
  }
}

bool LatencyTracker::isTracking(const QString& connectionId) const {
  QMutexLocker locker(&mutex_);
  return trackingStates_.contains(connectionId);
}

double LatencyTracker::getCurrentLatency(const QString& connectionId) const {
  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return -1.0;
  }

  return trackingStates_[connectionId].currentLatency;
}

LatencyStats LatencyTracker::getStats(const QString& connectionId) const {
  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return LatencyStats();
  }

  return trackingStates_[connectionId].stats;
}

QList<LatencyMeasurement> LatencyTracker::getHistory(const QString& connectionId, int maxItems) const {
  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return QList<LatencyMeasurement>();
  }

  const auto& history = trackingStates_[connectionId].history;

  if (history.size() <= maxItems) {
    return history;
  }

  return history.mid(history.size() - maxItems);
}

void LatencyTracker::setAlertThreshold(double thresholdMs) {
  alertThresholdMs_ = thresholdMs;
}

void LatencyTracker::setEnabled(bool enabled) {
  enabled_ = enabled;
}

void LatencyTracker::setMaxHistorySize(int size) {
  maxHistorySize_ = size;
}

QStringList LatencyTracker::trackedConnections() const {
  QMutexLocker locker(&mutex_);
  return trackingStates_.keys();
}

void LatencyTracker::onStatsUpdateTimer() {
  QMutexLocker locker(&mutex_);

  for (auto it = trackingStates_.begin(); it != trackingStates_.end(); ++it) {
    cleanupPendingMessages(it.value());
    updateStats(it.value());
  }
}

QString LatencyTracker::extractCorrelationValue(
    const std::shared_ptr<rclcpp::SerializedMessage>& msg,
    const QString& messageType,
    const QString& fieldPath) {
  Q_UNUSED(msg)
  Q_UNUSED(messageType)

  // For messages with std_msgs/Header, extract stamp
  // This is a simplified implementation that uses receive time as fallback
  // A full implementation would deserialize the message and extract the field

  if (fieldPath == "header.stamp") {
    // Use current time as correlation (simplified)
    // In a full implementation, we would deserialize and extract header.stamp
    return QString::number(QDateTime::currentMSecsSinceEpoch());
  }

  // Fallback: use message hash or receive time
  return QString::number(QDateTime::currentMSecsSinceEpoch());
}

void LatencyTracker::handleInputMessage(
    const QString& connectionId,
    const std::shared_ptr<rclcpp::SerializedMessage>& msg,
    const QString& messageType) {

  if (!enabled_) return;

  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return;
  }

  auto& state = trackingStates_[connectionId];

  // Record receive time
  qint64 receiveTime = QDateTime::currentMSecsSinceEpoch();

  // Extract correlation value
  QString correlationValue = extractCorrelationValue(msg, messageType, state.correlationField);

  // Store pending message
  state.pendingInputMessages[correlationValue] = receiveTime;
}

void LatencyTracker::handleOutputMessage(
    const QString& connectionId,
    const std::shared_ptr<rclcpp::SerializedMessage>& msg,
    const QString& messageType) {

  if (!enabled_) return;

  QMutexLocker locker(&mutex_);

  if (!trackingStates_.contains(connectionId)) {
    return;
  }

  auto& state = trackingStates_[connectionId];

  qint64 receiveTime = QDateTime::currentMSecsSinceEpoch();

  // Extract correlation value
  QString correlationValue = extractCorrelationValue(msg, messageType, state.correlationField);

  // Look for matching input message
  if (state.pendingInputMessages.contains(correlationValue)) {
    qint64 inputTime = state.pendingInputMessages[correlationValue];
    double latencyMs = static_cast<double>(receiveTime - inputTime);

    // Record measurement
    LatencyMeasurement measurement;
    measurement.connectionId = connectionId;
    measurement.sourceTopic = state.inputTopic;
    measurement.targetTopic = state.outputTopic;
    measurement.latencyMs = latencyMs;
    measurement.timestamp = receiveTime;

    state.history.append(measurement);

    // Trim history if needed
    while (state.history.size() > maxHistorySize_) {
      state.history.removeFirst();
    }

    state.currentLatency = latencyMs;

    // Remove matched message
    state.pendingInputMessages.remove(correlationValue);

    locker.unlock();

    emit latencyUpdated(connectionId, latencyMs);

    if (latencyMs > alertThresholdMs_) {
      emit latencyAlert(connectionId, latencyMs, alertThresholdMs_);
    }
  }
}

void LatencyTracker::cleanupPendingMessages(TrackingState& state) {
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 timeout = PENDING_MESSAGE_TIMEOUT_MS;

  QList<QString> toRemove;
  for (auto it = state.pendingInputMessages.begin();
       it != state.pendingInputMessages.end(); ++it) {
    if (now - it.value() > timeout) {
      toRemove.append(it.key());
    }
  }

  for (const QString& key : toRemove) {
    state.pendingInputMessages.remove(key);
  }
}

void LatencyTracker::updateStats(TrackingState& state) {
  if (state.history.isEmpty()) {
    return;
  }

  // Extract latency values
  QList<double> latencies;
  latencies.reserve(state.history.size());
  for (const auto& m : state.history) {
    latencies.append(m.latencyMs);
  }

  // Calculate statistics
  LatencyStats stats;
  stats.sampleCount = latencies.size();

  // Average
  double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
  stats.averageMs = sum / stats.sampleCount;

  // Min/Max
  auto minmax = std::minmax_element(latencies.begin(), latencies.end());
  stats.minMs = *minmax.first;
  stats.maxMs = *minmax.second;

  // Sort for percentiles
  std::sort(latencies.begin(), latencies.end());

  // Percentiles
  auto percentileIndex = [&](double p) -> int {
    return static_cast<int>(std::ceil(p * stats.sampleCount / 100.0)) - 1;
  };

  stats.p50Ms = latencies[percentileIndex(50)];
  stats.p95Ms = latencies[percentileIndex(95)];
  stats.p99Ms = latencies[percentileIndex(99)];

  // Standard deviation (jitter)
  double sqSum = 0.0;
  for (double lat : latencies) {
    sqSum += (lat - stats.averageMs) * (lat - stats.averageMs);
  }
  stats.jitterMs = std::sqrt(sqSum / stats.sampleCount);

  stats.lastUpdateTime = QDateTime::currentMSecsSinceEpoch();

  state.stats = stats;
}

}  // namespace ros_weaver
