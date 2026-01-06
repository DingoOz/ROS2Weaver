#include "ros_weaver/core/node_health_monitor.hpp"

#include <QDebug>
#include <QRandomGenerator>
#include <cmath>

namespace ros_weaver {

NodeHealthMonitor::NodeHealthMonitor(QObject* parent)
    : QObject(parent) {

  qRegisterMetaType<HealthStatus>("HealthStatus");
  qRegisterMetaType<NodeHealthData>("NodeHealthData");
  qRegisterMetaType<HealthThresholds>("HealthThresholds");

  updateTimer_ = new QTimer(this);
  updateTimer_->setInterval(updateIntervalMs_);
  connect(updateTimer_, &QTimer::timeout, this, &NodeHealthMonitor::onUpdateTimerTick);
}

void NodeHealthMonitor::start() {
  if (isMonitoring_) return;

  isMonitoring_ = true;
  updateTimer_->start();
  emit monitoringStateChanged(true);

  qDebug() << "NodeHealthMonitor: Started monitoring" << nodeHealthData_.size() << "nodes";
}

void NodeHealthMonitor::stop() {
  if (!isMonitoring_) return;

  isMonitoring_ = false;
  updateTimer_->stop();
  emit monitoringStateChanged(false);

  qDebug() << "NodeHealthMonitor: Stopped monitoring";
}

void NodeHealthMonitor::setUpdateInterval(int ms) {
  updateIntervalMs_ = qBound(100, ms, 10000);
  updateTimer_->setInterval(updateIntervalMs_);
}

void NodeHealthMonitor::setThresholds(const HealthThresholds& thresholds) {
  thresholds_ = thresholds;

  // Re-evaluate all node health with new thresholds
  for (auto it = nodeHealthData_.begin(); it != nodeHealthData_.end(); ++it) {
    evaluateHealthStatus(it.value());
  }
}

NodeHealthData NodeHealthMonitor::getNodeHealth(const QString& nodeName) const {
  return nodeHealthData_.value(nodeName);
}

QList<NodeHealthData> NodeHealthMonitor::getAllNodeHealth() const {
  return nodeHealthData_.values();
}

QStringList NodeHealthMonitor::monitoredNodes() const {
  return nodeHealthData_.keys();
}

void NodeHealthMonitor::addNode(const QString& nodeName) {
  if (nodeHealthData_.contains(nodeName)) return;

  NodeHealthData health;
  health.nodeName = nodeName;

  // Parse namespace from node name (e.g., "/ns/node_name")
  int lastSlash = nodeName.lastIndexOf('/');
  if (lastSlash > 0) {
    health.namespace_ = nodeName.left(lastSlash);
  }

  health.isAlive = true;
  health.lastHeartbeat = QDateTime::currentDateTime();

  nodeHealthData_[nodeName] = health;

  qDebug() << "NodeHealthMonitor: Added node" << nodeName;
}

void NodeHealthMonitor::removeNode(const QString& nodeName) {
  if (nodeHealthData_.remove(nodeName) > 0) {
    qDebug() << "NodeHealthMonitor: Removed node" << nodeName;
  }
}

void NodeHealthMonitor::clearNodes() {
  nodeHealthData_.clear();
  qDebug() << "NodeHealthMonitor: Cleared all nodes";
}

void NodeHealthMonitor::setHistorySize(int size) {
  historySize_ = qBound(10, size, 1000);
}

void NodeHealthMonitor::onUpdateTimerTick() {
  for (const QString& nodeName : nodeHealthData_.keys()) {
    updateNodeHealth(nodeName);
  }
}

void NodeHealthMonitor::updateNodeHealth(const QString& nodeName) {
  if (!nodeHealthData_.contains(nodeName)) return;

  NodeHealthData& health = nodeHealthData_[nodeName];
  HealthStatus previousStatus = health.status;

  // Update heartbeat
  health.lastHeartbeat = QDateTime::currentDateTime();
  health.isAlive = true;

  // Simulate metrics (in a real implementation, these would come from actual system monitoring)
  // For demo purposes, we generate semi-realistic varying values
  health.cpuPercent = simulateMetric(health.cpuPercent > 0 ? health.cpuPercent : 15.0, 5.0);
  health.memoryMB = simulateMetric(health.memoryMB > 0 ? health.memoryMB : 50.0, 10.0);
  health.memoryPercent = qBound(0.0, health.memoryMB / 10.0, 100.0);  // Simulated percentage
  health.callbackLatencyMs = simulateMetric(health.callbackLatencyMs > 0 ? health.callbackLatencyMs : 5.0, 2.0);
  health.maxCallbackLatencyMs = qMax(health.maxCallbackLatencyMs, health.callbackLatencyMs);
  health.messageRate = simulateMetric(health.messageRate > 0 ? health.messageRate : 30.0, 5.0);

  // Occasionally add dropped messages for demo
  if (QRandomGenerator::global()->bounded(100) < 5) {  // 5% chance
    health.droppedMessages += QRandomGenerator::global()->bounded(1, 3);
  }

  // Add to history
  addHistoryPoint(health.cpuHistory, health.cpuPercent);
  addHistoryPoint(health.memoryHistory, health.memoryMB);
  addHistoryPoint(health.latencyHistory, health.callbackLatencyMs);

  // Evaluate overall health status
  evaluateHealthStatus(health);

  // Emit signals
  emit healthUpdated(nodeName, health);

  // Check for status transitions
  if (previousStatus != health.status) {
    if (health.status == HealthStatus::Warning) {
      emit warningTriggered(nodeName, "One or more metrics elevated");
    } else if (health.status == HealthStatus::Critical) {
      emit criticalTriggered(nodeName, "One or more metrics at critical level");
    } else if (health.status == HealthStatus::Healthy &&
               (previousStatus == HealthStatus::Warning || previousStatus == HealthStatus::Critical)) {
      emit healthRecovered(nodeName);
    }
  }
}

void NodeHealthMonitor::evaluateHealthStatus(NodeHealthData& health) {
  // Check for critical conditions
  if (health.cpuPercent >= thresholds_.cpuCritical ||
      health.memoryPercent >= thresholds_.memoryCritical ||
      health.callbackLatencyMs >= thresholds_.latencyCritical ||
      health.droppedMessages >= thresholds_.droppedMessagesCritical) {
    health.status = HealthStatus::Critical;
    return;
  }

  // Check for warning conditions
  if (health.cpuPercent >= thresholds_.cpuWarning ||
      health.memoryPercent >= thresholds_.memoryWarning ||
      health.callbackLatencyMs >= thresholds_.latencyWarning ||
      health.droppedMessages >= thresholds_.droppedMessagesWarning) {
    health.status = HealthStatus::Warning;
    return;
  }

  health.status = HealthStatus::Healthy;
}

void NodeHealthMonitor::addHistoryPoint(QList<HealthMetricPoint>& history, double value) {
  HealthMetricPoint point;
  point.timestamp = QDateTime::currentDateTime();
  point.value = value;

  history.append(point);

  // Trim history to max size
  while (history.size() > historySize_) {
    history.removeFirst();
  }
}

double NodeHealthMonitor::simulateMetric(double base, double variance) {
  // Generate a value that varies around the base with some randomness
  // This creates somewhat realistic varying metrics
  double noise = (QRandomGenerator::global()->generateDouble() - 0.5) * 2.0 * variance;
  double drift = (QRandomGenerator::global()->generateDouble() - 0.5) * 0.5;  // Small drift
  double newValue = base + noise + drift;

  // Apply some bounds to keep it realistic
  return qBound(0.0, newValue, 100.0);
}

}  // namespace ros_weaver
