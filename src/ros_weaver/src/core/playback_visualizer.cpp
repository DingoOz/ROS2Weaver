#include "ros_weaver/core/playback_visualizer.hpp"
#include "ros_weaver/core/playback_controller.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QDebug>

namespace ros_weaver {

PlaybackVisualizer::PlaybackVisualizer(QObject* parent)
    : QObject(parent) {

  decayTimer_ = new QTimer(this);
  decayTimer_->setInterval(DECAY_CHECK_INTERVAL_MS);
  connect(decayTimer_, &QTimer::timeout, this, &PlaybackVisualizer::onDecayTimerTick);
}

PlaybackVisualizer::~PlaybackVisualizer() {
  clearActiveTopics();
}

void PlaybackVisualizer::setPlaybackController(PlaybackController* controller) {
  if (playbackController_) {
    disconnect(playbackController_, nullptr, this, nullptr);
  }

  playbackController_ = controller;

  if (playbackController_) {
    connect(playbackController_, &PlaybackController::messagePublished,
            this, &PlaybackVisualizer::onMessagePublished);
    connect(playbackController_, &PlaybackController::stateChanged,
            this, [this](PlaybackState state) { onPlaybackStateChanged(static_cast<int>(state)); });
  }
}

void PlaybackVisualizer::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void PlaybackVisualizer::setEnabled(bool enabled) {
  if (enabled_ == enabled) return;

  enabled_ = enabled;

  if (!enabled_) {
    clearActiveTopics();
    decayTimer_->stop();
  }

  emit enabledChanged(enabled_);
}

void PlaybackVisualizer::setPulseDurationMs(int ms) {
  pulseDurationMs_ = qBound(50, ms, 1000);
}

void PlaybackVisualizer::setActivityDecayMs(int ms) {
  activityDecayMs_ = qBound(100, ms, 2000);
}

void PlaybackVisualizer::resetStats() {
  messageCount_ = 0;
  clearActiveTopics();
}

void PlaybackVisualizer::onMessagePublished(const QString& topic, qint64 timeNs) {
  if (!enabled_ || !canvas_) return;

  currentTimeNs_ = timeNs;
  messageCount_++;

  activateTopic(topic);

  // Start decay timer if not running
  if (!decayTimer_->isActive()) {
    decayTimer_->start();
  }
}

void PlaybackVisualizer::onPlaybackStateChanged(int state) {
  auto playbackState = static_cast<PlaybackState>(state);

  if (playbackState == PlaybackState::Stopped) {
    clearActiveTopics();
    decayTimer_->stop();
  } else if (playbackState == PlaybackState::Paused) {
    // Keep current visualization but stop decay
    decayTimer_->stop();
  } else if (playbackState == PlaybackState::Playing) {
    // Resume decay checking
    if (enabled_ && !decayTimer_->isActive()) {
      decayTimer_->start();
    }
  }
}

void PlaybackVisualizer::onDecayTimerTick() {
  if (!enabled_ || activeTopics_.isEmpty()) {
    decayTimer_->stop();
    return;
  }

  // Check for topics that should be deactivated
  QStringList topicsToDeactivate;
  qint64 decayThresholdNs = static_cast<qint64>(activityDecayMs_) * 1000000;

  for (auto it = topicLastActiveTime_.begin(); it != topicLastActiveTime_.end(); ++it) {
    qint64 elapsed = currentTimeNs_ - it.value();
    if (elapsed > decayThresholdNs) {
      topicsToDeactivate.append(it.key());
    }
  }

  for (const QString& topic : topicsToDeactivate) {
    deactivateTopic(topic);
  }

  // Stop timer if no more active topics
  if (activeTopics_.isEmpty()) {
    decayTimer_->stop();
  }
}

void PlaybackVisualizer::activateTopic(const QString& topicName) {
  bool wasInactive = !activeTopics_.contains(topicName);

  activeTopics_.insert(topicName);
  topicLastActiveTime_[topicName] = currentTimeNs_;

  // Find and update connection visualization
  ConnectionLine* conn = findConnectionForTopic(topicName);
  if (conn) {
    updateConnectionVisualization(conn, true);
    conn->pulseActivity();
  }

  if (wasInactive) {
    emit topicActivated(topicName);
  }
}

void PlaybackVisualizer::deactivateTopic(const QString& topicName) {
  if (!activeTopics_.contains(topicName)) return;

  activeTopics_.remove(topicName);
  topicLastActiveTime_.remove(topicName);

  // Find and update connection visualization
  ConnectionLine* conn = findConnectionForTopic(topicName);
  if (conn) {
    updateConnectionVisualization(conn, false);
  }

  emit topicDeactivated(topicName);
}

void PlaybackVisualizer::clearActiveTopics() {
  QStringList topics = activeTopics_.values();
  for (const QString& topic : topics) {
    deactivateTopic(topic);
  }

  activeTopics_.clear();
  topicLastActiveTime_.clear();

  // Reset all connections to inactive state
  if (canvas_) {
    for (auto* conn : canvas_->connections()) {
      conn->setActivityState(TopicActivityState::Inactive);
    }
  }
}

ConnectionLine* PlaybackVisualizer::findConnectionForTopic(const QString& topicName) {
  if (!canvas_) return nullptr;

  for (auto* conn : canvas_->connections()) {
    if (conn->topicName() == topicName) {
      return conn;
    }
  }

  // Also try to match by checking the output pin names of source blocks
  for (auto* conn : canvas_->connections()) {
    PackageBlock* sourceBlock = conn->sourceBlock();
    if (sourceBlock) {
      int pinIndex = conn->sourcePinIndex();
      const QList<Pin>& pins = sourceBlock->outputPins();
      if (pinIndex >= 0 && pinIndex < pins.size()) {
        QString pinTopic = pins[pinIndex].name;
        // Check if topic matches (possibly with namespace differences)
        if (topicName.endsWith(pinTopic) || pinTopic.endsWith(topicName)) {
          return conn;
        }
      }
    }
  }

  return nullptr;
}

void PlaybackVisualizer::updateConnectionVisualization(ConnectionLine* conn, bool active) {
  if (!conn) return;

  if (active) {
    conn->setActivityState(TopicActivityState::Active);
    conn->setLiveMonitoringEnabled(true);
  } else {
    conn->setActivityState(TopicActivityState::Inactive);
  }
}

}  // namespace ros_weaver
