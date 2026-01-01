#include "ros_weaver/core/playback_animator.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/core/playback_controller.hpp"

#include <QDateTime>

namespace ros_weaver {

PlaybackAnimator::PlaybackAnimator(QObject* parent)
    : QObject(parent)
    , canvas_(nullptr)
    , playbackController_(nullptr)
    , enabled_(true)
    , isPlaying_(false)
    , animationSpeed_(1.0)
    , showRateLabels_(true)
    , pulseOnMessage_(true)
    , animationTimer_(new QTimer(this))
    , decayTimer_(new QTimer(this))
    , lastUpdateTime_(0.0)
{
  connect(animationTimer_, &QTimer::timeout,
          this, &PlaybackAnimator::updateAnimations);
  connect(decayTimer_, &QTimer::timeout,
          this, &PlaybackAnimator::decayAnimations);
}

PlaybackAnimator::~PlaybackAnimator() {
  stopAnimationTimer();
}

void PlaybackAnimator::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void PlaybackAnimator::setPlaybackController(PlaybackController* controller) {
  if (playbackController_) {
    disconnect(playbackController_, nullptr, this, nullptr);
  }

  playbackController_ = controller;

  if (playbackController_) {
    connect(playbackController_, &PlaybackController::stateChanged,
            this, [this](PlaybackState state) {
              if (state == PlaybackState::Playing) {
                onPlaybackStarted();
              } else if (state == PlaybackState::Stopped) {
                onPlaybackStopped();
              } else if (state == PlaybackState::Paused) {
                onPlaybackPaused();
              }
            });
    connect(playbackController_, &PlaybackController::timeChanged,
            this, [this](qint64 timeNs) {
              onPlaybackPositionChanged(timeNs / 1e9);  // Convert to seconds
            });
    connect(playbackController_, &PlaybackController::messagePublished,
            this, [this](const QString& topic, qint64 timeNs) {
              onMessageReceived(topic, timeNs / 1e9);  // Convert to seconds
            });
  }
}

void PlaybackAnimator::setEnabled(bool enabled) {
  enabled_ = enabled;

  if (!enabled_) {
    stopAnimationTimer();
    resetAllAnimations();
  } else if (isPlaying_) {
    startAnimationTimer();
  }
}

void PlaybackAnimator::setAnimationSpeed(double speed) {
  animationSpeed_ = qBound(0.1, speed, 10.0);
}

void PlaybackAnimator::setShowRateLabels(bool show) {
  showRateLabels_ = show;

  // Update all connections
  if (canvas_) {
    for (ConnectionLine* conn : canvas_->connections()) {
      conn->setShowRateLabel(show && isPlaying_);
    }
  }
}

void PlaybackAnimator::setPulseOnMessage(bool pulse) {
  pulseOnMessage_ = pulse;
}

void PlaybackAnimator::notifyMessage(const QString& topicName, double timestamp) {
  if (!enabled_ || !isPlaying_) return;

  ConnectionAnimationState& state = topicStates_[topicName];

  // Calculate time since last message
  double timeDelta = timestamp - state.lastMessageTime;
  if (timeDelta > 0 && state.lastMessageTime > 0) {
    // Smooth the rate estimate
    double instantRate = 1.0 / timeDelta;
    state.estimatedRate = state.estimatedRate * 0.7 + instantRate * 0.3;
  }

  state.topicName = topicName;
  state.lastMessageTime = timestamp;
  state.messageCount++;
  state.isActive = true;

  // Update the connection visualization
  updateConnectionForTopic(topicName);

  emit topicActivityChanged(topicName, state.estimatedRate);
}

void PlaybackAnimator::onPlaybackStarted() {
  isPlaying_ = true;
  topicStates_.clear();
  lastUpdateTime_ = 0.0;

  if (enabled_) {
    startAnimationTimer();

    // Enable live monitoring on all connections
    if (canvas_) {
      for (ConnectionLine* conn : canvas_->connections()) {
        conn->setLiveMonitoringEnabled(true);
        conn->setShowRateLabel(showRateLabels_);
      }
    }
  }
}

void PlaybackAnimator::onPlaybackStopped() {
  isPlaying_ = false;
  stopAnimationTimer();
  resetAllAnimations();
}

void PlaybackAnimator::onPlaybackPaused() {
  animationTimer_->stop();
}

void PlaybackAnimator::onPlaybackResumed() {
  if (enabled_ && isPlaying_) {
    animationTimer_->start(ANIMATION_INTERVAL_MS);
  }
}

void PlaybackAnimator::onPlaybackPositionChanged(double timestamp) {
  lastUpdateTime_ = timestamp;
}

void PlaybackAnimator::onMessageReceived(const QString& topicName, double timestamp) {
  notifyMessage(topicName, timestamp);
}

void PlaybackAnimator::updateAnimations() {
  if (!canvas_ || !enabled_) return;

  // Update data flow animations for active connections
  for (ConnectionLine* conn : canvas_->connections()) {
    QString topic = conn->topicName();
    if (!topic.isEmpty() && topicStates_.contains(topic)) {
      const ConnectionAnimationState& state = topicStates_[topic];
      if (state.isActive) {
        // Update rate display
        conn->setMessageRate(state.estimatedRate);

        // Set activity state based on rate
        if (state.estimatedRate > ConnectionLine::HIGH_RATE_THRESHOLD) {
          conn->setActivityState(TopicActivityState::HighRate);
        } else if (state.estimatedRate > ConnectionLine::LOW_RATE_THRESHOLD) {
          conn->setActivityState(TopicActivityState::Active);
        } else {
          conn->setActivityState(TopicActivityState::Inactive);
        }
      }
    }
  }

  emit animationUpdated();
}

void PlaybackAnimator::decayAnimations() {
  if (!canvas_) return;

  double currentTime = lastUpdateTime_;
  QList<QString> topicsToDeactivate;

  // Check for inactive topics
  for (auto it = topicStates_.begin(); it != topicStates_.end(); ++it) {
    ConnectionAnimationState& state = it.value();
    if (state.isActive) {
      double timeSinceLastMessage = currentTime - state.lastMessageTime;
      if (timeSinceLastMessage > ACTIVITY_DECAY_TIME) {
        state.isActive = false;
        state.estimatedRate = 0.0;
        topicsToDeactivate.append(it.key());
      }
    }
  }

  // Update connections for deactivated topics
  for (const QString& topic : topicsToDeactivate) {
    ConnectionLine* conn = findConnectionForTopic(topic);
    if (conn) {
      conn->setActivityState(TopicActivityState::Inactive);
      conn->setMessageRate(0.0);
    }
  }
}

void PlaybackAnimator::startAnimationTimer() {
  if (!animationTimer_->isActive()) {
    animationTimer_->start(ANIMATION_INTERVAL_MS);
  }
  if (!decayTimer_->isActive()) {
    decayTimer_->start(DECAY_INTERVAL_MS);
  }
}

void PlaybackAnimator::stopAnimationTimer() {
  animationTimer_->stop();
  decayTimer_->stop();
}

void PlaybackAnimator::updateConnectionForTopic(const QString& topicName) {
  ConnectionLine* conn = findConnectionForTopic(topicName);
  if (!conn) return;

  const ConnectionAnimationState& state = topicStates_[topicName];

  // Trigger pulse animation if enabled
  if (pulseOnMessage_) {
    conn->pulseActivity();
  }

  // Update rate and activity state
  conn->setMessageRate(state.estimatedRate);

  if (state.estimatedRate > ConnectionLine::HIGH_RATE_THRESHOLD) {
    conn->setActivityState(TopicActivityState::HighRate);
  } else if (state.estimatedRate > ConnectionLine::LOW_RATE_THRESHOLD) {
    conn->setActivityState(TopicActivityState::Active);
  } else {
    conn->setActivityState(TopicActivityState::Inactive);
  }
}

ConnectionLine* PlaybackAnimator::findConnectionForTopic(const QString& topicName) {
  if (!canvas_) return nullptr;

  for (ConnectionLine* conn : canvas_->connections()) {
    if (conn->topicName() == topicName) {
      return conn;
    }
  }

  return nullptr;
}

void PlaybackAnimator::resetAllAnimations() {
  topicStates_.clear();

  if (canvas_) {
    for (ConnectionLine* conn : canvas_->connections()) {
      conn->setActivityState(TopicActivityState::Unknown);
      conn->setMessageRate(0.0);
      conn->setLiveMonitoringEnabled(false);
      conn->setShowRateLabel(false);
    }
  }
}

}  // namespace ros_weaver
