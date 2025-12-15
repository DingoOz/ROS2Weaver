#include "ros_weaver/core/playback_controller.hpp"
#include "ros_weaver/core/bag_manager.hpp"

#include <QDateTime>
#include <QDebug>

// ROS 2 Humble uses time_stamp, Jazzy uses recv_timestamp
#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#endif

// Humble is version 16.x, Jazzy is version 28.x
#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->recv_timestamp
#else
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->time_stamp
#endif

namespace ros_weaver {

PlaybackController::PlaybackController(QObject* parent)
    : QObject(parent)
    , currentTime_(0, 0)
    , bagStartTime_(0, 0)
    , bagEndTime_(0, 0)
    , playbackStartWallTime_(0, 0)
    , playbackStartBagTime_(0, 0)
    , nextMessageTime_(0, 0) {
  qRegisterMetaType<PlaybackState>("PlaybackState");

  // Create playback timer
  playbackTimer_ = new QTimer(this);
  playbackTimer_->setTimerType(Qt::PreciseTimer);
  connect(playbackTimer_, &QTimer::timeout, this, &PlaybackController::onPlaybackTimerTick);
}

PlaybackController::~PlaybackController() {
  stop();
  shutdownRosNode();
}

void PlaybackController::play() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!bagManager_ || !bagManager_->isOpen()) {
    emit errorOccurred(tr("Cannot play: no bag is open"));
    return;
  }

  if (state_ == PlaybackState::Playing) {
    return;
  }

  // Initialize ROS node if needed
  if (!node_) {
    initializeRosNode();
    createPublishers();
  }

  // Set start times
  playbackStartWallTime_ = rclcpp::Clock().now();

  if (state_ == PlaybackState::Stopped) {
    bagManager_->seekToStart();
    currentTime_ = bagStartTime_;
    playbackStartBagTime_ = bagStartTime_;

    // Read first message
    pendingMessage_ = bagManager_->readNext();
    if (pendingMessage_) {
      nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
    }
  } else {
    // Resuming from pause
    playbackStartBagTime_ = currentTime_;
  }

  state_ = PlaybackState::Playing;
  emit stateChanged(state_);

  // Start playback timer
  playbackTimer_->start(TIMER_INTERVAL_MS);

  qDebug() << "PlaybackController: Started playback at rate" << playbackRate_;
}

void PlaybackController::pause() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ != PlaybackState::Playing) {
    return;
  }

  playbackTimer_->stop();
  state_ = PlaybackState::Paused;
  emit stateChanged(state_);

  qDebug() << "PlaybackController: Paused at" << formatTime(currentTime_);
}

void PlaybackController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  playbackTimer_->stop();

  if (bagManager_ && bagManager_->isOpen()) {
    bagManager_->seekToStart();
  }

  currentTime_ = bagStartTime_;
  pendingMessage_.reset();
  state_ = PlaybackState::Stopped;

  emit stateChanged(state_);
  emit timeChanged(currentTime_.nanoseconds());
  emit progressChanged(0.0);

  qDebug() << "PlaybackController: Stopped";
}

void PlaybackController::seek(const rclcpp::Time& time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!bagManager_ || !bagManager_->isOpen()) {
    return;
  }

  bool wasPlaying = (state_ == PlaybackState::Playing);
  if (wasPlaying) {
    playbackTimer_->stop();
  }

  bagManager_->seekToTime(time);
  currentTime_ = time;
  playbackStartBagTime_ = time;
  playbackStartWallTime_ = rclcpp::Clock().now();

  // Read next message after seek
  pendingMessage_ = bagManager_->readNext();
  if (pendingMessage_) {
    nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
  }

  emit timeChanged(currentTime_.nanoseconds());
  updateProgress();

  if (wasPlaying) {
    playbackTimer_->start(TIMER_INTERVAL_MS);
  }
}

void PlaybackController::stepForward() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!bagManager_ || !bagManager_->isOpen()) {
    return;
  }

  // Make sure we're paused or stopped
  if (state_ == PlaybackState::Playing) {
    playbackTimer_->stop();
    state_ = PlaybackState::Paused;
    emit stateChanged(state_);
  }

  // Read and publish one message
  if (!pendingMessage_) {
    pendingMessage_ = bagManager_->readNext();
  }

  if (pendingMessage_) {
    publishMessage(pendingMessage_);
    currentTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
    emit timeChanged(currentTime_.nanoseconds());
    updateProgress();

    // Read next
    pendingMessage_ = bagManager_->readNext();
    if (pendingMessage_) {
      nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
    }
  }

  if (!bagManager_->hasNext() && !pendingMessage_) {
    emit playbackFinished();
  }
}

void PlaybackController::stepBackward(int numMessages) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!bagManager_ || !bagManager_->isOpen()) {
    return;
  }

  // Seeking backward is tricky - seek to slightly earlier time
  int64_t stepBackNs = static_cast<int64_t>(numMessages * 100000000);  // ~100ms per step
  int64_t newTimeNs = currentTime_.nanoseconds() - stepBackNs;

  if (newTimeNs < bagStartTime_.nanoseconds()) {
    newTimeNs = bagStartTime_.nanoseconds();
  }

  rclcpp::Time newTime(newTimeNs);
  bagManager_->seekToTime(newTime);
  currentTime_ = newTime;

  pendingMessage_ = bagManager_->readNext();
  if (pendingMessage_) {
    nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
  }

  emit timeChanged(currentTime_.nanoseconds());
  updateProgress();
}

void PlaybackController::setPlaybackRate(double rate) {
  std::lock_guard<std::mutex> lock(mutex_);

  rate = qBound(0.1, rate, 10.0);
  if (qFuzzyCompare(playbackRate_, rate)) {
    return;
  }

  playbackRate_ = rate;

  // Reset timing reference when rate changes during playback
  if (state_ == PlaybackState::Playing) {
    playbackStartWallTime_ = rclcpp::Clock().now();
    playbackStartBagTime_ = currentTime_;
  }

  emit playbackRateChanged(rate);
}

void PlaybackController::setLooping(bool loop) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (looping_ == loop) {
    return;
  }

  looping_ = loop;
  emit loopingChanged(loop);
}

void PlaybackController::setPublishClock(bool publish) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (publishClock_ == publish) {
    return;
  }

  publishClock_ = publish;

  // Create or destroy clock publisher
  if (node_) {
    if (publish && !clockPublisher_) {
      clockPublisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(10).best_effort());
    } else if (!publish) {
      clockPublisher_.reset();
    }
  }

  emit clockPublishingChanged(publish);
}

PlaybackState PlaybackController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

rclcpp::Time PlaybackController::currentTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentTime_;
}

double PlaybackController::playbackRate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return playbackRate_;
}

bool PlaybackController::isLooping() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return looping_;
}

bool PlaybackController::isPublishingClock() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return publishClock_;
}

double PlaybackController::progressPercent() const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (bagEndTime_.nanoseconds() <= bagStartTime_.nanoseconds()) {
    return 0.0;
  }

  int64_t elapsed = currentTime_.nanoseconds() - bagStartTime_.nanoseconds();
  int64_t total = bagEndTime_.nanoseconds() - bagStartTime_.nanoseconds();

  return static_cast<double>(elapsed) / static_cast<double>(total) * 100.0;
}

QString PlaybackController::currentTimeString() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return formatTime(currentTime_);
}

QString PlaybackController::durationString() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!bagManager_) return "00:00.000";
  return formatTime(rclcpp::Time(bagEndTime_.nanoseconds() - bagStartTime_.nanoseconds()));
}

void PlaybackController::setBagManager(BagManager* manager) {
  if (bagManager_) {
    disconnect(bagManager_, nullptr, this, nullptr);
  }

  bagManager_ = manager;

  if (bagManager_) {
    connect(bagManager_, &BagManager::bagOpened, this, &PlaybackController::onBagOpened);
    connect(bagManager_, &BagManager::bagClosed, this, &PlaybackController::onBagClosed);
  }
}

BagManager* PlaybackController::bagManager() const {
  return bagManager_;
}

void PlaybackController::onBagOpened() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!bagManager_) return;

  BagMetadata meta = bagManager_->metadata();
  bagStartTime_ = meta.startTime();
  bagEndTime_ = meta.endTime();
  currentTime_ = bagStartTime_;

  // Recreate publishers for new bag
  destroyPublishers();

  emit timeChanged(currentTime_.nanoseconds());
  emit progressChanged(0.0);
}

void PlaybackController::onBagClosed() {
  stop();
  destroyPublishers();
  shutdownRosNode();
}

void PlaybackController::onPlaybackTimerTick() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ != PlaybackState::Playing || !bagManager_) {
    return;
  }

  // Calculate current bag time based on wall time elapsed
  rclcpp::Time wallNow = rclcpp::Clock().now();
  int64_t wallElapsedNs = wallNow.nanoseconds() - playbackStartWallTime_.nanoseconds();
  int64_t bagElapsedNs = static_cast<int64_t>(wallElapsedNs * playbackRate_);
  int64_t targetBagTimeNs = playbackStartBagTime_.nanoseconds() + bagElapsedNs;

  // Publish all messages up to target time
  while (pendingMessage_ && nextMessageTime_.nanoseconds() <= targetBagTimeNs) {
    publishMessage(pendingMessage_);
    currentTime_ = nextMessageTime_;

    // Read next message
    pendingMessage_ = bagManager_->readNext();
    if (pendingMessage_) {
      nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
    } else {
      break;
    }
  }

  // Update current time for display (even if no messages)
  if (targetBagTimeNs > currentTime_.nanoseconds()) {
    currentTime_ = rclcpp::Time(qMin(targetBagTimeNs, bagEndTime_.nanoseconds()));
  }

  // Publish clock
  if (publishClock_ && clockPublisher_) {
    publishClock(currentTime_);
  }

  emit timeChanged(currentTime_.nanoseconds());
  updateProgress();

  // Check for end of bag
  if (!pendingMessage_ && !bagManager_->hasNext()) {
    if (looping_) {
      // Loop back to start
      bagManager_->seekToStart();
      currentTime_ = bagStartTime_;
      playbackStartBagTime_ = bagStartTime_;
      playbackStartWallTime_ = rclcpp::Clock().now();

      pendingMessage_ = bagManager_->readNext();
      if (pendingMessage_) {
        nextMessageTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(pendingMessage_));
      }
    } else {
      playbackTimer_->stop();
      state_ = PlaybackState::Stopped;
      emit stateChanged(state_);
      emit playbackFinished();
    }
  }
}

void PlaybackController::initializeRosNode() {
  if (node_) return;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("ros_weaver_playback");

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    // Create clock publisher if enabled
    if (publishClock_) {
      clockPublisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(10).best_effort());
    }

    qDebug() << "PlaybackController: ROS node initialized";
  }
  catch (const std::exception& e) {
    emit errorOccurred(tr("Failed to initialize ROS node: %1").arg(e.what()));
  }
}

void PlaybackController::shutdownRosNode() {
  spinning_ = false;

  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();

  destroyPublishers();
  clockPublisher_.reset();
  node_.reset();
}

void PlaybackController::createPublishers() {
  if (!node_ || !bagManager_) return;

  destroyPublishers();

  QList<TopicConfig> configs = bagManager_->topicConfigs();
  for (const auto& config : configs) {
    if (config.enabled) {
      createPublisher(config.remappedName, config.messageType,
                      config.qosHistoryDepth, config.qosReliable);
    }
  }

  qDebug() << "PlaybackController: Created" << publishers_.size() << "publishers";
}

void PlaybackController::destroyPublishers() {
  publishers_.clear();
}

void PlaybackController::createPublisher(const QString& topic, const QString& messageType,
                                         int qosDepth, bool reliable) {
  if (!node_) return;

  try {
    rclcpp::QoS qos(qosDepth);
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    auto publisher = node_->create_generic_publisher(
        topic.toStdString(),
        messageType.toStdString(),
        qos);

    publishers_[topic.toStdString()] = publisher;
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to create publisher for" << topic << ":" << e.what();
  }
}

void PlaybackController::publishMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
  if (!msg || !bagManager_) return;

  QString originalTopic = QString::fromStdString(msg->topic_name);
  TopicConfig config = bagManager_->topicConfig(originalTopic);

  if (!config.enabled) {
    return;
  }

  QString publishTopic = config.remappedName;
  auto it = publishers_.find(publishTopic.toStdString());

  if (it != publishers_.end() && it->second) {
    try {
      auto serialized = std::make_shared<rclcpp::SerializedMessage>(msg->serialized_data->buffer_length);
      memcpy(serialized->get_rcl_serialized_message().buffer,
             msg->serialized_data->buffer,
             msg->serialized_data->buffer_length);
      serialized->get_rcl_serialized_message().buffer_length = msg->serialized_data->buffer_length;

      it->second->publish(*serialized);

      emit messagePublished(publishTopic, ROSBAG2_MSG_TIMESTAMP(msg));
    }
    catch (const std::exception& e) {
      qWarning() << "Failed to publish message on" << publishTopic << ":" << e.what();
    }
  }
}

void PlaybackController::publishClock(const rclcpp::Time& time) {
  if (!clockPublisher_) return;

  rosgraph_msgs::msg::Clock clockMsg;
  clockMsg.clock = time;
  clockPublisher_->publish(clockMsg);
}

void PlaybackController::updateProgress() {
  double progress = progressPercent();
  emit progressChanged(progress);
}

QString PlaybackController::formatTime(const rclcpp::Time& time) const {
  int64_t totalMs = time.nanoseconds() / 1000000;
  int minutes = static_cast<int>(totalMs / 60000);
  int seconds = static_cast<int>((totalMs % 60000) / 1000);
  int ms = static_cast<int>(totalMs % 1000);

  return QString("%1:%2.%3")
      .arg(minutes, 2, 10, QChar('0'))
      .arg(seconds, 2, 10, QChar('0'))
      .arg(ms, 3, 10, QChar('0'));
}

}  // namespace ros_weaver
