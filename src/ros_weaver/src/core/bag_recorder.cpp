#include "ros_weaver/core/bag_recorder.hpp"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QDebug>

#include <chrono>

namespace ros_weaver {

BagRecorder::BagRecorder(QObject* parent)
    : QObject(parent) {
  // Register metatypes for signal/slot
  qRegisterMetaType<RecordingState>("RecordingState");
  qRegisterMetaType<RecordingStats>("RecordingStats");

  // Statistics update timer
  statsTimer_ = new QTimer(this);
  statsTimer_->setInterval(500);  // Update every 500ms
  connect(statsTimer_, &QTimer::timeout, this, &BagRecorder::onUpdateStatistics);

  // Default output path
  outputPath_ = QDir::homePath() + "/rosbag_recordings";
}

BagRecorder::~BagRecorder() {
  if (state_ != RecordingState::Idle) {
    stopRecording();
  }
}

void BagRecorder::setOutputPath(const QString& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  outputPath_ = path;
}

QString BagRecorder::outputPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return outputPath_;
}

void BagRecorder::setStorageFormat(const QString& format) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (format == "sqlite3" || format == "mcap") {
    storageFormat_ = format;
  }
}

QString BagRecorder::storageFormat() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return storageFormat_;
}

void BagRecorder::setTopicsToRecord(const QStringList& topics) {
  std::lock_guard<std::mutex> lock(mutex_);
  topicsToRecord_ = topics;
}

void BagRecorder::setRecordAllTopics(bool recordAll) {
  std::lock_guard<std::mutex> lock(mutex_);
  recordAllTopics_ = recordAll;
}

QStringList BagRecorder::topicsToRecord() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return topicsToRecord_;
}

bool BagRecorder::recordAllTopics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return recordAllTopics_;
}

void BagRecorder::setNode(rclcpp::Node::SharedPtr node) {
  node_ = node;
}

bool BagRecorder::startRecording() {
  if (state_ != RecordingState::Idle) {
    emit recordingError(tr("Recording already in progress"));
    return false;
  }

  // Create our own ROS node for recording subscriptions
  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    QString nodeName = QString("ros_weaver_recorder_%1").arg(QDateTime::currentMSecsSinceEpoch());
    node_ = std::make_shared<rclcpp::Node>(nodeName.toStdString());
  } catch (const std::exception& e) {
    emit recordingError(tr("Failed to create ROS node: %1").arg(e.what()));
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Create output directory if needed
  QDir dir(outputPath_);
  if (!dir.exists()) {
    if (!dir.mkpath(".")) {
      emit recordingError(tr("Failed to create output directory: %1").arg(outputPath_));
      return false;
    }
  }

  // Generate bag filename with timestamp
  QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  QString bagName = QString("recording_%1").arg(timestamp);
  QString fullPath = outputPath_ + "/" + bagName;

  try {
    // Create writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = fullPath.toStdString();
    storage_options.storage_id = storageFormat_.toStdString();

    writer_->open(storage_options);

    // Determine which topics to record
    QStringList topics = recordAllTopics_ ? discoverAllTopics() : topicsToRecord_;

    if (topics.isEmpty()) {
      emit recordingError(tr("No topics available to record"));
      writer_.reset();
      return false;
    }

    // Reset statistics
    stats_ = RecordingStats{};
    stats_.topicCount = topics.size();
    recordingStartTime_ = QDateTime::currentMSecsSinceEpoch();

    // Create subscriptions
    createSubscriptions();

    // Start spin thread to process incoming messages
    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    setState(RecordingState::Recording);
    paused_ = false;
    statsTimer_->start();

    qDebug() << "Started recording to" << fullPath << "with" << topics.size() << "topics";
    emit recordingStarted(fullPath);
    return true;

  } catch (const std::exception& e) {
    emit recordingError(tr("Failed to start recording: %1").arg(e.what()));
    writer_.reset();
    return false;
  }
}

void BagRecorder::stopRecording() {
  if (state_ == RecordingState::Idle) {
    return;
  }

  statsTimer_->stop();

  // Stop the spin thread first
  spinning_ = false;
  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();

  destroySubscriptions();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (writer_) {
      writer_.reset();
    }
  }

  // Clean up the node
  node_.reset();

  setState(RecordingState::Idle);
  paused_ = false;

  qDebug() << "Recording stopped. Total messages:" << stats_.messageCount;
  emit recordingStopped();
}

void BagRecorder::pauseRecording() {
  if (state_ != RecordingState::Recording) {
    return;
  }

  paused_ = true;
  setState(RecordingState::Paused);
  emit recordingPaused();
}

void BagRecorder::resumeRecording() {
  if (state_ != RecordingState::Paused) {
    return;
  }

  paused_ = false;
  setState(RecordingState::Recording);
  emit recordingResumed();
}

RecordingState BagRecorder::state() const {
  return state_.load();
}

bool BagRecorder::isRecording() const {
  return state_ == RecordingState::Recording;
}

bool BagRecorder::isPaused() const {
  return state_ == RecordingState::Paused;
}

RecordingStats BagRecorder::statistics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void BagRecorder::onUpdateStatistics() {
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.durationMs = QDateTime::currentMSecsSinceEpoch() - recordingStartTime_;
  emit statisticsUpdated(stats_);
}

void BagRecorder::createSubscriptions() {
  if (!node_) return;

  QStringList topics = recordAllTopics_ ? discoverAllTopics() : topicsToRecord_;

  for (const QString& topic : topics) {
    QString msgType = inferMessageType(topic);
    if (msgType.isEmpty()) {
      qWarning() << "Could not determine message type for topic:" << topic;
      continue;
    }

    try {
      auto sub = node_->create_generic_subscription(
          topic.toStdString(),
          msgType.toStdString(),
          rclcpp::QoS(10).best_effort().durability_volatile(),
          [this, topic, msgType](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            messageCallback(msg, topic, msgType);
          });

      subscriptions_.push_back(sub);
    } catch (const std::exception& e) {
      qWarning() << "Failed to create subscription for" << topic << ":" << e.what();
    }
  }

  qDebug() << "Created" << subscriptions_.size() << "subscriptions for recording";
}

void BagRecorder::destroySubscriptions() {
  subscriptions_.clear();
}

void BagRecorder::messageCallback(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                                  const QString& topicName,
                                  const QString& topicType) {
  if (paused_ || !writer_) {
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(mutex_);

    rclcpp::Time time_stamp = node_->now();

    // Use the write API with shared_ptr to avoid deprecation warning
    writer_->write(
        msg,
        topicName.toStdString(),
        topicType.toStdString(),
        time_stamp);

    auto& rcl_msg = msg->get_rcl_serialized_message();
    stats_.messageCount++;
    stats_.bytesWritten += static_cast<qint64>(rcl_msg.buffer_length);

  } catch (const std::exception& e) {
    qWarning() << "Failed to write message:" << e.what();
  }
}

QStringList BagRecorder::discoverAllTopics() const {
  QStringList topics;

  if (!node_) {
    return topics;
  }

  auto topic_names_and_types = node_->get_topic_names_and_types();
  for (const auto& [name, types] : topic_names_and_types) {
    // Skip internal ROS topics
    QString topicName = QString::fromStdString(name);
    if (topicName.startsWith("/rosout") ||
        topicName.startsWith("/parameter_events") ||
        topicName.contains("/_")) {
      continue;
    }
    topics.append(topicName);
  }

  return topics;
}

QString BagRecorder::inferMessageType(const QString& topicName) const {
  if (!node_) {
    return QString();
  }

  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topicName.toStdString());
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    return QString::fromStdString(it->second[0]);
  }

  return QString();
}

void BagRecorder::setState(RecordingState newState) {
  state_ = newState;
  emit stateChanged(newState);
}

}  // namespace ros_weaver
