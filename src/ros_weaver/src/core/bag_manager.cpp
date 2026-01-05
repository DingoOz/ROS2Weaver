#include "ros_weaver/core/bag_manager.hpp"

#include <QFileInfo>
#include <QDir>
#include <QDebug>

#include <rosbag2_storage/metadata_io.hpp>

// ROS 2 Humble uses time_stamp, Jazzy uses recv_timestamp
// Check for Jazzy+ by detecting the new member name
#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#endif

// Humble is version 16.x, Jazzy is version 28.x
// In Humble, SerializedBagMessage uses time_stamp; in Jazzy it uses recv_timestamp
#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->recv_timestamp
#else
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->time_stamp
#endif

namespace ros_weaver {

BagManager::BagManager(QObject* parent)
    : QObject(parent)
    , reader_(nullptr)
    , isOpen_(false)
    , currentReadTime_(0, 0) {
  // Register metatypes for cross-thread signals
  qRegisterMetaType<BagMetadata>("BagMetadata");
  qRegisterMetaType<BagTopicInfo>("BagTopicInfo");
  qRegisterMetaType<TopicConfig>("TopicConfig");
}

BagManager::~BagManager() {
  closeBag();
}

bool BagManager::openBag(const QString& path) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Close any existing bag (use internal method since we already hold the lock)
  if (isOpen_) {
    closeBagInternal();
  }

  QFileInfo fileInfo(path);
  if (!fileInfo.exists()) {
    emit errorOccurred(tr("Bag path does not exist: %1").arg(path));
    return false;
  }

  try {
    // Create reader
    reader_ = std::make_unique<rosbag2_cpp::Reader>();

    // Detect storage format
    QString storageId = inferStorageId(path);

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = path.toStdString();
    storageOptions.storage_id = storageId.toStdString();

    rosbag2_cpp::ConverterOptions converterOptions;
    converterOptions.input_serialization_format = "cdr";
    converterOptions.output_serialization_format = "cdr";

    reader_->open(storageOptions, converterOptions);

    isOpen_ = true;
    metadata_.path = path;
    metadata_.storageId = storageId;

    // Parse metadata from the bag
    parseMetadata();

    // Build topic configurations
    buildTopicConfigs();

    // Emit success signal
    emit bagOpened(metadata_);

    qDebug() << "BagManager: Opened bag" << path
             << "with" << metadata_.topicCount << "topics,"
             << metadata_.messageCount << "messages";

    return true;
  }
  catch (const std::exception& e) {
    reader_.reset();
    isOpen_ = false;
    emit errorOccurred(tr("Failed to open bag: %1").arg(e.what()));
    return false;
  }
}

void BagManager::closeBag() {
  std::lock_guard<std::mutex> lock(mutex_);
  closeBagInternal();
  emit bagClosed();
}

void BagManager::closeBagInternal() {
  // Internal method - caller must hold mutex_
  if (reader_) {
    reader_.reset();
  }

  isOpen_ = false;
  metadata_ = BagMetadata();
  topicInfos_.clear();
  topicConfigs_.clear();
  currentReadTime_ = rclcpp::Time(0, 0);
}

bool BagManager::isOpen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return isOpen_;
}

BagMetadata BagManager::metadata() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return metadata_;
}

QList<BagTopicInfo> BagManager::topics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return topicInfos_;
}

rclcpp::Time BagManager::startTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return metadata_.startTime();
}

rclcpp::Time BagManager::endTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return metadata_.endTime();
}

rclcpp::Duration BagManager::duration() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return metadata_.duration();
}

void BagManager::setTopicEnabled(const QString& topic, bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (topicConfigs_.contains(topic)) {
    topicConfigs_[topic].enabled = enabled;
    emit topicConfigChanged(topic);
  }
}

void BagManager::setTopicRemap(const QString& originalTopic, const QString& remappedTopic) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (topicConfigs_.contains(originalTopic)) {
    topicConfigs_[originalTopic].remappedName = remappedTopic;
    emit topicConfigChanged(originalTopic);
  }
}

void BagManager::setTopicQos(const QString& topic, int historyDepth, bool reliable, bool durabilityTransient) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (topicConfigs_.contains(topic)) {
    topicConfigs_[topic].qosHistoryDepth = historyDepth;
    topicConfigs_[topic].qosReliable = reliable;
    topicConfigs_[topic].qosDurabilityTransient = durabilityTransient;
    emit topicConfigChanged(topic);
  }
}

TopicConfig BagManager::topicConfig(const QString& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return topicConfigs_.value(topic);
}

QList<TopicConfig> BagManager::topicConfigs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return topicConfigs_.values();
}

QStringList BagManager::enabledTopics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  QStringList enabled;
  for (auto it = topicConfigs_.constBegin(); it != topicConfigs_.constEnd(); ++it) {
    if (it.value().enabled) {
      enabled.append(it.key());
    }
  }
  return enabled;
}

bool BagManager::seekToTime(const rclcpp::Time& time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!isOpen_ || !reader_) {
    emit errorOccurred(tr("Cannot seek: no bag is open"));
    return false;
  }

  try {
    reader_->seek(time.nanoseconds());
    currentReadTime_ = time;
    emit seekCompleted(time);
    return true;
  }
  catch (const std::exception& e) {
    emit errorOccurred(tr("Seek failed: %1").arg(e.what()));
    return false;
  }
}

bool BagManager::seekToStart() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!isOpen_ || !reader_) {
    return false;
  }

  try {
    reader_->seek(metadata_.startTimeNs);
    currentReadTime_ = rclcpp::Time(metadata_.startTimeNs);
    emit seekCompleted(rclcpp::Time(metadata_.startTimeNs));
    return true;
  }
  catch (const std::exception& e) {
    emit errorOccurred(tr("Seek to start failed: %1").arg(e.what()));
    return false;
  }
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> BagManager::readNext() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!isOpen_ || !reader_ || !reader_->has_next()) {
    return nullptr;
  }

  try {
    auto msg = reader_->read_next();
    if (msg) {
      currentReadTime_ = rclcpp::Time(ROSBAG2_MSG_TIMESTAMP(msg));
    }
    return msg;
  }
  catch (const std::exception& e) {
    emit errorOccurred(tr("Read failed: %1").arg(e.what()));
    return nullptr;
  }
}

bool BagManager::hasNext() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return isOpen_ && reader_ && reader_->has_next();
}

rclcpp::Time BagManager::currentReadTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentReadTime_;
}

QString BagManager::detectStorageFormat(const QString& path) {
  QFileInfo fileInfo(path);

  // Check for MCAP file extension
  if (fileInfo.suffix().toLower() == "mcap") {
    return "mcap";
  }

  // Check for db3 file extension
  if (fileInfo.suffix().toLower() == "db3") {
    return "sqlite3";
  }

  // Check if it's a directory (typical bag folder)
  if (fileInfo.isDir()) {
    QDir bagDir(path);

    // Look for MCAP file
    QStringList mcapFiles = bagDir.entryList(QStringList() << "*.mcap", QDir::Files);
    if (!mcapFiles.isEmpty()) {
      return "mcap";
    }

    // Look for SQLite3 file
    QStringList dbFiles = bagDir.entryList(QStringList() << "*.db3", QDir::Files);
    if (!dbFiles.isEmpty()) {
      return "sqlite3";
    }

    // Check metadata.yaml for storage_identifier
    if (bagDir.exists("metadata.yaml")) {
      // Read metadata.yaml to determine storage format
      // For now, default to sqlite3 as it's most common
      return "sqlite3";
    }
  }

  // Default to sqlite3
  return "sqlite3";
}

bool BagManager::isValidBagPath(const QString& path) {
  QFileInfo fileInfo(path);

  if (!fileInfo.exists()) {
    return false;
  }

  // Check if it's a valid bag file or directory
  if (fileInfo.isFile()) {
    QString suffix = fileInfo.suffix().toLower();
    return suffix == "db3" || suffix == "mcap";
  }

  if (fileInfo.isDir()) {
    QDir bagDir(path);
    // Check for metadata.yaml or bag files
    return bagDir.exists("metadata.yaml") ||
           !bagDir.entryList(QStringList() << "*.db3" << "*.mcap", QDir::Files).isEmpty();
  }

  return false;
}

void BagManager::parseMetadata() {
  if (!reader_) return;

  try {
    const auto& bagMetadata = reader_->get_metadata();

    metadata_.serializationFormat = QString::fromStdString(
        bagMetadata.topics_with_message_count.empty() ? "cdr" :
        bagMetadata.topics_with_message_count.front().topic_metadata.serialization_format);

    // Calculate start/end times and message count
    if (bagMetadata.starting_time.time_since_epoch().count() > 0) {
      metadata_.startTimeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
          bagMetadata.starting_time.time_since_epoch()).count();
    }

    if (bagMetadata.duration.count() > 0) {
      metadata_.durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
          bagMetadata.duration).count();
      metadata_.endTimeNs = metadata_.startTimeNs + metadata_.durationNs;
    }

    metadata_.messageCount = bagMetadata.message_count;
    metadata_.topicCount = static_cast<int>(bagMetadata.topics_with_message_count.size());

    // Parse topic information
    topicInfos_.clear();
    for (const auto& topicWithCount : bagMetadata.topics_with_message_count) {
      BagTopicInfo info;
      info.name = QString::fromStdString(topicWithCount.topic_metadata.name);
      info.messageType = QString::fromStdString(topicWithCount.topic_metadata.type);
      info.serialization_format = QString::fromStdString(topicWithCount.topic_metadata.serialization_format);
      info.messageCount = static_cast<int64_t>(topicWithCount.message_count);

      // Parse QoS from offered_qos_profiles if available
      // In Jazzy, offered_qos_profiles is a vector of rclcpp::QoS objects
      // In Humble, offered_qos_profiles is a YAML string
#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
      if (!topicWithCount.topic_metadata.offered_qos_profiles.empty()) {
        const auto& firstQos = topicWithCount.topic_metadata.offered_qos_profiles.front();
        info.reliabilityReliable =
            firstQos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
        info.durabilityTransient =
            firstQos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
      }
#else
      // Humble: offered_qos_profiles is a YAML string - parse it simply
      const std::string& qosYaml = topicWithCount.topic_metadata.offered_qos_profiles;
      if (!qosYaml.empty()) {
        // Simple string search for reliability and durability settings
        info.reliabilityReliable = qosYaml.find("reliability: reliable") != std::string::npos;
        info.durabilityTransient = qosYaml.find("durability: transient_local") != std::string::npos;
      }
#endif

      topicInfos_.append(info);
    }

    // Get file size
    QFileInfo fileInfo(metadata_.path);
    if (fileInfo.isDir()) {
      QDir bagDir(metadata_.path);
      QFileInfoList files = bagDir.entryInfoList(QDir::Files);
      metadata_.fileSizeBytes = 0;
      for (const auto& fi : files) {
        metadata_.fileSizeBytes += fi.size();
      }
    } else {
      metadata_.fileSizeBytes = fileInfo.size();
    }
  }
  catch (const std::exception& e) {
    qWarning() << "BagManager: Error parsing metadata:" << e.what();
  }
}

void BagManager::buildTopicConfigs() {
  topicConfigs_.clear();

  for (const auto& info : topicInfos_) {
    TopicConfig config;
    config.originalName = info.name;
    config.remappedName = info.name;  // Default: no remap
    config.messageType = info.messageType;
    config.enabled = true;
    config.qosHistoryDepth = 10;
    config.qosReliable = info.reliabilityReliable;
    config.qosDurabilityTransient = info.durabilityTransient;
    config.messageCount = info.messageCount;

    topicConfigs_[info.name] = config;
  }
}

QString BagManager::inferStorageId(const QString& path) const {
  return detectStorageFormat(path);
}

}  // namespace ros_weaver
