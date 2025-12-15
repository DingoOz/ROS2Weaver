#ifndef ROS_WEAVER_CORE_BAG_MANAGER_HPP
#define ROS_WEAVER_CORE_BAG_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QMutex>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <memory>
#include <mutex>

namespace ros_weaver {

// Forward declarations
struct BagMetadata;
struct BagTopicInfo;
struct TopicConfig;

/**
 * @brief Information about a topic in the bag
 */
struct BagTopicInfo {
  QString name;
  QString messageType;
  QString serialization_format;
  int64_t messageCount = 0;

  // QoS profile from bag metadata
  bool durabilityTransient = false;
  bool reliabilityReliable = true;
};

/**
 * @brief Per-topic configuration for playback
 */
struct TopicConfig {
  QString originalName;
  QString remappedName;
  QString messageType;
  bool enabled = true;
  int qosHistoryDepth = 10;
  bool qosReliable = true;
  bool qosDurabilityTransient = false;
  int64_t messageCount = 0;
};

/**
 * @brief Metadata about an opened bag file
 */
struct BagMetadata {
  QString path;
  QString storageId;           // "sqlite3" or "mcap"
  QString serializationFormat; // Usually "cdr"
  int64_t startTimeNs = 0;     // Start time in nanoseconds
  int64_t endTimeNs = 0;       // End time in nanoseconds
  int64_t durationNs = 0;      // Duration in nanoseconds
  int64_t messageCount = 0;
  int64_t fileSizeBytes = 0;
  int topicCount = 0;

  // Helper methods
  rclcpp::Time startTime() const { return rclcpp::Time(startTimeNs); }
  rclcpp::Time endTime() const { return rclcpp::Time(endTimeNs); }
  rclcpp::Duration duration() const { return rclcpp::Duration::from_nanoseconds(durationNs); }
};

/**
 * @brief Manages rosbag2 reading operations
 *
 * Provides functionality to open rosbag2 files (SQLite3 and MCAP formats),
 * enumerate topics, configure playback options, and read messages.
 * Thread-safe for use with Qt signal/slot system.
 */
class BagManager : public QObject {
  Q_OBJECT

public:
  explicit BagManager(QObject* parent = nullptr);
  ~BagManager() override;

  // Bag file operations
  bool openBag(const QString& path);
  void closeBag();
  bool isOpen() const;

  // Metadata access
  BagMetadata metadata() const;
  QList<BagTopicInfo> topics() const;
  rclcpp::Time startTime() const;
  rclcpp::Time endTime() const;
  rclcpp::Duration duration() const;

  // Topic configuration
  void setTopicEnabled(const QString& topic, bool enabled);
  void setTopicRemap(const QString& originalTopic, const QString& remappedTopic);
  void setTopicQos(const QString& topic, int historyDepth, bool reliable, bool durabilityTransient);
  TopicConfig topicConfig(const QString& topic) const;
  QList<TopicConfig> topicConfigs() const;
  QStringList enabledTopics() const;

  // Message reading
  bool seekToTime(const rclcpp::Time& time);
  bool seekToStart();
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> readNext();
  bool hasNext() const;
  rclcpp::Time currentReadTime() const;

  // Utility
  static QString detectStorageFormat(const QString& path);
  static bool isValidBagPath(const QString& path);

signals:
  void bagOpened(const BagMetadata& metadata);
  void bagClosed();
  void errorOccurred(const QString& error);
  void topicConfigChanged(const QString& topic);
  void seekCompleted(const rclcpp::Time& time);

private:
  void parseMetadata();
  void buildTopicConfigs();
  QString inferStorageId(const QString& path) const;

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  BagMetadata metadata_;
  QList<BagTopicInfo> topicInfos_;
  QMap<QString, TopicConfig> topicConfigs_;

  bool isOpen_ = false;
  rclcpp::Time currentReadTime_;

  mutable std::mutex mutex_;
};

}  // namespace ros_weaver

// Register metatypes for Qt signal/slot
Q_DECLARE_METATYPE(ros_weaver::BagMetadata)
Q_DECLARE_METATYPE(ros_weaver::BagTopicInfo)
Q_DECLARE_METATYPE(ros_weaver::TopicConfig)

#endif  // ROS_WEAVER_CORE_BAG_MANAGER_HPP
