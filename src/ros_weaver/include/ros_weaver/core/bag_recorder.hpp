#ifndef ROS_WEAVER_CORE_BAG_RECORDER_HPP
#define ROS_WEAVER_CORE_BAG_RECORDER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <thread>

namespace ros_weaver {

/**
 * @brief Recording state enum
 */
enum class RecordingState {
  Idle,
  Recording,
  Paused
};

/**
 * @brief Statistics about the current recording session
 */
struct RecordingStats {
  qint64 messageCount = 0;
  qint64 bytesWritten = 0;
  qint64 durationMs = 0;
  int topicCount = 0;
};

/**
 * @brief Manages rosbag2 recording operations
 *
 * Provides functionality to record ROS 2 topics to rosbag2 files
 * in SQLite3 or MCAP formats. Supports topic selection, pause/resume,
 * and real-time statistics.
 */
class BagRecorder : public QObject {
  Q_OBJECT

public:
  explicit BagRecorder(QObject* parent = nullptr);
  ~BagRecorder() override;

  // Recording path management
  void setOutputPath(const QString& path);
  QString outputPath() const;

  // Storage format
  void setStorageFormat(const QString& format);  // "sqlite3" or "mcap"
  QString storageFormat() const;

  // Topic selection
  void setTopicsToRecord(const QStringList& topics);
  void setRecordAllTopics(bool recordAll);
  QStringList topicsToRecord() const;
  bool recordAllTopics() const;

  // Recording operations
  bool startRecording();
  void stopRecording();
  void pauseRecording();
  void resumeRecording();

  // State
  RecordingState state() const;
  bool isRecording() const;
  bool isPaused() const;

  // Statistics
  RecordingStats statistics() const;

  // ROS node
  void setNode(rclcpp::Node::SharedPtr node);

signals:
  void recordingStarted(const QString& path);
  void recordingStopped();
  void recordingPaused();
  void recordingResumed();
  void recordingError(const QString& error);
  void statisticsUpdated(const RecordingStats& stats);
  void stateChanged(RecordingState state);

private slots:
  void onUpdateStatistics();

private:
  void createSubscriptions();
  void destroySubscriptions();
  void messageCallback(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                       const QString& topicName,
                       const QString& topicType);
  QStringList discoverAllTopics() const;
  QString inferMessageType(const QString& topicName) const;
  void setState(RecordingState newState);

  // ROS node and spin thread
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_{false};

  // Writer
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  // Subscriptions for recording
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  // Configuration
  QString outputPath_;
  QString storageFormat_ = "sqlite3";
  QStringList topicsToRecord_;
  bool recordAllTopics_ = true;

  // State
  std::atomic<RecordingState> state_{RecordingState::Idle};
  std::atomic<bool> paused_{false};

  // Statistics
  RecordingStats stats_;
  QTimer* statsTimer_ = nullptr;
  qint64 recordingStartTime_ = 0;

  mutable std::mutex mutex_;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::RecordingState)
Q_DECLARE_METATYPE(ros_weaver::RecordingStats)

#endif  // ROS_WEAVER_CORE_BAG_RECORDER_HPP
