#ifndef ROS_WEAVER_CORE_PLAYBACK_CONTROLLER_HPP
#define ROS_WEAVER_CORE_PLAYBACK_CONTROLLER_HPP

#include <QObject>
#include <QTimer>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>

namespace ros_weaver {

class BagManager;

/**
 * @brief Playback state enumeration
 */
enum class PlaybackState {
  Stopped,
  Playing,
  Paused
};

/**
 * @brief Controls rosbag playback timing and message publishing
 *
 * Manages the playback of rosbag messages to ROS 2 topics, including
 * timing control (rate adjustment, seek, stepping) and /clock publishing
 * for sim_time support.
 */
class PlaybackController : public QObject {
  Q_OBJECT

public:
  explicit PlaybackController(QObject* parent = nullptr);
  ~PlaybackController() override;

  // Playback control
  void play();
  void pause();
  void stop();
  void seek(const rclcpp::Time& time);
  void stepForward();
  void stepBackward(int numMessages = 1);

  // Playback settings
  void setPlaybackRate(double rate);  // 0.1x to 10x
  void setLooping(bool loop);
  void setPublishClock(bool publish);

  // State access
  PlaybackState state() const;
  rclcpp::Time currentTime() const;
  double playbackRate() const;
  bool isLooping() const;
  bool isPublishingClock() const;

  // Progress info
  double progressPercent() const;
  QString currentTimeString() const;
  QString durationString() const;

  // Bag manager connection
  void setBagManager(BagManager* manager);
  BagManager* bagManager() const;

signals:
  void stateChanged(PlaybackState state);
  void timeChanged(qint64 timeNs);  // Time in nanoseconds
  void progressChanged(double percent);
  void playbackRateChanged(double rate);
  void messagePublished(const QString& topic, qint64 timeNs);
  void playbackFinished();
  void errorOccurred(const QString& error);
  void loopingChanged(bool looping);
  void clockPublishingChanged(bool publishing);

public slots:
  void onBagOpened();
  void onBagClosed();

private slots:
  void onPlaybackTimerTick();

private:
  void initializeRosNode();
  void shutdownRosNode();
  void createPublishers();
  void destroyPublishers();
  void createPublisher(const QString& topic, const QString& messageType, int qosDepth, bool reliable);
  void publishMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg);
  void publishClock(const rclcpp::Time& time);
  void updateProgress();
  QString formatTime(const rclcpp::Time& time) const;

  // Bag manager reference
  BagManager* bagManager_ = nullptr;

  // ROS 2 node and publishers
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_{false};

  // Topic publishers (original topic name -> publisher)
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher_;

  // Playback state
  PlaybackState state_ = PlaybackState::Stopped;
  rclcpp::Time currentTime_;
  rclcpp::Time bagStartTime_;
  rclcpp::Time bagEndTime_;
  rclcpp::Time playbackStartWallTime_;  // Wall time when playback started
  rclcpp::Time playbackStartBagTime_;   // Bag time when playback started

  // Playback settings
  double playbackRate_ = 1.0;
  bool looping_ = false;
  bool publishClock_ = true;

  // Timer for playback loop
  QTimer* playbackTimer_ = nullptr;
  static constexpr int TIMER_INTERVAL_MS = 1;  // 1ms resolution

  // Thread safety
  mutable std::mutex mutex_;

  // Message queue for rate control
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> pendingMessage_;
  rclcpp::Time nextMessageTime_;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::PlaybackState)

#endif  // ROS_WEAVER_CORE_PLAYBACK_CONTROLLER_HPP
