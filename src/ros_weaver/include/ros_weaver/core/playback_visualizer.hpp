#ifndef ROS_WEAVER_CORE_PLAYBACK_VISUALIZER_HPP
#define ROS_WEAVER_CORE_PLAYBACK_VISUALIZER_HPP

#include <QObject>
#include <QTimer>
#include <QMap>
#include <QSet>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

class PlaybackController;
class WeaverCanvas;
class ConnectionLine;

/**
 * @brief Manages time-synced playback visualization on the canvas
 *
 * During bag file playback, this class:
 * - Animates data flow on canvas connections
 * - Shows pulse/glow effects when topics are active
 * - Synchronizes with the PlaybackController
 * - Tracks which topics are active at each timestamp
 */
class PlaybackVisualizer : public QObject {
  Q_OBJECT

public:
  explicit PlaybackVisualizer(QObject* parent = nullptr);
  ~PlaybackVisualizer() override;

  /**
   * @brief Set the playback controller to sync with
   */
  void setPlaybackController(PlaybackController* controller);
  PlaybackController* playbackController() const { return playbackController_; }

  /**
   * @brief Set the canvas for visualization
   */
  void setCanvas(WeaverCanvas* canvas);
  WeaverCanvas* canvas() const { return canvas_; }

  /**
   * @brief Enable or disable visualization
   */
  void setEnabled(bool enabled);
  bool isEnabled() const { return enabled_; }

  /**
   * @brief Activity pulse duration in milliseconds
   */
  void setPulseDurationMs(int ms);
  int pulseDurationMs() const { return pulseDurationMs_; }

  /**
   * @brief Activity decay time (how long a topic stays "active" after last message)
   */
  void setActivityDecayMs(int ms);
  int activityDecayMs() const { return activityDecayMs_; }

  /**
   * @brief Get topics that were active in the most recent time window
   */
  QSet<QString> activeTopics() const { return activeTopics_; }

  /**
   * @brief Get total count of messages visualized in this session
   */
  qint64 messageCount() const { return messageCount_; }

  /**
   * @brief Reset statistics
   */
  void resetStats();

signals:
  /**
   * @brief Emitted when a topic becomes active on the canvas
   */
  void topicActivated(const QString& topicName);

  /**
   * @brief Emitted when a topic becomes inactive
   */
  void topicDeactivated(const QString& topicName);

  /**
   * @brief Emitted when visualization is enabled/disabled
   */
  void enabledChanged(bool enabled);

private slots:
  void onMessagePublished(const QString& topic, qint64 timeNs);
  void onPlaybackStateChanged(int state);
  void onDecayTimerTick();

private:
  void activateTopic(const QString& topicName);
  void deactivateTopic(const QString& topicName);
  void clearActiveTopics();
  ConnectionLine* findConnectionForTopic(const QString& topicName);
  void updateConnectionVisualization(ConnectionLine* conn, bool active);

  PlaybackController* playbackController_ = nullptr;
  WeaverCanvas* canvas_ = nullptr;

  bool enabled_ = true;
  int pulseDurationMs_ = 150;
  int activityDecayMs_ = 300;

  QTimer* decayTimer_ = nullptr;
  static constexpr int DECAY_CHECK_INTERVAL_MS = 50;

  QSet<QString> activeTopics_;
  QMap<QString, qint64> topicLastActiveTime_;  // Topic -> last message timestamp (ns)
  qint64 currentTimeNs_ = 0;
  qint64 messageCount_ = 0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_PLAYBACK_VISUALIZER_HPP
