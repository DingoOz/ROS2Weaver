#ifndef ROS_WEAVER_CORE_PLAYBACK_ANIMATOR_HPP
#define ROS_WEAVER_CORE_PLAYBACK_ANIMATOR_HPP

#include <QObject>
#include <QMap>
#include <QString>
#include <QTimer>
#include <QDateTime>

namespace ros_weaver {

class WeaverCanvas;
class ConnectionLine;
class PlaybackController;

// Stores animation state for a connection
struct ConnectionAnimationState {
  QString topicName;
  double lastMessageTime = 0.0;
  int messageCount = 0;
  double estimatedRate = 0.0;
  bool isActive = false;
};

// Animates canvas connections during rosbag playback
class PlaybackAnimator : public QObject {
  Q_OBJECT

public:
  explicit PlaybackAnimator(QObject* parent = nullptr);
  ~PlaybackAnimator() override;

  // Set the canvas to animate
  void setCanvas(WeaverCanvas* canvas);
  WeaverCanvas* canvas() const { return canvas_; }

  // Connect to playback controller
  void setPlaybackController(PlaybackController* controller);

  // Enable/disable animation
  void setEnabled(bool enabled);
  bool isEnabled() const { return enabled_; }

  // Animation settings
  void setAnimationSpeed(double speed);  // 1.0 = realtime, 2.0 = 2x, etc.
  double animationSpeed() const { return animationSpeed_; }

  void setShowRateLabels(bool show);
  bool showRateLabels() const { return showRateLabels_; }

  void setPulseOnMessage(bool pulse);
  bool pulseOnMessage() const { return pulseOnMessage_; }

  // Manual message notification (for external use)
  void notifyMessage(const QString& topicName, double timestamp);

public slots:
  void onPlaybackStarted();
  void onPlaybackStopped();
  void onPlaybackPaused();
  void onPlaybackResumed();
  void onPlaybackPositionChanged(double timestamp);
  void onMessageReceived(const QString& topicName, double timestamp);

signals:
  void animationUpdated();
  void topicActivityChanged(const QString& topicName, double rate);

private slots:
  void updateAnimations();
  void decayAnimations();

private:
  void startAnimationTimer();
  void stopAnimationTimer();
  void updateConnectionForTopic(const QString& topicName);
  ConnectionLine* findConnectionForTopic(const QString& topicName);
  void resetAllAnimations();

  WeaverCanvas* canvas_;
  PlaybackController* playbackController_;

  bool enabled_;
  bool isPlaying_;
  double animationSpeed_;
  bool showRateLabels_;
  bool pulseOnMessage_;

  // Animation state per topic
  QMap<QString, ConnectionAnimationState> topicStates_;

  // Timers
  QTimer* animationTimer_;
  QTimer* decayTimer_;

  // Timing
  double lastUpdateTime_;

  static constexpr int ANIMATION_INTERVAL_MS = 50;    // 20 FPS
  static constexpr int DECAY_INTERVAL_MS = 500;       // Check for inactive topics
  static constexpr double ACTIVITY_DECAY_TIME = 1.0;  // Seconds before marking inactive
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_PLAYBACK_ANIMATOR_HPP
