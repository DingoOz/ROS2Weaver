#ifndef ROS_WEAVER_WIDGETS_PLAYBACK_CONTROL_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_PLAYBACK_CONTROL_WIDGET_HPP

#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QDoubleSpinBox>

namespace ros_weaver {

class PlaybackController;

/**
 * @brief Widget providing playback control buttons and speed adjustment
 *
 * Contains play/pause/stop buttons, loop toggle, speed slider,
 * and clock publishing toggle.
 */
class PlaybackControlWidget : public QWidget {
  Q_OBJECT

public:
  explicit PlaybackControlWidget(QWidget* parent = nullptr);
  ~PlaybackControlWidget() override = default;

  // Controller connection
  void setPlaybackController(PlaybackController* controller);

  // State access
  bool isPlaying() const;
  bool isPaused() const;
  bool isStopped() const;
  double playbackRate() const;
  bool isLooping() const;

public slots:
  void onPlaybackStateChanged(int state);
  void onPlaybackRateChanged(double rate);
  void onLoopingChanged(bool looping);

signals:
  void playClicked();
  void pauseClicked();
  void stopClicked();
  void stepForwardClicked();
  void stepBackwardClicked();
  void playbackRateRequested(double rate);
  void loopingToggled(bool enabled);
  void clockPublishingToggled(bool enabled);

private slots:
  void onPlayPauseClicked();
  void onStopClicked();
  void onStepForwardClicked();
  void onStepBackwardClicked();
  void onSpeedSliderChanged(int value);
  void onSpeedSpinBoxChanged(double value);
  void onLoopCheckBoxToggled(bool checked);
  void onClockCheckBoxToggled(bool checked);

private:
  void setupUi();
  void setupConnections();
  void updatePlayPauseButton();
  double sliderValueToRate(int value) const;
  int rateToSliderValue(double rate) const;

  PlaybackController* controller_ = nullptr;

  // Transport controls
  QPushButton* playPauseButton_ = nullptr;
  QPushButton* stopButton_ = nullptr;
  QPushButton* stepBackButton_ = nullptr;
  QPushButton* stepForwardButton_ = nullptr;

  // Speed control
  QSlider* speedSlider_ = nullptr;
  QDoubleSpinBox* speedSpinBox_ = nullptr;
  QLabel* speedLabel_ = nullptr;

  // Options
  QCheckBox* loopCheckBox_ = nullptr;
  QCheckBox* clockCheckBox_ = nullptr;

  // State
  bool isPlaying_ = false;
  bool isPaused_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_PLAYBACK_CONTROL_WIDGET_HPP
