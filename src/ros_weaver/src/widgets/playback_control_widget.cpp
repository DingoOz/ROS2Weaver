#include "ros_weaver/widgets/playback_control_widget.hpp"
#include "ros_weaver/core/playback_controller.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStyle>
#include <cmath>

namespace ros_weaver {

PlaybackControlWidget::PlaybackControlWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();
  setupConnections();
}

void PlaybackControlWidget::setPlaybackController(PlaybackController* controller) {
  if (controller_) {
    disconnect(controller_, nullptr, this, nullptr);
  }

  controller_ = controller;

  if (controller_) {
    connect(this, &PlaybackControlWidget::playClicked, controller_, &PlaybackController::play);
    connect(this, &PlaybackControlWidget::pauseClicked, controller_, &PlaybackController::pause);
    connect(this, &PlaybackControlWidget::stopClicked, controller_, &PlaybackController::stop);
    connect(this, &PlaybackControlWidget::stepForwardClicked, controller_, &PlaybackController::stepForward);
    connect(this, &PlaybackControlWidget::playbackRateRequested, controller_, &PlaybackController::setPlaybackRate);
    connect(this, &PlaybackControlWidget::loopingToggled, controller_, &PlaybackController::setLooping);
    connect(this, &PlaybackControlWidget::clockPublishingToggled, controller_, &PlaybackController::setPublishClock);

    connect(controller_, &PlaybackController::stateChanged, this, [this](PlaybackState state) {
      onPlaybackStateChanged(static_cast<int>(state));
    });
    connect(controller_, &PlaybackController::playbackRateChanged, this, &PlaybackControlWidget::onPlaybackRateChanged);
    connect(controller_, &PlaybackController::loopingChanged, this, &PlaybackControlWidget::onLoopingChanged);

    // Initialize from controller state
    loopCheckBox_->setChecked(controller_->isLooping());
    clockCheckBox_->setChecked(controller_->isPublishingClock());
    speedSpinBox_->setValue(controller_->playbackRate());
  }
}

bool PlaybackControlWidget::isPlaying() const {
  return isPlaying_;
}

bool PlaybackControlWidget::isPaused() const {
  return isPaused_;
}

bool PlaybackControlWidget::isStopped() const {
  return !isPlaying_ && !isPaused_;
}

double PlaybackControlWidget::playbackRate() const {
  return speedSpinBox_->value();
}

bool PlaybackControlWidget::isLooping() const {
  return loopCheckBox_->isChecked();
}

void PlaybackControlWidget::onPlaybackStateChanged(int state) {
  auto playbackState = static_cast<PlaybackState>(state);

  isPlaying_ = (playbackState == PlaybackState::Playing);
  isPaused_ = (playbackState == PlaybackState::Paused);

  updatePlayPauseButton();
  stopButton_->setEnabled(playbackState != PlaybackState::Stopped);
}

void PlaybackControlWidget::onPlaybackRateChanged(double rate) {
  // Block signals to avoid feedback loop
  speedSpinBox_->blockSignals(true);
  speedSlider_->blockSignals(true);

  speedSpinBox_->setValue(rate);
  speedSlider_->setValue(rateToSliderValue(rate));

  speedSpinBox_->blockSignals(false);
  speedSlider_->blockSignals(false);
}

void PlaybackControlWidget::onLoopingChanged(bool looping) {
  loopCheckBox_->blockSignals(true);
  loopCheckBox_->setChecked(looping);
  loopCheckBox_->blockSignals(false);
}

void PlaybackControlWidget::onPlayPauseClicked() {
  if (isPlaying_) {
    emit pauseClicked();
  } else {
    emit playClicked();
  }
}

void PlaybackControlWidget::onStopClicked() {
  emit stopClicked();
}

void PlaybackControlWidget::onStepForwardClicked() {
  emit stepForwardClicked();
}

void PlaybackControlWidget::onStepBackwardClicked() {
  emit stepBackwardClicked();
}

void PlaybackControlWidget::onSpeedSliderChanged(int value) {
  double rate = sliderValueToRate(value);
  speedSpinBox_->blockSignals(true);
  speedSpinBox_->setValue(rate);
  speedSpinBox_->blockSignals(false);

  emit playbackRateRequested(rate);
}

void PlaybackControlWidget::onSpeedSpinBoxChanged(double value) {
  speedSlider_->blockSignals(true);
  speedSlider_->setValue(rateToSliderValue(value));
  speedSlider_->blockSignals(false);

  emit playbackRateRequested(value);
}

void PlaybackControlWidget::onLoopCheckBoxToggled(bool checked) {
  emit loopingToggled(checked);
}

void PlaybackControlWidget::onClockCheckBoxToggled(bool checked) {
  emit clockPublishingToggled(checked);
}

void PlaybackControlWidget::setupUi() {
  auto* mainLayout = new QHBoxLayout(this);
  mainLayout->setContentsMargins(5, 2, 5, 2);
  mainLayout->setSpacing(5);

  // Transport controls
  stepBackButton_ = new QPushButton(this);
  stepBackButton_->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
  stepBackButton_->setToolTip(tr("Step Backward"));
  stepBackButton_->setFixedSize(32, 32);

  playPauseButton_ = new QPushButton(this);
  playPauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  playPauseButton_->setToolTip(tr("Play"));
  playPauseButton_->setFixedSize(40, 40);

  stopButton_ = new QPushButton(this);
  stopButton_->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
  stopButton_->setToolTip(tr("Stop"));
  stopButton_->setFixedSize(32, 32);
  stopButton_->setEnabled(false);

  stepForwardButton_ = new QPushButton(this);
  stepForwardButton_->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
  stepForwardButton_->setToolTip(tr("Step Forward"));
  stepForwardButton_->setFixedSize(32, 32);

  mainLayout->addWidget(stepBackButton_);
  mainLayout->addWidget(playPauseButton_);
  mainLayout->addWidget(stopButton_);
  mainLayout->addWidget(stepForwardButton_);

  // Separator
  auto* sep1 = new QFrame(this);
  sep1->setFrameShape(QFrame::VLine);
  sep1->setFrameShadow(QFrame::Sunken);
  mainLayout->addWidget(sep1);

  // Speed control
  speedLabel_ = new QLabel(tr("Speed:"), this);
  mainLayout->addWidget(speedLabel_);

  speedSlider_ = new QSlider(Qt::Horizontal, this);
  speedSlider_->setRange(-20, 20);  // Maps to 0.1x to 10x
  speedSlider_->setValue(0);  // 1.0x
  speedSlider_->setFixedWidth(100);
  speedSlider_->setToolTip(tr("Playback Speed"));
  mainLayout->addWidget(speedSlider_);

  speedSpinBox_ = new QDoubleSpinBox(this);
  speedSpinBox_->setRange(0.1, 10.0);
  speedSpinBox_->setValue(1.0);
  speedSpinBox_->setSingleStep(0.1);
  speedSpinBox_->setSuffix("x");
  speedSpinBox_->setFixedWidth(70);
  mainLayout->addWidget(speedSpinBox_);

  // Separator
  auto* sep2 = new QFrame(this);
  sep2->setFrameShape(QFrame::VLine);
  sep2->setFrameShadow(QFrame::Sunken);
  mainLayout->addWidget(sep2);

  // Options
  loopCheckBox_ = new QCheckBox(tr("Loop"), this);
  loopCheckBox_->setToolTip(tr("Loop playback when reaching end"));
  mainLayout->addWidget(loopCheckBox_);

  clockCheckBox_ = new QCheckBox(tr("Publish /clock"), this);
  clockCheckBox_->setToolTip(tr("Publish /clock topic for sim_time"));
  clockCheckBox_->setChecked(true);
  mainLayout->addWidget(clockCheckBox_);

  mainLayout->addStretch();
}

void PlaybackControlWidget::setupConnections() {
  connect(playPauseButton_, &QPushButton::clicked, this, &PlaybackControlWidget::onPlayPauseClicked);
  connect(stopButton_, &QPushButton::clicked, this, &PlaybackControlWidget::onStopClicked);
  connect(stepForwardButton_, &QPushButton::clicked, this, &PlaybackControlWidget::onStepForwardClicked);
  connect(stepBackButton_, &QPushButton::clicked, this, &PlaybackControlWidget::onStepBackwardClicked);
  connect(speedSlider_, &QSlider::valueChanged, this, &PlaybackControlWidget::onSpeedSliderChanged);
  connect(speedSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &PlaybackControlWidget::onSpeedSpinBoxChanged);
  connect(loopCheckBox_, &QCheckBox::toggled, this, &PlaybackControlWidget::onLoopCheckBoxToggled);
  connect(clockCheckBox_, &QCheckBox::toggled, this, &PlaybackControlWidget::onClockCheckBoxToggled);
}

void PlaybackControlWidget::updatePlayPauseButton() {
  if (isPlaying_) {
    playPauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    playPauseButton_->setToolTip(tr("Pause"));
  } else {
    playPauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    playPauseButton_->setToolTip(tr("Play"));
  }
}

double PlaybackControlWidget::sliderValueToRate(int value) const {
  // Logarithmic scale: -20 = 0.1x, 0 = 1.0x, 20 = 10.0x
  if (value == 0) return 1.0;
  return std::pow(10.0, value / 20.0);
}

int PlaybackControlWidget::rateToSliderValue(double rate) const {
  if (rate <= 0.1) return -20;
  if (rate >= 10.0) return 20;
  return static_cast<int>(std::round(20.0 * std::log10(rate)));
}

}  // namespace ros_weaver
