#include "ros_weaver/widgets/rosbag_workbench_panel.hpp"
#include "ros_weaver/core/bag_manager.hpp"
#include "ros_weaver/core/bag_recorder.hpp"
#include "ros_weaver/core/playback_controller.hpp"
#include "ros_weaver/core/slam_pipeline_manager.hpp"
#include "ros_weaver/widgets/timeline_widget.hpp"
#include "ros_weaver/widgets/playback_control_widget.hpp"
#include "ros_weaver/widgets/topic_selector_widget.hpp"
#include "ros_weaver/widgets/slam_config_widget.hpp"
#include "ros_weaver/widgets/bookmark_manager.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QStyle>
#include <QDebug>
#include <QDir>

namespace ros_weaver {

RosbagWorkbenchPanel::RosbagWorkbenchPanel(QWidget* parent)
    : QWidget(parent) {
  // Create core components
  bagManager_ = new BagManager(this);
  bagRecorder_ = new BagRecorder(this);
  playbackController_ = new PlaybackController(this);
  slamManager_ = new SlamPipelineManager(this);

  // Wire up core components
  playbackController_->setBagManager(bagManager_);
  slamManager_->setBagManager(bagManager_);
  slamManager_->setPlaybackController(playbackController_);

  setupUi();
  setupConnections();
}

RosbagWorkbenchPanel::~RosbagWorkbenchPanel() {
  // Stop playback, recording, and SLAM before destruction
  playbackController_->stop();
  bagRecorder_->stopRecording();
  slamManager_->stopSlam();
}

void RosbagWorkbenchPanel::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void RosbagWorkbenchPanel::setTopicViewerPanel(TopicViewerPanel* panel) {
  topicViewerPanel_ = panel;
}

void RosbagWorkbenchPanel::setPlotPanel(PlotPanel* panel) {
  plotPanel_ = panel;
}

void RosbagWorkbenchPanel::setParamDashboard(ParamDashboard* dashboard) {
  paramDashboard_ = dashboard;
}

bool RosbagWorkbenchPanel::openBag(const QString& path) {
  return bagManager_->openBag(path);
}

void RosbagWorkbenchPanel::closeBag() {
  playbackController_->stop();
  bagManager_->closeBag();
}

bool RosbagWorkbenchPanel::isBagOpen() const {
  return bagManager_->isOpen();
}

QString RosbagWorkbenchPanel::currentBagPath() const {
  return currentBagPath_;
}

BagManager* RosbagWorkbenchPanel::bagManager() const {
  return bagManager_;
}

BagRecorder* RosbagWorkbenchPanel::bagRecorder() const {
  return bagRecorder_;
}

PlaybackController* RosbagWorkbenchPanel::playbackController() const {
  return playbackController_;
}

SlamPipelineManager* RosbagWorkbenchPanel::slamPipelineManager() const {
  return slamManager_;
}

void RosbagWorkbenchPanel::setRecordingPath(const QString& path) {
  bagRecorder_->setOutputPath(path);
}

QString RosbagWorkbenchPanel::recordingPath() const {
  return bagRecorder_->outputPath();
}

void RosbagWorkbenchPanel::onOpenBagClicked() {
  QString path = QFileDialog::getOpenFileName(
      this,
      tr("Open Rosbag"),
      QString(),
      tr("Rosbag Files (*.db3 *.mcap);;All Files (*)"));

  if (path.isEmpty()) return;

  // Also check for directory (bag folder)
  if (QFileInfo(path).isDir() || path.endsWith(".db3") || path.endsWith(".mcap")) {
    if (!openBag(path)) {
      QMessageBox::warning(this, tr("Error"),
                           tr("Failed to open bag file:\n%1").arg(path));
    }
  }
}

void RosbagWorkbenchPanel::onCloseBagClicked() {
  closeBag();
}

void RosbagWorkbenchPanel::onRecordClicked() {
  if (bagRecorder_->isRecording() || bagRecorder_->isPaused()) {
    bagRecorder_->stopRecording();
  } else {
    if (!bagRecorder_->startRecording()) {
      // Error handled by signal
    }
  }
}

void RosbagWorkbenchPanel::onSetRecordingPathClicked() {
  QString currentPath = bagRecorder_->outputPath();
  QString path = QFileDialog::getExistingDirectory(
      this,
      tr("Select Recording Directory"),
      currentPath,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (!path.isEmpty()) {
    bagRecorder_->setOutputPath(path);
    updateRecordingInfo();
  }
}

void RosbagWorkbenchPanel::onRecordingStarted(const QString& path) {
  recordButton_->setText(tr("Stop Recording"));
  recordButton_->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
  recordButton_->setStyleSheet("QPushButton { background-color: #cc4444; color: white; }");
  setRecordingPathButton_->setEnabled(false);
  openBagButton_->setEnabled(false);

  updateRecordingInfo();
  emit recordingStarted(path);
}

void RosbagWorkbenchPanel::onRecordingStopped() {
  recordButton_->setText(tr("Record"));
  recordButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  recordButton_->setStyleSheet("");
  setRecordingPathButton_->setEnabled(true);
  openBagButton_->setEnabled(true);

  recordingInfoLabel_->setText(tr("Not recording"));
  emit recordingStopped();
}

void RosbagWorkbenchPanel::onRecordingError(const QString& error) {
  QMessageBox::warning(this, tr("Recording Error"), error);
}

void RosbagWorkbenchPanel::onRecordingStatsUpdated(const RecordingStats& stats) {
  // Format duration
  int seconds = static_cast<int>(stats.durationMs / 1000);
  int minutes = seconds / 60;
  seconds = seconds % 60;

  // Format size
  QString sizeStr;
  if (stats.bytesWritten >= 1073741824) {
    sizeStr = QString("%1 GB").arg(stats.bytesWritten / 1073741824.0, 0, 'f', 2);
  } else if (stats.bytesWritten >= 1048576) {
    sizeStr = QString("%1 MB").arg(stats.bytesWritten / 1048576.0, 0, 'f', 1);
  } else {
    sizeStr = QString("%1 KB").arg(stats.bytesWritten / 1024.0, 0, 'f', 1);
  }

  QString info = QString("REC %1:%2 | %3 msgs | %4")
      .arg(minutes, 2, 10, QChar('0'))
      .arg(seconds, 2, 10, QChar('0'))
      .arg(stats.messageCount)
      .arg(sizeStr);

  recordingInfoLabel_->setText(info);
  recordingInfoLabel_->setStyleSheet("QLabel { color: #cc4444; font-weight: bold; }");
}

void RosbagWorkbenchPanel::onBagOpened() {
  BagMetadata meta = bagManager_->metadata();
  currentBagPath_ = meta.path;

  // Update timeline
  timelineWidget_->setTimeRange(meta.startTime(), meta.endTime());
  timelineWidget_->setCurrentTime(meta.startTime());

  // Update UI
  updateBagInfo();
  closeBagButton_->setEnabled(true);
  progressBar_->setValue(0);

  emit bagOpened(currentBagPath_);
}

void RosbagWorkbenchPanel::onBagClosed() {
  currentBagPath_.clear();

  timelineWidget_->setTimeRange(rclcpp::Time(0, 0), rclcpp::Time(0, 0));

  bagInfoLabel_->setText(tr("No bag loaded"));
  closeBagButton_->setEnabled(false);
  progressBar_->setValue(0);

  emit bagClosed();
}

void RosbagWorkbenchPanel::onBagError(const QString& error) {
  QMessageBox::warning(this, tr("Bag Error"), error);
}

void RosbagWorkbenchPanel::onPlaybackStateChanged(int state) {
  auto playbackState = static_cast<PlaybackState>(state);

  switch (playbackState) {
    case PlaybackState::Playing:
      emit playbackStarted();
      break;
    case PlaybackState::Paused:
      emit playbackPaused();
      break;
    case PlaybackState::Stopped:
      emit playbackStopped();
      break;
  }
}

void RosbagWorkbenchPanel::onPlaybackTimeChanged(qint64 timeNs) {
  timelineWidget_->setCurrentTime(rclcpp::Time(timeNs));
  updatePlaybackInfo();
}

void RosbagWorkbenchPanel::onPlaybackProgressChanged(double percent) {
  progressBar_->setValue(static_cast<int>(percent));
}

void RosbagWorkbenchPanel::onPlaybackFinished() {
  progressBar_->setValue(100);
}

void RosbagWorkbenchPanel::onSeekRequested(qint64 timeNs) {
  playbackController_->seek(rclcpp::Time(timeNs));
}

void RosbagWorkbenchPanel::onBookmarkSelected(qint64 timeNs) {
  playbackController_->seek(rclcpp::Time(timeNs));
}

void RosbagWorkbenchPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(5, 5, 5, 5);
  mainLayout->setSpacing(5);

  createToolbar();
  createMainContent();
}

void RosbagWorkbenchPanel::setupConnections() {
  // Bag manager signals
  connect(bagManager_, &BagManager::bagOpened, this, [this](const BagMetadata&) {
    onBagOpened();
  });
  connect(bagManager_, &BagManager::bagClosed, this, &RosbagWorkbenchPanel::onBagClosed);
  connect(bagManager_, &BagManager::errorOccurred, this, &RosbagWorkbenchPanel::onBagError);

  // Playback controller signals
  connect(playbackController_, &PlaybackController::stateChanged, this, [this](PlaybackState state) {
    onPlaybackStateChanged(static_cast<int>(state));
  });
  connect(playbackController_, &PlaybackController::timeChanged, this, &RosbagWorkbenchPanel::onPlaybackTimeChanged);
  connect(playbackController_, &PlaybackController::progressChanged, this, &RosbagWorkbenchPanel::onPlaybackProgressChanged);
  connect(playbackController_, &PlaybackController::playbackFinished, this, &RosbagWorkbenchPanel::onPlaybackFinished);

  // Timeline signals
  connect(timelineWidget_, &TimelineWidget::seekRequested, this, &RosbagWorkbenchPanel::onSeekRequested);

  // Bookmark manager signals
  connect(bookmarkManager_, &BookmarkManager::bookmarkSelected, this, &RosbagWorkbenchPanel::onBookmarkSelected);

  // SLAM manager signals
  connect(slamManager_, &SlamPipelineManager::slamStarted, this, [this]() {
    emit slamLaunched();
  });
  connect(slamManager_, &SlamPipelineManager::slamStopped, this, [this]() {
    emit slamStopped();
  });

  // Recording signals
  connect(bagRecorder_, &BagRecorder::recordingStarted, this, &RosbagWorkbenchPanel::onRecordingStarted);
  connect(bagRecorder_, &BagRecorder::recordingStopped, this, &RosbagWorkbenchPanel::onRecordingStopped);
  connect(bagRecorder_, &BagRecorder::recordingError, this, &RosbagWorkbenchPanel::onRecordingError);
  connect(bagRecorder_, &BagRecorder::statisticsUpdated, this, &RosbagWorkbenchPanel::onRecordingStatsUpdated);
}

void RosbagWorkbenchPanel::createToolbar() {
  auto* toolbarLayout = new QHBoxLayout();

  // Open button
  openBagButton_ = new QPushButton(this);
  openBagButton_->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
  openBagButton_->setText(tr("Open Bag"));
  openBagButton_->setToolTip(tr("Open a rosbag file (.db3 or .mcap)"));
  connect(openBagButton_, &QPushButton::clicked, this, &RosbagWorkbenchPanel::onOpenBagClicked);
  toolbarLayout->addWidget(openBagButton_);

  // Close button
  closeBagButton_ = new QPushButton(this);
  closeBagButton_->setIcon(style()->standardIcon(QStyle::SP_DialogCloseButton));
  closeBagButton_->setText(tr("Close"));
  closeBagButton_->setToolTip(tr("Close current bag"));
  closeBagButton_->setEnabled(false);
  connect(closeBagButton_, &QPushButton::clicked, this, &RosbagWorkbenchPanel::onCloseBagClicked);
  toolbarLayout->addWidget(closeBagButton_);

  // Separator
  auto* sep1 = new QFrame(this);
  sep1->setFrameShape(QFrame::VLine);
  sep1->setFrameShadow(QFrame::Sunken);
  toolbarLayout->addWidget(sep1);

  // Record button
  recordButton_ = new QPushButton(this);
  recordButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  recordButton_->setText(tr("Record"));
  recordButton_->setToolTip(tr("Start/stop recording topics to a new rosbag"));
  connect(recordButton_, &QPushButton::clicked, this, &RosbagWorkbenchPanel::onRecordClicked);
  toolbarLayout->addWidget(recordButton_);

  // Set recording path button
  setRecordingPathButton_ = new QPushButton(this);
  setRecordingPathButton_->setIcon(style()->standardIcon(QStyle::SP_DirIcon));
  setRecordingPathButton_->setText(tr("Set Path"));
  setRecordingPathButton_->setToolTip(tr("Set the directory for saving recordings"));
  connect(setRecordingPathButton_, &QPushButton::clicked, this, &RosbagWorkbenchPanel::onSetRecordingPathClicked);
  toolbarLayout->addWidget(setRecordingPathButton_);

  // Separator
  auto* sep2 = new QFrame(this);
  sep2->setFrameShape(QFrame::VLine);
  sep2->setFrameShadow(QFrame::Sunken);
  toolbarLayout->addWidget(sep2);

  // Bag info label
  bagInfoLabel_ = new QLabel(tr("No bag loaded"), this);
  toolbarLayout->addWidget(bagInfoLabel_, 1);

  // Recording info label
  recordingInfoLabel_ = new QLabel(tr("Not recording"), this);
  recordingInfoLabel_->setMinimumWidth(200);
  toolbarLayout->addWidget(recordingInfoLabel_);

  // Progress bar
  progressBar_ = new QProgressBar(this);
  progressBar_->setRange(0, 100);
  progressBar_->setValue(0);
  progressBar_->setTextVisible(true);
  progressBar_->setFixedWidth(150);
  toolbarLayout->addWidget(progressBar_);

  static_cast<QVBoxLayout*>(layout())->addLayout(toolbarLayout);
}

void RosbagWorkbenchPanel::createMainContent() {
  // Timeline and playback controls (top section)
  auto* playbackSection = new QWidget(this);
  auto* playbackLayout = new QVBoxLayout(playbackSection);
  playbackLayout->setContentsMargins(0, 0, 0, 0);
  playbackLayout->setSpacing(2);

  timelineWidget_ = new TimelineWidget(this);
  playbackLayout->addWidget(timelineWidget_);

  playbackControlWidget_ = new PlaybackControlWidget(this);
  playbackControlWidget_->setPlaybackController(playbackController_);
  playbackLayout->addWidget(playbackControlWidget_);

  static_cast<QVBoxLayout*>(layout())->addWidget(playbackSection);

  // Main splitter for left/right content
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  // Left tabs (Topics, Bookmarks)
  leftTabs_ = new QTabWidget(this);

  topicSelectorWidget_ = new TopicSelectorWidget(this);
  topicSelectorWidget_->setBagManager(bagManager_);
  leftTabs_->addTab(topicSelectorWidget_, tr("Topics"));

  bookmarkManager_ = new BookmarkManager(this);
  bookmarkManager_->setTimelineWidget(timelineWidget_);
  bookmarkManager_->setPlaybackController(playbackController_);
  leftTabs_->addTab(bookmarkManager_, tr("Bookmarks"));

  mainSplitter_->addWidget(leftTabs_);

  // Right tabs (SLAM)
  rightTabs_ = new QTabWidget(this);

  slamConfigWidget_ = new SlamConfigWidget(this);
  slamConfigWidget_->setSlamPipelineManager(slamManager_);
  rightTabs_->addTab(slamConfigWidget_, tr("SLAM Toolbox"));

  mainSplitter_->addWidget(rightTabs_);

  // Set splitter proportions
  mainSplitter_->setStretchFactor(0, 1);
  mainSplitter_->setStretchFactor(1, 1);
  mainSplitter_->setSizes({400, 400});

  static_cast<QVBoxLayout*>(layout())->addWidget(mainSplitter_, 1);
}

void RosbagWorkbenchPanel::updateBagInfo() {
  if (!bagManager_->isOpen()) {
    bagInfoLabel_->setText(tr("No bag loaded"));
    return;
  }

  BagMetadata meta = bagManager_->metadata();
  QFileInfo fileInfo(meta.path);

  // Format duration
  int64_t durationMs = meta.duration().nanoseconds() / 1000000;
  int minutes = static_cast<int>(durationMs / 60000);
  int seconds = static_cast<int>((durationMs % 60000) / 1000);

  // Format file size
  QString sizeStr;
  if (meta.fileSizeBytes >= 1073741824) {
    sizeStr = QString("%1 GB").arg(meta.fileSizeBytes / 1073741824.0, 0, 'f', 2);
  } else if (meta.fileSizeBytes >= 1048576) {
    sizeStr = QString("%1 MB").arg(meta.fileSizeBytes / 1048576.0, 0, 'f', 1);
  } else {
    sizeStr = QString("%1 KB").arg(meta.fileSizeBytes / 1024.0, 0, 'f', 1);
  }

  QString info = QString("%1 | %2:%3 | %4 msgs | %5 topics | %6 | %7")
      .arg(fileInfo.fileName())
      .arg(minutes, 2, 10, QChar('0'))
      .arg(seconds, 2, 10, QChar('0'))
      .arg(meta.messageCount)
      .arg(meta.topicCount)
      .arg(sizeStr)
      .arg(meta.storageId.toUpper());

  bagInfoLabel_->setText(info);
}

void RosbagWorkbenchPanel::updatePlaybackInfo() {
  // Additional playback info could be shown here
}

void RosbagWorkbenchPanel::updateRecordingInfo() {
  if (bagRecorder_->isRecording()) {
    // Recording info is updated by the stats callback
    return;
  }

  QString path = bagRecorder_->outputPath();
  QDir dir(path);
  QString dirName = dir.dirName();
  if (dirName.isEmpty()) {
    dirName = path;
  }

  recordingInfoLabel_->setText(tr("Save to: %1").arg(dirName));
  recordingInfoLabel_->setStyleSheet("");
  recordingInfoLabel_->setToolTip(path);
}

}  // namespace ros_weaver
