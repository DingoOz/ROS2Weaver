#ifndef ROS_WEAVER_WIDGETS_ROSBAG_WORKBENCH_PANEL_HPP
#define ROS_WEAVER_WIDGETS_ROSBAG_WORKBENCH_PANEL_HPP

#include <QWidget>
#include <QSplitter>
#include <QTabWidget>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>

namespace ros_weaver {

// Forward declarations
class WeaverCanvas;
class BagManager;
class BagRecorder;
class PlaybackController;
class SlamPipelineManager;
class TimelineWidget;
class PlaybackControlWidget;
class TopicSelectorWidget;
class SlamConfigWidget;
class BookmarkManager;
class TopicViewerPanel;
class PlotPanel;
class ParamDashboard;

struct RecordingStats;

/**
 * @brief Main rosbag workbench panel integrating all playback and SLAM components
 *
 * This is the top-level widget for the rosbag workbench feature. It integrates:
 * - Bag loading and management
 * - Playback controls and timeline
 * - Topic selection and filtering
 * - SLAM configuration and launching
 * - Bookmark management
 *
 * It also provides integration points with existing panels like TopicViewer,
 * PlotPanel, and ParamDashboard.
 */
class RosbagWorkbenchPanel : public QWidget {
  Q_OBJECT

public:
  explicit RosbagWorkbenchPanel(QWidget* parent = nullptr);
  ~RosbagWorkbenchPanel() override;

  // External panel integration
  void setCanvas(WeaverCanvas* canvas);
  void setTopicViewerPanel(TopicViewerPanel* panel);
  void setPlotPanel(PlotPanel* panel);
  void setParamDashboard(ParamDashboard* dashboard);

  // Bag operations
  bool openBag(const QString& path);
  void closeBag();
  bool isBagOpen() const;
  QString currentBagPath() const;

  // Core component access
  BagManager* bagManager() const;
  BagRecorder* bagRecorder() const;
  PlaybackController* playbackController() const;
  SlamPipelineManager* slamPipelineManager() const;

  // Recording path
  void setRecordingPath(const QString& path);
  QString recordingPath() const;

public slots:
  void onOpenBagClicked();
  void onCloseBagClicked();
  void onRecordClicked();
  void onSetRecordingPathClicked();

signals:
  void bagOpened(const QString& path);
  void bagClosed();
  void playbackStarted();
  void playbackPaused();
  void playbackStopped();
  void slamLaunched();
  void slamStopped();
  void recordingStarted(const QString& path);
  void recordingStopped();

private slots:
  void onBagOpened();
  void onBagClosed();
  void onBagError(const QString& error);
  void onPlaybackStateChanged(int state);
  void onPlaybackTimeChanged(qint64 timeNs);
  void onPlaybackProgressChanged(double percent);
  void onPlaybackFinished();
  void onSeekRequested(qint64 timeNs);
  void onBookmarkSelected(qint64 timeNs);
  void onRecordingStarted(const QString& path);
  void onRecordingStopped();
  void onRecordingError(const QString& error);
  void onRecordingStatsUpdated(const RecordingStats& stats);

private:
  void setupUi();
  void setupConnections();
  void createToolbar();
  void createMainContent();
  void updateBagInfo();
  void updatePlaybackInfo();
  void updateRecordingInfo();

  // Core components
  BagManager* bagManager_ = nullptr;
  BagRecorder* bagRecorder_ = nullptr;
  PlaybackController* playbackController_ = nullptr;
  SlamPipelineManager* slamManager_ = nullptr;

  // UI - Toolbar
  QPushButton* openBagButton_ = nullptr;
  QPushButton* closeBagButton_ = nullptr;
  QPushButton* recordButton_ = nullptr;
  QPushButton* setRecordingPathButton_ = nullptr;
  QLabel* bagInfoLabel_ = nullptr;
  QLabel* recordingInfoLabel_ = nullptr;
  QProgressBar* progressBar_ = nullptr;

  // UI - Main content
  QSplitter* mainSplitter_ = nullptr;
  QTabWidget* leftTabs_ = nullptr;
  QTabWidget* rightTabs_ = nullptr;

  // UI - Playback section
  TimelineWidget* timelineWidget_ = nullptr;
  PlaybackControlWidget* playbackControlWidget_ = nullptr;

  // UI - Topic section
  TopicSelectorWidget* topicSelectorWidget_ = nullptr;

  // UI - SLAM section
  SlamConfigWidget* slamConfigWidget_ = nullptr;

  // UI - Bookmarks
  BookmarkManager* bookmarkManager_ = nullptr;

  // External panel references
  WeaverCanvas* canvas_ = nullptr;
  TopicViewerPanel* topicViewerPanel_ = nullptr;
  PlotPanel* plotPanel_ = nullptr;
  ParamDashboard* paramDashboard_ = nullptr;

  // Current state
  QString currentBagPath_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ROSBAG_WORKBENCH_PANEL_HPP
