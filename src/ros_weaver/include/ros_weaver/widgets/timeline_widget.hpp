#ifndef ROS_WEAVER_WIDGETS_TIMELINE_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_TIMELINE_WIDGET_HPP

#include <QWidget>
#include <QList>
#include <QString>
#include <QColor>
#include <QMap>
#include <QVector>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

/**
 * @brief Bookmark on the timeline
 */
struct TimelineBookmark {
  rclcpp::Time time;
  QString label;
  QColor color = QColor(255, 200, 0);  // Yellow/gold default
};

/**
 * @brief Custom timeline widget for rosbag playback visualization
 *
 * Displays a timeline with playhead, tick marks, bookmarks, and optional
 * topic activity heatmap. Supports mouse scrubbing, zoom, and pan.
 */
class TimelineWidget : public QWidget {
  Q_OBJECT

public:
  explicit TimelineWidget(QWidget* parent = nullptr);
  ~TimelineWidget() override = default;

  // Time range
  void setTimeRange(const rclcpp::Time& start, const rclcpp::Time& end);
  void setCurrentTime(const rclcpp::Time& time);
  rclcpp::Time startTime() const;
  rclcpp::Time endTime() const;
  rclcpp::Time currentTime() const;

  // Progress
  void setProgress(double percent);
  double progress() const;

  // Bookmarks
  void addBookmark(const rclcpp::Time& time, const QString& label,
                   const QColor& color = QColor(255, 200, 0));
  void removeBookmark(int index);
  void clearBookmarks();
  QList<TimelineBookmark> bookmarks() const;

  // Topic activity visualization
  void setShowTopicActivity(bool show);
  bool showTopicActivity() const;
  void setTopicActivityData(const QString& topic, const QVector<qint64>& timestamps);
  void clearTopicActivity();

  // Zoom and pan
  void setZoomLevel(double level);  // 1.0 = full range visible
  double zoomLevel() const;
  void resetView();

signals:
  void seekRequested(qint64 timeNs);  // Time in nanoseconds
  void bookmarkClicked(int index);
  void bookmarkDoubleClicked(int index);
  void progressDragged(double percent);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private:
  void drawBackground(QPainter& painter);
  void drawTickMarks(QPainter& painter);
  void drawBookmarks(QPainter& painter);
  void drawPlayhead(QPainter& painter);
  void drawTopicActivity(QPainter& painter);
  void drawTimeLabels(QPainter& painter);

  rclcpp::Time timeFromX(int x) const;
  int xFromTime(const rclcpp::Time& time) const;
  int bookmarkAtPos(const QPoint& pos) const;
  QString formatTime(const rclcpp::Time& time) const;
  QString formatDuration(int64_t ns) const;

  // Time range
  rclcpp::Time startTime_;
  rclcpp::Time endTime_;
  rclcpp::Time currentTime_;

  // Visible range (for zoom/pan)
  rclcpp::Time visibleStart_;
  rclcpp::Time visibleEnd_;

  // Bookmarks
  QList<TimelineBookmark> bookmarks_;

  // Topic activity heatmap
  bool showTopicActivity_ = false;
  QMap<QString, QVector<qint64>> topicActivity_;

  // Interaction state
  bool isDragging_ = false;
  bool isPanning_ = false;
  QPoint lastMousePos_;
  int hoveredBookmark_ = -1;

  // Visual settings
  double zoomLevel_ = 1.0;
  int playheadHeight_ = 60;
  int tickHeight_ = 40;
  int bookmarkHeight_ = 15;
  int margin_ = 10;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_TIMELINE_WIDGET_HPP
