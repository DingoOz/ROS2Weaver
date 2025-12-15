#ifndef ROS_WEAVER_WIDGETS_BOOKMARK_MANAGER_HPP
#define ROS_WEAVER_WIDGETS_BOOKMARK_MANAGER_HPP

#include <QWidget>
#include <QListWidget>
#include <QPushButton>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>

#include "ros_weaver/widgets/timeline_widget.hpp"

namespace ros_weaver {

class PlaybackController;

/**
 * @brief Widget for managing timeline bookmarks
 *
 * Provides UI for adding, removing, and navigating to bookmarks
 * in the rosbag timeline.
 */
class BookmarkManager : public QWidget {
  Q_OBJECT

public:
  explicit BookmarkManager(QWidget* parent = nullptr);
  ~BookmarkManager() override = default;

  // Controller connections
  void setTimelineWidget(TimelineWidget* timeline);
  void setPlaybackController(PlaybackController* controller);

  // Bookmark management
  void addBookmark(const QString& label = QString());
  void addBookmarkAtTime(const rclcpp::Time& time, const QString& label);
  void removeSelectedBookmark();
  void clearAllBookmarks();

  // Persistence
  void saveBookmarks(const QString& filePath);
  void loadBookmarks(const QString& filePath);

  // Access
  QList<TimelineBookmark> bookmarks() const;

signals:
  void bookmarkSelected(qint64 timeNs);  // Time in nanoseconds
  void bookmarkAdded(const TimelineBookmark& bookmark);
  void bookmarkRemoved(int index);

public slots:
  void onCurrentTimeChanged(qint64 timeNs);

private slots:
  void onAddClicked();
  void onRemoveClicked();
  void onClearClicked();
  void onBookmarkDoubleClicked(QListWidgetItem* item);
  void onBookmarkSelectionChanged();

private:
  void setupUi();
  void setupConnections();
  void updateBookmarkList();
  QString formatTime(const rclcpp::Time& time) const;

  TimelineWidget* timeline_ = nullptr;
  PlaybackController* controller_ = nullptr;

  QListWidget* bookmarkList_ = nullptr;
  QPushButton* addButton_ = nullptr;
  QPushButton* removeButton_ = nullptr;
  QPushButton* clearButton_ = nullptr;
  QLabel* infoLabel_ = nullptr;

  rclcpp::Time currentTime_;
  rclcpp::Time bagStartTime_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_BOOKMARK_MANAGER_HPP
