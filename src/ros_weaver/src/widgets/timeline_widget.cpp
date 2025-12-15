#include "ros_weaver/widgets/timeline_widget.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QToolTip>
#include <cmath>

namespace ros_weaver {

TimelineWidget::TimelineWidget(QWidget* parent)
    : QWidget(parent)
    , startTime_(0, 0)
    , endTime_(0, 0)
    , currentTime_(0, 0)
    , visibleStart_(0, 0)
    , visibleEnd_(0, 0) {
  setMinimumHeight(80);
  setMouseTracking(true);
  setCursor(Qt::ArrowCursor);
}

void TimelineWidget::setTimeRange(const rclcpp::Time& start, const rclcpp::Time& end) {
  startTime_ = start;
  endTime_ = end;
  visibleStart_ = start;
  visibleEnd_ = end;
  currentTime_ = start;
  update();
}

void TimelineWidget::setCurrentTime(const rclcpp::Time& time) {
  currentTime_ = time;
  update();
}

rclcpp::Time TimelineWidget::startTime() const {
  return startTime_;
}

rclcpp::Time TimelineWidget::endTime() const {
  return endTime_;
}

rclcpp::Time TimelineWidget::currentTime() const {
  return currentTime_;
}

void TimelineWidget::setProgress(double percent) {
  if (endTime_.nanoseconds() <= startTime_.nanoseconds()) return;

  int64_t totalNs = endTime_.nanoseconds() - startTime_.nanoseconds();
  int64_t currentNs = startTime_.nanoseconds() + static_cast<int64_t>(totalNs * percent / 100.0);
  currentTime_ = rclcpp::Time(currentNs);
  update();
}

double TimelineWidget::progress() const {
  if (endTime_.nanoseconds() <= startTime_.nanoseconds()) return 0.0;

  int64_t totalNs = endTime_.nanoseconds() - startTime_.nanoseconds();
  int64_t elapsedNs = currentTime_.nanoseconds() - startTime_.nanoseconds();
  return static_cast<double>(elapsedNs) / totalNs * 100.0;
}

void TimelineWidget::addBookmark(const rclcpp::Time& time, const QString& label,
                                  const QColor& color) {
  TimelineBookmark bookmark;
  bookmark.time = time;
  bookmark.label = label;
  bookmark.color = color;
  bookmarks_.append(bookmark);
  update();
}

void TimelineWidget::removeBookmark(int index) {
  if (index >= 0 && index < bookmarks_.size()) {
    bookmarks_.removeAt(index);
    update();
  }
}

void TimelineWidget::clearBookmarks() {
  bookmarks_.clear();
  update();
}

QList<TimelineBookmark> TimelineWidget::bookmarks() const {
  return bookmarks_;
}

void TimelineWidget::setShowTopicActivity(bool show) {
  showTopicActivity_ = show;
  update();
}

bool TimelineWidget::showTopicActivity() const {
  return showTopicActivity_;
}

void TimelineWidget::setTopicActivityData(const QString& topic, const QVector<qint64>& timestamps) {
  topicActivity_[topic] = timestamps;
  update();
}

void TimelineWidget::clearTopicActivity() {
  topicActivity_.clear();
  update();
}

void TimelineWidget::setZoomLevel(double level) {
  zoomLevel_ = qBound(0.1, level, 10.0);

  // Adjust visible range based on zoom
  int64_t totalNs = endTime_.nanoseconds() - startTime_.nanoseconds();
  int64_t visibleNs = static_cast<int64_t>(totalNs / zoomLevel_);

  // Center on current time
  int64_t centerNs = currentTime_.nanoseconds();
  int64_t newStartNs = centerNs - visibleNs / 2;
  int64_t newEndNs = centerNs + visibleNs / 2;

  // Clamp to valid range
  if (newStartNs < startTime_.nanoseconds()) {
    newStartNs = startTime_.nanoseconds();
    newEndNs = newStartNs + visibleNs;
  }
  if (newEndNs > endTime_.nanoseconds()) {
    newEndNs = endTime_.nanoseconds();
    newStartNs = newEndNs - visibleNs;
    if (newStartNs < startTime_.nanoseconds()) {
      newStartNs = startTime_.nanoseconds();
    }
  }

  visibleStart_ = rclcpp::Time(newStartNs);
  visibleEnd_ = rclcpp::Time(newEndNs);

  update();
}

double TimelineWidget::zoomLevel() const {
  return zoomLevel_;
}

void TimelineWidget::resetView() {
  zoomLevel_ = 1.0;
  visibleStart_ = startTime_;
  visibleEnd_ = endTime_;
  update();
}

void TimelineWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  drawBackground(painter);
  drawTickMarks(painter);

  if (showTopicActivity_) {
    drawTopicActivity(painter);
  }

  drawBookmarks(painter);
  drawPlayhead(painter);
  drawTimeLabels(painter);
}

void TimelineWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    int bookmarkIndex = bookmarkAtPos(event->pos());
    if (bookmarkIndex >= 0) {
      emit bookmarkClicked(bookmarkIndex);
      emit seekRequested(bookmarks_[bookmarkIndex].time.nanoseconds());
    } else {
      isDragging_ = true;
      rclcpp::Time time = timeFromX(event->pos().x());
      emit seekRequested(time.nanoseconds());
    }
  } else if (event->button() == Qt::MiddleButton) {
    isPanning_ = true;
    lastMousePos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
}

void TimelineWidget::mouseMoveEvent(QMouseEvent* event) {
  if (isDragging_) {
    rclcpp::Time time = timeFromX(event->pos().x());
    emit seekRequested(time.nanoseconds());

    double percent = progress();
    emit progressDragged(percent);
  } else if (isPanning_) {
    int dx = event->pos().x() - lastMousePos_.x();
    lastMousePos_ = event->pos();

    // Pan the visible range
    int64_t visibleNs = visibleEnd_.nanoseconds() - visibleStart_.nanoseconds();
    int64_t panNs = static_cast<int64_t>(-dx * visibleNs / width());

    int64_t newStartNs = visibleStart_.nanoseconds() + panNs;
    int64_t newEndNs = visibleEnd_.nanoseconds() + panNs;

    // Clamp to valid range
    if (newStartNs < startTime_.nanoseconds()) {
      newStartNs = startTime_.nanoseconds();
      newEndNs = newStartNs + visibleNs;
    }
    if (newEndNs > endTime_.nanoseconds()) {
      newEndNs = endTime_.nanoseconds();
      newStartNs = newEndNs - visibleNs;
    }

    visibleStart_ = rclcpp::Time(newStartNs);
    visibleEnd_ = rclcpp::Time(newEndNs);
    update();
  } else {
    // Update hover state
    int bookmarkIndex = bookmarkAtPos(event->pos());
    if (bookmarkIndex >= 0) {
      setCursor(Qt::PointingHandCursor);
      if (bookmarkIndex != hoveredBookmark_) {
        hoveredBookmark_ = bookmarkIndex;
        QString tooltip = QString("%1\n%2")
            .arg(bookmarks_[bookmarkIndex].label)
            .arg(formatTime(bookmarks_[bookmarkIndex].time));
        QToolTip::showText(event->globalPos(), tooltip);
      }
    } else {
      setCursor(Qt::ArrowCursor);
      hoveredBookmark_ = -1;
    }
  }
}

void TimelineWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    isDragging_ = false;
  } else if (event->button() == Qt::MiddleButton) {
    isPanning_ = false;
    setCursor(Qt::ArrowCursor);
  }
}

void TimelineWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  int bookmarkIndex = bookmarkAtPos(event->pos());
  if (bookmarkIndex >= 0) {
    emit bookmarkDoubleClicked(bookmarkIndex);
  }
}

void TimelineWidget::wheelEvent(QWheelEvent* event) {
  // Zoom with scroll wheel
  double delta = event->angleDelta().y() / 120.0;
  double newZoom = zoomLevel_ * (1.0 + delta * 0.1);
  setZoomLevel(newZoom);
}

void TimelineWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  update();
}

void TimelineWidget::drawBackground(QPainter& painter) {
  // Background gradient
  QLinearGradient gradient(0, 0, 0, height());
  gradient.setColorAt(0, QColor(50, 50, 55));
  gradient.setColorAt(1, QColor(35, 35, 40));
  painter.fillRect(rect(), gradient);

  // Timeline track
  int trackY = height() / 2 - 3;
  int trackHeight = 6;
  painter.fillRect(margin_, trackY, width() - 2 * margin_, trackHeight, QColor(70, 70, 75));

  // Progress fill
  if (visibleEnd_.nanoseconds() > visibleStart_.nanoseconds()) {
    int currentX = xFromTime(currentTime_);
    int startX = xFromTime(visibleStart_);
    if (currentX > startX) {
      QLinearGradient progressGradient(startX, 0, currentX, 0);
      progressGradient.setColorAt(0, QColor(42, 130, 218));
      progressGradient.setColorAt(1, QColor(30, 100, 180));
      painter.fillRect(startX, trackY, currentX - startX, trackHeight, progressGradient);
    }
  }
}

void TimelineWidget::drawTickMarks(QPainter& painter) {
  if (visibleEnd_.nanoseconds() <= visibleStart_.nanoseconds()) return;

  int64_t visibleNs = visibleEnd_.nanoseconds() - visibleStart_.nanoseconds();

  // Determine tick interval based on visible range
  int64_t tickInterval;
  if (visibleNs > 60000000000LL) {
    tickInterval = 10000000000LL;  // 10 seconds
  } else if (visibleNs > 10000000000LL) {
    tickInterval = 1000000000LL;   // 1 second
  } else if (visibleNs > 1000000000LL) {
    tickInterval = 100000000LL;    // 100ms
  } else {
    tickInterval = 10000000LL;     // 10ms
  }

  painter.setPen(QColor(100, 100, 105));

  int64_t firstTick = (visibleStart_.nanoseconds() / tickInterval) * tickInterval;
  for (int64_t tickNs = firstTick; tickNs <= visibleEnd_.nanoseconds(); tickNs += tickInterval) {
    if (tickNs < visibleStart_.nanoseconds()) continue;

    int x = xFromTime(rclcpp::Time(tickNs));
    bool isMajor = (tickNs % (tickInterval * 5)) == 0;

    int tickH = isMajor ? 10 : 5;
    int trackY = height() / 2;
    painter.drawLine(x, trackY - tickH, x, trackY + tickH);
  }
}

void TimelineWidget::drawBookmarks(QPainter& painter) {
  int trackY = height() / 2;

  for (int i = 0; i < bookmarks_.size(); ++i) {
    const auto& bookmark = bookmarks_[i];
    if (bookmark.time.nanoseconds() < visibleStart_.nanoseconds() ||
        bookmark.time.nanoseconds() > visibleEnd_.nanoseconds()) {
      continue;
    }

    int x = xFromTime(bookmark.time);

    // Draw bookmark flag
    QPainterPath path;
    path.moveTo(x, trackY - bookmarkHeight_);
    path.lineTo(x + 8, trackY - bookmarkHeight_ + 5);
    path.lineTo(x + 8, trackY - 3);
    path.lineTo(x, trackY - 3);
    path.closeSubpath();

    QColor color = bookmark.color;
    if (i == hoveredBookmark_) {
      color = color.lighter(130);
    }
    painter.fillPath(path, color);
    painter.setPen(color.darker(120));
    painter.drawPath(path);
  }
}

void TimelineWidget::drawPlayhead(QPainter& painter) {
  if (currentTime_.nanoseconds() < visibleStart_.nanoseconds() ||
      currentTime_.nanoseconds() > visibleEnd_.nanoseconds()) {
    return;
  }

  int x = xFromTime(currentTime_);
  int trackY = height() / 2;

  // Vertical line
  painter.setPen(QPen(QColor(255, 100, 100), 2));
  painter.drawLine(x, 5, x, height() - 5);

  // Triangle marker at top
  QPainterPath path;
  path.moveTo(x - 6, 3);
  path.lineTo(x + 6, 3);
  path.lineTo(x, 12);
  path.closeSubpath();
  painter.fillPath(path, QColor(255, 100, 100));
}

void TimelineWidget::drawTopicActivity(QPainter& painter) {
  if (topicActivity_.isEmpty()) return;

  int trackY = height() / 2;
  int activityY = trackY + 15;
  int rowHeight = 4;
  int row = 0;

  for (auto it = topicActivity_.constBegin(); it != topicActivity_.constEnd(); ++it, ++row) {
    const QVector<qint64>& timestamps = it.value();

    // Generate activity color based on topic name hash
    uint hash = qHash(it.key());
    QColor color = QColor::fromHsl(hash % 360, 150, 100);

    for (qint64 ts : timestamps) {
      if (ts < visibleStart_.nanoseconds() || ts > visibleEnd_.nanoseconds()) continue;

      int x = xFromTime(rclcpp::Time(ts));
      painter.fillRect(x, activityY + row * rowHeight, 2, rowHeight - 1, color);
    }
  }
}

void TimelineWidget::drawTimeLabels(QPainter& painter) {
  painter.setPen(QColor(200, 200, 200));
  QFont font = painter.font();
  font.setPointSize(9);
  painter.setFont(font);

  // Current time
  QString currentStr = formatTime(currentTime_);
  painter.drawText(margin_, height() - 5, currentStr);

  // Duration
  QString durationStr = formatDuration(endTime_.nanoseconds() - startTime_.nanoseconds());
  QFontMetrics fm(font);
  int durWidth = fm.horizontalAdvance(durationStr);
  painter.drawText(width() - margin_ - durWidth, height() - 5, durationStr);
}

rclcpp::Time TimelineWidget::timeFromX(int x) const {
  if (visibleEnd_.nanoseconds() <= visibleStart_.nanoseconds()) {
    return visibleStart_;
  }

  int effectiveWidth = width() - 2 * margin_;
  int relX = x - margin_;
  double ratio = static_cast<double>(relX) / effectiveWidth;
  ratio = qBound(0.0, ratio, 1.0);

  int64_t visibleNs = visibleEnd_.nanoseconds() - visibleStart_.nanoseconds();
  int64_t timeNs = visibleStart_.nanoseconds() + static_cast<int64_t>(visibleNs * ratio);

  return rclcpp::Time(timeNs);
}

int TimelineWidget::xFromTime(const rclcpp::Time& time) const {
  if (visibleEnd_.nanoseconds() <= visibleStart_.nanoseconds()) {
    return margin_;
  }

  int64_t visibleNs = visibleEnd_.nanoseconds() - visibleStart_.nanoseconds();
  int64_t relNs = time.nanoseconds() - visibleStart_.nanoseconds();
  double ratio = static_cast<double>(relNs) / visibleNs;

  int effectiveWidth = width() - 2 * margin_;
  return margin_ + static_cast<int>(effectiveWidth * ratio);
}

int TimelineWidget::bookmarkAtPos(const QPoint& pos) const {
  int trackY = height() / 2;

  for (int i = 0; i < bookmarks_.size(); ++i) {
    const auto& bookmark = bookmarks_[i];
    if (bookmark.time.nanoseconds() < visibleStart_.nanoseconds() ||
        bookmark.time.nanoseconds() > visibleEnd_.nanoseconds()) {
      continue;
    }

    int x = xFromTime(bookmark.time);
    QRect hitRect(x - 2, trackY - bookmarkHeight_ - 2, 12, bookmarkHeight_ + 4);
    if (hitRect.contains(pos)) {
      return i;
    }
  }

  return -1;
}

QString TimelineWidget::formatTime(const rclcpp::Time& time) const {
  int64_t relNs = time.nanoseconds() - startTime_.nanoseconds();
  return formatDuration(relNs);
}

QString TimelineWidget::formatDuration(int64_t ns) const {
  int64_t totalMs = ns / 1000000;
  int minutes = static_cast<int>(totalMs / 60000);
  int seconds = static_cast<int>((totalMs % 60000) / 1000);
  int ms = static_cast<int>(totalMs % 1000);

  return QString("%1:%2.%3")
      .arg(minutes, 2, 10, QChar('0'))
      .arg(seconds, 2, 10, QChar('0'))
      .arg(ms, 3, 10, QChar('0'));
}

}  // namespace ros_weaver
