#include "ros_weaver/widgets/bookmark_manager.hpp"
#include "ros_weaver/core/playback_controller.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

namespace ros_weaver {

BookmarkManager::BookmarkManager(QWidget* parent)
    : QWidget(parent)
    , currentTime_(0, 0)
    , bagStartTime_(0, 0) {
  setupUi();
  setupConnections();
}

void BookmarkManager::setTimelineWidget(TimelineWidget* timeline) {
  timeline_ = timeline;

  if (timeline_) {
    connect(timeline_, &TimelineWidget::bookmarkClicked, this, [this](int index) {
      if (index >= 0 && index < bookmarkList_->count()) {
        bookmarkList_->setCurrentRow(index);
      }
    });
  }
}

void BookmarkManager::setPlaybackController(PlaybackController* controller) {
  if (controller_) {
    disconnect(controller_, nullptr, this, nullptr);
  }

  controller_ = controller;

  if (controller_) {
    connect(controller_, &PlaybackController::timeChanged,
            this, &BookmarkManager::onCurrentTimeChanged);
  }
}

void BookmarkManager::addBookmark(const QString& label) {
  addBookmarkAtTime(currentTime_, label);
}

void BookmarkManager::addBookmarkAtTime(const rclcpp::Time& time, const QString& label) {
  QString bookmarkLabel = label;
  if (bookmarkLabel.isEmpty()) {
    bookmarkLabel = QString("Bookmark %1").arg(timeline_ ? timeline_->bookmarks().size() + 1 : 1);
  }

  TimelineBookmark bookmark;
  bookmark.time = time;
  bookmark.label = bookmarkLabel;
  bookmark.color = QColor(255, 200, 0);

  if (timeline_) {
    timeline_->addBookmark(time, bookmarkLabel, bookmark.color);
  }

  updateBookmarkList();
  emit bookmarkAdded(bookmark);
}

void BookmarkManager::removeSelectedBookmark() {
  int row = bookmarkList_->currentRow();
  if (row >= 0 && timeline_) {
    timeline_->removeBookmark(row);
    updateBookmarkList();
    emit bookmarkRemoved(row);
  }
}

void BookmarkManager::clearAllBookmarks() {
  if (timeline_) {
    timeline_->clearBookmarks();
  }
  bookmarkList_->clear();
}

void BookmarkManager::saveBookmarks(const QString& filePath) {
  if (!timeline_) return;

  QJsonArray array;
  for (const auto& bookmark : timeline_->bookmarks()) {
    QJsonObject obj;
    obj["time_ns"] = static_cast<qint64>(bookmark.time.nanoseconds());
    obj["label"] = bookmark.label;
    obj["color"] = bookmark.color.name();
    array.append(obj);
  }

  QJsonDocument doc(array);
  QFile file(filePath);
  if (file.open(QIODevice::WriteOnly)) {
    file.write(doc.toJson());
  }
}

void BookmarkManager::loadBookmarks(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) return;

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  QJsonArray array = doc.array();

  clearAllBookmarks();

  for (const QJsonValue& val : array) {
    QJsonObject obj = val.toObject();
    rclcpp::Time time(obj["time_ns"].toVariant().toLongLong());
    QString label = obj["label"].toString();
    QColor color(obj["color"].toString());

    if (timeline_) {
      timeline_->addBookmark(time, label, color);
    }
  }

  updateBookmarkList();
}

QList<TimelineBookmark> BookmarkManager::bookmarks() const {
  if (timeline_) {
    return timeline_->bookmarks();
  }
  return {};
}

void BookmarkManager::onCurrentTimeChanged(qint64 timeNs) {
  currentTime_ = rclcpp::Time(timeNs);
}

void BookmarkManager::onAddClicked() {
  bool ok;
  QString label = QInputDialog::getText(this, tr("Add Bookmark"),
                                        tr("Bookmark label:"),
                                        QLineEdit::Normal,
                                        QString("Bookmark at %1").arg(formatTime(currentTime_)),
                                        &ok);
  if (ok) {
    addBookmark(label);
  }
}

void BookmarkManager::onRemoveClicked() {
  removeSelectedBookmark();
}

void BookmarkManager::onClearClicked() {
  clearAllBookmarks();
}

void BookmarkManager::onBookmarkDoubleClicked(QListWidgetItem* item) {
  int row = bookmarkList_->row(item);
  if (row >= 0 && timeline_) {
    QList<TimelineBookmark> bm = timeline_->bookmarks();
    if (row < bm.size()) {
      emit bookmarkSelected(bm[row].time.nanoseconds());
    }
  }
}

void BookmarkManager::onBookmarkSelectionChanged() {
  removeButton_->setEnabled(bookmarkList_->currentRow() >= 0);
}

void BookmarkManager::setupUi() {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(5);

  // Toolbar
  auto* toolbar = new QHBoxLayout();

  addButton_ = new QPushButton(tr("+ Add"), this);
  addButton_->setToolTip(tr("Add bookmark at current time"));
  toolbar->addWidget(addButton_);

  removeButton_ = new QPushButton(tr("- Remove"), this);
  removeButton_->setToolTip(tr("Remove selected bookmark"));
  removeButton_->setEnabled(false);
  toolbar->addWidget(removeButton_);

  clearButton_ = new QPushButton(tr("Clear All"), this);
  clearButton_->setToolTip(tr("Remove all bookmarks"));
  toolbar->addWidget(clearButton_);

  toolbar->addStretch();
  layout->addLayout(toolbar);

  // Bookmark list
  bookmarkList_ = new QListWidget(this);
  bookmarkList_->setAlternatingRowColors(true);
  layout->addWidget(bookmarkList_, 1);

  // Info label
  infoLabel_ = new QLabel(tr("Double-click to seek"), this);
  infoLabel_->setStyleSheet("color: gray; font-size: 10px;");
  layout->addWidget(infoLabel_);
}

void BookmarkManager::setupConnections() {
  connect(addButton_, &QPushButton::clicked, this, &BookmarkManager::onAddClicked);
  connect(removeButton_, &QPushButton::clicked, this, &BookmarkManager::onRemoveClicked);
  connect(clearButton_, &QPushButton::clicked, this, &BookmarkManager::onClearClicked);
  connect(bookmarkList_, &QListWidget::itemDoubleClicked,
          this, &BookmarkManager::onBookmarkDoubleClicked);
  connect(bookmarkList_, &QListWidget::itemSelectionChanged,
          this, &BookmarkManager::onBookmarkSelectionChanged);
}

void BookmarkManager::updateBookmarkList() {
  bookmarkList_->clear();

  if (!timeline_) return;

  for (const auto& bookmark : timeline_->bookmarks()) {
    QString text = QString("%1 - %2").arg(formatTime(bookmark.time)).arg(bookmark.label);
    auto* item = new QListWidgetItem(text, bookmarkList_);
    item->setForeground(bookmark.color);
  }

  clearButton_->setEnabled(bookmarkList_->count() > 0);
}

QString BookmarkManager::formatTime(const rclcpp::Time& time) const {
  int64_t relNs = time.nanoseconds() - bagStartTime_.nanoseconds();
  int64_t totalMs = relNs / 1000000;
  int minutes = static_cast<int>(totalMs / 60000);
  int seconds = static_cast<int>((totalMs % 60000) / 1000);
  int ms = static_cast<int>(totalMs % 1000);

  return QString("%1:%2.%3")
      .arg(minutes, 2, 10, QChar('0'))
      .arg(seconds, 2, 10, QChar('0'))
      .arg(ms, 3, 10, QChar('0'));
}

}  // namespace ros_weaver
