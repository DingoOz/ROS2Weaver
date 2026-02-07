#include "ros_weaver/widgets/topic_list_model.hpp"
#include "ros_weaver/widgets/topic_viewer_panel.hpp"

#include <QColor>
#include <QFont>
#include <QtGlobal>

namespace ros_weaver {

TopicListModel::TopicListModel(QObject* parent)
  : QAbstractTableModel(parent)
{
}

int TopicListModel::rowCount(const QModelIndex& parent) const {
  if (parent.isValid()) return 0;
  return topics_.size();
}

int TopicListModel::columnCount(const QModelIndex& parent) const {
  if (parent.isValid()) return 0;
  return static_cast<int>(TopicColumn::ColumnCount);
}

QVariant TopicListModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid() || index.row() >= topics_.size()) {
    return QVariant();
  }

  const TopicDisplayInfo& topic = topics_.at(index.row());
  TopicColumn col = static_cast<TopicColumn>(index.column());

  if (role == Qt::DisplayRole) {
    switch (col) {
      case TopicColumn::Name:
        return topic.name;
      case TopicColumn::Type:
        return topic.shortType;
      case TopicColumn::Rate:
        if (topic.publishRate > 0) {
          return QString("%1 Hz").arg(topic.publishRate, 0, 'f', 1);
        }
        return "-";
      case TopicColumn::Publishers:
        return topic.publisherCount;
      case TopicColumn::Subscribers:
        return topic.subscriberCount;
      default:
        return QVariant();
    }
  }

  if (role == Qt::ToolTipRole) {
    switch (col) {
      case TopicColumn::Name:
        return topic.name;
      case TopicColumn::Type:
        return topic.type;  // Full type in tooltip
      case TopicColumn::Rate:
        if (topic.publishRate > 0) {
          return tr("Publishing at %1 Hz").arg(topic.publishRate, 0, 'f', 2);
        }
        return tr("No messages detected");
      default:
        return QVariant();
    }
  }

  if (role == Qt::ForegroundRole) {
    // Dim inactive topics
    if (topic.publishRate == 0 && topic.publisherCount == 0) {
      return QColor(128, 128, 128);
    }
    // Highlight monitored topics
    if (monitoredTopics_.contains(topic.name)) {
      return QColor(0, 180, 0);
    }
  }

  if (role == Qt::FontRole) {
    if (monitoredTopics_.contains(topic.name)) {
      QFont font;
      font.setBold(true);
      return font;
    }
  }

  if (role == Qt::BackgroundRole) {
    // Subtle highlight for active topics
    if (topic.publishRate > 0) {
      return QColor(40, 60, 40);
    }
  }

  if (role == Qt::DecorationRole && col == TopicColumn::Name) {
    if (monitoredTopics_.contains(topic.name)) {
      return QColor(0, 200, 0);  // Green dot for monitored
    }
    if (topic.publishRate > 0) {
      return QColor(100, 180, 100);  // Light green for active
    }
    return QColor(80, 80, 80);  // Gray for inactive
  }

  // Custom role for sorting by rate (numeric)
  if (role == Qt::UserRole && col == TopicColumn::Rate) {
    return topic.publishRate;
  }

  return QVariant();
}

QVariant TopicListModel::headerData(int section, Qt::Orientation orientation, int role) const {
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
    return QVariant();
  }

  TopicColumn col = static_cast<TopicColumn>(section);
  switch (col) {
    case TopicColumn::Name:
      return tr("Topic");
    case TopicColumn::Type:
      return tr("Type");
    case TopicColumn::Rate:
      return tr("Rate");
    case TopicColumn::Publishers:
      return tr("Pubs");
    case TopicColumn::Subscribers:
      return tr("Subs");
    default:
      return QVariant();
  }
}

Qt::ItemFlags TopicListModel::flags(const QModelIndex& index) const {
  if (!index.isValid()) {
    return Qt::NoItemFlags;
  }
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

void TopicListModel::setTopics(const QList<TopicDisplayInfo>& topics) {
  beginResetModel();
  topics_ = topics;

  // Rebuild index
  topicIndex_.clear();
  for (int i = 0; i < topics_.size(); ++i) {
    topicIndex_[topics_[i].name] = i;
  }

  endResetModel();
}

void TopicListModel::updateTopic(const TopicDisplayInfo& topic) {
  int row = findTopicRow(topic.name);
  if (row >= 0) {
    topics_[row] = topic;
    emit dataChanged(index(row, 0), index(row, columnCount() - 1));
    emit topicUpdated(topic.name);
  } else {
    // Add new topic
    beginInsertRows(QModelIndex(), topics_.size(), topics_.size());
    topicIndex_[topic.name] = topics_.size();
    topics_.append(topic);
    endInsertRows();
  }
}

void TopicListModel::clear() {
  beginResetModel();
  topics_.clear();
  topicIndex_.clear();
  endResetModel();
}

TopicDisplayInfo TopicListModel::getTopicAt(int row) const {
  if (row >= 0 && row < topics_.size()) {
    return topics_.at(row);
  }
  return TopicDisplayInfo();
}

TopicDisplayInfo TopicListModel::getTopicByName(const QString& name) const {
  int row = findTopicRow(name);
  if (row >= 0) {
    return topics_.at(row);
  }
  return TopicDisplayInfo();
}

int TopicListModel::findTopicRow(const QString& name) const {
  auto it = topicIndex_.find(name);
  if (it != topicIndex_.end()) {
    return it.value();
  }
  return -1;
}

void TopicListModel::setTopicMonitored(const QString& topicName, bool monitored) {
  if (monitored) {
    monitoredTopics_.insert(topicName);
  } else {
    monitoredTopics_.remove(topicName);
  }

  int row = findTopicRow(topicName);
  if (row >= 0) {
    emit dataChanged(index(row, 0), index(row, columnCount() - 1));
  }
}

bool TopicListModel::isTopicMonitored(const QString& topicName) const {
  return monitoredTopics_.contains(topicName);
}

void TopicListModel::updateTopicRate(const QString& topicName, double rate) {
  int row = findTopicRow(topicName);
  if (row >= 0) {
    double oldRate = topics_[row].publishRate;
    topics_[row].publishRate = rate;

    // Emit rate changed for rolling digit animation
    if (!qFuzzyCompare(1.0 + oldRate, 1.0 + rate)) {
      emit rateChanged(row, oldRate, rate);
    }

    emit dataChanged(index(row, static_cast<int>(TopicColumn::Rate)),
                     index(row, static_cast<int>(TopicColumn::Rate)));
  }
}

void TopicListModel::updateTopicMessage(const QString& topicName, const QString& preview, qint64 timestamp) {
  int row = findTopicRow(topicName);
  if (row >= 0) {
    topics_[row].lastMessagePreview = preview;
    topics_[row].lastMessageTime = timestamp;
    emit topicUpdated(topicName);
  }
}

}  // namespace ros_weaver
