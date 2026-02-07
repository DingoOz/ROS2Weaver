#ifndef ROS_WEAVER_TOPIC_LIST_MODEL_HPP
#define ROS_WEAVER_TOPIC_LIST_MODEL_HPP

#include <QAbstractTableModel>
#include <QList>
#include <QMap>
#include <QSet>
#include <QIcon>

namespace ros_weaver {

struct TopicDisplayInfo;

// Column indices for the topic list
enum class TopicColumn {
  Name = 0,
  Type,
  Rate,
  Publishers,
  Subscribers,
  ColumnCount
};

class TopicListModel : public QAbstractTableModel {
  Q_OBJECT

public:
  explicit TopicListModel(QObject* parent = nullptr);
  ~TopicListModel() override = default;

  // QAbstractTableModel interface
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
  Qt::ItemFlags flags(const QModelIndex& index) const override;

  // Data management
  void setTopics(const QList<TopicDisplayInfo>& topics);
  void updateTopic(const TopicDisplayInfo& topic);
  void clear();

  // Get topic info
  TopicDisplayInfo getTopicAt(int row) const;
  TopicDisplayInfo getTopicByName(const QString& name) const;
  int findTopicRow(const QString& name) const;

  // Monitoring state
  void setTopicMonitored(const QString& topicName, bool monitored);
  bool isTopicMonitored(const QString& topicName) const;

  // Update rate for a topic
  void updateTopicRate(const QString& topicName, double rate);
  void updateTopicMessage(const QString& topicName, const QString& preview, qint64 timestamp);

signals:
  void topicUpdated(const QString& topicName);
  void rateChanged(int row, double oldRate, double newRate);

private:
  QList<TopicDisplayInfo> topics_;
  QMap<QString, int> topicIndex_;  // name -> row mapping for fast lookup
  QSet<QString> monitoredTopics_;

  // Icons
  QIcon activeIcon_;
  QIcon inactiveIcon_;
  QIcon monitoredIcon_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_TOPIC_LIST_MODEL_HPP
