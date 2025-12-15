#ifndef ROS_WEAVER_WIDGETS_TOPIC_SELECTOR_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_TOPIC_SELECTOR_WIDGET_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>

#include "ros_weaver/core/bag_manager.hpp"

namespace ros_weaver {

/**
 * @brief Widget for selecting and configuring bag topics for playback
 *
 * Displays topics from the loaded bag with enable/disable checkboxes,
 * remapping options, and QoS configuration.
 */
class TopicSelectorWidget : public QWidget {
  Q_OBJECT

public:
  explicit TopicSelectorWidget(QWidget* parent = nullptr);
  ~TopicSelectorWidget() override = default;

  // Bag manager connection
  void setBagManager(BagManager* manager);

  // Topic operations
  void selectAllTopics();
  void deselectAllTopics();
  void invertSelection();
  QStringList selectedTopics() const;

  // Filter
  void setFilter(const QString& filter);

signals:
  void topicSelectionChanged(const QString& topic, bool selected);
  void topicRemapChanged(const QString& originalTopic, const QString& newTopic);
  void topicQosChanged(const QString& topic, int historyDepth, bool reliable);
  void plotFieldRequested(const QString& topic, const QString& field);

public slots:
  void onBagOpened(const BagMetadata& metadata);
  void onBagClosed();

private slots:
  void onFilterChanged(const QString& text);
  void onSelectAllClicked();
  void onDeselectAllClicked();
  void onInvertClicked();
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onContextMenuRequested(const QPoint& pos);

private:
  void setupUi();
  void setupConnections();
  void populateTopicTree();
  void applyFilter();
  QTreeWidgetItem* createTopicItem(const TopicConfig& config);
  QString formatMessageCount(int64_t count) const;

  BagManager* bagManager_ = nullptr;

  // UI elements
  QLineEdit* filterEdit_ = nullptr;
  QTreeWidget* topicTree_ = nullptr;
  QPushButton* selectAllButton_ = nullptr;
  QPushButton* deselectAllButton_ = nullptr;
  QPushButton* invertButton_ = nullptr;
  QLabel* summaryLabel_ = nullptr;

  // Tree columns
  enum Column {
    ColEnabled = 0,
    ColTopic,
    ColType,
    ColMessageCount,
    ColRemap
  };
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_TOPIC_SELECTOR_WIDGET_HPP
