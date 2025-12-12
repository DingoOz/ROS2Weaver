#ifndef ROS_WEAVER_TOPIC_INSPECTOR_HPP
#define ROS_WEAVER_TOPIC_INSPECTOR_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QFrame>
#include <QGraphicsDropShadowEffect>

#include "ros_weaver/core/topic_monitor.hpp"

namespace ros_weaver {

class ConnectionLine;

/**
 * @brief Popup widget showing detailed topic information
 *
 * Displays real-time information about a topic including:
 * - Topic name and message type
 * - Publisher and subscriber counts
 * - Message rate and statistics
 * - Recent message preview
 */
class TopicInspectorPopup : public QFrame {
  Q_OBJECT

public:
  explicit TopicInspectorPopup(QWidget* parent = nullptr);
  ~TopicInspectorPopup() override;

  /**
   * @brief Show popup for a specific connection
   */
  void showForConnection(ConnectionLine* connection, const QPoint& globalPos);

  /**
   * @brief Update with topic stats from monitor
   */
  void updateStats(const TopicStats& stats);

  /**
   * @brief Set the topic monitor for live updates
   */
  void setTopicMonitor(TopicMonitor* monitor);

signals:
  void echoTopicRequested(const QString& topicName);
  void monitorTopicRequested(const QString& topicName);
  void showOnCanvasRequested(const QString& topicName);

protected:
  void paintEvent(QPaintEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;
  bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
  void onEchoClicked();
  void onMonitorClicked();
  void onShowOnCanvasClicked();
  void onCloseClicked();
  void onTopicStatsUpdated(const QString& topicName, const TopicStats& stats);

private:
  void setupUi();
  void updateDisplay();

  // UI Elements
  QLabel* titleLabel_;
  QLabel* topicNameLabel_;
  QLabel* messageTypeLabel_;
  QLabel* rateLabel_;
  QLabel* statsLabel_;
  QLabel* publishersLabel_;
  QLabel* subscribersLabel_;
  QTextEdit* messagePreview_;

  QPushButton* echoButton_;
  QPushButton* monitorButton_;
  QPushButton* showOnCanvasButton_;
  QPushButton* closeButton_;

  // Data
  ConnectionLine* currentConnection_;
  QString topicName_;
  QString messageType_;
  TopicMonitor* topicMonitor_;

  // Update timer for live stats
  QTimer* updateTimer_;
};

/**
 * @brief Compact inline tooltip showing basic topic info
 *
 * Shows minimal information on hover without blocking interaction.
 */
class TopicTooltip : public QLabel {
  Q_OBJECT

public:
  explicit TopicTooltip(QWidget* parent = nullptr);

  /**
   * @brief Show tooltip near a position
   */
  void showAt(const QPoint& globalPos, const QString& topicName,
              const QString& messageType, double rate);

  /**
   * @brief Hide the tooltip
   */
  void hideTooltip();

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  QTimer* hideTimer_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_TOPIC_INSPECTOR_HPP
