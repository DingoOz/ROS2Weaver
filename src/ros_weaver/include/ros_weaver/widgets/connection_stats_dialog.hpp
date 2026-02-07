#ifndef ROS_WEAVER_WIDGETS_CONNECTION_STATS_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_CONNECTION_STATS_DIALOG_HPP

#include <QDialog>
#include <QLabel>
#include <QPointer>
#include <QProgressBar>
#include <QTimer>

namespace ros_weaver {

class ConnectionLine;
class RollingDigitLabel;

/**
 * @brief Dialog showing detailed statistics for a connection
 *
 * Displays live connection statistics including:
 * - Message rate (Hz)
 * - Bandwidth (B/s, KB/s, MB/s)
 * - Latency and jitter
 * - Queue depth and dropped messages
 * - Total messages and bytes
 * - Connection health status
 */
class ConnectionStatsDialog : public QDialog {
  Q_OBJECT

public:
  explicit ConnectionStatsDialog(ConnectionLine* connection, QWidget* parent = nullptr);
  ~ConnectionStatsDialog() override;

public slots:
  void updateStats();

private:
  void setupUI();
  void updateHealthIndicator();
  QString formatBytes(qint64 bytes) const;
  QString formatDuration(qint64 milliseconds) const;

  QPointer<ConnectionLine> connection_;
  QTimer* updateTimer_;

  // UI elements
  QLabel* topicNameLabel_;
  QLabel* messageTypeLabel_;
  QLabel* healthStatusLabel_;
  RollingDigitLabel* messageRateLabel_;
  RollingDigitLabel* bandwidthLabel_;
  QLabel* latencyLabel_;
  QLabel* jitterLabel_;
  QLabel* queueDepthLabel_;
  QLabel* droppedCountLabel_;
  QLabel* totalMessagesLabel_;
  QLabel* totalBytesLabel_;
  QLabel* lastMessageLabel_;
  QLabel* expectedRateLabel_;
  QProgressBar* healthBar_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_CONNECTION_STATS_DIALOG_HPP
