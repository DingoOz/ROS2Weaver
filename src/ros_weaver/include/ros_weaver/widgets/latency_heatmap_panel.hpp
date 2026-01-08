#ifndef ROS_WEAVER_WIDGETS_LATENCY_HEATMAP_PANEL_HPP
#define ROS_WEAVER_WIDGETS_LATENCY_HEATMAP_PANEL_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QTimer>

#include "ros_weaver/core/latency_tracker.hpp"

namespace ros_weaver {

class WeaverCanvas;
class ConnectionLine;

class LatencyHeatmapPanel : public QWidget {
  Q_OBJECT

public:
  explicit LatencyHeatmapPanel(QWidget* parent = nullptr);
  ~LatencyHeatmapPanel() override;

  void setCanvas(WeaverCanvas* canvas);
  void setLatencyTracker(LatencyTracker* tracker);
  void setRosNode(rclcpp::Node::SharedPtr node);

signals:
  void heatmapEnabledChanged(bool enabled);
  void connectionSelected(const QString& connectionId);

public slots:
  void setHeatmapEnabled(bool enabled);
  void refreshConnections();
  void updateLatencyDisplay();

private slots:
  void onEnableToggled(bool enabled);
  void onConnectionClicked(int row, int column);
  void onThresholdChanged();
  void onAutoTrackToggled(bool enabled);

  void onLatencyUpdated(const QString& connectionId, double latencyMs);
  void onLatencyAlert(const QString& connectionId, double latencyMs, double thresholdMs);
  void onTrackingStarted(const QString& connectionId);
  void onTrackingStopped(const QString& connectionId);

private:
  void setupUi();
  void setupThresholdControls();
  void setupConnectionTable();
  void setupStatsDisplay();

  void updateConnectionRow(int row, const QString& connectionId);
  void addConnectionRow(const QString& connectionId, ConnectionLine* connection);
  int findConnectionRow(const QString& connectionId);

  QString formatLatency(double latencyMs) const;
  QColor getLatencyColor(double latencyMs) const;

  // UI Components
  QCheckBox* enableCheck_;
  QCheckBox* autoTrackCheck_;

  // Threshold controls
  QDoubleSpinBox* goodThresholdSpin_;
  QDoubleSpinBox* warningThresholdSpin_;
  QDoubleSpinBox* criticalThresholdSpin_;
  QDoubleSpinBox* alertThresholdSpin_;

  // Connection table
  QTableWidget* connectionTable_;

  // Stats display
  QLabel* totalConnectionsLabel_;
  QLabel* activeConnectionsLabel_;
  QLabel* alertCountLabel_;
  QLabel* avgLatencyLabel_;

  // Legend
  QWidget* legendWidget_;

  // Update timer
  QTimer* updateTimer_;

  // Core
  WeaverCanvas* canvas_;
  LatencyTracker* tracker_;
  rclcpp::Node::SharedPtr node_;

  bool heatmapEnabled_;
  int alertCount_;

  // Threshold values
  double goodThresholdMs_;
  double warningThresholdMs_;
  double criticalThresholdMs_;

  static constexpr int UPDATE_INTERVAL_MS = 500;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_LATENCY_HEATMAP_PANEL_HPP
