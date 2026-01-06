#ifndef ROS_WEAVER_WIDGETS_NODE_HEALTH_DASHBOARD_HPP
#define ROS_WEAVER_WIDGETS_NODE_HEALTH_DASHBOARD_HPP

#include <QWidget>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QProgressBar>
#include <QTimer>

#include "ros_weaver/core/node_health_monitor.hpp"

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Dashboard panel for monitoring node health
 *
 * Features:
 * - Real-time table of node health metrics
 * - Heatmap visualization toggle (colors canvas nodes by resource usage)
 * - Alert thresholds configuration
 * - Historical graphs access
 * - Overall system health summary
 */
class NodeHealthDashboard : public QWidget {
  Q_OBJECT

public:
  explicit NodeHealthDashboard(QWidget* parent = nullptr);
  ~NodeHealthDashboard() override = default;

  /**
   * @brief Set the canvas for heatmap visualization
   */
  void setCanvas(WeaverCanvas* canvas);
  WeaverCanvas* canvas() const { return canvas_; }

  /**
   * @brief Get the health monitor
   */
  NodeHealthMonitor* healthMonitor() const { return healthMonitor_; }

  /**
   * @brief Sync nodes from canvas
   */
  void syncNodesFromCanvas();

public slots:
  /**
   * @brief Start monitoring
   */
  void startMonitoring();

  /**
   * @brief Stop monitoring
   */
  void stopMonitoring();

  /**
   * @brief Toggle heatmap visualization on canvas
   */
  void setHeatmapEnabled(bool enabled);

signals:
  /**
   * @brief Emitted when user wants to view history for a node
   */
  void showHistoryRequested(const QString& nodeName);

  /**
   * @brief Emitted when user double-clicks a node row
   */
  void nodeSelected(const QString& nodeName);

private slots:
  void onHealthUpdated(const QString& nodeName, const NodeHealthData& health);
  void onWarningTriggered(const QString& nodeName, const QString& reason);
  void onCriticalTriggered(const QString& nodeName, const QString& reason);
  void onHealthRecovered(const QString& nodeName);
  void onMonitoringStateChanged(bool isMonitoring);

  void onStartStopClicked();
  void onHeatmapToggled(bool checked);
  void onHeatmapModeChanged(int index);
  void onTableDoubleClicked(int row, int column);
  void onRefreshClicked();
  void onUpdateIntervalChanged(int value);

  void updateSummary();

private:
  void setupUi();
  void setupConnections();
  void updateNodeRow(const QString& nodeName, const NodeHealthData& health);
  int findNodeRow(const QString& nodeName);
  void addNodeRow(const QString& nodeName, const NodeHealthData& health);
  void updateHeatmapVisualization();
  QColor getHeatmapColor(const NodeHealthData& health);

  WeaverCanvas* canvas_ = nullptr;
  NodeHealthMonitor* healthMonitor_ = nullptr;

  // UI - Controls
  QPushButton* startStopButton_ = nullptr;
  QPushButton* refreshButton_ = nullptr;
  QCheckBox* heatmapCheck_ = nullptr;
  QComboBox* heatmapModeCombo_ = nullptr;
  QSpinBox* intervalSpin_ = nullptr;

  // UI - Summary
  QLabel* summaryLabel_ = nullptr;
  QProgressBar* overallHealthBar_ = nullptr;
  QLabel* healthyCountLabel_ = nullptr;
  QLabel* warningCountLabel_ = nullptr;
  QLabel* criticalCountLabel_ = nullptr;

  // UI - Table
  QTableWidget* healthTable_ = nullptr;

  // Heatmap
  bool heatmapEnabled_ = false;
  int heatmapMode_ = 0;  // 0 = CPU, 1 = Memory, 2 = Status

  // Column indices
  static constexpr int COL_STATUS = 0;
  static constexpr int COL_NAME = 1;
  static constexpr int COL_CPU = 2;
  static constexpr int COL_MEMORY = 3;
  static constexpr int COL_LATENCY = 4;
  static constexpr int COL_DROPPED = 5;
  static constexpr int COL_RATE = 6;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_NODE_HEALTH_DASHBOARD_HPP
