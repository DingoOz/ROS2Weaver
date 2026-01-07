#ifndef ROS_WEAVER_WIDGETS_LIFECYCLE_PANEL_HPP
#define ROS_WEAVER_WIDGETS_LIFECYCLE_PANEL_HPP

#include <QWidget>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QComboBox>
#include <QGroupBox>
#include <QTimer>

#include "ros_weaver/core/lifecycle_manager.hpp"

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Panel for visualizing and controlling ROS2 lifecycle nodes
 *
 * Features:
 * - Table view of all lifecycle nodes with their states
 * - Visual state machine diagram for selected node
 * - State transition buttons
 * - Real-time state monitoring
 * - Batch operations on multiple nodes
 */
class LifecyclePanel : public QWidget {
  Q_OBJECT

public:
  explicit LifecyclePanel(QWidget* parent = nullptr);
  ~LifecyclePanel() override;

  /**
   * @brief Set the canvas reference
   */
  void setCanvas(WeaverCanvas* canvas);

  /**
   * @brief Get the lifecycle manager
   */
  LifecycleManager* lifecycleManager() const { return lifecycleManager_; }

  /**
   * @brief Sync nodes from canvas
   */
  void syncNodesFromCanvas();

public slots:
  /**
   * @brief Start discovering lifecycle nodes
   */
  void discoverNodes();

  /**
   * @brief Refresh state of all nodes
   */
  void refreshStates();

signals:
  /**
   * @brief Emitted when user selects a node
   */
  void nodeSelected(const QString& nodeName);

  /**
   * @brief Emitted when user wants to highlight a node on canvas
   */
  void highlightNodeRequested(const QString& nodeName);

private slots:
  void onDiscoverClicked();
  void onRefreshClicked();
  void onTableSelectionChanged();
  void onTableDoubleClicked(int row, int column);

  void onConfigureClicked();
  void onActivateClicked();
  void onDeactivateClicked();
  void onCleanupClicked();
  void onShutdownClicked();

  void onStateChanged(const QString& nodeName, LifecycleState newState);
  void onTransitionFailed(const QString& nodeName, const QString& error);
  void onTransitionSucceeded(const QString& nodeName, LifecycleState newState);
  void onLifecycleNodesDiscovered(const QStringList& nodes);

  void updateStateMachineDisplay();

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  void setupUi();
  void setupConnections();
  void initializeRosNode();
  void shutdownRosNode();

  void updateNodeRow(const QString& nodeName, const LifecycleNodeInfo& info);
  int findNodeRow(const QString& nodeName);
  void addNodeRow(const QString& nodeName, const LifecycleNodeInfo& info);
  void updateButtonStates();

  QString getSelectedNodeName() const;
  QStringList getSelectedNodeNames() const;

  QColor getStateColor(LifecycleState state) const;
  QString getStateIcon(LifecycleState state) const;

  void drawStateMachine(QPainter* painter, const QRect& rect);

  // UI - Controls
  QPushButton* discoverButton_;
  QPushButton* refreshButton_;
  QLabel* statusLabel_;

  // UI - Table
  QTableWidget* nodeTable_;

  // UI - State machine visualization
  QGroupBox* stateMachineGroup_;
  QWidget* stateMachineWidget_;
  QLabel* selectedNodeLabel_;

  // UI - Transition buttons
  QGroupBox* controlsGroup_;
  QPushButton* configureButton_;
  QPushButton* activateButton_;
  QPushButton* deactivateButton_;
  QPushButton* cleanupButton_;
  QPushButton* shutdownButton_;

  // UI - Batch controls
  QGroupBox* batchGroup_;
  QPushButton* configureAllButton_;
  QPushButton* activateAllButton_;
  QPushButton* deactivateAllButton_;

  // Core components
  LifecycleManager* lifecycleManager_;
  WeaverCanvas* canvas_;
  rclcpp::Node::SharedPtr rosNode_;
  std::thread spinThread_;
  std::atomic<bool> spinning_;

  // Auto-refresh timer
  QTimer* refreshTimer_;

  // Currently selected node
  QString selectedNode_;

  // Column indices
  static constexpr int COL_STATUS = 0;
  static constexpr int COL_NAME = 1;
  static constexpr int COL_STATE = 2;
  static constexpr int COL_LAST_CHANGE = 3;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_LIFECYCLE_PANEL_HPP
