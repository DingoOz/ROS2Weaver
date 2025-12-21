// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#ifndef ROS_WEAVER_ROS_CONTROL_PANEL_HPP
#define ROS_WEAVER_ROS_CONTROL_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QToolButton>
#include <QLabel>
#include <QSplitter>
#include <QTimer>
#include <QGroupBox>
#include <QProcess>
#include <QVBoxLayout>
#include <QDateTime>
#include <QFuture>
#include <QFutureWatcher>
#include <memory>
#include <thread>
#include <atomic>

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Controller lifecycle states (from ros2_control)
 */
enum class ControllerState {
  Unconfigured,   // Initial state - not configured
  Inactive,       // Configured but not running
  Active,         // Running and controlling
  Finalized,      // Cleanup complete
  Unknown
};

/**
 * @brief Hardware interface claim state
 */
enum class InterfaceState {
  Available,      // Can be claimed by a controller
  Claimed,        // Currently in use by a controller
  Unavailable     // Not accessible
};

/**
 * @brief Information about a single controller
 */
struct ControllerInfo {
  QString name;
  QString type;
  ControllerState state = ControllerState::Unknown;
  QStringList claimedInterfaces;
  QStringList requiredCommandInterfaces;
  QStringList requiredStateInterfaces;
  QString controllerManagerName;
  QDateTime lastStateChange;
  bool isChainedController = false;
  QString chainParent;

  bool operator==(const ControllerInfo& other) const {
    return name == other.name && type == other.type && state == other.state;
  }
};

/**
 * @brief Information about a hardware interface
 */
struct HardwareInterfaceInfo {
  QString name;
  QString interfaceType;    // "command" or "state"
  InterfaceState state = InterfaceState::Available;
  QString claimedBy;        // Controller name if claimed
  QString componentName;    // Hardware component name
};

/**
 * @brief Panel for monitoring and controlling ros2_control controllers
 *
 * Provides visual lifecycle control and monitoring of ros2_control controllers.
 * Shows controller states with color coding, hardware interfaces, and provides
 * buttons for lifecycle management (activate, deactivate, etc.)
 */
class RosControlPanel : public QWidget {
  Q_OBJECT

public:
  explicit RosControlPanel(QWidget* parent = nullptr);
  ~RosControlPanel() override;

  void setCanvas(WeaverCanvas* canvas) { canvas_ = canvas; }
  bool isMonitoring() const { return monitoring_; }

public slots:
  // Discovery and refresh
  void refreshControllers();
  void refreshHardwareInterfaces();
  void refreshAll();
  void refreshAllAsync();  // Non-blocking version for initial load

  // Auto-refresh control
  void startMonitoring();
  void stopMonitoring();
  void toggleMonitoring();

  // Lifecycle management
  void activateController(const QString& controllerName);
  void deactivateController(const QString& controllerName);
  void switchControllers(const QStringList& activate, const QStringList& deactivate);
  void emergencyStopAll();

signals:
  // AI integration signals
  void controllerStateChanged(const QString& name, const QString& oldState,
                              const QString& newState);
  void controllerActionRequested(const QString& action, const QString& controller);
  void errorOccurred(const QString& message);

private slots:
  void onControllerSelected(QTreeWidgetItem* item, int column);
  void onInterfaceSelected(QTreeWidgetItem* item, int column);
  void onAutoRefreshTimer();
  void onControllerContextMenu(const QPoint& pos);
  void onInterfaceContextMenu(const QPoint& pos);
  void onFilterChanged();
  void onCommandFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onAsyncRefreshComplete();

private:
  void setupUi();
  QWidget* createToolbar();
  void setupControllerTree();
  void setupInterfaceTree();
  void setupDetailsPanel();
  void setupActionButtons();
  void setupConnections();

  // CLI execution helpers
  QString runRos2ControlCommand(const QStringList& args, int timeoutMs = 5000);
  QString runRos2ControlCommandForManager(const QStringList& args,
                                          const QString& manager, int timeoutMs);

  // Parsing helpers
  QList<ControllerInfo> parseControllerList(const QString& output);
  QList<HardwareInterfaceInfo> parseHardwareInterfaces(const QString& output);
  ControllerState parseControllerState(const QString& stateStr);

  // UI update helpers
  void updateControllerTree(const QList<ControllerInfo>& controllers);
  void updateInterfaceTree(const QList<HardwareInterfaceInfo>& interfaces);
  void updateDetailsPanel(const ControllerInfo* info);
  void updateActionButtonsState();
  void updateStatusLabel();

  QColor getStateColor(ControllerState state) const;
  QString getStateText(ControllerState state) const;
  QIcon getStateIcon(ControllerState state) const;
  QColor getInterfaceStateColor(InterfaceState state) const;
  QString getInterfaceStateText(InterfaceState state) const;

  // UI Components - Toolbar
  QLineEdit* searchEdit_;
  QComboBox* managerCombo_;
  QPushButton* refreshButton_;
  QToolButton* liveButton_;
  QLabel* statusLabel_;

  // UI Components - Main layout
  QSplitter* mainSplitter_;
  QSplitter* leftSplitter_;

  // Controller tree
  QTreeWidget* controllerTree_;

  // Hardware interface tree
  QTreeWidget* interfaceTree_;

  // Details panel
  QWidget* detailsPanel_;
  QLabel* controllerNameLabel_;
  QLabel* controllerTypeLabel_;
  QLabel* controllerStateLabel_;
  QLabel* claimedInterfacesLabel_;
  QLabel* lastStateChangeLabel_;

  // Action buttons
  QGroupBox* actionsGroupBox_;
  QPushButton* activateButton_;
  QPushButton* deactivateButton_;
  QPushButton* emergencyStopButton_;

  // Canvas integration
  WeaverCanvas* canvas_ = nullptr;

  // Data storage
  QList<ControllerInfo> controllers_;
  QList<HardwareInterfaceInfo> interfaces_;
  QString selectedController_;
  QString selectedManager_;

  // Monitoring state
  QTimer* autoRefreshTimer_;
  bool monitoring_ = false;

  // Async process for commands
  QProcess* commandProcess_ = nullptr;
  QString pendingAction_;
  QString pendingController_;

  // Async refresh thread
  std::unique_ptr<std::thread> refreshThread_;
  std::atomic<bool> refreshThreadRunning_{false};
  QList<ControllerInfo> pendingControllers_;
  QList<HardwareInterfaceInfo> pendingInterfaces_;
  bool asyncRefreshHasData_ = false;

  // Configuration
  static constexpr int DEFAULT_REFRESH_INTERVAL_MS = 2000;
  static constexpr int CLI_TIMEOUT_MS = 5000;
  static constexpr int ASYNC_CLI_TIMEOUT_MS = 2000;  // Shorter timeout for async initial load
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::ControllerInfo)
Q_DECLARE_METATYPE(ros_weaver::HardwareInterfaceInfo)
Q_DECLARE_METATYPE(ros_weaver::ControllerState)

#endif  // ROS_WEAVER_ROS_CONTROL_PANEL_HPP
