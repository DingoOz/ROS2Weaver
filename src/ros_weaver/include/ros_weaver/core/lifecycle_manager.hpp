#ifndef ROS_WEAVER_CORE_LIFECYCLE_MANAGER_HPP
#define ROS_WEAVER_CORE_LIFECYCLE_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QMap>
#include <QDateTime>
#include <QMutex>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/get_available_transitions.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

namespace ros_weaver {

/**
 * @brief Enum representing lifecycle node states
 */
enum class LifecycleState {
  Unknown = 0,
  Unconfigured = 1,
  Inactive = 2,
  Active = 3,
  Finalized = 4,
  // Transitioning states
  Configuring = 10,
  CleaningUp = 11,
  ShuttingDown = 12,
  Activating = 13,
  Deactivating = 14,
  ErrorProcessing = 15
};

/**
 * @brief Available lifecycle transitions
 */
enum class LifecycleTransition {
  Configure = 1,
  CleanUp = 2,
  Activate = 3,
  Deactivate = 4,
  UnconfiguredShutdown = 5,
  InactiveShutdown = 6,
  ActiveShutdown = 7,
  Destroy = 8
};

/**
 * @brief Information about a lifecycle node
 */
struct LifecycleNodeInfo {
  QString nodeName;
  LifecycleState currentState;
  QDateTime lastStateChange;
  QStringList availableTransitions;
  bool isResponsive;
};

/**
 * @brief Manager class for discovering and controlling ROS2 lifecycle nodes
 *
 * Features:
 * - Discover lifecycle nodes in the ROS graph
 * - Query node states
 * - Request state transitions
 * - Subscribe to transition events
 */
class LifecycleManager : public QObject {
  Q_OBJECT

public:
  explicit LifecycleManager(QObject* parent = nullptr);
  ~LifecycleManager() override;

  /**
   * @brief Initialize with a ROS node
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

  /**
   * @brief Discover lifecycle nodes in the ROS graph
   * @return List of lifecycle node names
   */
  QStringList discoverLifecycleNodes();

  /**
   * @brief Get the current state of a lifecycle node
   */
  LifecycleState getNodeState(const QString& nodeName);

  /**
   * @brief Get available transitions for a node
   */
  QStringList getAvailableTransitions(const QString& nodeName);

  /**
   * @brief Request a state transition
   * @param nodeName The node to transition
   * @param transition The transition to request
   */
  void requestTransition(const QString& nodeName, LifecycleTransition transition);

  /**
   * @brief Request transition by name
   */
  void requestTransitionByName(const QString& nodeName, const QString& transitionName);

  /**
   * @brief Get info for all tracked nodes
   */
  QList<LifecycleNodeInfo> getAllNodeInfo() const;

  /**
   * @brief Get info for a specific node
   */
  LifecycleNodeInfo getNodeInfo(const QString& nodeName) const;

  /**
   * @brief Start monitoring a lifecycle node
   */
  void monitorNode(const QString& nodeName);

  /**
   * @brief Stop monitoring a node
   */
  void stopMonitoringNode(const QString& nodeName);

  /**
   * @brief Check if a node is a lifecycle node
   */
  bool isLifecycleNode(const QString& nodeName);

  /**
   * @brief Convert state enum to string
   */
  static QString stateToString(LifecycleState state);

  /**
   * @brief Convert state ID to enum
   */
  static LifecycleState stateFromId(uint8_t id);

  /**
   * @brief Convert transition name to enum
   */
  static LifecycleTransition transitionFromName(const QString& name);

  // Batch operations
  void configureAll(const QStringList& nodes);
  void activateAll(const QStringList& nodes);
  void deactivateAll(const QStringList& nodes);
  void shutdownAll(const QStringList& nodes);

signals:
  /**
   * @brief Emitted when a node's state changes
   */
  void stateChanged(const QString& nodeName, LifecycleState newState);

  /**
   * @brief Emitted when a transition request fails
   */
  void transitionFailed(const QString& nodeName, const QString& error);

  /**
   * @brief Emitted when a transition request succeeds
   */
  void transitionSucceeded(const QString& nodeName, LifecycleState newState);

  /**
   * @brief Emitted when lifecycle nodes are discovered
   */
  void lifecycleNodesDiscovered(const QStringList& nodes);

  /**
   * @brief Emitted when node discovery starts
   */
  void discoveryStarted();

  /**
   * @brief Emitted when node discovery completes
   */
  void discoveryCompleted();

private:
  void subscribeToTransitionEvents(const QString& nodeName);
  void handleTransitionEvent(const QString& nodeName,
                              const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  // Service clients per node
  QMap<QString, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> getStateClients_;
  QMap<QString, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> changeStateClients_;
  QMap<QString, rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr>
      getTransitionsClients_;

  // Transition event subscriptions
  QMap<QString, rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr>
      transitionSubs_;

  // Cached node info
  QMap<QString, LifecycleNodeInfo> nodeInfoCache_;

  mutable QMutex mutex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LIFECYCLE_MANAGER_HPP
