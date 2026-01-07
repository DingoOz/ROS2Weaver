#include "ros_weaver/core/lifecycle_manager.hpp"

#include <QMutexLocker>
#include <iostream>

namespace ros_weaver {

LifecycleManager::LifecycleManager(QObject* parent)
  : QObject(parent)
  , node_(nullptr)
{
}

LifecycleManager::~LifecycleManager() {
  QMutexLocker locker(&mutex_);
  getStateClients_.clear();
  changeStateClients_.clear();
  getTransitionsClients_.clear();
  transitionSubs_.clear();
}

void LifecycleManager::setRosNode(rclcpp::Node::SharedPtr node) {
  QMutexLocker locker(&mutex_);
  node_ = node;
}

QStringList LifecycleManager::discoverLifecycleNodes() {
  QMutexLocker locker(&mutex_);
  QStringList lifecycleNodes;

  if (!node_) {
    return lifecycleNodes;
  }

  emit discoveryStarted();

  try {
    // Get all nodes
    auto nodeNames = node_->get_node_names();

    for (const auto& name : nodeNames) {
      // Check if it has lifecycle services
      QString nodeName = QString::fromStdString(name);

      // Skip our own node
      if (nodeName.contains("ros_weaver")) {
        continue;
      }

      // Look for lifecycle services
      auto services = node_->get_service_names_and_types();
      QString getStateService = nodeName + "/get_state";
      QString changeStateService = nodeName + "/change_state";

      bool hasGetState = false;
      bool hasChangeState = false;

      for (const auto& [svcName, types] : services) {
        QString svc = QString::fromStdString(svcName);
        if (svc.endsWith("/get_state")) {
          for (const auto& type : types) {
            if (type == "lifecycle_msgs/srv/GetState") {
              hasGetState = true;
              break;
            }
          }
        }
        if (svc.endsWith("/change_state")) {
          for (const auto& type : types) {
            if (type == "lifecycle_msgs/srv/ChangeState") {
              hasChangeState = true;
              break;
            }
          }
        }
      }

      if (hasGetState && hasChangeState) {
        lifecycleNodes.append(nodeName);
        std::cerr << "LifecycleManager: Found lifecycle node: "
                  << nodeName.toStdString() << std::endl;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "LifecycleManager: Discovery error: " << e.what() << std::endl;
  }

  emit discoveryCompleted();
  emit lifecycleNodesDiscovered(lifecycleNodes);

  return lifecycleNodes;
}

LifecycleState LifecycleManager::getNodeState(const QString& nodeName) {
  QMutexLocker locker(&mutex_);

  if (!node_) {
    return LifecycleState::Unknown;
  }

  try {
    // Create client if needed
    if (!getStateClients_.contains(nodeName)) {
      QString serviceName = nodeName + "/get_state";
      auto client = node_->create_client<lifecycle_msgs::srv::GetState>(
          serviceName.toStdString());
      getStateClients_[nodeName] = client;
    }

    auto client = getStateClients_[nodeName];

    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      return LifecycleState::Unknown;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);

    // Wait for result (with timeout)
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto response = future.get();
      return stateFromId(response->current_state.id);
    }
  } catch (const std::exception& e) {
    std::cerr << "LifecycleManager: Failed to get state: " << e.what() << std::endl;
  }

  return LifecycleState::Unknown;
}

QStringList LifecycleManager::getAvailableTransitions(const QString& nodeName) {
  QMutexLocker locker(&mutex_);
  QStringList transitions;

  if (!node_) {
    return transitions;
  }

  try {
    // Create client if needed
    if (!getTransitionsClients_.contains(nodeName)) {
      QString serviceName = nodeName + "/get_available_transitions";
      auto client = node_->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
          serviceName.toStdString());
      getTransitionsClients_[nodeName] = client;
    }

    auto client = getTransitionsClients_[nodeName];

    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      return transitions;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetAvailableTransitions::Request>();
    auto future = client->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto response = future.get();
      for (const auto& transition : response->available_transitions) {
        transitions.append(QString::fromStdString(transition.transition.label));
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "LifecycleManager: Failed to get transitions: " << e.what() << std::endl;
  }

  return transitions;
}

void LifecycleManager::requestTransition(const QString& nodeName,
                                          LifecycleTransition transition) {
  QMutexLocker locker(&mutex_);

  if (!node_) {
    emit transitionFailed(nodeName, tr("ROS node not initialized"));
    return;
  }

  try {
    // Create client if needed
    if (!changeStateClients_.contains(nodeName)) {
      QString serviceName = nodeName + "/change_state";
      auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>(
          serviceName.toStdString());
      changeStateClients_[nodeName] = client;
    }

    auto client = changeStateClients_[nodeName];

    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      emit transitionFailed(nodeName, tr("Service not available"));
      return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = static_cast<uint8_t>(transition);

    // Send request asynchronously
    auto future = client->async_send_request(
        request,
        [this, nodeName](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
          try {
            auto response = future.get();
            if (response->success) {
              LifecycleState newState = getNodeState(nodeName);
              QMetaObject::invokeMethod(this, [this, nodeName, newState]() {
                emit stateChanged(nodeName, newState);
                emit transitionSucceeded(nodeName, newState);
              }, Qt::QueuedConnection);
            } else {
              QMetaObject::invokeMethod(this, [this, nodeName]() {
                emit transitionFailed(nodeName, tr("Transition failed"));
              }, Qt::QueuedConnection);
            }
          } catch (const std::exception& e) {
            QString error = QString::fromStdString(e.what());
            QMetaObject::invokeMethod(this, [this, nodeName, error]() {
              emit transitionFailed(nodeName, error);
            }, Qt::QueuedConnection);
          }
        });

    std::cerr << "LifecycleManager: Requested transition "
              << static_cast<int>(transition) << " for " << nodeName.toStdString() << std::endl;

  } catch (const std::exception& e) {
    emit transitionFailed(nodeName, QString::fromStdString(e.what()));
  }
}

void LifecycleManager::requestTransitionByName(const QString& nodeName,
                                                 const QString& transitionName) {
  LifecycleTransition transition = transitionFromName(transitionName);
  requestTransition(nodeName, transition);
}

QList<LifecycleNodeInfo> LifecycleManager::getAllNodeInfo() const {
  QMutexLocker locker(&mutex_);
  return nodeInfoCache_.values();
}

LifecycleNodeInfo LifecycleManager::getNodeInfo(const QString& nodeName) const {
  QMutexLocker locker(&mutex_);
  return nodeInfoCache_.value(nodeName);
}

void LifecycleManager::monitorNode(const QString& nodeName) {
  QMutexLocker locker(&mutex_);

  if (!node_ || transitionSubs_.contains(nodeName)) {
    return;
  }

  subscribeToTransitionEvents(nodeName);

  // Initialize cache entry
  LifecycleNodeInfo info;
  info.nodeName = nodeName;
  info.currentState = LifecycleState::Unknown;
  info.lastStateChange = QDateTime::currentDateTime();
  info.isResponsive = true;
  nodeInfoCache_[nodeName] = info;
}

void LifecycleManager::stopMonitoringNode(const QString& nodeName) {
  QMutexLocker locker(&mutex_);

  transitionSubs_.remove(nodeName);
  nodeInfoCache_.remove(nodeName);
}

bool LifecycleManager::isLifecycleNode(const QString& nodeName) {
  QMutexLocker locker(&mutex_);

  if (!node_) {
    return false;
  }

  try {
    auto services = node_->get_service_names_and_types();
    QString getStateService = nodeName + "/get_state";

    for (const auto& [svcName, types] : services) {
      if (QString::fromStdString(svcName) == getStateService) {
        for (const auto& type : types) {
          if (type == "lifecycle_msgs/srv/GetState") {
            return true;
          }
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "LifecycleManager: Error checking lifecycle node: "
              << e.what() << std::endl;
  }

  return false;
}

QString LifecycleManager::stateToString(LifecycleState state) {
  switch (state) {
    case LifecycleState::Unknown:
      return tr("Unknown");
    case LifecycleState::Unconfigured:
      return tr("Unconfigured");
    case LifecycleState::Inactive:
      return tr("Inactive");
    case LifecycleState::Active:
      return tr("Active");
    case LifecycleState::Finalized:
      return tr("Finalized");
    case LifecycleState::Configuring:
      return tr("Configuring");
    case LifecycleState::CleaningUp:
      return tr("Cleaning Up");
    case LifecycleState::ShuttingDown:
      return tr("Shutting Down");
    case LifecycleState::Activating:
      return tr("Activating");
    case LifecycleState::Deactivating:
      return tr("Deactivating");
    case LifecycleState::ErrorProcessing:
      return tr("Error Processing");
    default:
      return tr("Unknown");
  }
}

LifecycleState LifecycleManager::stateFromId(uint8_t id) {
  switch (id) {
    case 1:
      return LifecycleState::Unconfigured;
    case 2:
      return LifecycleState::Inactive;
    case 3:
      return LifecycleState::Active;
    case 4:
      return LifecycleState::Finalized;
    case 10:
      return LifecycleState::Configuring;
    case 11:
      return LifecycleState::CleaningUp;
    case 12:
      return LifecycleState::ShuttingDown;
    case 13:
      return LifecycleState::Activating;
    case 14:
      return LifecycleState::Deactivating;
    case 15:
      return LifecycleState::ErrorProcessing;
    default:
      return LifecycleState::Unknown;
  }
}

LifecycleTransition LifecycleManager::transitionFromName(const QString& name) {
  QString lower = name.toLower();

  if (lower == "configure") {
    return LifecycleTransition::Configure;
  } else if (lower == "cleanup" || lower == "clean_up") {
    return LifecycleTransition::CleanUp;
  } else if (lower == "activate") {
    return LifecycleTransition::Activate;
  } else if (lower == "deactivate") {
    return LifecycleTransition::Deactivate;
  } else if (lower == "shutdown") {
    return LifecycleTransition::InactiveShutdown;
  } else if (lower == "destroy") {
    return LifecycleTransition::Destroy;
  }

  return LifecycleTransition::Configure;  // Default
}

void LifecycleManager::configureAll(const QStringList& nodes) {
  for (const QString& node : nodes) {
    requestTransition(node, LifecycleTransition::Configure);
  }
}

void LifecycleManager::activateAll(const QStringList& nodes) {
  for (const QString& node : nodes) {
    requestTransition(node, LifecycleTransition::Activate);
  }
}

void LifecycleManager::deactivateAll(const QStringList& nodes) {
  for (const QString& node : nodes) {
    requestTransition(node, LifecycleTransition::Deactivate);
  }
}

void LifecycleManager::shutdownAll(const QStringList& nodes) {
  for (const QString& node : nodes) {
    requestTransition(node, LifecycleTransition::InactiveShutdown);
  }
}

void LifecycleManager::subscribeToTransitionEvents(const QString& nodeName) {
  try {
    QString topicName = nodeName + "/transition_event";

    auto callback = [this, nodeName](
        const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
      handleTransitionEvent(nodeName, msg);
    };

    auto subscription = node_->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        topicName.toStdString(),
        rclcpp::QoS(10),
        callback);

    transitionSubs_[nodeName] = subscription;

    std::cerr << "LifecycleManager: Subscribed to " << topicName.toStdString() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "LifecycleManager: Failed to subscribe to transition events: "
              << e.what() << std::endl;
  }
}

void LifecycleManager::handleTransitionEvent(
    const QString& nodeName,
    const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
  LifecycleState newState = stateFromId(msg->goal_state.id);

  // Update cache
  {
    QMutexLocker locker(&mutex_);
    if (nodeInfoCache_.contains(nodeName)) {
      nodeInfoCache_[nodeName].currentState = newState;
      nodeInfoCache_[nodeName].lastStateChange = QDateTime::currentDateTime();
    }
  }

  // Emit signal on Qt thread
  QMetaObject::invokeMethod(this, [this, nodeName, newState]() {
    emit stateChanged(nodeName, newState);
  }, Qt::QueuedConnection);
}

}  // namespace ros_weaver
