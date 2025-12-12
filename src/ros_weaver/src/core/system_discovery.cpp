#include "ros_weaver/core/system_discovery.hpp"

#include <QDateTime>
#include <QThread>
#include <QDebug>

namespace ros_weaver {

SystemDiscovery::SystemDiscovery(QObject* parent)
  : QObject(parent)
  , rosNode_(nullptr)
  , spinning_(false)
  , scanning_(false)
  , autoScanEnabled_(false)
  , autoScanIntervalSec_(10)
  , autoScanTimer_(nullptr)
  , scanTimeoutSec_(5)
  , scanTimeoutTimer_(nullptr)
  , scanStartTime_(0)
{
  // Register meta types for cross-thread signal/slot connections
  static bool typesRegistered = false;
  if (!typesRegistered) {
    qRegisterMetaType<DiscoveredTopic>("DiscoveredTopic");
    qRegisterMetaType<DiscoveredNode>("DiscoveredNode");
    qRegisterMetaType<SystemGraph>("SystemGraph");
    typesRegistered = true;
  }

  autoScanTimer_ = new QTimer(this);
  connect(autoScanTimer_, &QTimer::timeout, this, &SystemDiscovery::onAutoScanTimer);

  scanTimeoutTimer_ = new QTimer(this);
  scanTimeoutTimer_->setSingleShot(true);
  connect(scanTimeoutTimer_, &QTimer::timeout, this, &SystemDiscovery::onScanTimeoutTimer);
}

SystemDiscovery::~SystemDiscovery() {
  stopScan();
  shutdownRosNode();
}

void SystemDiscovery::initializeRosNode() {
  if (rosNode_) {
    return;  // Already initialized
  }

  try {
    // Initialize ROS2 if not already done
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create a unique node name for discovery
    std::string nodeName = "ros_weaver_discovery_" +
                           std::to_string(QDateTime::currentMSecsSinceEpoch());

    rosNode_ = std::make_shared<rclcpp::Node>(nodeName);

    // Start spinning in a separate thread
    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  } catch (const std::exception& e) {
    emit scanFailed(tr("Failed to initialize ROS2 node: %1").arg(e.what()));
  }
}

void SystemDiscovery::shutdownRosNode() {
  spinning_ = false;

  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();
  rosNode_.reset();
}

void SystemDiscovery::scan() {
  if (scanning_.load()) {
    return;  // Already scanning
  }

  scanning_ = true;
  scanStartTime_ = QDateTime::currentMSecsSinceEpoch();
  emit scanStarted();
  emit scanProgress(0, tr("Initializing ROS2 discovery..."));

  // Start timeout timer
  scanTimeoutTimer_->start(scanTimeoutSec_ * 1000);

  // Perform scan in a separate thread to avoid blocking UI
  QThread* thread = QThread::create([this]() {
    performScan();
  });

  connect(thread, &QThread::finished, thread, &QThread::deleteLater);
  thread->start();
}

qint64 SystemDiscovery::elapsedScanTime() const {
  if (!scanning_.load() || scanStartTime_ == 0) {
    return 0;
  }
  return QDateTime::currentMSecsSinceEpoch() - scanStartTime_;
}

void SystemDiscovery::performScan() {
  try {
    initializeRosNode();

    if (!rosNode_) {
      emit scanFailed(tr("ROS2 node not available"));
      scanning_ = false;
      return;
    }

    // Small delay to ensure node is fully initialized
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    emit scanProgress(10, tr("Discovering topics..."));

    SystemGraph graph;
    graph.timestamp = QDateTime::currentMSecsSinceEpoch();

    // Discover topics
    graph.topics = discoverTopics();
    emit scanProgress(40, tr("Found %1 topics. Discovering nodes...").arg(graph.topics.size()));

    // Discover nodes
    graph.nodes = discoverNodes();
    emit scanProgress(70, tr("Found %1 nodes. Building graph...").arg(graph.nodes.size()));

    // Build publisher/subscriber relationships
    buildPublisherSubscriberMap(graph);
    emit scanProgress(90, tr("Finalizing..."));

    // Store results
    systemGraph_ = graph;

    // Stop timeout timer (from main thread)
    QMetaObject::invokeMethod(scanTimeoutTimer_, "stop", Qt::QueuedConnection);

    emit scanProgress(100, tr("Scan complete"));
    emit scanCompleted(graph);

  } catch (const std::exception& e) {
    // Stop timeout timer (from main thread)
    QMetaObject::invokeMethod(scanTimeoutTimer_, "stop", Qt::QueuedConnection);
    emit scanFailed(tr("Scan failed: %1").arg(e.what()));
  }

  scanning_ = false;
  scanStartTime_ = 0;
}

QList<DiscoveredTopic> SystemDiscovery::discoverTopics() {
  QList<DiscoveredTopic> topics;

  if (!rosNode_) {
    return topics;
  }

  try {
    // Get all topics with their types
    auto topicNamesAndTypes = rosNode_->get_topic_names_and_types();

    for (const auto& [topicName, types] : topicNamesAndTypes) {
      DiscoveredTopic topic;
      topic.name = QString::fromStdString(topicName);

      // Get the first type (usually there's only one)
      if (!types.empty()) {
        topic.type = QString::fromStdString(types[0]);
      }

      // Get publisher and subscriber counts
      auto pubInfo = rosNode_->get_publishers_info_by_topic(topicName);
      auto subInfo = rosNode_->get_subscriptions_info_by_topic(topicName);

      topic.publisherCount = static_cast<int>(pubInfo.size());
      topic.subscriberCount = static_cast<int>(subInfo.size());

      // Extract node names from endpoint info
      for (const auto& info : pubInfo) {
        QString nodeName = QString::fromStdString(info.node_name());
        QString nodeNs = QString::fromStdString(info.node_namespace());
        QString fullName = nodeNs + "/" + nodeName;
        if (nodeNs == "/") {
          fullName = "/" + nodeName;
        }
        if (!topic.publishers.contains(fullName)) {
          topic.publishers.append(fullName);
        }
      }

      for (const auto& info : subInfo) {
        QString nodeName = QString::fromStdString(info.node_name());
        QString nodeNs = QString::fromStdString(info.node_namespace());
        QString fullName = nodeNs + "/" + nodeName;
        if (nodeNs == "/") {
          fullName = "/" + nodeName;
        }
        if (!topic.subscribers.contains(fullName)) {
          topic.subscribers.append(fullName);
        }
      }

      topics.append(topic);
    }
  } catch (const std::exception& e) {
    qWarning() << "Error discovering topics:" << e.what();
  }

  return topics;
}

QList<DiscoveredNode> SystemDiscovery::discoverNodes() {
  QList<DiscoveredNode> nodes;

  if (!rosNode_) {
    return nodes;
  }

  try {
    // Get all node names with namespaces
    auto nodeNamesAndNamespaces = rosNode_->get_node_names();

    for (const auto& fullName : nodeNamesAndNamespaces) {
      // Skip our own discovery node
      if (QString::fromStdString(fullName).contains("ros_weaver_discovery")) {
        continue;
      }

      DiscoveredNode node;
      node.fullName = QString::fromStdString(fullName);

      // Parse namespace and name from full name
      QString qFullName = node.fullName;
      int lastSlash = qFullName.lastIndexOf('/');
      if (lastSlash >= 0) {
        node.name = qFullName.mid(lastSlash + 1);
        node.namespacePath = qFullName.left(lastSlash);
        if (node.namespacePath.isEmpty()) {
          node.namespacePath = "/";
        }
      } else {
        node.name = qFullName;
        node.namespacePath = "/";
      }

      // Get node info - services (publishers/subscribers are obtained via topics)
      try {
        // Get services for this node
        auto srvTopics = rosNode_->get_service_names_and_types_by_node(
          node.name.toStdString(), node.namespacePath.toStdString());
        for (const auto& [service, types] : srvTopics) {
          node.services.append(QString::fromStdString(service));
        }

        // Publishers and subscribers are determined from topic info
        // We'll fill them in from the topic discovery data
      } catch (const std::exception& e) {
        qWarning() << "Error getting info for node" << node.fullName << ":" << e.what();
      }

      nodes.append(node);
    }
  } catch (const std::exception& e) {
    qWarning() << "Error discovering nodes:" << e.what();
  }

  return nodes;
}

void SystemDiscovery::buildPublisherSubscriberMap(SystemGraph& graph) {
  // Build topic type map
  for (const auto& topic : graph.topics) {
    graph.topicTypes[topic.name] = topic.type;
  }

  // Build publisher/subscriber lists for each node from topic data
  for (DiscoveredNode& node : graph.nodes) {
    for (const DiscoveredTopic& topic : graph.topics) {
      // Check if this node publishes to this topic
      if (topic.publishers.contains(node.fullName)) {
        if (!node.publishers.contains(topic.name)) {
          node.publishers.append(topic.name);
        }
      }
      // Check if this node subscribes to this topic
      if (topic.subscribers.contains(node.fullName)) {
        if (!node.subscribers.contains(topic.name)) {
          node.subscribers.append(topic.name);
        }
      }
    }
  }
}

void SystemDiscovery::stopScan() {
  scanning_ = false;
}

void SystemDiscovery::setAutoScanEnabled(bool enabled) {
  autoScanEnabled_ = enabled;

  if (enabled) {
    autoScanTimer_->start(autoScanIntervalSec_ * 1000);
  } else {
    autoScanTimer_->stop();
  }
}

void SystemDiscovery::setAutoScanInterval(int seconds) {
  autoScanIntervalSec_ = qBound(5, seconds, 60);

  if (autoScanEnabled_) {
    autoScanTimer_->setInterval(autoScanIntervalSec_ * 1000);
  }
}

void SystemDiscovery::onAutoScanTimer() {
  if (!scanning_.load()) {
    scan();
  }
}

void SystemDiscovery::setScanTimeout(int seconds) {
  scanTimeoutSec_ = qBound(1, seconds, 30);
}

void SystemDiscovery::onScanTimeoutTimer() {
  if (scanning_.load()) {
    // Scan is still running - it timed out
    scanning_ = false;
    scanStartTime_ = 0;

    // Emit with whatever data we have so far (may be partial)
    if (!systemGraph_.isEmpty()) {
      emit scanProgress(100, tr("Scan timed out (partial results)"));
      emit scanCompleted(systemGraph_);
    } else {
      emit scanProgress(100, tr("Scan timed out"));
    }
    emit scanTimedOut();
  }
}

}  // namespace ros_weaver
