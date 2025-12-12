#ifndef ROS_WEAVER_SYSTEM_DISCOVERY_HPP
#define ROS_WEAVER_SYSTEM_DISCOVERY_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>

namespace ros_weaver {

// Information about a discovered topic
struct DiscoveredTopic {
  QString name;
  QString type;
  QStringList publishers;   // Node names that publish to this topic
  QStringList subscribers;  // Node names that subscribe to this topic
  int publisherCount = 0;
  int subscriberCount = 0;
};

// Information about a discovered node
struct DiscoveredNode {
  QString name;
  QString namespacePath;
  QString fullName;         // namespace + name
  QStringList publishers;   // Topics this node publishes
  QStringList subscribers;  // Topics this node subscribes to
  QStringList services;     // Services this node provides
  QStringList clients;      // Service clients this node has
};

// Complete system graph
struct SystemGraph {
  QList<DiscoveredNode> nodes;
  QList<DiscoveredTopic> topics;
  QMap<QString, QString> topicTypes;  // topic name -> message type
  qint64 timestamp = 0;  // When the scan was performed

  bool isEmpty() const { return nodes.isEmpty() && topics.isEmpty(); }
  void clear() {
    nodes.clear();
    topics.clear();
    topicTypes.clear();
    timestamp = 0;
  }
};

class SystemDiscovery : public QObject {
  Q_OBJECT

public:
  explicit SystemDiscovery(QObject* parent = nullptr);
  ~SystemDiscovery() override;

  // Get the last discovered system graph
  const SystemGraph& systemGraph() const { return systemGraph_; }

  // Check if a scan is currently in progress
  bool isScanning() const { return scanning_.load(); }

  // Check if auto-scan is enabled
  bool isAutoScanEnabled() const { return autoScanEnabled_; }

  // Get auto-scan interval in seconds
  int autoScanInterval() const { return autoScanIntervalSec_; }

  // Get scan timeout in seconds
  int scanTimeout() const { return scanTimeoutSec_; }

  // Get elapsed scan time in milliseconds (only valid during scanning)
  qint64 elapsedScanTime() const;

public slots:
  // Trigger a manual scan
  void scan();

  // Enable/disable automatic periodic scanning
  void setAutoScanEnabled(bool enabled);

  // Set auto-scan interval in seconds (5-60)
  void setAutoScanInterval(int seconds);

  // Set scan timeout in seconds (1-30)
  void setScanTimeout(int seconds);

  // Stop any ongoing scan
  void stopScan();

signals:
  // Emitted when a scan starts
  void scanStarted();

  // Emitted with progress during scan (0-100)
  void scanProgress(int percent, const QString& message);

  // Emitted when scan completes successfully
  void scanCompleted(const SystemGraph& graph);

  // Emitted if scan fails
  void scanFailed(const QString& error);

  // Emitted if scan times out
  void scanTimedOut();

private slots:
  void onAutoScanTimer();
  void onScanTimeoutTimer();

private:
  void performScan();
  void initializeRosNode();
  void shutdownRosNode();

  // Discovery operations
  QList<DiscoveredTopic> discoverTopics();
  QList<DiscoveredNode> discoverNodes();
  void buildPublisherSubscriberMap(SystemGraph& graph);

  // ROS2 node for discovery
  std::shared_ptr<rclcpp::Node> rosNode_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_;

  // State
  SystemGraph systemGraph_;
  std::atomic<bool> scanning_;

  // Auto-scan settings
  bool autoScanEnabled_;
  int autoScanIntervalSec_;
  QTimer* autoScanTimer_;

  // Scan timeout
  int scanTimeoutSec_;
  QTimer* scanTimeoutTimer_;
  qint64 scanStartTime_;
};

}  // namespace ros_weaver

// Register types for cross-thread signal/slot connections
Q_DECLARE_METATYPE(ros_weaver::DiscoveredTopic)
Q_DECLARE_METATYPE(ros_weaver::DiscoveredNode)
Q_DECLARE_METATYPE(ros_weaver::SystemGraph)

#endif  // ROS_WEAVER_SYSTEM_DISCOVERY_HPP
