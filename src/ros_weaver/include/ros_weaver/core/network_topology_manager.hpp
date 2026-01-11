#ifndef ROS_WEAVER_CORE_NETWORK_TOPOLOGY_MANAGER_HPP
#define ROS_WEAVER_CORE_NETWORK_TOPOLOGY_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QList>
#include <QMap>
#include <QTimer>
#include <QDateTime>
#include <QColor>
#include <QProcess>

#include <atomic>

namespace ros_weaver {

/**
 * @brief DDS discovery method used by the middleware
 */
enum class DiscoveryType {
  Unknown,
  Multicast,        // Standard DDS multicast discovery
  Unicast,          // Unicast peer list discovery
  DiscoveryServer   // FastDDS Discovery Server
};

/**
 * @brief Information about a network host running ROS2 nodes
 */
struct HostInfo {
  QString hostname;           // Resolved hostname or IP
  QString ipAddress;          // IP address
  bool isLocal = false;       // True if this is the local machine
  QStringList nodeNames;      // Full node names running on this host
  int nodeCount = 0;          // Number of nodes on this host
  qint64 lastSeen = 0;        // Timestamp of last discovery

  /**
   * @brief Get display name (hostname if available, else IP)
   */
  QString displayName() const {
    return hostname.isEmpty() ? ipAddress : hostname;
  }
};

/**
 * @brief Information about a ROS2 node/participant
 */
struct ParticipantInfo {
  QString nodeName;           // Full ROS2 node name (with namespace)
  QString hostAddress;        // IP address of the host
  QString hostname;           // Resolved hostname
  QStringList publishers;     // Topics this node publishes
  QStringList subscribers;    // Topics this node subscribes to
  QStringList services;       // Services this node provides
  QStringList clients;        // Service clients this node has
};

/**
 * @brief Connection between two hosts
 */
struct HostConnection {
  QString sourceHost;
  QString targetHost;
  QStringList topics;         // Topics flowing between these hosts
  int connectionCount = 0;    // Number of topic connections
};

/**
 * @brief Complete network topology snapshot
 */
struct NetworkTopology {
  int domainId = 0;
  QString rmwImplementation;
  DiscoveryType discoveryType = DiscoveryType::Unknown;
  QList<HostInfo> hosts;
  QList<ParticipantInfo> participants;
  QList<HostConnection> connections;
  qint64 timestamp = 0;

  bool isEmpty() const { return hosts.isEmpty() && participants.isEmpty(); }

  void clear() {
    hosts.clear();
    participants.clear();
    connections.clear();
    timestamp = 0;
  }

  QString discoveryTypeString() const;
};

/**
 * @brief Manages DDS network topology discovery
 *
 * Features:
 * - Discovers ROS2 nodes and their network locations
 * - Detects RMW implementation and domain ID
 * - Identifies discovery method (multicast/unicast/discovery server)
 * - Groups nodes by host
 * - Maps inter-host connections
 * - Supports manual and auto-refresh
 */
class NetworkTopologyManager : public QObject {
  Q_OBJECT

public:
  explicit NetworkTopologyManager(QObject* parent = nullptr);
  ~NetworkTopologyManager() override;

  /**
   * @brief Get the last discovered network topology
   */
  const NetworkTopology& topology() const { return topology_; }

  /**
   * @brief Check if a scan is currently in progress
   */
  bool isScanning() const { return scanning_.load(); }

  /**
   * @brief Check if auto-refresh is enabled
   */
  bool isAutoRefreshEnabled() const { return autoRefreshEnabled_; }

  /**
   * @brief Get auto-refresh interval in seconds
   */
  int autoRefreshInterval() const { return autoRefreshIntervalSec_; }

  /**
   * @brief Get current ROS_DOMAIN_ID
   */
  int getCurrentDomainId() const;

  /**
   * @brief Get current RMW_IMPLEMENTATION
   */
  QString detectRmwImplementation() const;

  /**
   * @brief Detect the discovery type from environment/config
   */
  DiscoveryType detectDiscoveryType() const;

  /**
   * @brief Load settings from QSettings
   */
  void loadSettings();

  /**
   * @brief Save settings to QSettings
   */
  void saveSettings();

  // Settings getters
  bool showBandwidth() const { return showBandwidth_; }
  QString viewMode() const { return viewMode_; }

public slots:
  /**
   * @brief Trigger a network topology scan
   */
  void scan();

  /**
   * @brief Stop any ongoing scan
   */
  void stopScan();

  /**
   * @brief Enable/disable auto-refresh
   */
  void setAutoRefreshEnabled(bool enabled);

  /**
   * @brief Set auto-refresh interval in seconds (10-300)
   */
  void setAutoRefreshInterval(int seconds);

  /**
   * @brief Set whether to show bandwidth info
   */
  void setShowBandwidth(bool show);

  /**
   * @brief Set default view mode ("graph" or "table")
   */
  void setViewMode(const QString& mode);

signals:
  /**
   * @brief Emitted when a scan starts
   */
  void scanStarted();

  /**
   * @brief Emitted during scan with progress info
   */
  void scanProgress(int percent, const QString& message);

  /**
   * @brief Emitted when scan completes successfully
   */
  void scanCompleted(const NetworkTopology& topology);

  /**
   * @brief Emitted if scan fails
   */
  void scanFailed(const QString& error);

  /**
   * @brief Emitted when topology changes
   */
  void topologyChanged();

  /**
   * @brief Emitted when a new host is discovered
   */
  void hostDiscovered(const HostInfo& host);

  /**
   * @brief Emitted when settings change
   */
  void settingsChanged();

private slots:
  void onAutoRefreshTimer();
  void onNodeListProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onNodeInfoProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
  void performScan();
  void parseNodeList(const QString& output);
  void parseNodeInfo(const QString& nodeName, const QString& output);
  void buildHostMap();
  void buildHostConnections();
  void finalizeScan();

  // State
  NetworkTopology topology_;
  std::atomic<bool> scanning_;
  QStringList pendingNodeInfoQueries_;
  QMap<QString, ParticipantInfo> discoveredParticipants_;

  // Auto-refresh
  bool autoRefreshEnabled_ = false;
  int autoRefreshIntervalSec_ = 30;
  QTimer* autoRefreshTimer_ = nullptr;

  // Processes for ROS2 CLI commands
  QProcess* nodeListProcess_ = nullptr;
  QProcess* nodeInfoProcess_ = nullptr;

  // Settings
  bool showBandwidth_ = true;
  QString viewMode_ = "graph";

  // Settings keys
  static constexpr const char* kSettingsGroup = "NetworkTopology";
  static constexpr const char* kKeyAutoRefresh = "autoRefreshEnabled";
  static constexpr const char* kKeyRefreshInterval = "refreshIntervalSec";
  static constexpr const char* kKeyShowBandwidth = "showBandwidth";
  static constexpr const char* kKeyViewMode = "viewMode";
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::DiscoveryType)
Q_DECLARE_METATYPE(ros_weaver::HostInfo)
Q_DECLARE_METATYPE(ros_weaver::ParticipantInfo)
Q_DECLARE_METATYPE(ros_weaver::NetworkTopology)

#endif  // ROS_WEAVER_CORE_NETWORK_TOPOLOGY_MANAGER_HPP
