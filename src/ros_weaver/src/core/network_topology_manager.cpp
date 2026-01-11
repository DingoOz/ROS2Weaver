#include "ros_weaver/core/network_topology_manager.hpp"

#include <QDateTime>
#include <QDebug>
#include <QHostInfo>
#include <QNetworkInterface>
#include <QSet>
#include <QSettings>

namespace ros_weaver {

namespace {

void registerMetaTypes() {
  static bool registered = false;
  if (registered) {
    return;
  }
  qRegisterMetaType<DiscoveryType>("DiscoveryType");
  qRegisterMetaType<HostInfo>("HostInfo");
  qRegisterMetaType<ParticipantInfo>("ParticipantInfo");
  qRegisterMetaType<NetworkTopology>("NetworkTopology");
  registered = true;
}

}  // namespace

QString NetworkTopology::discoveryTypeString() const {
  switch (discoveryType) {
    case DiscoveryType::Multicast:
      return "Multicast";
    case DiscoveryType::Unicast:
      return "Unicast";
    case DiscoveryType::DiscoveryServer:
      return "Discovery Server";
    default:
      return "Unknown";
  }
}

NetworkTopologyManager::NetworkTopologyManager(QObject* parent)
    : QObject(parent)
    , scanning_(false) {
  registerMetaTypes();

  autoRefreshTimer_ = new QTimer(this);
  connect(autoRefreshTimer_, &QTimer::timeout,
          this, &NetworkTopologyManager::onAutoRefreshTimer);

  loadSettings();
}

NetworkTopologyManager::~NetworkTopologyManager() {
  stopScan();
  saveSettings();
}

int NetworkTopologyManager::getCurrentDomainId() const {
  bool ok = false;
  int domainId = qEnvironmentVariableIntValue("ROS_DOMAIN_ID", &ok);
  return ok ? domainId : 0;
}

QString NetworkTopologyManager::detectRmwImplementation() const {
  QString rmw = qEnvironmentVariable("RMW_IMPLEMENTATION");
  return rmw.isEmpty() ? QStringLiteral("rmw_fastrtps_cpp") : rmw;
}

DiscoveryType NetworkTopologyManager::detectDiscoveryType() const {
  // Check for FastDDS Discovery Server
  if (!qEnvironmentVariable("ROS_DISCOVERY_SERVER").isEmpty()) {
    return DiscoveryType::DiscoveryServer;
  }

  // Check FastDDS or CycloneDDS config for unicast
  if (!qEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE").isEmpty() ||
      !qEnvironmentVariable("CYCLONEDDS_URI").isEmpty()) {
    return DiscoveryType::Unicast;
  }

  return DiscoveryType::Multicast;
}

void NetworkTopologyManager::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(kSettingsGroup);

  autoRefreshEnabled_ = settings.value(kKeyAutoRefresh, false).toBool();
  autoRefreshIntervalSec_ = settings.value(kKeyRefreshInterval, 30).toInt();
  showBandwidth_ = settings.value(kKeyShowBandwidth, true).toBool();
  viewMode_ = settings.value(kKeyViewMode, "graph").toString();

  settings.endGroup();

  // Apply auto-refresh setting
  if (autoRefreshEnabled_) {
    autoRefreshTimer_->start(autoRefreshIntervalSec_ * 1000);
  }
}

void NetworkTopologyManager::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(kSettingsGroup);

  settings.setValue(kKeyAutoRefresh, autoRefreshEnabled_);
  settings.setValue(kKeyRefreshInterval, autoRefreshIntervalSec_);
  settings.setValue(kKeyShowBandwidth, showBandwidth_);
  settings.setValue(kKeyViewMode, viewMode_);

  settings.endGroup();
}

void NetworkTopologyManager::scan() {
  if (scanning_.load()) {
    return;  // Already scanning
  }

  scanning_ = true;
  topology_.clear();
  discoveredParticipants_.clear();
  pendingNodeInfoQueries_.clear();

  emit scanStarted();
  emit scanProgress(0, tr("Starting network topology scan..."));

  // Get environment info
  topology_.domainId = getCurrentDomainId();
  topology_.rmwImplementation = detectRmwImplementation();
  topology_.discoveryType = detectDiscoveryType();

  emit scanProgress(10, tr("Discovering nodes..."));

  // Start by getting node list
  if (!nodeListProcess_) {
    nodeListProcess_ = new QProcess(this);
    connect(nodeListProcess_,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &NetworkTopologyManager::onNodeListProcessFinished);
  }

  nodeListProcess_->start("ros2", QStringList() << "node" << "list");
}

void NetworkTopologyManager::stopScan() {
  scanning_ = false;

  if (nodeListProcess_ && nodeListProcess_->state() != QProcess::NotRunning) {
    nodeListProcess_->kill();
    nodeListProcess_->waitForFinished(1000);
  }

  if (nodeInfoProcess_ && nodeInfoProcess_->state() != QProcess::NotRunning) {
    nodeInfoProcess_->kill();
    nodeInfoProcess_->waitForFinished(1000);
  }
}

void NetworkTopologyManager::setAutoRefreshEnabled(bool enabled) {
  if (autoRefreshEnabled_ != enabled) {
    autoRefreshEnabled_ = enabled;

    if (enabled) {
      autoRefreshTimer_->start(autoRefreshIntervalSec_ * 1000);
    } else {
      autoRefreshTimer_->stop();
    }

    saveSettings();
    emit settingsChanged();
  }
}

void NetworkTopologyManager::setAutoRefreshInterval(int seconds) {
  seconds = qBound(10, seconds, 300);

  if (autoRefreshIntervalSec_ != seconds) {
    autoRefreshIntervalSec_ = seconds;

    if (autoRefreshEnabled_) {
      autoRefreshTimer_->setInterval(autoRefreshIntervalSec_ * 1000);
    }

    saveSettings();
    emit settingsChanged();
  }
}

void NetworkTopologyManager::setShowBandwidth(bool show) {
  if (showBandwidth_ != show) {
    showBandwidth_ = show;
    saveSettings();
    emit settingsChanged();
  }
}

void NetworkTopologyManager::setViewMode(const QString& mode) {
  if (viewMode_ != mode) {
    viewMode_ = mode;
    saveSettings();
    emit settingsChanged();
  }
}

void NetworkTopologyManager::onAutoRefreshTimer() {
  if (!scanning_.load()) {
    scan();
  }
}

void NetworkTopologyManager::onNodeListProcessFinished(int exitCode,
                                                        QProcess::ExitStatus exitStatus) {
  if (exitStatus != QProcess::NormalExit || exitCode != 0) {
    scanning_ = false;
    emit scanFailed(tr("Failed to get node list. Is ROS2 environment sourced?"));
    return;
  }

  QString output = QString::fromUtf8(nodeListProcess_->readAllStandardOutput());
  parseNodeList(output);

  emit scanProgress(30, tr("Found %1 nodes. Getting node details...")
                        .arg(pendingNodeInfoQueries_.size()));

  // Start querying node info for each node
  if (!pendingNodeInfoQueries_.isEmpty()) {
    if (!nodeInfoProcess_) {
      nodeInfoProcess_ = new QProcess(this);
      connect(nodeInfoProcess_,
              QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
              this, &NetworkTopologyManager::onNodeInfoProcessFinished);
    }

    QString nodeName = pendingNodeInfoQueries_.first();
    nodeInfoProcess_->start("ros2", QStringList() << "node" << "info" << nodeName);
  } else {
    // No nodes found, finalize
    finalizeScan();
  }
}

void NetworkTopologyManager::onNodeInfoProcessFinished(int exitCode,
                                                        QProcess::ExitStatus exitStatus) {
  QString nodeName = pendingNodeInfoQueries_.takeFirst();

  if (exitStatus == QProcess::NormalExit && exitCode == 0) {
    QString output = QString::fromUtf8(nodeInfoProcess_->readAllStandardOutput());
    parseNodeInfo(nodeName, output);
  }

  int total = discoveredParticipants_.size() + pendingNodeInfoQueries_.size();
  int done = discoveredParticipants_.size();
  int progress = 30 + (done * 50 / qMax(1, total));

  emit scanProgress(progress, tr("Processing node %1 of %2...")
                              .arg(done).arg(total));

  // Continue with next node
  if (!pendingNodeInfoQueries_.isEmpty()) {
    QString nextNode = pendingNodeInfoQueries_.first();
    nodeInfoProcess_->start("ros2", QStringList() << "node" << "info" << nextNode);
  } else {
    // All nodes processed
    finalizeScan();
  }
}

void NetworkTopologyManager::parseNodeList(const QString& output) {
  QStringList lines = output.split('\n', Qt::SkipEmptyParts);

  for (const QString& line : lines) {
    QString nodeName = line.trimmed();
    if (!nodeName.isEmpty() && nodeName.startsWith('/')) {
      // Skip our own nodes
      if (!nodeName.contains("ros_weaver")) {
        pendingNodeInfoQueries_.append(nodeName);
      }
    }
  }
}

void NetworkTopologyManager::parseNodeInfo(const QString& nodeName,
                                           const QString& output) {
  ParticipantInfo info;
  info.nodeName = nodeName;

  // Parse sections from ros2 node info output
  QStringList lines = output.split('\n');
  enum class Section { None, Subscribers, Publishers, Services, Clients };
  Section currentSection = Section::None;

  for (const QString& line : lines) {
    QString trimmed = line.trimmed();
    if (trimmed.isEmpty()) {
      continue;
    }

    // Detect section headers
    if (trimmed.startsWith("Subscribers:")) {
      currentSection = Section::Subscribers;
    } else if (trimmed.startsWith("Publishers:")) {
      currentSection = Section::Publishers;
    } else if (trimmed.startsWith("Service Servers:")) {
      currentSection = Section::Services;
    } else if (trimmed.startsWith("Service Clients:")) {
      currentSection = Section::Clients;
    } else if (trimmed.startsWith("Action Servers:") ||
               trimmed.startsWith("Action Clients:")) {
      currentSection = Section::None;
    } else if (trimmed.startsWith('/')) {
      // Extract topic/service name (before the colon if present)
      int colonPos = trimmed.indexOf(':');
      QString name = colonPos > 0 ? trimmed.left(colonPos).trimmed() : trimmed;

      switch (currentSection) {
        case Section::Subscribers:
          info.subscribers.append(name);
          break;
        case Section::Publishers:
          info.publishers.append(name);
          break;
        case Section::Services:
          info.services.append(name);
          break;
        case Section::Clients:
          info.clients.append(name);
          break;
        case Section::None:
          break;
      }
    }
  }

  // For now, assume all nodes are on localhost
  // In a real multi-host setup, you'd need to parse DDS participant info
  info.hostAddress = QStringLiteral("127.0.0.1");
  info.hostname = QHostInfo::localHostName();

  discoveredParticipants_[nodeName] = info;
}

void NetworkTopologyManager::buildHostMap() {
  QMap<QString, HostInfo> hostMap;

  // Get local addresses for comparison
  QSet<QString> localAddresses;
  for (const QHostAddress& addr : QNetworkInterface::allAddresses()) {
    if (addr.protocol() == QAbstractSocket::IPv4Protocol) {
      localAddresses.insert(addr.toString());
    }
  }
  localAddresses.insert(QStringLiteral("127.0.0.1"));
  localAddresses.insert(QStringLiteral("localhost"));

  // Group participants by host
  for (const ParticipantInfo& participant : discoveredParticipants_) {
    QString hostKey = participant.hostAddress.isEmpty()
                          ? QStringLiteral("localhost")
                          : participant.hostAddress;

    if (!hostMap.contains(hostKey)) {
      HostInfo host;
      host.ipAddress = hostKey;
      host.hostname = participant.hostname.isEmpty() ? hostKey : participant.hostname;
      host.isLocal = localAddresses.contains(hostKey) ||
                     localAddresses.contains(host.hostname);
      host.lastSeen = QDateTime::currentMSecsSinceEpoch();
      hostMap[hostKey] = host;
    }

    hostMap[hostKey].nodeNames.append(participant.nodeName);
    hostMap[hostKey].nodeCount = hostMap[hostKey].nodeNames.size();
  }

  topology_.hosts = hostMap.values();

  for (const HostInfo& host : topology_.hosts) {
    emit hostDiscovered(host);
  }
}

void NetworkTopologyManager::buildHostConnections() {
  // Build connections based on topics that span multiple hosts
  // For now, with single-host detection, we'll show connections
  // between nodes on the same host via shared topics

  QMap<QString, HostConnection> connectionMap;

  // For each topic, find publisher/subscriber pairs across hosts
  for (const ParticipantInfo& pub : discoveredParticipants_) {
    for (const QString& topic : pub.publishers) {
      for (const ParticipantInfo& sub : discoveredParticipants_) {
        if (pub.nodeName == sub.nodeName) {
          continue;  // Skip self-connections
        }

        if (sub.subscribers.contains(topic)) {
          QString connKey = pub.hostAddress + "->" + sub.hostAddress;

          if (!connectionMap.contains(connKey)) {
            HostConnection conn;
            conn.sourceHost = pub.hostAddress;
            conn.targetHost = sub.hostAddress;
            connectionMap[connKey] = conn;
          }

          if (!connectionMap[connKey].topics.contains(topic)) {
            connectionMap[connKey].topics.append(topic);
            connectionMap[connKey].connectionCount++;
          }
        }
      }
    }
  }

  topology_.connections = connectionMap.values();
}

void NetworkTopologyManager::finalizeScan() {
  emit scanProgress(85, tr("Building host map..."));

  // Convert participants to topology
  topology_.participants = discoveredParticipants_.values();

  // Build host groupings
  buildHostMap();

  emit scanProgress(95, tr("Building connections..."));

  // Build inter-host connections
  buildHostConnections();

  // Set timestamp
  topology_.timestamp = QDateTime::currentMSecsSinceEpoch();

  scanning_ = false;

  emit scanProgress(100, tr("Scan complete"));
  emit scanCompleted(topology_);
  emit topologyChanged();
}

}  // namespace ros_weaver
