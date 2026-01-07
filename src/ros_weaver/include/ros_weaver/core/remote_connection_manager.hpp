#ifndef ROS_WEAVER_CORE_REMOTE_CONNECTION_MANAGER_HPP
#define ROS_WEAVER_CORE_REMOTE_CONNECTION_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QList>
#include <QProcess>
#include <QSettings>

namespace ros_weaver {

/**
 * @brief Robot connection profile
 */
struct RobotProfile {
  QString name;               // Profile name
  QString hostname;           // Robot IP or hostname
  int port = 22;              // SSH port
  QString username;           // SSH username
  QString sshKeyPath;         // Path to SSH private key
  int rosDomainId = 0;        // ROS_DOMAIN_ID
  QString rosDistro;          // ROS distro (humble, jazzy, etc.)
  bool useSshTunnel = true;   // Use SSH tunnel for connection
  QStringList environmentVariables;  // Additional env vars

  bool isValid() const {
    return !name.isEmpty() && !hostname.isEmpty() && !username.isEmpty();
  }
};

/**
 * @brief SSH tunnel wrapper for remote ROS2 connections
 */
class SshTunnel : public QObject {
  Q_OBJECT

public:
  explicit SshTunnel(QObject* parent = nullptr);
  ~SshTunnel() override;

  /**
   * @brief Establish an SSH tunnel
   */
  void establish(const QString& host, int port, const QString& user,
                 const QString& keyPath, const QStringList& portMappings);

  /**
   * @brief Close the tunnel
   */
  void close();

  /**
   * @brief Check if tunnel is active
   */
  bool isActive() const;

signals:
  void tunnelEstablished();
  void tunnelError(const QString& error);
  void tunnelClosed();

private slots:
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onProcessError(QProcess::ProcessError error);
  void onReadyReadStandardError();

private:
  QProcess* sshProcess_;
  bool active_;
};

/**
 * @brief Manager for remote robot connections
 *
 * Features:
 * - Save/load robot connection profiles
 * - Establish SSH tunnels for remote ROS2 discovery
 * - Configure DDS for remote communication
 * - Manage connection state
 */
class RemoteConnectionManager : public QObject {
  Q_OBJECT

public:
  explicit RemoteConnectionManager(QObject* parent = nullptr);
  ~RemoteConnectionManager() override;

  // Profile management
  void saveProfile(const RobotProfile& profile);
  void deleteProfile(const QString& name);
  QList<RobotProfile> loadProfiles();
  RobotProfile getProfile(const QString& name) const;

  // Connection
  void connectToRobot(const RobotProfile& profile);
  void disconnect();
  bool isConnected() const;
  QString connectedRobotName() const;

  // Get current profile
  RobotProfile currentProfile() const { return currentProfile_; }

  // Test connection
  void testConnection(const RobotProfile& profile);

  // Get FastDDS config for remote discovery
  QString generateFastDdsConfig(const QString& remoteHost);

signals:
  void connectionEstablished(const QString& robotName);
  void connectionFailed(const QString& error);
  void connectionLost();
  void connectionTestResult(bool success, const QString& message);
  void profilesChanged();

private:
  void setupDdsEnvironment(const RobotProfile& profile);
  void restoreDdsEnvironment();
  void startDiscovery();

  SshTunnel* sshTunnel_;
  RobotProfile currentProfile_;
  bool connected_;

  // Saved environment variables
  QString savedDomainId_;
  QString savedFastDdsConfig_;

  // Settings for profile storage
  QSettings settings_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_REMOTE_CONNECTION_MANAGER_HPP
