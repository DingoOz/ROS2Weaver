#include "ros_weaver/core/remote_connection_manager.hpp"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QTimer>

#include <iostream>
#include <cstdlib>

namespace ros_weaver {

// SshTunnel implementation

SshTunnel::SshTunnel(QObject* parent)
  : QObject(parent)
  , sshProcess_(nullptr)
  , active_(false)
{
}

SshTunnel::~SshTunnel() {
  close();
}

void SshTunnel::establish(const QString& host, int port, const QString& user,
                           const QString& keyPath, const QStringList& portMappings) {
  if (sshProcess_) {
    close();
  }

  sshProcess_ = new QProcess(this);

  connect(sshProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &SshTunnel::onProcessFinished);
  connect(sshProcess_, &QProcess::errorOccurred,
          this, &SshTunnel::onProcessError);
  connect(sshProcess_, &QProcess::readyReadStandardError,
          this, &SshTunnel::onReadyReadStandardError);

  QStringList args;
  args << "-N";  // Don't execute remote command
  args << "-T";  // Disable pseudo-terminal allocation

  // Add port forwarding
  for (const QString& mapping : portMappings) {
    args << "-L" << mapping;
  }

  // SSH key
  if (!keyPath.isEmpty()) {
    args << "-i" << keyPath;
  }

  // Port
  args << "-p" << QString::number(port);

  // User@host
  args << QString("%1@%2").arg(user, host);

  std::cerr << "SshTunnel: Starting ssh with args: "
            << args.join(" ").toStdString() << std::endl;

  sshProcess_->start("ssh", args);

  if (!sshProcess_->waitForStarted(5000)) {
    emit tunnelError(tr("Failed to start SSH process"));
    return;
  }

  // Give it a moment to establish
  QTimer::singleShot(2000, this, [this]() {
    if (sshProcess_ && sshProcess_->state() == QProcess::Running) {
      active_ = true;
      emit tunnelEstablished();
    }
  });
}

void SshTunnel::close() {
  if (sshProcess_) {
    sshProcess_->terminate();
    if (!sshProcess_->waitForFinished(3000)) {
      sshProcess_->kill();
    }
    delete sshProcess_;
    sshProcess_ = nullptr;
  }
  active_ = false;
  emit tunnelClosed();
}

bool SshTunnel::isActive() const {
  return active_ && sshProcess_ && sshProcess_->state() == QProcess::Running;
}

void SshTunnel::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  Q_UNUSED(exitStatus);
  active_ = false;

  if (exitCode != 0) {
    emit tunnelError(tr("SSH tunnel closed with exit code %1").arg(exitCode));
  }
  emit tunnelClosed();
}

void SshTunnel::onProcessError(QProcess::ProcessError error) {
  QString errorMsg;
  switch (error) {
    case QProcess::FailedToStart:
      errorMsg = tr("SSH process failed to start");
      break;
    case QProcess::Crashed:
      errorMsg = tr("SSH process crashed");
      break;
    case QProcess::Timedout:
      errorMsg = tr("SSH connection timed out");
      break;
    default:
      errorMsg = tr("SSH error occurred");
  }

  active_ = false;
  emit tunnelError(errorMsg);
}

void SshTunnel::onReadyReadStandardError() {
  if (sshProcess_) {
    QString error = QString::fromUtf8(sshProcess_->readAllStandardError());
    std::cerr << "SSH: " << error.toStdString() << std::endl;

    if (error.contains("Permission denied") ||
        error.contains("Connection refused") ||
        error.contains("Host key verification failed")) {
      emit tunnelError(error.trimmed());
    }
  }
}

// RemoteConnectionManager implementation

RemoteConnectionManager::RemoteConnectionManager(QObject* parent)
  : QObject(parent)
  , sshTunnel_(nullptr)
  , connected_(false)
  , settings_("ROS2Weaver", "RemoteConnections")
{
  sshTunnel_ = new SshTunnel(this);

  connect(sshTunnel_, &SshTunnel::tunnelEstablished, this, [this]() {
    connected_ = true;
    emit connectionEstablished(currentProfile_.name);
  });

  connect(sshTunnel_, &SshTunnel::tunnelError, this, [this](const QString& error) {
    connected_ = false;
    restoreDdsEnvironment();
    emit connectionFailed(error);
  });

  connect(sshTunnel_, &SshTunnel::tunnelClosed, this, [this]() {
    if (connected_) {
      connected_ = false;
      restoreDdsEnvironment();
      emit connectionLost();
    }
  });
}

RemoteConnectionManager::~RemoteConnectionManager() {
  disconnect();
}

void RemoteConnectionManager::saveProfile(const RobotProfile& profile) {
  settings_.beginGroup("Profiles");
  settings_.beginGroup(profile.name);

  settings_.setValue("hostname", profile.hostname);
  settings_.setValue("port", profile.port);
  settings_.setValue("username", profile.username);
  settings_.setValue("sshKeyPath", profile.sshKeyPath);
  settings_.setValue("rosDomainId", profile.rosDomainId);
  settings_.setValue("rosDistro", profile.rosDistro);
  settings_.setValue("useSshTunnel", profile.useSshTunnel);
  settings_.setValue("environmentVariables", profile.environmentVariables);

  settings_.endGroup();
  settings_.endGroup();
  settings_.sync();

  emit profilesChanged();
}

void RemoteConnectionManager::deleteProfile(const QString& name) {
  settings_.beginGroup("Profiles");
  settings_.remove(name);
  settings_.endGroup();
  settings_.sync();

  emit profilesChanged();
}

QList<RobotProfile> RemoteConnectionManager::loadProfiles() {
  QList<RobotProfile> profiles;

  settings_.beginGroup("Profiles");
  QStringList profileNames = settings_.childGroups();

  for (const QString& name : profileNames) {
    settings_.beginGroup(name);

    RobotProfile profile;
    profile.name = name;
    profile.hostname = settings_.value("hostname").toString();
    profile.port = settings_.value("port", 22).toInt();
    profile.username = settings_.value("username").toString();
    profile.sshKeyPath = settings_.value("sshKeyPath").toString();
    profile.rosDomainId = settings_.value("rosDomainId", 0).toInt();
    profile.rosDistro = settings_.value("rosDistro", "humble").toString();
    profile.useSshTunnel = settings_.value("useSshTunnel", true).toBool();
    profile.environmentVariables = settings_.value("environmentVariables").toStringList();

    profiles.append(profile);
    settings_.endGroup();
  }

  settings_.endGroup();
  return profiles;
}

RobotProfile RemoteConnectionManager::getProfile(const QString& name) const {
  QList<RobotProfile> profiles = const_cast<RemoteConnectionManager*>(this)->loadProfiles();
  for (const RobotProfile& profile : profiles) {
    if (profile.name == name) {
      return profile;
    }
  }
  return RobotProfile();
}

void RemoteConnectionManager::connectToRobot(const RobotProfile& profile) {
  if (connected_) {
    disconnect();
  }

  currentProfile_ = profile;

  // Setup DDS environment for remote discovery
  setupDdsEnvironment(profile);

  if (profile.useSshTunnel) {
    // Setup port forwarding for DDS discovery ports
    QStringList portMappings;
    // Fast-DDS default discovery port
    portMappings << "7400:localhost:7400";
    // Additional DDS ports
    portMappings << "7410:localhost:7410";
    portMappings << "7411:localhost:7411";

    sshTunnel_->establish(profile.hostname, profile.port,
                          profile.username, profile.sshKeyPath,
                          portMappings);
  } else {
    // Direct connection - just setup DDS
    connected_ = true;
    emit connectionEstablished(profile.name);
  }
}

void RemoteConnectionManager::disconnect() {
  if (sshTunnel_->isActive()) {
    sshTunnel_->close();
  }

  restoreDdsEnvironment();
  connected_ = false;
  currentProfile_ = RobotProfile();
}

bool RemoteConnectionManager::isConnected() const {
  return connected_;
}

QString RemoteConnectionManager::connectedRobotName() const {
  return connected_ ? currentProfile_.name : QString();
}

void RemoteConnectionManager::testConnection(const RobotProfile& profile) {
  QProcess* testProcess = new QProcess(this);

  connect(testProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, testProcess](int exitCode, QProcess::ExitStatus status) {
    Q_UNUSED(status);
    bool success = (exitCode == 0);
    QString message = success ? tr("Connection successful!") :
                                tr("Connection failed (exit code %1)").arg(exitCode);

    QString output = QString::fromUtf8(testProcess->readAllStandardError());
    if (!output.isEmpty() && !success) {
      message += "\n" + output.trimmed();
    }

    emit connectionTestResult(success, message);
    testProcess->deleteLater();
  });

  QStringList args;
  args << "-o" << "BatchMode=yes";
  args << "-o" << "ConnectTimeout=5";
  args << "-o" << "StrictHostKeyChecking=accept-new";

  if (!profile.sshKeyPath.isEmpty()) {
    args << "-i" << profile.sshKeyPath;
  }

  args << "-p" << QString::number(profile.port);
  args << QString("%1@%2").arg(profile.username, profile.hostname);
  args << "echo" << "Connection test successful";

  testProcess->start("ssh", args);
}

QString RemoteConnectionManager::generateFastDdsConfig(const QString& remoteHost) {
  return QString(R"(<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="ros_weaver_remote" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>%1</address>
              </udpv4>
            </locator>
          </initialPeersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
)").arg(remoteHost);
}

void RemoteConnectionManager::setupDdsEnvironment(const RobotProfile& profile) {
  // Save current environment
  savedDomainId_ = QString::fromUtf8(qgetenv("ROS_DOMAIN_ID"));
  savedFastDdsConfig_ = QString::fromUtf8(qgetenv("FASTRTPS_DEFAULT_PROFILES_FILE"));

  // Set ROS_DOMAIN_ID
  qputenv("ROS_DOMAIN_ID", QString::number(profile.rosDomainId).toUtf8());

  // Generate and save Fast-DDS config
  QString xmlConfig = generateFastDdsConfig(profile.hostname);
  QString configDir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
  QString configPath = configDir + "/ros_weaver_dds_config.xml";

  QFile file(configPath);
  if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream stream(&file);
    stream << xmlConfig;
    file.close();

    qputenv("FASTRTPS_DEFAULT_PROFILES_FILE", configPath.toUtf8());
    std::cerr << "RemoteConnectionManager: DDS config written to "
              << configPath.toStdString() << std::endl;
  }

  // Set custom environment variables
  for (const QString& envVar : profile.environmentVariables) {
    QStringList parts = envVar.split('=');
    if (parts.size() == 2) {
      qputenv(parts[0].toUtf8(), parts[1].toUtf8());
    }
  }
}

void RemoteConnectionManager::restoreDdsEnvironment() {
  if (!savedDomainId_.isEmpty()) {
    qputenv("ROS_DOMAIN_ID", savedDomainId_.toUtf8());
  } else {
    qunsetenv("ROS_DOMAIN_ID");
  }

  if (!savedFastDdsConfig_.isEmpty()) {
    qputenv("FASTRTPS_DEFAULT_PROFILES_FILE", savedFastDdsConfig_.toUtf8());
  } else {
    qunsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
  }

  savedDomainId_.clear();
  savedFastDdsConfig_.clear();
}

}  // namespace ros_weaver
