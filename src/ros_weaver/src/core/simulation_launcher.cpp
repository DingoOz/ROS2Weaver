#include "ros_weaver/core/simulation_launcher.hpp"

#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include <QPointF>
#include <QProcessEnvironment>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QtMath>

namespace ros_weaver {

SimulationLauncher& SimulationLauncher::instance() {
  static SimulationLauncher instance;
  return instance;
}

SimulationLauncher::SimulationLauncher()
    : QObject(nullptr)
    , gazeboAvailable_(false)
    , ignitionAvailable_(false)
    , simProcess_(nullptr)
    , bridgeProcess_(nullptr)
    , statusTimer_(new QTimer(this))
{
  detectSimulators();

  connect(statusTimer_, &QTimer::timeout, this, &SimulationLauncher::updateStatus);
}

SimulationLauncher::~SimulationLauncher() {
  stop();
}

void SimulationLauncher::detectSimulators() {
  // Check for Gazebo (gz sim)
  QProcess gazeboCheck;
  gazeboCheck.start("gz", {"sim", "--version"});
  if (gazeboCheck.waitForFinished(5000)) {
    if (gazeboCheck.exitCode() == 0) {
      ignitionAvailable_ = true;
      ignitionVersion_ = QString::fromUtf8(gazeboCheck.readAllStandardOutput()).trimmed();
    }
  }

  // Check for classic Gazebo
  QProcess classicCheck;
  classicCheck.start("gazebo", {"--version"});
  if (classicCheck.waitForFinished(5000)) {
    if (classicCheck.exitCode() == 0) {
      gazeboAvailable_ = true;
      gazeboVersion_ = QString::fromUtf8(classicCheck.readAllStandardOutput()).trimmed();
    }
  }

  // Also check gzserver for headless
  if (!gazeboAvailable_) {
    QProcess gzserverCheck;
    gzserverCheck.start("gzserver", {"--version"});
    if (gzserverCheck.waitForFinished(5000)) {
      if (gzserverCheck.exitCode() == 0) {
        gazeboAvailable_ = true;
        gazeboVersion_ = QString::fromUtf8(gzserverCheck.readAllStandardOutput()).trimmed();
      }
    }
  }
}

bool SimulationLauncher::isGazeboAvailable() const {
  return gazeboAvailable_;
}

bool SimulationLauncher::isIgnitionAvailable() const {
  return ignitionAvailable_;
}

SimulatorType SimulationLauncher::detectBestSimulator() const {
  // Prefer Ignition/Gazebo Sim (newer)
  if (ignitionAvailable_) {
    return SimulatorType::Ignition;
  }
  if (gazeboAvailable_) {
    return SimulatorType::Gazebo;
  }
  return SimulatorType::AutoDetect;  // None available
}

QString SimulationLauncher::simulatorVersion(SimulatorType type) const {
  switch (type) {
    case SimulatorType::Gazebo:
      return gazeboVersion_;
    case SimulatorType::Ignition:
      return ignitionVersion_;
    case SimulatorType::AutoDetect:
      return ignitionAvailable_ ? ignitionVersion_ : gazeboVersion_;
  }
  return QString();
}

bool SimulationLauncher::launch(const SimulationConfig& config) {
  if (isRunning()) {
    stop();
  }

  SimulatorType simType = config.simulatorType;
  if (simType == SimulatorType::AutoDetect) {
    simType = detectBestSimulator();
  }

  if (simType == SimulatorType::AutoDetect) {
    emit errorOccurred(tr("No simulator available. Install Gazebo or Ignition."));
    return false;
  }

  // Build launch command
  QStringList args = buildLaunchCommand(config);
  if (args.isEmpty()) {
    emit errorOccurred(tr("Failed to build launch command"));
    return false;
  }

  QString program = args.takeFirst();

  // Create process
  simProcess_ = new QProcess(this);

  connect(simProcess_, &QProcess::started,
          this, &SimulationLauncher::onProcessStarted);
  connect(simProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &SimulationLauncher::onProcessFinished);
  connect(simProcess_, &QProcess::errorOccurred,
          this, &SimulationLauncher::onProcessError);
  connect(simProcess_, &QProcess::readyReadStandardOutput,
          this, &SimulationLauncher::onReadyReadStdout);
  connect(simProcess_, &QProcess::readyReadStandardError,
          this, &SimulationLauncher::onReadyReadStderr);

  // Setup environment
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  setupEnvironment(env, config);
  simProcess_->setProcessEnvironment(env);

  currentConfig_ = config;

  // Start simulation
  simProcess_->start(program, args);

  if (!simProcess_->waitForStarted(10000)) {
    emit errorOccurred(tr("Failed to start simulator: %1").arg(simProcess_->errorString()));
    delete simProcess_;
    simProcess_ = nullptr;
    return false;
  }

  // Start ros_gz_bridge if requested
  if (config.useBridgeNode && !config.bridgeTopics.isEmpty()) {
    startBridgeNode(config);
  }

  return true;
}

void SimulationLauncher::stop() {
  stopBridgeNode();

  if (simProcess_) {
    simProcess_->terminate();
    if (!simProcess_->waitForFinished(5000)) {
      simProcess_->kill();
      simProcess_->waitForFinished(2000);
    }
    delete simProcess_;
    simProcess_ = nullptr;
  }

  statusTimer_->stop();
  currentStatus_ = SimulationStatus();
  spawnedModels_.clear();

  emit simulationStopped();
}

void SimulationLauncher::pause() {
  if (!isRunning()) return;

  // Send pause command via service
  QProcess pauseCmd;
  if (currentConfig_.simulatorType == SimulatorType::Ignition ||
      detectBestSimulator() == SimulatorType::Ignition) {
    pauseCmd.start("gz", {"service", "-s", "/world/default/control", "--reqtype",
                          "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                          "--req", "pause: true"});
  } else {
    pauseCmd.start("gz", {"world", "-p", "1"});
  }
  pauseCmd.waitForFinished(3000);

  currentStatus_.isPaused = true;
  emit simulationPaused();
}

void SimulationLauncher::resume() {
  if (!isRunning()) return;

  QProcess resumeCmd;
  if (currentConfig_.simulatorType == SimulatorType::Ignition ||
      detectBestSimulator() == SimulatorType::Ignition) {
    resumeCmd.start("gz", {"service", "-s", "/world/default/control", "--reqtype",
                           "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                           "--req", "pause: false"});
  } else {
    resumeCmd.start("gz", {"world", "-p", "0"});
  }
  resumeCmd.waitForFinished(3000);

  currentStatus_.isPaused = false;
  emit simulationResumed();
}

void SimulationLauncher::reset() {
  if (!isRunning()) return;

  QProcess resetCmd;
  if (currentConfig_.simulatorType == SimulatorType::Ignition ||
      detectBestSimulator() == SimulatorType::Ignition) {
    resetCmd.start("gz", {"service", "-s", "/world/default/control", "--reqtype",
                          "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                          "--req", "reset: {all: true}"});
  } else {
    resetCmd.start("gz", {"world", "-r"});
  }
  resetCmd.waitForFinished(3000);

  currentStatus_.simTime = 0.0;
  emit simulationReset();
}

SimulationStatus SimulationLauncher::status() const {
  return currentStatus_;
}

bool SimulationLauncher::isRunning() const {
  return simProcess_ != nullptr && simProcess_->state() == QProcess::Running;
}

QString SimulationLauncher::currentWorldFile() const {
  return currentConfig_.worldFile;
}

bool SimulationLauncher::spawnModel(const QString& name, const QString& sdfPath,
                                     const QPointF& pos, double z, double yaw) {
  if (!isRunning()) return false;

  QString modelPath = findRobotModel(sdfPath);
  if (modelPath.isEmpty()) {
    modelPath = sdfPath;
  }

  QProcess spawnCmd;
  QStringList args;

  if (currentConfig_.simulatorType == SimulatorType::Ignition ||
      detectBestSimulator() == SimulatorType::Ignition) {
    args << "service" << "-s" << "/world/default/create"
         << "--reqtype" << "gz.msgs.EntityFactory"
         << "--reptype" << "gz.msgs.Boolean"
         << "--req" << QString("sdf_filename: '%1', name: '%2', "
                                "pose: {position: {x: %3, y: %4, z: %5}, "
                                "orientation: {z: %6}}")
                         .arg(modelPath, name)
                         .arg(pos.x()).arg(pos.y()).arg(z)
                         .arg(qSin(yaw / 2));
    spawnCmd.start("gz", args);
  } else {
    args << "model" << "-m" << name << "-f" << modelPath
         << "-x" << QString::number(pos.x())
         << "-y" << QString::number(pos.y())
         << "-z" << QString::number(z)
         << "-Y" << QString::number(yaw);
    spawnCmd.start("gz", args);
  }

  if (spawnCmd.waitForFinished(10000) && spawnCmd.exitCode() == 0) {
    spawnedModels_.append(name);
    emit modelSpawned(name);
    return true;
  }

  emit errorOccurred(tr("Failed to spawn model '%1'").arg(name));
  return false;
}

bool SimulationLauncher::deleteModel(const QString& name) {
  if (!isRunning()) return false;

  QProcess deleteCmd;

  if (currentConfig_.simulatorType == SimulatorType::Ignition ||
      detectBestSimulator() == SimulatorType::Ignition) {
    deleteCmd.start("gz", {"service", "-s", "/world/default/remove",
                           "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                           "--req", QString("name: '%1', type: MODEL").arg(name)});
  } else {
    deleteCmd.start("gz", {"model", "-d", "-m", name});
  }

  if (deleteCmd.waitForFinished(5000) && deleteCmd.exitCode() == 0) {
    spawnedModels_.removeAll(name);
    emit modelDeleted(name);
    return true;
  }

  return false;
}

QStringList SimulationLauncher::spawnedModels() const {
  return spawnedModels_;
}

QStringList SimulationLauncher::availableWorlds() const {
  QStringList worlds;

  // Check common world locations
  QStringList searchPaths = {
    "/usr/share/gazebo/worlds",
    "/usr/share/gz/worlds",
    QDir::homePath() + "/.gazebo/worlds",
    QDir::homePath() + "/.gz/worlds"
  };

  // Add paths from environment
  QString gazeboResourcePath = qEnvironmentVariable("GAZEBO_RESOURCE_PATH");
  if (!gazeboResourcePath.isEmpty()) {
    searchPaths << gazeboResourcePath.split(':');
  }

  QString gzSimResourcePath = qEnvironmentVariable("GZ_SIM_RESOURCE_PATH");
  if (!gzSimResourcePath.isEmpty()) {
    searchPaths << gzSimResourcePath.split(':');
  }

  for (const QString& path : searchPaths) {
    QDir dir(path);
    if (dir.exists()) {
      QStringList filters = {"*.world", "*.sdf"};
      for (const QString& file : dir.entryList(filters, QDir::Files)) {
        worlds.append(dir.filePath(file));
      }
    }
  }

  return worlds;
}

QStringList SimulationLauncher::availableRobotModels() const {
  QStringList models;

  // Check model paths
  QStringList searchPaths = {
    "/usr/share/gazebo/models",
    "/usr/share/gz/models",
    QDir::homePath() + "/.gazebo/models",
    QDir::homePath() + "/.gz/models"
  };

  QString gazeboModelPath = qEnvironmentVariable("GAZEBO_MODEL_PATH");
  if (!gazeboModelPath.isEmpty()) {
    searchPaths << gazeboModelPath.split(':');
  }

  for (const QString& path : searchPaths) {
    QDir dir(path);
    if (dir.exists()) {
      for (const QString& modelDir : dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        QString modelConfig = dir.filePath(modelDir + "/model.config");
        QString modelSdf = dir.filePath(modelDir + "/model.sdf");
        if (QFileInfo::exists(modelConfig) || QFileInfo::exists(modelSdf)) {
          models.append(modelDir);
        }
      }
    }
  }

  return models;
}

bool SimulationLauncher::launchEmptyWorld(SimulatorType type) {
  SimulationConfig config;
  config.simulatorType = type;
  config.worldFile = "empty.sdf";
  config.paused = false;
  return launch(config);
}

bool SimulationLauncher::launchTurtleBot3World(const QString& worldName) {
  SimulationConfig config = createTurtleBot3Config("waffle");
  config.worldFile = worldName;
  return launch(config);
}

bool SimulationLauncher::launchCustomWorld(const QString& worldPath) {
  SimulationConfig config;
  config.worldFile = worldPath;
  return launch(config);
}

SimulationConfig SimulationLauncher::createDefaultConfig() {
  SimulationConfig config;
  config.simulatorType = SimulatorType::AutoDetect;
  config.worldFile = "empty.sdf";
  config.paused = true;
  config.useBridgeNode = true;
  config.bridgeTopics << "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock";
  return config;
}

SimulationConfig SimulationLauncher::createTurtleBot3Config(const QString& variant) {
  SimulationConfig config = createDefaultConfig();
  config.configName = QString("TurtleBot3 %1").arg(variant);
  config.robotModel = QString("turtlebot3_%1").arg(variant.toLower());
  config.robotNamespace = "tb3";
  config.worldFile = "turtlebot3_world.world";
  config.spawnPosition = QPointF(0, 0);
  config.spawnZ = 0.01;

  // Common TurtleBot3 topics to bridge
  config.bridgeTopics.clear();
  config.bridgeTopics << "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
                      << "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
                      << "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
                      << "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
                      << "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
                      << "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image";

  return config;
}

SimulationConfig SimulationLauncher::createDiffDriveConfig(const QString& robotName) {
  SimulationConfig config = createDefaultConfig();
  config.configName = QString("Differential Drive - %1").arg(robotName);
  config.robotNamespace = robotName.toLower().replace(" ", "_");
  config.worldFile = "empty.sdf";

  config.bridgeTopics.clear();
  config.bridgeTopics << "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
                      << QString("/%1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist").arg(config.robotNamespace)
                      << QString("/%1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry").arg(config.robotNamespace);

  return config;
}

void SimulationLauncher::onProcessStarted() {
  currentStatus_.isRunning = true;
  currentStatus_.isPaused = currentConfig_.paused;
  statusTimer_->start(1000);  // Update status every second
  emit simulationStarted();
}

void SimulationLauncher::onProcessFinished(int exitCode, QProcess::ExitStatus status) {
  Q_UNUSED(exitCode)
  Q_UNUSED(status)
  statusTimer_->stop();
  currentStatus_.isRunning = false;
  emit simulationStopped();
}

void SimulationLauncher::onProcessError(QProcess::ProcessError error) {
  QString errorMsg;
  switch (error) {
    case QProcess::FailedToStart:
      errorMsg = tr("Simulator failed to start");
      break;
    case QProcess::Crashed:
      errorMsg = tr("Simulator crashed");
      break;
    case QProcess::Timedout:
      errorMsg = tr("Simulator operation timed out");
      break;
    default:
      errorMsg = tr("Unknown simulator error");
  }
  emit errorOccurred(errorMsg);
}

void SimulationLauncher::onReadyReadStdout() {
  if (simProcess_) {
    QString output = QString::fromUtf8(simProcess_->readAllStandardOutput());
    emit outputReceived(output);
  }
}

void SimulationLauncher::onReadyReadStderr() {
  if (simProcess_) {
    QString output = QString::fromUtf8(simProcess_->readAllStandardError());
    emit outputReceived(output);
  }
}

void SimulationLauncher::updateStatus() {
  // Query simulation status via gz service (in a real implementation)
  // For now, just emit the current status
  emit statusChanged(currentStatus_);
}

QStringList SimulationLauncher::buildLaunchCommand(const SimulationConfig& config) const {
  QStringList args;

  SimulatorType type = config.simulatorType;
  if (type == SimulatorType::AutoDetect) {
    type = detectBestSimulator();
  }

  if (type == SimulatorType::Ignition) {
    args << "gz" << "sim";

    if (!config.worldFile.isEmpty()) {
      QString worldPath = findWorldFile(config.worldFile);
      args << (worldPath.isEmpty() ? config.worldFile : worldPath);
    }

    if (config.headless) {
      args << "-s";  // Server only
    }

    if (!config.paused) {
      args << "-r";  // Run immediately
    }

  } else if (type == SimulatorType::Gazebo) {
    args << (config.headless ? "gzserver" : "gazebo");

    if (!config.worldFile.isEmpty()) {
      QString worldPath = findWorldFile(config.worldFile);
      args << (worldPath.isEmpty() ? config.worldFile : worldPath);
    }

    if (config.paused) {
      args << "-u";  // Start paused
    }
  }

  return args;
}

void SimulationLauncher::setupEnvironment(QProcessEnvironment& env,
                                           const SimulationConfig& config) const {
  // Add user-specified environment variables
  for (auto it = config.envVars.constBegin(); it != config.envVars.constEnd(); ++it) {
    env.insert(it.key(), it.value());
  }
}

void SimulationLauncher::startBridgeNode(const SimulationConfig& config) {
  if (bridgeProcess_) {
    stopBridgeNode();
  }

  bridgeProcess_ = new QProcess(this);

  QStringList args = {"run", "ros_gz_bridge", "parameter_bridge"};
  for (const QString& topic : config.bridgeTopics) {
    args << topic;
  }

  bridgeProcess_->start("ros2", args);
}

void SimulationLauncher::stopBridgeNode() {
  if (bridgeProcess_) {
    bridgeProcess_->terminate();
    if (!bridgeProcess_->waitForFinished(3000)) {
      bridgeProcess_->kill();
    }
    delete bridgeProcess_;
    bridgeProcess_ = nullptr;
  }
}

QString SimulationLauncher::findWorldFile(const QString& worldName) const {
  // If it's already an absolute path, return it
  if (QFileInfo(worldName).isAbsolute() && QFileInfo::exists(worldName)) {
    return worldName;
  }

  // Search in common locations
  for (const QString& worldPath : availableWorlds()) {
    if (worldPath.endsWith("/" + worldName) ||
        QFileInfo(worldPath).baseName() == QFileInfo(worldName).baseName()) {
      return worldPath;
    }
  }

  return QString();
}

QString SimulationLauncher::findRobotModel(const QString& modelName) const {
  // If it's already an absolute path, return it
  if (QFileInfo(modelName).isAbsolute() && QFileInfo::exists(modelName)) {
    return modelName;
  }

  // Search in model directories
  QStringList searchPaths = {
    "/usr/share/gazebo/models",
    "/usr/share/gz/models",
    QDir::homePath() + "/.gazebo/models",
    QDir::homePath() + "/.gz/models"
  };

  QString gazeboModelPath = qEnvironmentVariable("GAZEBO_MODEL_PATH");
  if (!gazeboModelPath.isEmpty()) {
    searchPaths << gazeboModelPath.split(':');
  }

  for (const QString& path : searchPaths) {
    QString modelDir = path + "/" + modelName;
    QString modelSdf = modelDir + "/model.sdf";
    if (QFileInfo::exists(modelSdf)) {
      return modelSdf;
    }
  }

  return QString();
}

}  // namespace ros_weaver
