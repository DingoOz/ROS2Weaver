#include "ros_weaver/core/slam_pipeline_manager.hpp"
#include "ros_weaver/core/bag_manager.hpp"
#include "ros_weaver/core/playback_controller.hpp"

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardPaths>
#include <QTimer>
#include <QDebug>

#include <yaml-cpp/yaml.h>

namespace ros_weaver {

SlamPipelineManager::SlamPipelineManager(QObject* parent)
    : QObject(parent) {
  qRegisterMetaType<SlamConfig>("SlamConfig");
  qRegisterMetaType<SlamPreset>("SlamPreset");
  qRegisterMetaType<SlamNodeStatus>("SlamNodeStatus");

  // Default presets directory
  presetsDirectory_ = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)
                      + "/slam_presets";
  QDir().mkpath(presetsDirectory_);

  // Load built-in presets
  loadPresetsFromDirectory();

  // Create parameter check timer
  paramCheckTimer_ = new QTimer(this);
  paramCheckTimer_->setInterval(1000);  // Check every second
  connect(paramCheckTimer_, &QTimer::timeout, this, &SlamPipelineManager::onParameterCheckTimer);
}

SlamPipelineManager::~SlamPipelineManager() {
  stopSlam();
  shutdownRosNode();
}

bool SlamPipelineManager::launchSlam(const SlamConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (status_ == SlamNodeStatus::Running || status_ == SlamNodeStatus::Starting) {
    emit slamError(tr("SLAM is already running"));
    return false;
  }

  // Store configuration
  currentConfig_ = config;
  currentParams_ = config.parameters;

  // Build launch command
  QString command = buildLaunchCommand(config);
  if (command.isEmpty()) {
    emit slamError(tr("Failed to build launch command"));
    return false;
  }

  // Create process
  if (slamProcess_) {
    delete slamProcess_;
  }

  slamProcess_ = new QProcess(this);

  connect(slamProcess_, &QProcess::started,
          this, &SlamPipelineManager::onProcessStarted);
  connect(slamProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &SlamPipelineManager::onProcessFinished);
  connect(slamProcess_, &QProcess::errorOccurred,
          this, &SlamPipelineManager::onProcessError);
  connect(slamProcess_, &QProcess::readyReadStandardOutput,
          this, &SlamPipelineManager::onProcessReadyReadStdout);
  connect(slamProcess_, &QProcess::readyReadStandardError,
          this, &SlamPipelineManager::onProcessReadyReadStderr);

  status_ = SlamNodeStatus::Starting;
  emit statusChanged(status_);

  qDebug() << "SlamPipelineManager: Launching:" << command;
  slamProcess_->start("bash", QStringList() << "-c" << command);

  return true;
}

void SlamPipelineManager::stopSlam() {
  std::lock_guard<std::mutex> lock(mutex_);

  paramCheckTimer_->stop();

  if (slamProcess_ && slamProcess_->state() != QProcess::NotRunning) {
    status_ = SlamNodeStatus::Stopping;
    emit statusChanged(status_);

    // Try graceful termination first
    slamProcess_->terminate();

    if (!slamProcess_->waitForFinished(5000)) {
      // Force kill if not responding
      slamProcess_->kill();
      slamProcess_->waitForFinished(2000);
    }
  }

  // Destroy parameter client
  paramClient_.reset();

  status_ = SlamNodeStatus::NotRunning;
  emit statusChanged(status_);
  emit slamStopped();
}

bool SlamPipelineManager::isRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_ == SlamNodeStatus::Running;
}

SlamNodeStatus SlamPipelineManager::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool SlamPipelineManager::launchSlamToolboxAsync(const QMap<QString, QVariant>& params) {
  SlamConfig config;
  config.packageName = "slam_toolbox";
  config.launchFile = "online_async_launch.py";
  config.mode = "mapping";
  config.parameters = params;

  // Default SLAM Toolbox parameters
  if (!config.parameters.contains("use_sim_time")) {
    config.parameters["use_sim_time"] = true;
  }
  if (!config.parameters.contains("max_laser_range")) {
    config.parameters["max_laser_range"] = 20.0;
  }

  return launchSlam(config);
}

bool SlamPipelineManager::launchSlamToolboxSync(const QMap<QString, QVariant>& params) {
  SlamConfig config;
  config.packageName = "slam_toolbox";
  config.launchFile = "online_sync_launch.py";
  config.mode = "mapping";
  config.parameters = params;

  if (!config.parameters.contains("use_sim_time")) {
    config.parameters["use_sim_time"] = true;
  }

  return launchSlam(config);
}

void SlamPipelineManager::setParameter(const QString& name, const QVariant& value) {
  bool shouldRerun = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    currentParams_[name] = value;

    // If SLAM is running, apply parameter dynamically
    if (status_ == SlamNodeStatus::Running && paramClient_) {
      try {
        rclcpp::Parameter param;

        if (value.type() == QVariant::Bool) {
          param = rclcpp::Parameter(name.toStdString(), value.toBool());
        } else if (value.type() == QVariant::Int) {
          param = rclcpp::Parameter(name.toStdString(), value.toInt());
        } else if (value.type() == QVariant::Double) {
          param = rclcpp::Parameter(name.toStdString(), value.toDouble());
        } else {
          param = rclcpp::Parameter(name.toStdString(), value.toString().toStdString());
        }

        auto future = paramClient_->set_parameters({param});
        // Note: In production, should handle future result asynchronously
      }
      catch (const std::exception& e) {
        qWarning() << "Failed to set parameter" << name << ":" << e.what();
      }
    }

    // Check if we need to trigger rerun (while still holding lock)
    shouldRerun = autoRerunEnabled_;
  }
  // Lock released here

  emit parameterChanged(name, value);

  // Trigger rerun outside the lock to avoid deadlock
  // (triggerRerun() calls stopSlam() which also acquires mutex_)
  if (shouldRerun) {
    triggerRerun();
  }
}

QVariant SlamPipelineManager::getParameter(const QString& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentParams_.value(name);
}

QMap<QString, QVariant> SlamPipelineManager::getAllParameters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentParams_;
}

QStringList SlamPipelineManager::parameterNames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentParams_.keys();
}

void SlamPipelineManager::loadPreset(const QString& presetName) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!presets_.contains(presetName)) {
    emit slamError(tr("Preset not found: %1").arg(presetName));
    return;
  }

  const SlamPreset& preset = presets_[presetName];
  currentConfig_ = preset.config;
  currentParams_ = preset.config.parameters;
  currentPresetName_ = presetName;

  emit presetLoaded(presetName);
  emit parametersUpdated();
}

bool SlamPipelineManager::savePreset(const QString& presetName, const QString& description) {
  std::lock_guard<std::mutex> lock(mutex_);

  SlamPreset preset;
  preset.name = presetName;
  preset.description = description;
  preset.packageName = currentConfig_.packageName;
  preset.config = currentConfig_;
  preset.config.parameters = currentParams_;

  presets_[presetName] = preset;
  savePresetToFile(preset);

  emit presetSaved(presetName);
  return true;
}

void SlamPipelineManager::deletePreset(const QString& presetName) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (presets_.contains(presetName)) {
    presets_.remove(presetName);

    // Delete file
    QString filePath = presetsDirectory_ + "/" + presetName + ".json";
    QFile::remove(filePath);
  }
}

QStringList SlamPipelineManager::availablePresets() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return presets_.keys();
}

SlamPreset SlamPipelineManager::currentPreset() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return presets_.value(currentPresetName_);
}

void SlamPipelineManager::setPresetsDirectory(const QString& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  presetsDirectory_ = path;
  QDir().mkpath(presetsDirectory_);
  loadPresetsFromDirectory();
}

void SlamPipelineManager::enableAutoRerun(bool enable) {
  std::lock_guard<std::mutex> lock(mutex_);
  autoRerunEnabled_ = enable;
}

bool SlamPipelineManager::isAutoRerunEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return autoRerunEnabled_;
}

void SlamPipelineManager::triggerRerun() {
  emit rerunTriggered();

  // Stop current SLAM
  stopSlam();

  // Reset playback if available
  if (playbackController_) {
    playbackController_->stop();
  }

  // Relaunch SLAM with current config
  launchSlam(currentConfig_);

  // Restart playback
  if (playbackController_) {
    QTimer::singleShot(2000, playbackController_, &PlaybackController::play);
  }
}

void SlamPipelineManager::setBagManager(BagManager* manager) {
  bagManager_ = manager;
}

void SlamPipelineManager::setPlaybackController(PlaybackController* controller) {
  playbackController_ = controller;
}

SlamConfig SlamPipelineManager::currentConfig() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return currentConfig_;
}

void SlamPipelineManager::onProcessStarted() {
  qDebug() << "SlamPipelineManager: SLAM process started";

  // Initialize ROS node for parameter client
  initializeRosNode();

  // Wait a bit for the SLAM node to initialize, then create parameter client
  QTimer::singleShot(3000, this, [this]() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (slamProcess_ && slamProcess_->state() == QProcess::Running) {
      status_ = SlamNodeStatus::Running;
      emit statusChanged(status_);
      emit slamStarted();

      // Create parameter client
      slamNodeName_ = "/slam_toolbox";  // Default for SLAM Toolbox
      createParameterClient(slamNodeName_);

      // Start parameter monitoring
      paramCheckTimer_->start();
    }
  });
}

void SlamPipelineManager::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  qDebug() << "SlamPipelineManager: Process finished with code" << exitCode;

  paramCheckTimer_->stop();
  paramClient_.reset();

  status_ = SlamNodeStatus::NotRunning;
  emit statusChanged(status_);

  if (exitStatus == QProcess::CrashExit) {
    emit slamError(tr("SLAM process crashed"));
  }

  emit slamStopped();
}

void SlamPipelineManager::onProcessError(QProcess::ProcessError error) {
  QString errorStr;
  switch (error) {
    case QProcess::FailedToStart:
      errorStr = tr("Failed to start SLAM process");
      break;
    case QProcess::Crashed:
      errorStr = tr("SLAM process crashed");
      break;
    case QProcess::Timedout:
      errorStr = tr("SLAM process timed out");
      break;
    default:
      errorStr = tr("SLAM process error");
  }

  status_ = SlamNodeStatus::Error;
  emit statusChanged(status_);
  emit slamError(errorStr);
}

void SlamPipelineManager::onProcessReadyReadStdout() {
  if (slamProcess_) {
    QString output = slamProcess_->readAllStandardOutput();
    emit outputReceived(output);
  }
}

void SlamPipelineManager::onProcessReadyReadStderr() {
  if (slamProcess_) {
    QString output = slamProcess_->readAllStandardError();
    emit outputReceived(output);
  }
}

void SlamPipelineManager::onParameterCheckTimer() {
  // Periodically check if SLAM node is still responsive
  if (!checkSlamNodeRunning()) {
    status_ = SlamNodeStatus::Error;
    emit statusChanged(status_);
    emit slamError(tr("SLAM node is not responding"));
  }
}

void SlamPipelineManager::initializeRosNode() {
  if (node_) return;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("ros_weaver_slam_manager");

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }
  catch (const std::exception& e) {
    emit slamError(tr("Failed to initialize ROS node: %1").arg(e.what()));
  }
}

void SlamPipelineManager::shutdownRosNode() {
  spinning_ = false;

  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();

  paramClient_.reset();
  node_.reset();
}

void SlamPipelineManager::createParameterClient(const QString& nodeName) {
  if (!node_) return;

  try {
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(
        node_, nodeName.toStdString());

    // Wait for service
    if (!paramClient_->wait_for_service(std::chrono::seconds(5))) {
      qWarning() << "Parameter service not available for" << nodeName;
    }
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to create parameter client:" << e.what();
  }
}

void SlamPipelineManager::loadPresetsFromDirectory() {
  presets_.clear();

  // Add built-in presets
  {
    SlamPreset indoor;
    indoor.name = "SLAM Toolbox Indoor";
    indoor.description = "Optimized for indoor mapping with short-range lidar";
    indoor.packageName = "slam_toolbox";
    indoor.config.packageName = "slam_toolbox";
    indoor.config.launchFile = "online_async_launch.py";
    indoor.config.mode = "mapping";
    indoor.config.parameters["use_sim_time"] = true;
    indoor.config.parameters["max_laser_range"] = 12.0;
    indoor.config.parameters["minimum_travel_distance"] = 0.3;
    indoor.config.parameters["minimum_travel_heading"] = 0.3;
    indoor.config.parameters["scan_buffer_size"] = 10;
    indoor.config.parameters["resolution"] = 0.05;
    presets_[indoor.name] = indoor;
  }

  {
    SlamPreset outdoor;
    outdoor.name = "SLAM Toolbox Outdoor";
    outdoor.description = "Optimized for outdoor mapping with long-range lidar";
    outdoor.packageName = "slam_toolbox";
    outdoor.config.packageName = "slam_toolbox";
    outdoor.config.launchFile = "online_async_launch.py";
    outdoor.config.mode = "mapping";
    outdoor.config.parameters["use_sim_time"] = true;
    outdoor.config.parameters["max_laser_range"] = 40.0;
    outdoor.config.parameters["minimum_travel_distance"] = 0.5;
    outdoor.config.parameters["minimum_travel_heading"] = 0.5;
    outdoor.config.parameters["scan_buffer_size"] = 5;
    outdoor.config.parameters["resolution"] = 0.1;
    presets_[outdoor.name] = outdoor;
  }

  // Load user presets from directory
  QDir presetsDir(presetsDirectory_);
  QStringList filters;
  filters << "*.json";

  for (const QString& fileName : presetsDir.entryList(filters, QDir::Files)) {
    QFile file(presetsDir.absoluteFilePath(fileName));
    if (file.open(QIODevice::ReadOnly)) {
      QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
      QJsonObject obj = doc.object();

      SlamPreset preset;
      preset.name = obj["name"].toString();
      preset.description = obj["description"].toString();
      preset.packageName = obj["package_name"].toString();
      preset.config.packageName = preset.packageName;
      preset.config.launchFile = obj["launch_file"].toString();
      preset.config.mode = obj["mode"].toString();
      preset.config.mapFrame = obj["map_frame"].toString("map");
      preset.config.odomFrame = obj["odom_frame"].toString("odom");
      preset.config.baseFrame = obj["base_frame"].toString("base_footprint");
      preset.config.scanTopic = obj["scan_topic"].toString("/scan");

      QJsonObject params = obj["parameters"].toObject();
      for (const QString& key : params.keys()) {
        QJsonValue val = params[key];
        if (val.isBool()) {
          preset.config.parameters[key] = val.toBool();
        } else if (val.isDouble()) {
          preset.config.parameters[key] = val.toDouble();
        } else if (val.isString()) {
          preset.config.parameters[key] = val.toString();
        }
      }

      presets_[preset.name] = preset;
    }
  }
}

void SlamPipelineManager::savePresetToFile(const SlamPreset& preset) {
  QJsonObject obj;
  obj["name"] = preset.name;
  obj["description"] = preset.description;
  obj["package_name"] = preset.packageName;
  obj["launch_file"] = preset.config.launchFile;
  obj["mode"] = preset.config.mode;
  obj["map_frame"] = preset.config.mapFrame;
  obj["odom_frame"] = preset.config.odomFrame;
  obj["base_frame"] = preset.config.baseFrame;
  obj["scan_topic"] = preset.config.scanTopic;

  QJsonObject params;
  for (auto it = preset.config.parameters.constBegin();
       it != preset.config.parameters.constEnd(); ++it) {
    const QVariant& val = it.value();
    if (val.type() == QVariant::Bool) {
      params[it.key()] = val.toBool();
    } else if (val.type() == QVariant::Int || val.type() == QVariant::Double) {
      params[it.key()] = val.toDouble();
    } else {
      params[it.key()] = val.toString();
    }
  }
  obj["parameters"] = params;

  QString filePath = presetsDirectory_ + "/" + preset.name + ".json";
  QFile file(filePath);
  if (file.open(QIODevice::WriteOnly)) {
    file.write(QJsonDocument(obj).toJson());
  }
}

bool SlamPipelineManager::checkSlamNodeRunning() {
  // Simple check: verify process is still running
  return slamProcess_ && slamProcess_->state() == QProcess::Running;
}

QString SlamPipelineManager::buildLaunchCommand(const SlamConfig& config) const {
  // Build ros2 launch command
  QStringList args;
  args << "ros2" << "launch"
       << config.packageName
       << config.launchFile;

  // Add parameters as launch arguments
  for (auto it = config.parameters.constBegin(); it != config.parameters.constEnd(); ++it) {
    QString paramArg = QString("%1:=%2").arg(it.key());

    const QVariant& val = it.value();
    if (val.type() == QVariant::Bool) {
      paramArg = paramArg.arg(val.toBool() ? "true" : "false");
    } else if (val.type() == QVariant::Int || val.type() == QVariant::Double) {
      paramArg = paramArg.arg(val.toDouble());
    } else {
      paramArg = paramArg.arg(val.toString());
    }

    args << paramArg;
  }

  return args.join(" ");
}

}  // namespace ros_weaver
