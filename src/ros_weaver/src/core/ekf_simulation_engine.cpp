#include "ros_weaver/core/ekf_simulation_engine.hpp"
#include "ros_weaver/core/bag_manager.hpp"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QStandardPaths>
#include <QDateTime>
#include <QRegularExpression>

#include <yaml-cpp/yaml.h>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/serialization.hpp>

// ROS 2 Humble uses time_stamp, Jazzy uses recv_timestamp
#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#endif

#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->recv_timestamp
#else
#define ROSBAG2_MSG_TIMESTAMP(msg) (msg)->time_stamp
#endif

namespace ros_weaver {

EKFSimulationEngine::EKFSimulationEngine(QObject* parent)
  : QObject(parent)
{
  // Set default output directory
  outputDirectory_ = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                     + "/ros_weaver_ekf";

  // Setup watchdog timer
  watchdogTimer_ = new QTimer(this);
  watchdogTimer_->setSingleShot(true);
  connect(watchdogTimer_, &QTimer::timeout, this, &EKFSimulationEngine::onWatchdogTimeout);

  // Default topics to record
  recordTopics_ << "/odometry/filtered" << "/tf" << "/tf_static";
}

EKFSimulationEngine::~EKFSimulationEngine() {
  stopSimulation();
}

bool EKFSimulationEngine::startSimulation(const EKFConfig& config, const QString& inputBagPath) {
  if (isRunning()) {
    emit errorOccurred("Simulation already running");
    return false;
  }

  if (!config.isValid()) {
    emit errorOccurred("Invalid EKF configuration");
    return false;
  }

  if (!QFile::exists(inputBagPath)) {
    emit errorOccurred(QString("Input bag not found: %1").arg(inputBagPath));
    return false;
  }

  currentConfig_ = config;
  inputBagPath_ = inputBagPath;
  stopRequested_ = false;

  // Clear previous trajectories
  ekfTrajectory_.poses.clear();
  rawOdomTrajectory_.poses.clear();
  groundTruthTrajectory_.poses.clear();

  setStatus(EKFSimulationStatus::Preparing);
  emit progressChanged(0, "Preparing simulation...");

  // Create temp directory for config
  tempDir_ = std::make_unique<QTemporaryDir>();
  if (!tempDir_->isValid()) {
    emit errorOccurred("Failed to create temporary directory");
    setStatus(EKFSimulationStatus::Error);
    return false;
  }

  // Generate YAML config
  if (!generateConfigFile()) {
    emit errorOccurred("Failed to generate config file");
    setStatus(EKFSimulationStatus::Error);
    return false;
  }

  // Start watchdog
  watchdogTimer_->start(WATCHDOG_TIMEOUT_MS);

  // Launch EKF node
  if (!launchEkfNode()) {
    setStatus(EKFSimulationStatus::Error);
    return false;
  }

  return true;
}

void EKFSimulationEngine::stopSimulation() {
  stopRequested_ = true;
  watchdogTimer_->stop();
  stopAllProcesses();
  setStatus(EKFSimulationStatus::Idle);
}

bool EKFSimulationEngine::isRunning() const {
  return status_ != EKFSimulationStatus::Idle &&
         status_ != EKFSimulationStatus::Completed &&
         status_ != EKFSimulationStatus::Error;
}

EKFSimulationStatus EKFSimulationEngine::status() const {
  return status_;
}

void EKFSimulationEngine::setOutputDirectory(const QString& path) {
  outputDirectory_ = path;
}

QString EKFSimulationEngine::outputDirectory() const {
  return outputDirectory_;
}

void EKFSimulationEngine::setPlaybackRate(double rate) {
  playbackRate_ = rate;
}

double EKFSimulationEngine::playbackRate() const {
  return playbackRate_;
}

void EKFSimulationEngine::setRecordTopics(const QStringList& topics) {
  recordTopics_ = topics;
}

QStringList EKFSimulationEngine::recordTopics() const {
  return recordTopics_;
}

QString EKFSimulationEngine::outputBagPath() const {
  return outputBagPath_;
}

Trajectory EKFSimulationEngine::ekfTrajectory() const {
  return ekfTrajectory_;
}

Trajectory EKFSimulationEngine::rawOdomTrajectory() const {
  return rawOdomTrajectory_;
}

Trajectory EKFSimulationEngine::groundTruthTrajectory() const {
  return groundTruthTrajectory_;
}

void EKFSimulationEngine::setGroundTruthTopic(const QString& topic) {
  groundTruthTopic_ = topic;
}

QString EKFSimulationEngine::groundTruthTopic() const {
  return groundTruthTopic_;
}

void EKFSimulationEngine::setStatus(EKFSimulationStatus newStatus) {
  if (status_ != newStatus) {
    status_ = newStatus;
    emit statusChanged(status_);
  }
}

bool EKFSimulationEngine::generateConfigFile() {
  tempConfigPath_ = tempDir_->path() + "/ekf_config.yaml";

  QFile file(tempConfigPath_);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    emit errorOccurred(QString("Failed to create config file: %1").arg(tempConfigPath_));
    return false;
  }

  QString yaml = configToYaml(currentConfig_);

  QTextStream stream(&file);
  stream << yaml;
  file.close();

  qDebug() << "Generated EKF config at:" << tempConfigPath_;
  return true;
}

QString EKFSimulationEngine::configToYaml(const EKFConfig& config) {
  YAML::Emitter out;
  out << YAML::BeginMap;

  // Node name with parameters
  out << YAML::Key << config.name.toStdString();
  out << YAML::Value << YAML::BeginMap;

  out << YAML::Key << "ros__parameters";
  out << YAML::Value << YAML::BeginMap;

  // Basic parameters
  out << YAML::Key << "frequency" << YAML::Value << config.frequency;
  out << YAML::Key << "two_d_mode" << YAML::Value << config.twoDMode;
  out << YAML::Key << "publish_tf" << YAML::Value << config.publishTf;

  // Frame IDs
  out << YAML::Key << "map_frame" << YAML::Value << config.mapFrame.toStdString();
  out << YAML::Key << "odom_frame" << YAML::Value << config.odomFrame.toStdString();
  out << YAML::Key << "base_link_frame" << YAML::Value << config.baseFrame.toStdString();
  out << YAML::Key << "world_frame" << YAML::Value << config.worldFrame.toStdString();

  // Transform settings
  if (config.transformTimeout > 0.0) {
    out << YAML::Key << "transform_timeout" << YAML::Value << config.transformTimeout;
  }
  if (config.transformTimeOffset > 0.0) {
    out << YAML::Key << "transform_time_offset" << YAML::Value << config.transformTimeOffset;
  }

  // Process noise covariance
  out << YAML::Key << "process_noise_covariance";
  out << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      if (i == j) {
        out << config.processNoiseCovariance[i];
      } else {
        out << 0.0;
      }
    }
  }
  out << YAML::EndSeq;

  // Initial estimate covariance
  out << YAML::Key << "initial_estimate_covariance";
  out << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      if (i == j) {
        out << config.initialEstimateCovariance[i];
      } else {
        out << 0.0;
      }
    }
  }
  out << YAML::EndSeq;

  // Odom sensors
  for (int i = 0; i < config.odomSensors.size(); ++i) {
    const auto& sensor = config.odomSensors[i];
    if (!sensor.enabled) continue;

    QString prefix = QString("odom%1").arg(i);
    out << YAML::Key << prefix.toStdString() << YAML::Value << sensor.topic.toStdString();

    // Config array
    out << YAML::Key << (prefix + "_config").toStdString();
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      out << sensor.config[j];
    }
    out << YAML::EndSeq;

    if (sensor.differential) {
      out << YAML::Key << (prefix + "_differential").toStdString() << YAML::Value << true;
    }
    if (sensor.relative) {
      out << YAML::Key << (prefix + "_relative").toStdString() << YAML::Value << true;
    }
    if (sensor.queueSize != 10) {
      out << YAML::Key << (prefix + "_queue_size").toStdString() << YAML::Value << sensor.queueSize;
    }
    if (sensor.rejectionThreshold > 0.0) {
      out << YAML::Key << (prefix + "_rejection_threshold").toStdString()
          << YAML::Value << sensor.rejectionThreshold;
    }
  }

  // IMU sensors
  for (int i = 0; i < config.imuSensors.size(); ++i) {
    const auto& sensor = config.imuSensors[i];
    if (!sensor.enabled) continue;

    QString prefix = QString("imu%1").arg(i);
    out << YAML::Key << prefix.toStdString() << YAML::Value << sensor.topic.toStdString();

    out << YAML::Key << (prefix + "_config").toStdString();
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      out << sensor.config[j];
    }
    out << YAML::EndSeq;

    if (sensor.differential) {
      out << YAML::Key << (prefix + "_differential").toStdString() << YAML::Value << true;
    }
    if (sensor.relative) {
      out << YAML::Key << (prefix + "_relative").toStdString() << YAML::Value << true;
    }
    if (sensor.queueSize != 10) {
      out << YAML::Key << (prefix + "_queue_size").toStdString() << YAML::Value << sensor.queueSize;
    }
    if (!sensor.removeGravitationalAcceleration) {
      out << YAML::Key << (prefix + "_remove_gravitational_acceleration").toStdString()
          << YAML::Value << false;
    }
  }

  // Pose sensors
  for (int i = 0; i < config.poseSensors.size(); ++i) {
    const auto& sensor = config.poseSensors[i];
    if (!sensor.enabled) continue;

    QString prefix = QString("pose%1").arg(i);
    out << YAML::Key << prefix.toStdString() << YAML::Value << sensor.topic.toStdString();

    out << YAML::Key << (prefix + "_config").toStdString();
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      out << sensor.config[j];
    }
    out << YAML::EndSeq;

    if (sensor.differential) {
      out << YAML::Key << (prefix + "_differential").toStdString() << YAML::Value << true;
    }
    if (sensor.relative) {
      out << YAML::Key << (prefix + "_relative").toStdString() << YAML::Value << true;
    }
  }

  // Twist sensors
  for (int i = 0; i < config.twistSensors.size(); ++i) {
    const auto& sensor = config.twistSensors[i];
    if (!sensor.enabled) continue;

    QString prefix = QString("twist%1").arg(i);
    out << YAML::Key << prefix.toStdString() << YAML::Value << sensor.topic.toStdString();

    out << YAML::Key << (prefix + "_config").toStdString();
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      out << sensor.config[j];
    }
    out << YAML::EndSeq;
  }

  // Debug/print diagnostics
  if (config.printDiagnostics) {
    out << YAML::Key << "print_diagnostics" << YAML::Value << true;
  }

  out << YAML::EndMap;  // ros__parameters
  out << YAML::EndMap;  // node name
  out << YAML::EndMap;  // root

  return QString::fromStdString(out.c_str());
}

EKFConfig EKFSimulationEngine::yamlToConfig(const QString& yaml) {
  EKFConfig config;

  try {
    YAML::Node root = YAML::Load(yaml.toStdString());

    // Find the node (first key is the node name)
    for (auto it = root.begin(); it != root.end(); ++it) {
      config.name = QString::fromStdString(it->first.as<std::string>());
      YAML::Node params = it->second["ros__parameters"];

      if (params["frequency"]) {
        config.frequency = params["frequency"].as<double>();
      }
      if (params["two_d_mode"]) {
        config.twoDMode = params["two_d_mode"].as<bool>();
      }
      if (params["publish_tf"]) {
        config.publishTf = params["publish_tf"].as<bool>();
      }
      if (params["map_frame"]) {
        config.mapFrame = QString::fromStdString(params["map_frame"].as<std::string>());
      }
      if (params["odom_frame"]) {
        config.odomFrame = QString::fromStdString(params["odom_frame"].as<std::string>());
      }
      if (params["base_link_frame"]) {
        config.baseFrame = QString::fromStdString(params["base_link_frame"].as<std::string>());
      }
      if (params["world_frame"]) {
        config.worldFrame = QString::fromStdString(params["world_frame"].as<std::string>());
      }

      // Parse process noise covariance (225-element array, extract diagonal)
      if (params["process_noise_covariance"]) {
        auto cov = params["process_noise_covariance"];
        for (int i = 0; i < EKF_STATE_SIZE; ++i) {
          int idx = i * EKF_STATE_SIZE + i;  // Diagonal element
          if (idx < static_cast<int>(cov.size())) {
            config.processNoiseCovariance[i] = cov[idx].as<double>();
          }
        }
      }

      // Parse initial estimate covariance
      if (params["initial_estimate_covariance"]) {
        auto cov = params["initial_estimate_covariance"];
        for (int i = 0; i < EKF_STATE_SIZE; ++i) {
          int idx = i * EKF_STATE_SIZE + i;
          if (idx < static_cast<int>(cov.size())) {
            config.initialEstimateCovariance[i] = cov[idx].as<double>();
          }
        }
      }

      // Parse odom sensors
      for (int i = 0; i < 4; ++i) {
        QString prefix = QString("odom%1").arg(i);
        if (params[prefix.toStdString()]) {
          EKFSensorConfig sensor;
          sensor.topic = QString::fromStdString(params[prefix.toStdString()].as<std::string>());
          sensor.enabled = true;

          QString configKey = prefix + "_config";
          if (params[configKey.toStdString()]) {
            auto cfgArray = params[configKey.toStdString()];
            for (size_t j = 0; j < cfgArray.size() && j < EKF_STATE_SIZE; ++j) {
              sensor.config[j] = cfgArray[j].as<bool>();
            }
          }

          if (params[(prefix + "_differential").toStdString()]) {
            sensor.differential = params[(prefix + "_differential").toStdString()].as<bool>();
          }
          if (params[(prefix + "_relative").toStdString()]) {
            sensor.relative = params[(prefix + "_relative").toStdString()].as<bool>();
          }

          config.odomSensors.append(sensor);
        }
      }

      // Parse IMU sensors
      for (int i = 0; i < 4; ++i) {
        QString prefix = QString("imu%1").arg(i);
        if (params[prefix.toStdString()]) {
          EKFSensorConfig sensor;
          sensor.topic = QString::fromStdString(params[prefix.toStdString()].as<std::string>());
          sensor.enabled = true;

          QString configKey = prefix + "_config";
          if (params[configKey.toStdString()]) {
            auto cfgArray = params[configKey.toStdString()];
            for (size_t j = 0; j < cfgArray.size() && j < EKF_STATE_SIZE; ++j) {
              sensor.config[j] = cfgArray[j].as<bool>();
            }
          }

          config.imuSensors.append(sensor);
        }
      }

      // Parse pose sensors
      for (int i = 0; i < 4; ++i) {
        QString prefix = QString("pose%1").arg(i);
        if (params[prefix.toStdString()]) {
          EKFSensorConfig sensor;
          sensor.topic = QString::fromStdString(params[prefix.toStdString()].as<std::string>());
          sensor.enabled = true;

          QString configKey = prefix + "_config";
          if (params[configKey.toStdString()]) {
            auto cfgArray = params[configKey.toStdString()];
            for (size_t j = 0; j < cfgArray.size() && j < EKF_STATE_SIZE; ++j) {
              sensor.config[j] = cfgArray[j].as<bool>();
            }
          }

          config.poseSensors.append(sensor);
        }
      }

      break;  // Only process first node
    }
  } catch (const YAML::Exception& e) {
    qWarning() << "Failed to parse EKF YAML:" << e.what();
  }

  return config;
}

bool EKFSimulationEngine::launchEkfNode() {
  setStatus(EKFSimulationStatus::LaunchingEKF);
  emit progressChanged(10, "Launching EKF node...");

  ekfProcess_ = new QProcess(this);
  connect(ekfProcess_, &QProcess::started, this, &EKFSimulationEngine::onEkfProcessStarted);
  connect(ekfProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &EKFSimulationEngine::onEkfProcessFinished);
  connect(ekfProcess_, &QProcess::errorOccurred, this, &EKFSimulationEngine::onEkfProcessError);
  connect(ekfProcess_, &QProcess::readyReadStandardOutput, this, &EKFSimulationEngine::onEkfProcessOutput);
  connect(ekfProcess_, &QProcess::readyReadStandardError, this, &EKFSimulationEngine::onEkfProcessOutput);

  // Build command
  QStringList args;
  args << "run" << "robot_localization" << "ekf_node"
       << "--ros-args"
       << "--params-file" << tempConfigPath_
       << "-p" << "use_sim_time:=true";

  qDebug() << "Launching EKF:" << "ros2" << args.join(" ");
  ekfProcess_->start("ros2", args);

  return true;
}

bool EKFSimulationEngine::startRecording() {
  setStatus(EKFSimulationStatus::Recording);
  emit progressChanged(30, "Starting output recording...");

  // Generate output bag path
  QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  outputBagPath_ = outputDirectory_ + "/ekf_output_" + timestamp;

  // Ensure output directory exists
  QDir().mkpath(outputDirectory_);

  recorderProcess_ = new QProcess(this);
  connect(recorderProcess_, &QProcess::started, this, &EKFSimulationEngine::onRecorderProcessStarted);
  connect(recorderProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &EKFSimulationEngine::onRecorderProcessFinished);
  connect(recorderProcess_, &QProcess::errorOccurred, this, &EKFSimulationEngine::onRecorderProcessError);

  QStringList args;
  args << "bag" << "record"
       << "-o" << outputBagPath_
       << "--use-sim-time";

  // Add topics to record
  for (const QString& topic : recordTopics_) {
    args << topic;
  }

  qDebug() << "Starting recorder:" << "ros2" << args.join(" ");
  recorderProcess_->start("ros2", args);

  return true;
}

bool EKFSimulationEngine::startPlayback() {
  setStatus(EKFSimulationStatus::Playing);
  emit progressChanged(50, "Playing input bag...");

  playerProcess_ = new QProcess(this);
  connect(playerProcess_, &QProcess::started, this, &EKFSimulationEngine::onPlayerProcessStarted);
  connect(playerProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &EKFSimulationEngine::onPlayerProcessFinished);
  connect(playerProcess_, &QProcess::errorOccurred, this, &EKFSimulationEngine::onPlayerProcessError);
  connect(playerProcess_, &QProcess::readyReadStandardOutput, this, &EKFSimulationEngine::onPlayerProcessOutput);
  connect(playerProcess_, &QProcess::readyReadStandardError, this, &EKFSimulationEngine::onPlayerProcessOutput);

  QStringList args;
  args << "bag" << "play" << inputBagPath_ << "--clock";

  if (playbackRate_ > 0.0) {
    args << "-r" << QString::number(playbackRate_);
  }

  qDebug() << "Starting playback:" << "ros2" << args.join(" ");
  playerProcess_->start("ros2", args);

  return true;
}

void EKFSimulationEngine::stopAllProcesses() {
  if (playerProcess_ && playerProcess_->state() != QProcess::NotRunning) {
    playerProcess_->terminate();
    if (!playerProcess_->waitForFinished(3000)) {
      playerProcess_->kill();
    }
  }

  if (recorderProcess_ && recorderProcess_->state() != QProcess::NotRunning) {
    // Send SIGINT for graceful shutdown
    recorderProcess_->terminate();
    if (!recorderProcess_->waitForFinished(5000)) {
      recorderProcess_->kill();
    }
  }

  if (ekfProcess_ && ekfProcess_->state() != QProcess::NotRunning) {
    ekfProcess_->terminate();
    if (!ekfProcess_->waitForFinished(3000)) {
      ekfProcess_->kill();
    }
  }
}

void EKFSimulationEngine::cleanup() {
  if (ekfProcess_) {
    ekfProcess_->deleteLater();
    ekfProcess_ = nullptr;
  }
  if (recorderProcess_) {
    recorderProcess_->deleteLater();
    recorderProcess_ = nullptr;
  }
  if (playerProcess_) {
    playerProcess_->deleteLater();
    playerProcess_ = nullptr;
  }
}

void EKFSimulationEngine::extractTrajectories() {
  setStatus(EKFSimulationStatus::Analyzing);
  emit progressChanged(80, "Extracting trajectories...");

  // Extract from output bag
  if (QFile::exists(outputBagPath_)) {
    try {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = outputBagPath_.toStdString();

      // Try to detect storage format
      if (QFile::exists(outputBagPath_ + "/metadata.yaml")) {
        storage_options.storage_id = "sqlite3";
      } else if (QFile::exists(outputBagPath_ + ".mcap")) {
        storage_options.storage_id = "mcap";
        storage_options.uri = (outputBagPath_ + ".mcap").toStdString();
      }

      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      reader.open(storage_options, converter_options);

      rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;
      double startTime = 0.0;
      bool firstMessage = true;

      ekfTrajectory_.name = "EKF Output";
      ekfTrajectory_.topic = "/odometry/filtered";
      ekfTrajectory_.color = Qt::blue;

      while (reader.has_next()) {
        auto msg = reader.read_next();

        if (firstMessage) {
          startTime = ROSBAG2_MSG_TIMESTAMP(msg) / 1e9;
          firstMessage = false;
        }

        if (msg->topic_name == "/odometry/filtered") {
          nav_msgs::msg::Odometry odom;
          rclcpp::SerializedMessage serialized(*msg->serialized_data);
          odom_serializer.deserialize_message(&serialized, &odom);

          TrajectoryPose pose;
          pose.timestamp = ROSBAG2_MSG_TIMESTAMP(msg) / 1e9 - startTime;
          pose.x = odom.pose.pose.position.x;
          pose.y = odom.pose.pose.position.y;
          pose.z = odom.pose.pose.position.z;

          // Convert quaternion to euler
          auto& q = odom.pose.pose.orientation;
          pose.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                                 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
          pose.pitch = std::asin(2.0 * (q.w * q.y - q.z * q.x));
          pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

          ekfTrajectory_.poses.append(pose);
        }
      }

      reader.close();
    } catch (const std::exception& e) {
      qWarning() << "Failed to read output bag:" << e.what();
    }
  }

  // Extract raw odom and ground truth from input bag
  if (QFile::exists(inputBagPath_)) {
    try {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = inputBagPath_.toStdString();

      // Detect storage format
      if (QFile::exists(inputBagPath_ + "/metadata.yaml")) {
        storage_options.storage_id = "sqlite3";
      } else {
        storage_options.storage_id = "mcap";
      }

      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      reader.open(storage_options, converter_options);

      rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;
      double startTime = 0.0;
      bool firstMessage = true;

      rawOdomTrajectory_.name = "Raw Odometry";
      rawOdomTrajectory_.topic = "/odom";
      rawOdomTrajectory_.color = Qt::gray;

      groundTruthTrajectory_.name = "Ground Truth";
      groundTruthTrajectory_.topic = groundTruthTopic_;
      groundTruthTrajectory_.color = Qt::green;

      while (reader.has_next()) {
        auto msg = reader.read_next();

        if (firstMessage) {
          startTime = ROSBAG2_MSG_TIMESTAMP(msg) / 1e9;
          firstMessage = false;
        }

        QString topic = QString::fromStdString(msg->topic_name);

        if (topic == "/odom" || topic == groundTruthTopic_) {
          nav_msgs::msg::Odometry odom;
          rclcpp::SerializedMessage serialized(*msg->serialized_data);
          odom_serializer.deserialize_message(&serialized, &odom);

          TrajectoryPose pose;
          pose.timestamp = ROSBAG2_MSG_TIMESTAMP(msg) / 1e9 - startTime;
          pose.x = odom.pose.pose.position.x;
          pose.y = odom.pose.pose.position.y;
          pose.z = odom.pose.pose.position.z;

          auto& q = odom.pose.pose.orientation;
          pose.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                                 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
          pose.pitch = std::asin(2.0 * (q.w * q.y - q.z * q.x));
          pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

          if (topic == "/odom") {
            rawOdomTrajectory_.poses.append(pose);
          } else {
            groundTruthTrajectory_.poses.append(pose);
          }
        }
      }

      reader.close();
    } catch (const std::exception& e) {
      qWarning() << "Failed to read input bag:" << e.what();
    }
  }

  emit trajectoriesReady(ekfTrajectory_, rawOdomTrajectory_, groundTruthTrajectory_);
}

// Process slots

void EKFSimulationEngine::onEkfProcessStarted() {
  qDebug() << "EKF process started";

  // Wait a bit for EKF to initialize, then start recording
  QTimer::singleShot(2000, this, [this]() {
    if (!stopRequested_ && ekfProcess_ && ekfProcess_->state() == QProcess::Running) {
      startRecording();
    }
  });
}

void EKFSimulationEngine::onEkfProcessFinished(int exitCode, QProcess::ExitStatus status) {
  qDebug() << "EKF process finished with code" << exitCode << "status" << status;

  if (status_ == EKFSimulationStatus::Stopping ||
      status_ == EKFSimulationStatus::Analyzing) {
    // Expected finish
    return;
  }

  if (!stopRequested_) {
    emit errorOccurred(QString("EKF node exited unexpectedly (code %1)").arg(exitCode));
    setStatus(EKFSimulationStatus::Error);
    stopAllProcesses();
    cleanup();
  }
}

void EKFSimulationEngine::onEkfProcessError(QProcess::ProcessError error) {
  if (error == QProcess::FailedToStart) {
    emit errorOccurred("Failed to start EKF node. Is robot_localization installed?");
    setStatus(EKFSimulationStatus::Error);
    stopAllProcesses();
    cleanup();
  }
}

void EKFSimulationEngine::onEkfProcessOutput() {
  if (ekfProcess_) {
    QString output = ekfProcess_->readAllStandardOutput();
    output += ekfProcess_->readAllStandardError();
    if (!output.isEmpty()) {
      qDebug() << "[EKF]" << output.trimmed();
    }
  }
}

void EKFSimulationEngine::onRecorderProcessStarted() {
  qDebug() << "Recorder started";

  // Wait for recorder to initialize, then start playback
  QTimer::singleShot(1000, this, [this]() {
    if (!stopRequested_ && recorderProcess_ && recorderProcess_->state() == QProcess::Running) {
      startPlayback();
    }
  });
}

void EKFSimulationEngine::onRecorderProcessFinished(int exitCode, QProcess::ExitStatus /*status*/) {
  qDebug() << "Recorder finished with code" << exitCode;

  if (status_ == EKFSimulationStatus::Stopping) {
    // After recorder finishes, extract trajectories
    extractTrajectories();

    setStatus(EKFSimulationStatus::Completed);
    emit progressChanged(100, "Simulation completed");
    emit simulationCompleted(true);

    watchdogTimer_->stop();
    cleanup();
  }
}

void EKFSimulationEngine::onRecorderProcessError(QProcess::ProcessError error) {
  if (error == QProcess::FailedToStart) {
    emit errorOccurred("Failed to start bag recorder");
    setStatus(EKFSimulationStatus::Error);
    stopAllProcesses();
    cleanup();
  }
}

void EKFSimulationEngine::onPlayerProcessStarted() {
  qDebug() << "Player started";
}

void EKFSimulationEngine::onPlayerProcessFinished(int exitCode, QProcess::ExitStatus /*status*/) {
  qDebug() << "Player finished with code" << exitCode;

  if (!stopRequested_) {
    // Playback complete - stop recording and EKF
    setStatus(EKFSimulationStatus::Stopping);
    emit progressChanged(70, "Stopping simulation...");

    // Stop EKF first
    if (ekfProcess_ && ekfProcess_->state() != QProcess::NotRunning) {
      ekfProcess_->terminate();
    }

    // Then stop recorder (this will trigger trajectory extraction)
    if (recorderProcess_ && recorderProcess_->state() != QProcess::NotRunning) {
      // Send SIGINT for clean shutdown
      recorderProcess_->terminate();
    }
  }
}

void EKFSimulationEngine::onPlayerProcessError(QProcess::ProcessError error) {
  if (error == QProcess::FailedToStart) {
    emit errorOccurred("Failed to start bag player");
    setStatus(EKFSimulationStatus::Error);
    stopAllProcesses();
    cleanup();
  }
}

void EKFSimulationEngine::onPlayerProcessOutput() {
  if (playerProcess_) {
    QString output = playerProcess_->readAllStandardOutput();
    output += playerProcess_->readAllStandardError();

    // Parse progress from bag player output
    QRegularExpression progressRegex(R"(\[(\d+(?:\.\d+)?)%\])");
    QRegularExpressionMatch match = progressRegex.match(output);
    if (match.hasMatch()) {
      double percent = match.captured(1).toDouble();
      int progress = 50 + static_cast<int>(percent * 0.2);  // Map to 50-70%
      emit progressChanged(progress, QString("Playing: %1%").arg(percent, 0, 'f', 1));
    }
  }
}

void EKFSimulationEngine::onWatchdogTimeout() {
  qWarning() << "Simulation watchdog timeout - forcing stop";
  emit errorOccurred("Simulation timed out");
  stopSimulation();
  setStatus(EKFSimulationStatus::Error);
  emit simulationCompleted(false);
}

}  // namespace ros_weaver
