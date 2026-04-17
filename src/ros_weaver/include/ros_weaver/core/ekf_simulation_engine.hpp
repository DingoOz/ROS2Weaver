#ifndef ROS_WEAVER_CORE_EKF_SIMULATION_ENGINE_HPP
#define ROS_WEAVER_CORE_EKF_SIMULATION_ENGINE_HPP

#include <QObject>
#include <QString>
#include <QProcess>
#include <QTemporaryDir>
#include <QTimer>

#include <memory>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

// Forward declarations
class BagManager;

/**
 * @brief Status of the EKF simulation
 */
enum class EKFSimulationStatus {
  Idle,
  Preparing,
  LaunchingEKF,
  Recording,
  Playing,
  Stopping,
  Analyzing,
  Completed,
  Error
};

/**
 * @brief Orchestrates EKF simulation runs using subprocesses
 *
 * This class manages the lifecycle of an EKF simulation:
 * 1. Generate temporary YAML config file
 * 2. Launch EKF node via ros2 run
 * 3. Start recording output bag
 * 4. Play input bag with --clock
 * 5. Wait for playback completion
 * 6. Stop recording and EKF node
 * 7. Extract trajectories from output bag
 *
 * Uses subprocess approach to run the exact same EKF node that runs on robot.
 */
class EKFSimulationEngine : public QObject {
  Q_OBJECT

public:
  explicit EKFSimulationEngine(QObject* parent = nullptr);
  ~EKFSimulationEngine() override;

  // Simulation control
  bool startSimulation(const EKFConfig& config, const QString& inputBagPath);
  void stopSimulation();
  bool isRunning() const;
  EKFSimulationStatus status() const;

  // Configuration
  void setOutputDirectory(const QString& path);
  QString outputDirectory() const;

  // Playback rate (1.0 = realtime, 0.0 = max speed)
  void setPlaybackRate(double rate);
  double playbackRate() const;

  // Optional: Set specific topics to record
  void setRecordTopics(const QStringList& topics);
  QStringList recordTopics() const;

  // Results access
  QString outputBagPath() const;
  Trajectory ekfTrajectory() const;
  Trajectory rawOdomTrajectory() const;
  Trajectory groundTruthTrajectory() const;

  // Ground truth topic detection
  void setGroundTruthTopic(const QString& topic);
  QString groundTruthTopic() const;

  // Static utility: Generate YAML from config
  static QString configToYaml(const EKFConfig& config);

  // Static utility: Parse YAML to config
  static EKFConfig yamlToConfig(const QString& yaml);

signals:
  void statusChanged(EKFSimulationStatus status);
  void progressChanged(int percent, const QString& message);
  void simulationCompleted(bool success);
  void errorOccurred(const QString& error);

  // Trajectory data ready after simulation
  void trajectoriesReady(const Trajectory& ekf,
                         const Trajectory& rawOdom,
                         const Trajectory& groundTruth);

private slots:
  void onEkfProcessStarted();
  void onEkfProcessFinished(int exitCode, QProcess::ExitStatus status);
  void onEkfProcessError(QProcess::ProcessError error);
  void onEkfProcessOutput();

  void onRecorderProcessStarted();
  void onRecorderProcessFinished(int exitCode, QProcess::ExitStatus status);
  void onRecorderProcessError(QProcess::ProcessError error);

  void onPlayerProcessStarted();
  void onPlayerProcessFinished(int exitCode, QProcess::ExitStatus status);
  void onPlayerProcessError(QProcess::ProcessError error);
  void onPlayerProcessOutput();

  void onWatchdogTimeout();

private:
  void setStatus(EKFSimulationStatus status);
  bool generateConfigFile();
  bool launchEkfNode();
  bool startRecording();
  bool startPlayback();
  void stopAllProcesses();
  void cleanup();
  void extractTrajectories();

  // Configuration
  EKFConfig currentConfig_;
  QString inputBagPath_;
  QString outputDirectory_;
  QString tempConfigPath_;
  QString outputBagPath_;
  double playbackRate_ = 0.0;  // 0 = max speed
  QStringList recordTopics_;
  QString groundTruthTopic_ = "/ground_truth/odom";

  // Processes
  QProcess* ekfProcess_ = nullptr;
  QProcess* recorderProcess_ = nullptr;
  QProcess* playerProcess_ = nullptr;

  // Temp directory for config files
  std::unique_ptr<QTemporaryDir> tempDir_;

  // State
  EKFSimulationStatus status_ = EKFSimulationStatus::Idle;
  bool stopRequested_ = false;

  // Watchdog timer
  QTimer* watchdogTimer_ = nullptr;
  static constexpr int WATCHDOG_TIMEOUT_MS = 300000;  // 5 minutes max

  // Extracted trajectories
  Trajectory ekfTrajectory_;
  Trajectory rawOdomTrajectory_;
  Trajectory groundTruthTrajectory_;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::EKFSimulationStatus)

#endif  // ROS_WEAVER_CORE_EKF_SIMULATION_ENGINE_HPP
