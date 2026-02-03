#ifndef ROS_WEAVER_CLI_EKF_ANALYZER_CLI_HPP
#define ROS_WEAVER_CLI_EKF_ANALYZER_CLI_HPP

#include <QObject>
#include <QString>
#include <QVector>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

class EKFSimulationEngine;
class EKFMetricsAnalyzer;
class EKFParameterSweep;

/**
 * @brief Output format for CLI results
 */
enum class OutputFormat {
  Text,
  Json,
  Csv
};

/**
 * @brief Exit codes for CLI tool
 */
enum class ExitCode {
  Success = 0,
  InvalidArguments = 1,
  ConfigFileError = 2,
  InputBagError = 3,
  RobotLocalizationNotInstalled = 4,
  SimulationFailed = 5,
  OutputWriteError = 6
};

/**
 * @brief CLI runner for EKF analysis without GUI
 *
 * Provides a synchronous interface to the async EKF simulation engine.
 * Uses QEventLoop internally to wait for simulation completion.
 */
class EKFAnalyzerCLI : public QObject {
  Q_OBJECT

public:
  explicit EKFAnalyzerCLI(QObject* parent = nullptr);
  ~EKFAnalyzerCLI() override;

  // Configuration
  void setInputBagPath(const QString& path);
  QString inputBagPath() const;

  void setConfigPath(const QString& path);
  QString configPath() const;

  void setOutputPath(const QString& path);
  QString outputPath() const;

  void setOutputFormat(OutputFormat format);
  OutputFormat outputFormat() const;

  void setGroundTruthTopic(const QString& topic);
  QString groundTruthTopic() const;

  void setPlaybackRate(double rate);
  double playbackRate() const;

  void setVerbose(bool verbose);
  bool verbose() const;

  // Parameter sweep configuration
  void enableSweepMode(bool enable);
  bool sweepModeEnabled() const;

  void addSweepParameter(const EKFSweepParameter& param);
  void clearSweepParameters();

  // Load config from YAML file
  bool loadConfig(const QString& path);
  EKFConfig config() const;

  // Run analysis (blocking)
  ExitCode run();

  // Check if robot_localization is available
  static bool isRobotLocalizationInstalled();

  // Format output
  QString formatResultsText(const EKFMetrics& metrics) const;
  QString formatResultsJson(const EKFMetrics& metrics) const;
  QString formatSweepResultsCsv(const QVector<EKFSweepResult>& results) const;

  // Write results to file or stdout
  bool writeResults(const QString& content);

signals:
  void progressChanged(int percent, const QString& message);

private slots:
  void onSimulationCompleted(bool success);
  void onTrajectoriesReady(const Trajectory& ekf,
                           const Trajectory& rawOdom,
                           const Trajectory& groundTruth);
  void onSimulationProgress(int percent, const QString& message);
  void onSweepCompleted(bool success);
  void onSweepProgress(int completed, int total, const QString& message);
  void onError(const QString& error);

private:
  ExitCode runSingleAnalysis();
  ExitCode runParameterSweep();
  void log(const QString& message) const;

  // Configuration
  QString inputBagPath_;
  QString configPath_;
  QString outputPath_;
  OutputFormat outputFormat_ = OutputFormat::Text;
  QString groundTruthTopic_ = "/ground_truth/odom";
  double playbackRate_ = 0.0;
  bool verbose_ = false;
  bool sweepMode_ = false;
  QVector<EKFSweepParameter> sweepParameters_;

  // Loaded config
  EKFConfig config_;
  bool configLoaded_ = false;

  // Components
  EKFSimulationEngine* simulationEngine_ = nullptr;
  EKFMetricsAnalyzer* metricsAnalyzer_ = nullptr;
  EKFParameterSweep* parameterSweep_ = nullptr;

  // Results from simulation
  EKFMetrics lastMetrics_;
  QVector<EKFSweepResult> sweepResults_;
  bool simulationSuccess_ = false;
  QString lastError_;

  // Event loop for synchronous operation
  bool waitingForCompletion_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CLI_EKF_ANALYZER_CLI_HPP
