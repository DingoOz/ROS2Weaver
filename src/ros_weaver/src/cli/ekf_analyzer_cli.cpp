#include "ros_weaver/cli/ekf_analyzer_cli.hpp"
#include "ros_weaver/core/ekf_simulation_engine.hpp"
#include "ros_weaver/core/ekf_metrics_analyzer.hpp"
#include "ros_weaver/core/ekf_parameter_sweep.hpp"

#include <QCoreApplication>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include <iostream>

namespace ros_weaver {

EKFAnalyzerCLI::EKFAnalyzerCLI(QObject* parent)
    : QObject(parent)
    , simulationEngine_(new EKFSimulationEngine(this))
    , metricsAnalyzer_(new EKFMetricsAnalyzer(this))
    , parameterSweep_(new EKFParameterSweep(this)) {
  // Connect simulation signals
  connect(simulationEngine_, &EKFSimulationEngine::simulationCompleted,
          this, &EKFAnalyzerCLI::onSimulationCompleted);
  connect(simulationEngine_, &EKFSimulationEngine::trajectoriesReady,
          this, &EKFAnalyzerCLI::onTrajectoriesReady);
  connect(simulationEngine_, &EKFSimulationEngine::progressChanged,
          this, &EKFAnalyzerCLI::onSimulationProgress);
  connect(simulationEngine_, &EKFSimulationEngine::errorOccurred,
          this, &EKFAnalyzerCLI::onError);

  // Connect sweep signals
  connect(parameterSweep_, &EKFParameterSweep::sweepCompleted,
          this, &EKFAnalyzerCLI::onSweepCompleted);
  connect(parameterSweep_, &EKFParameterSweep::sweepProgress,
          this, &EKFAnalyzerCLI::onSweepProgress);
  connect(parameterSweep_, &EKFParameterSweep::errorOccurred,
          this, &EKFAnalyzerCLI::onError);
}

EKFAnalyzerCLI::~EKFAnalyzerCLI() = default;

void EKFAnalyzerCLI::setInputBagPath(const QString& path) {
  inputBagPath_ = path;
}

QString EKFAnalyzerCLI::inputBagPath() const {
  return inputBagPath_;
}

void EKFAnalyzerCLI::setConfigPath(const QString& path) {
  configPath_ = path;
}

QString EKFAnalyzerCLI::configPath() const {
  return configPath_;
}

void EKFAnalyzerCLI::setOutputPath(const QString& path) {
  outputPath_ = path;
}

QString EKFAnalyzerCLI::outputPath() const {
  return outputPath_;
}

void EKFAnalyzerCLI::setOutputFormat(OutputFormat format) {
  outputFormat_ = format;
}

OutputFormat EKFAnalyzerCLI::outputFormat() const {
  return outputFormat_;
}

void EKFAnalyzerCLI::setGroundTruthTopic(const QString& topic) {
  groundTruthTopic_ = topic;
}

QString EKFAnalyzerCLI::groundTruthTopic() const {
  return groundTruthTopic_;
}

void EKFAnalyzerCLI::setPlaybackRate(double rate) {
  playbackRate_ = rate;
}

double EKFAnalyzerCLI::playbackRate() const {
  return playbackRate_;
}

void EKFAnalyzerCLI::setVerbose(bool verbose) {
  verbose_ = verbose;
}

bool EKFAnalyzerCLI::verbose() const {
  return verbose_;
}

void EKFAnalyzerCLI::enableSweepMode(bool enable) {
  sweepMode_ = enable;
}

bool EKFAnalyzerCLI::sweepModeEnabled() const {
  return sweepMode_;
}

void EKFAnalyzerCLI::addSweepParameter(const EKFSweepParameter& param) {
  sweepParameters_.append(param);
}

void EKFAnalyzerCLI::clearSweepParameters() {
  sweepParameters_.clear();
}

bool EKFAnalyzerCLI::loadConfig(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    lastError_ = QString("Cannot open config file: %1").arg(path);
    return false;
  }

  QTextStream in(&file);
  QString yaml = in.readAll();
  file.close();

  config_ = EKFSimulationEngine::yamlToConfig(yaml);
  configLoaded_ = true;

  if (!config_.isValid()) {
    lastError_ = "Invalid EKF configuration: no sensors configured";
    configLoaded_ = false;
    return false;
  }

  return true;
}

EKFConfig EKFAnalyzerCLI::config() const {
  return config_;
}

bool EKFAnalyzerCLI::isRobotLocalizationInstalled() {
  QProcess process;
  process.start("ros2", QStringList() << "pkg" << "prefix" << "robot_localization");
  process.waitForFinished(5000);
  return process.exitCode() == 0;
}

ExitCode EKFAnalyzerCLI::run() {
  // Validate inputs
  if (inputBagPath_.isEmpty()) {
    std::cerr << "Error: No input bag path specified" << std::endl;
    return ExitCode::InvalidArguments;
  }

  QFileInfo bagInfo(inputBagPath_);
  if (!bagInfo.exists()) {
    std::cerr << "Error: Input bag not found: " << inputBagPath_.toStdString() << std::endl;
    return ExitCode::InputBagError;
  }

  if (!configLoaded_) {
    std::cerr << "Error: No configuration loaded" << std::endl;
    return ExitCode::ConfigFileError;
  }

  // Check robot_localization availability
  if (!isRobotLocalizationInstalled()) {
    std::cerr << "Error: robot_localization package not found" << std::endl;
    std::cerr << "Install with: sudo apt install ros-${ROS_DISTRO}-robot-localization" << std::endl;
    return ExitCode::RobotLocalizationNotInstalled;
  }

  log("Starting EKF analysis...");
  log(QString("Input bag: %1").arg(inputBagPath_));
  log(QString("Ground truth topic: %1").arg(groundTruthTopic_));

  if (sweepMode_ && !sweepParameters_.isEmpty()) {
    return runParameterSweep();
  } else {
    return runSingleAnalysis();
  }
}

ExitCode EKFAnalyzerCLI::runSingleAnalysis() {
  // Configure simulation engine
  simulationEngine_->setGroundTruthTopic(groundTruthTopic_);
  simulationEngine_->setPlaybackRate(playbackRate_);

  // Start simulation
  waitingForCompletion_ = true;
  simulationSuccess_ = false;

  if (!simulationEngine_->startSimulation(config_, inputBagPath_)) {
    std::cerr << "Error: Failed to start simulation" << std::endl;
    return ExitCode::SimulationFailed;
  }

  // Wait for completion using event loop
  QEventLoop eventLoop;
  connect(simulationEngine_, &EKFSimulationEngine::simulationCompleted,
          &eventLoop, &QEventLoop::quit);
  connect(simulationEngine_, &EKFSimulationEngine::errorOccurred,
          &eventLoop, &QEventLoop::quit);
  eventLoop.exec();

  waitingForCompletion_ = false;

  if (!simulationSuccess_) {
    std::cerr << "Error: Simulation failed";
    if (!lastError_.isEmpty()) {
      std::cerr << ": " << lastError_.toStdString();
    }
    std::cerr << std::endl;
    return ExitCode::SimulationFailed;
  }

  // Format and output results
  QString output;
  switch (outputFormat_) {
    case OutputFormat::Text:
      output = formatResultsText(lastMetrics_);
      break;
    case OutputFormat::Json:
      output = formatResultsJson(lastMetrics_);
      break;
    case OutputFormat::Csv:
      // For single analysis, CSV shows one row
      output = formatSweepResultsCsv({EKFSweepResult{config_, lastMetrics_, 0, {}}});
      break;
  }

  if (!writeResults(output)) {
    return ExitCode::OutputWriteError;
  }

  return ExitCode::Success;
}

ExitCode EKFAnalyzerCLI::runParameterSweep() {
  log(QString("Running parameter sweep with %1 parameter(s)").arg(sweepParameters_.size()));

  // Configure sweep
  parameterSweep_->setBaseConfig(config_);
  parameterSweep_->setInputBagPath(inputBagPath_);
  parameterSweep_->setGroundTruthTopic(groundTruthTopic_);

  for (const auto& param : sweepParameters_) {
    parameterSweep_->addSweepParameter(param);
    log(QString("  %1: %2 to %3 (%4 steps%5)")
            .arg(param.name)
            .arg(param.minValue)
            .arg(param.maxValue)
            .arg(param.steps)
            .arg(param.logScale ? ", log scale" : ""));
  }

  int totalConfigs = parameterSweep_->totalConfigurations();
  log(QString("Total configurations to test: %1").arg(totalConfigs));

  // Start sweep
  waitingForCompletion_ = true;
  simulationSuccess_ = false;
  sweepResults_.clear();

  if (!parameterSweep_->startSweep()) {
    std::cerr << "Error: Failed to start parameter sweep" << std::endl;
    return ExitCode::SimulationFailed;
  }

  // Wait for completion
  QEventLoop eventLoop;
  connect(parameterSweep_, &EKFParameterSweep::sweepCompleted,
          &eventLoop, &QEventLoop::quit);
  connect(parameterSweep_, &EKFParameterSweep::errorOccurred,
          &eventLoop, &QEventLoop::quit);
  eventLoop.exec();

  waitingForCompletion_ = false;

  if (!simulationSuccess_) {
    std::cerr << "Error: Parameter sweep failed";
    if (!lastError_.isEmpty()) {
      std::cerr << ": " << lastError_.toStdString();
    }
    std::cerr << std::endl;
    return ExitCode::SimulationFailed;
  }

  // Get results
  sweepResults_ = parameterSweep_->results();

  // Format output
  QString output;
  switch (outputFormat_) {
    case OutputFormat::Text:
      // For sweep, text shows best result prominently
      if (!sweepResults_.isEmpty()) {
        EKFSweepResult best = parameterSweep_->bestResult();
        output = "=== Best Configuration ===\n";
        output += QString("Index: %1\n").arg(best.sweepIndex);
        for (auto it = best.parameterValues.begin(); it != best.parameterValues.end(); ++it) {
          output += QString("%1: %2\n").arg(it.key()).arg(it.value());
        }
        output += "\n";
        output += formatResultsText(best.metrics);
        output += "\n\n=== All Results ===\n";
        for (const auto& result : sweepResults_) {
          output += QString("\n--- Configuration %1 ---\n").arg(result.sweepIndex);
          output += QString("ATE RMSE: %1 m\n").arg(result.metrics.ateRmse, 0, 'f', 4);
          output += QString("RPE RMSE: %1 m\n").arg(result.metrics.rpeRmse, 0, 'f', 4);
        }
      }
      break;
    case OutputFormat::Json: {
      QJsonObject root;
      QJsonArray resultsArray;
      for (const auto& result : sweepResults_) {
        QJsonObject obj;
        obj["index"] = result.sweepIndex;

        QJsonObject params;
        for (auto it = result.parameterValues.begin(); it != result.parameterValues.end(); ++it) {
          params[it.key()] = it.value();
        }
        obj["parameters"] = params;

        QJsonObject metrics;
        metrics["ate_rmse"] = result.metrics.ateRmse;
        metrics["ate_mean"] = result.metrics.ateMean;
        metrics["ate_median"] = result.metrics.ateMedian;
        metrics["ate_std"] = result.metrics.ateStd;
        metrics["rpe_rmse"] = result.metrics.rpeRmse;
        metrics["rpe_mean"] = result.metrics.rpeMean;
        metrics["yaw_drift"] = result.metrics.yawDrift;
        obj["metrics"] = metrics;

        resultsArray.append(obj);
      }
      root["results"] = resultsArray;

      if (!sweepResults_.isEmpty()) {
        EKFSweepResult best = parameterSweep_->bestResult();
        root["best_index"] = best.sweepIndex;
      }

      QJsonDocument doc(root);
      output = doc.toJson(QJsonDocument::Indented);
      break;
    }
    case OutputFormat::Csv:
      output = formatSweepResultsCsv(sweepResults_);
      break;
  }

  if (!writeResults(output)) {
    return ExitCode::OutputWriteError;
  }

  return ExitCode::Success;
}

void EKFAnalyzerCLI::onSimulationCompleted(bool success) {
  simulationSuccess_ = success;
}

void EKFAnalyzerCLI::onTrajectoriesReady(const Trajectory& ekf,
                                          const Trajectory& rawOdom,
                                          const Trajectory& groundTruth) {
  log("Trajectories extracted, computing metrics...");

  // Compute metrics
  if (!groundTruth.poses.isEmpty()) {
    lastMetrics_ = metricsAnalyzer_->computeMetrics(ekf, groundTruth);
    lastMetrics_.hasGroundTruth = true;
  } else {
    lastMetrics_ = metricsAnalyzer_->computeRelativeMetrics(ekf, rawOdom);
    lastMetrics_.hasGroundTruth = false;
    log("Warning: No ground truth available, using relative metrics");
  }

  lastMetrics_.bagPath = inputBagPath_;
  lastMetrics_.trajectoryLength = ekf.length();
  lastMetrics_.poseCount = ekf.poses.size();

  log(QString("Metrics computed: ATE RMSE = %1 m").arg(lastMetrics_.ateRmse, 0, 'f', 4));
}

void EKFAnalyzerCLI::onSimulationProgress(int percent, const QString& message) {
  if (verbose_) {
    log(QString("[%1%] %2").arg(percent, 3).arg(message));
  }
  emit progressChanged(percent, message);
}

void EKFAnalyzerCLI::onSweepCompleted(bool success) {
  simulationSuccess_ = success;
}

void EKFAnalyzerCLI::onSweepProgress(int completed, int total, const QString& message) {
  int percent = (total > 0) ? (completed * 100 / total) : 0;
  log(QString("[%1/%2] %3").arg(completed).arg(total).arg(message));
  emit progressChanged(percent, message);
}

void EKFAnalyzerCLI::onError(const QString& error) {
  lastError_ = error;
  std::cerr << "Error: " << error.toStdString() << std::endl;
}

QString EKFAnalyzerCLI::formatResultsText(const EKFMetrics& metrics) const {
  QString output;
  QTextStream out(&output);

  out << "=== EKF Analysis Results ===" << Qt::endl;
  out << "Bag: " << metrics.bagPath << Qt::endl;
  out << Qt::endl;

  out << "--- Absolute Trajectory Error (ATE) ---" << Qt::endl;
  out << QString("RMSE:    %1 m").arg(metrics.ateRmse, 0, 'f', 4) << Qt::endl;
  out << QString("Mean:    %1 m").arg(metrics.ateMean, 0, 'f', 4) << Qt::endl;
  out << QString("Median:  %1 m").arg(metrics.ateMedian, 0, 'f', 4) << Qt::endl;
  out << QString("Std:     %1 m").arg(metrics.ateStd, 0, 'f', 4) << Qt::endl;
  out << QString("Min:     %1 m").arg(metrics.ateMin, 0, 'f', 4) << Qt::endl;
  out << QString("Max:     %1 m").arg(metrics.ateMax, 0, 'f', 4) << Qt::endl;
  out << Qt::endl;

  out << "--- Relative Pose Error (RPE) ---" << Qt::endl;
  out << QString("RMSE:    %1 m").arg(metrics.rpeRmse, 0, 'f', 4) << Qt::endl;
  out << QString("Mean:    %1 m").arg(metrics.rpeMean, 0, 'f', 4) << Qt::endl;
  out << QString("Median:  %1 m").arg(metrics.rpeMedian, 0, 'f', 4) << Qt::endl;
  out << QString("Std:     %1 m").arg(metrics.rpeStd, 0, 'f', 4) << Qt::endl;
  out << Qt::endl;

  out << "--- Additional Metrics ---" << Qt::endl;
  out << QString("Yaw drift:          %1 deg/m").arg(metrics.yawDrift, 0, 'f', 4) << Qt::endl;
  out << QString("Trajectory length:  %1 m").arg(metrics.trajectoryLength, 0, 'f', 2) << Qt::endl;
  out << QString("Pose count:         %1").arg(metrics.poseCount) << Qt::endl;
  out << QString("Ground truth:       %1").arg(metrics.hasGroundTruth ? "yes" : "no") << Qt::endl;

  return output;
}

QString EKFAnalyzerCLI::formatResultsJson(const EKFMetrics& metrics) const {
  QJsonObject root;

  QJsonObject ate;
  ate["rmse"] = metrics.ateRmse;
  ate["mean"] = metrics.ateMean;
  ate["median"] = metrics.ateMedian;
  ate["std"] = metrics.ateStd;
  ate["min"] = metrics.ateMin;
  ate["max"] = metrics.ateMax;
  root["ate"] = ate;

  QJsonObject rpe;
  rpe["rmse"] = metrics.rpeRmse;
  rpe["mean"] = metrics.rpeMean;
  rpe["median"] = metrics.rpeMedian;
  rpe["std"] = metrics.rpeStd;
  root["rpe"] = rpe;

  root["yaw_drift"] = metrics.yawDrift;
  root["trajectory_length"] = metrics.trajectoryLength;
  root["pose_count"] = metrics.poseCount;
  root["has_ground_truth"] = metrics.hasGroundTruth;
  root["bag_path"] = metrics.bagPath;

  QJsonDocument doc(root);
  return doc.toJson(QJsonDocument::Indented);
}

QString EKFAnalyzerCLI::formatSweepResultsCsv(const QVector<EKFSweepResult>& results) const {
  QString output;
  QTextStream out(&output);

  if (results.isEmpty()) {
    return output;
  }

  // Build header
  QStringList headers;
  headers << "Index";

  // Get parameter names from first result
  if (!results.first().parameterValues.isEmpty()) {
    for (auto it = results.first().parameterValues.begin();
         it != results.first().parameterValues.end(); ++it) {
      headers << it.key();
    }
  }

  headers << "ATE_RMSE" << "ATE_Mean" << "ATE_Median" << "ATE_Std"
          << "RPE_RMSE" << "RPE_Mean" << "RPE_Median" << "RPE_Std"
          << "Yaw_Drift" << "Trajectory_Length" << "Pose_Count";

  out << headers.join(",") << Qt::endl;

  // Write data rows
  for (const auto& result : results) {
    QStringList row;
    row << QString::number(result.sweepIndex);

    for (auto it = result.parameterValues.begin();
         it != result.parameterValues.end(); ++it) {
      row << QString::number(it.value(), 'g', 6);
    }

    row << QString::number(result.metrics.ateRmse, 'g', 6)
        << QString::number(result.metrics.ateMean, 'g', 6)
        << QString::number(result.metrics.ateMedian, 'g', 6)
        << QString::number(result.metrics.ateStd, 'g', 6)
        << QString::number(result.metrics.rpeRmse, 'g', 6)
        << QString::number(result.metrics.rpeMean, 'g', 6)
        << QString::number(result.metrics.rpeMedian, 'g', 6)
        << QString::number(result.metrics.rpeStd, 'g', 6)
        << QString::number(result.metrics.yawDrift, 'g', 6)
        << QString::number(result.metrics.trajectoryLength, 'f', 2)
        << QString::number(result.metrics.poseCount);

    out << row.join(",") << Qt::endl;
  }

  return output;
}

bool EKFAnalyzerCLI::writeResults(const QString& content) {
  if (outputPath_.isEmpty()) {
    // Write to stdout
    std::cout << content.toStdString();
    return true;
  }

  // Write to file
  QFile file(outputPath_);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    std::cerr << "Error: Cannot write to output file: " << outputPath_.toStdString() << std::endl;
    return false;
  }

  QTextStream out(&file);
  out << content;
  file.close();

  log(QString("Results written to: %1").arg(outputPath_));
  return true;
}

void EKFAnalyzerCLI::log(const QString& message) const {
  if (verbose_) {
    std::cerr << message.toStdString() << std::endl;
  }
}

}  // namespace ros_weaver
