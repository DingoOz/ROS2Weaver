#include "ros_weaver/core/ekf_parameter_sweep.hpp"
#include "ros_weaver/core/ekf_simulation_engine.hpp"
#include "ros_weaver/core/ekf_metrics_analyzer.hpp"

#include <QFile>
#include <QTextStream>
#include <QDebug>

#include <cmath>

namespace ros_weaver {

EKFParameterSweep::EKFParameterSweep(QObject* parent)
  : QObject(parent)
{
  simulationEngine_ = new EKFSimulationEngine(this);
  metricsAnalyzer_ = new EKFMetricsAnalyzer(this);

  connect(simulationEngine_, &EKFSimulationEngine::simulationCompleted,
          this, &EKFParameterSweep::onSimulationCompleted);
  connect(simulationEngine_, &EKFSimulationEngine::trajectoriesReady,
          this, &EKFParameterSweep::onTrajectoriesReady);
  connect(simulationEngine_, &EKFSimulationEngine::errorOccurred,
          this, &EKFParameterSweep::errorOccurred);
}

EKFParameterSweep::~EKFParameterSweep() {
  cancelSweep();
}

void EKFParameterSweep::setBaseConfig(const EKFConfig& config) {
  baseConfig_ = config;
}

EKFConfig EKFParameterSweep::baseConfig() const {
  return baseConfig_;
}

void EKFParameterSweep::setInputBagPath(const QString& path) {
  inputBagPath_ = path;
}

QString EKFParameterSweep::inputBagPath() const {
  return inputBagPath_;
}

void EKFParameterSweep::setGroundTruthTopic(const QString& topic) {
  groundTruthTopic_ = topic;
  simulationEngine_->setGroundTruthTopic(topic);
}

QString EKFParameterSweep::groundTruthTopic() const {
  return groundTruthTopic_;
}

void EKFParameterSweep::addSweepParameter(const EKFSweepParameter& param) {
  sweepParameters_.append(param);
}

void EKFParameterSweep::clearSweepParameters() {
  sweepParameters_.clear();
}

QVector<EKFSweepParameter> EKFParameterSweep::sweepParameters() const {
  return sweepParameters_;
}

int EKFParameterSweep::totalConfigurations() const {
  if (sweepParameters_.isEmpty()) {
    return 1;  // Just the base config
  }

  int total = 1;
  for (const auto& param : sweepParameters_) {
    total *= param.steps;
  }
  return total;
}

QVector<EKFConfig> EKFParameterSweep::generateConfigurations() const {
  QVector<EKFConfig> configs;

  if (sweepParameters_.isEmpty()) {
    configs.append(baseConfig_);
    return configs;
  }

  // Generate Cartesian product of all parameter values
  int total = totalConfigurations();

  for (int i = 0; i < total; ++i) {
    QMap<QString, double> values;

    // Decode index into parameter values
    int remainder = i;
    for (int p = sweepParameters_.size() - 1; p >= 0; --p) {
      const auto& param = sweepParameters_[p];
      auto paramValues = param.values();
      int valueIdx = remainder % param.steps;
      remainder /= param.steps;

      values[param.name] = paramValues[valueIdx];
    }

    EKFConfig config = applyParameterValues(baseConfig_, values);
    configs.append(config);
  }

  return configs;
}

bool EKFParameterSweep::startSweep() {
  if (running_) {
    emit errorOccurred("Sweep already running");
    return false;
  }

  if (!baseConfig_.isValid()) {
    emit errorOccurred("Invalid base configuration");
    return false;
  }

  if (inputBagPath_.isEmpty()) {
    emit errorOccurred("No input bag specified");
    return false;
  }

  // Generate all configurations
  configurations_ = generateConfigurations();

  // Store parameter values for each config
  parameterValueSets_.clear();
  int total = totalConfigurations();
  for (int i = 0; i < total; ++i) {
    QMap<QString, double> values;
    int remainder = i;
    for (int p = sweepParameters_.size() - 1; p >= 0; --p) {
      const auto& param = sweepParameters_[p];
      auto paramValues = param.values();
      int valueIdx = remainder % param.steps;
      remainder /= param.steps;
      values[param.name] = paramValues[valueIdx];
    }
    parameterValueSets_.append(values);
  }

  results_.clear();
  currentConfigIndex_ = 0;
  running_ = true;
  cancelled_ = false;

  emit sweepStarted(configurations_.size());

  runNextConfiguration();
  return true;
}

void EKFParameterSweep::cancelSweep() {
  if (running_) {
    cancelled_ = true;
    simulationEngine_->stopSimulation();
    running_ = false;
    emit sweepCancelled();
  }
}

bool EKFParameterSweep::isRunning() const {
  return running_;
}

QVector<EKFSweepResult> EKFParameterSweep::results() const {
  return results_;
}

EKFSweepResult EKFParameterSweep::bestResult() const {
  if (results_.isEmpty()) {
    return EKFSweepResult();
  }

  // Find result with lowest ATE RMSE
  int bestIdx = 0;
  double bestRmse = results_[0].metrics.ateRmse;

  for (int i = 1; i < results_.size(); ++i) {
    if (results_[i].metrics.ateRmse < bestRmse) {
      bestRmse = results_[i].metrics.ateRmse;
      bestIdx = i;
    }
  }

  return results_[bestIdx];
}

bool EKFParameterSweep::exportResultsCSV(const QString& path) const {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QTextStream stream(&file);

  // Header
  stream << "Index,Config Name";
  for (const auto& param : sweepParameters_) {
    stream << "," << param.name;
  }
  stream << ",ATE RMSE,ATE Mean,ATE Max,RPE RMSE,Yaw Drift,Trajectory Length\n";

  // Data rows
  for (int i = 0; i < results_.size(); ++i) {
    const auto& result = results_[i];
    stream << result.sweepIndex << "," << result.metrics.configName;

    for (const auto& param : sweepParameters_) {
      stream << "," << result.parameterValues.value(param.name, 0.0);
    }

    stream << "," << result.metrics.ateRmse
           << "," << result.metrics.ateMean
           << "," << result.metrics.ateMax
           << "," << result.metrics.rpeRmse
           << "," << result.metrics.yawDrift
           << "," << result.metrics.trajectoryLength
           << "\n";
  }

  file.close();
  return true;
}

bool EKFParameterSweep::exportBestConfig(const QString& path) const {
  if (results_.isEmpty()) {
    return false;
  }

  EKFSweepResult best = bestResult();
  QString yaml = EKFSimulationEngine::configToYaml(best.config);

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QTextStream stream(&file);
  stream << "# Best EKF configuration from parameter sweep\n";
  stream << "# ATE RMSE: " << best.metrics.ateRmse << " m\n";
  stream << "# Parameters:\n";
  for (auto it = best.parameterValues.begin(); it != best.parameterValues.end(); ++it) {
    stream << "#   " << it.key() << ": " << it.value() << "\n";
  }
  stream << "\n";
  stream << yaml;

  file.close();
  return true;
}

void EKFParameterSweep::runNextConfiguration() {
  if (cancelled_ || currentConfigIndex_ >= configurations_.size()) {
    running_ = false;
    emit sweepCompleted(!cancelled_ && currentConfigIndex_ >= configurations_.size());
    return;
  }

  const EKFConfig& config = configurations_[currentConfigIndex_];

  emit configurationStarted(currentConfigIndex_, config);
  emit sweepProgress(currentConfigIndex_, configurations_.size(),
                     QString("Running config %1 of %2")
                       .arg(currentConfigIndex_ + 1)
                       .arg(configurations_.size()));

  simulationEngine_->startSimulation(config, inputBagPath_);
}

void EKFParameterSweep::onSimulationCompleted(bool success) {
  if (cancelled_) {
    return;
  }

  if (!success) {
    qWarning() << "Simulation failed for config" << currentConfigIndex_;
    // Continue with next config anyway
  }

  // Metrics will be computed when trajectories are ready
}

void EKFParameterSweep::onTrajectoriesReady(const Trajectory& ekf,
                                             const Trajectory& rawOdom,
                                             const Trajectory& groundTruth) {
  if (cancelled_) {
    return;
  }

  currentGroundTruth_ = groundTruth;

  // Compute metrics
  EKFMetrics metrics;
  if (!groundTruth.poses.isEmpty()) {
    metrics = metricsAnalyzer_->computeMetrics(ekf, groundTruth);
  } else {
    metrics = metricsAnalyzer_->computeRelativeMetrics(ekf, rawOdom);
  }

  // Store result
  EKFSweepResult result;
  result.config = configurations_[currentConfigIndex_];
  result.metrics = metrics;
  result.sweepIndex = currentConfigIndex_;
  result.parameterValues = parameterValueSets_[currentConfigIndex_];

  // Generate config name
  QStringList parts;
  for (auto it = result.parameterValues.begin(); it != result.parameterValues.end(); ++it) {
    QString paramName = it.key().split(".").last();
    parts << QString("%1=%2").arg(paramName).arg(it.value(), 0, 'g', 4);
  }
  result.metrics.configName = parts.isEmpty() ? "Base" : parts.join(", ");

  results_.append(result);
  emit configurationCompleted(currentConfigIndex_, result);

  // Move to next configuration
  currentConfigIndex_++;
  runNextConfiguration();
}

EKFConfig EKFParameterSweep::applyParameterValues(const EKFConfig& base,
                                                   const QMap<QString, double>& values) const {
  EKFConfig config = base;

  for (auto it = values.begin(); it != values.end(); ++it) {
    setConfigParameter(config, it.key(), it.value());
  }

  return config;
}

void EKFParameterSweep::setConfigParameter(EKFConfig& config,
                                            const QString& path,
                                            double value) const {
  // Parse parameter path like "processNoiseCovariance.0" or "frequency"
  QStringList parts = path.split(".");

  if (parts.isEmpty()) {
    return;
  }

  QString paramName = parts[0];

  if (paramName == "frequency") {
    config.frequency = value;
  }
  else if (paramName == "processNoiseCovariance" && parts.size() > 1) {
    int idx = parts[1].toInt();
    if (idx >= 0 && idx < EKF_STATE_SIZE) {
      config.processNoiseCovariance[idx] = value;
    }
  }
  else if (paramName == "initialEstimateCovariance" && parts.size() > 1) {
    int idx = parts[1].toInt();
    if (idx >= 0 && idx < EKF_STATE_SIZE) {
      config.initialEstimateCovariance[idx] = value;
    }
  }
  else if (paramName == "transformTimeout") {
    config.transformTimeout = value;
  }
  else if (paramName == "transformTimeOffset") {
    config.transformTimeOffset = value;
  }
  else if (paramName == "odomRejectionThreshold" && parts.size() > 1) {
    int sensorIdx = parts[1].toInt();
    if (sensorIdx >= 0 && sensorIdx < config.odomSensors.size()) {
      config.odomSensors[sensorIdx].rejectionThreshold = value;
    }
  }
  else if (paramName == "imuRejectionThreshold" && parts.size() > 1) {
    int sensorIdx = parts[1].toInt();
    if (sensorIdx >= 0 && sensorIdx < config.imuSensors.size()) {
      config.imuSensors[sensorIdx].rejectionThreshold = value;
    }
  }
  else {
    qWarning() << "Unknown sweep parameter:" << path;
  }
}

}  // namespace ros_weaver
