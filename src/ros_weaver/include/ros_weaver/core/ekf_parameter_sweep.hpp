#ifndef ROS_WEAVER_CORE_EKF_PARAMETER_SWEEP_HPP
#define ROS_WEAVER_CORE_EKF_PARAMETER_SWEEP_HPP

#include <QObject>
#include <QString>
#include <QVector>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

// Forward declarations
class EKFSimulationEngine;
class EKFMetricsAnalyzer;

/**
 * @brief Manages batch execution of EKF parameter sweeps
 *
 * Generates a Cartesian product of parameter configurations,
 * runs simulations sequentially, and collects metrics for comparison.
 */
class EKFParameterSweep : public QObject {
  Q_OBJECT

public:
  explicit EKFParameterSweep(QObject* parent = nullptr);
  ~EKFParameterSweep() override;

  // Configuration
  void setBaseConfig(const EKFConfig& config);
  EKFConfig baseConfig() const;

  void setInputBagPath(const QString& path);
  QString inputBagPath() const;

  void setGroundTruthTopic(const QString& topic);
  QString groundTruthTopic() const;

  // Parameter sweep definition
  void addSweepParameter(const EKFSweepParameter& param);
  void clearSweepParameters();
  QVector<EKFSweepParameter> sweepParameters() const;

  // Total number of configurations in the sweep
  int totalConfigurations() const;

  // Generate all configurations
  QVector<EKFConfig> generateConfigurations() const;

  // Sweep execution
  bool startSweep();
  void cancelSweep();
  bool isRunning() const;

  // Results
  QVector<EKFSweepResult> results() const;
  EKFSweepResult bestResult() const;

  // Export results
  bool exportResultsCSV(const QString& path) const;
  bool exportBestConfig(const QString& path) const;

signals:
  void sweepStarted(int totalConfigs);
  void configurationStarted(int index, const EKFConfig& config);
  void configurationCompleted(int index, const EKFSweepResult& result);
  void sweepProgress(int completed, int total, const QString& message);
  void sweepCompleted(bool success);
  void sweepCancelled();
  void errorOccurred(const QString& error);

private slots:
  void onSimulationCompleted(bool success);
  void onTrajectoriesReady(const Trajectory& ekf,
                           const Trajectory& rawOdom,
                           const Trajectory& groundTruth);

private:
  void runNextConfiguration();
  EKFConfig applyParameterValues(const EKFConfig& base,
                                  const QMap<QString, double>& values) const;
  void setConfigParameter(EKFConfig& config,
                          const QString& path,
                          double value) const;

  // Configuration
  EKFConfig baseConfig_;
  QString inputBagPath_;
  QString groundTruthTopic_ = "/ground_truth/odom";
  QVector<EKFSweepParameter> sweepParameters_;

  // Execution state
  bool running_ = false;
  bool cancelled_ = false;
  int currentConfigIndex_ = 0;
  QVector<EKFConfig> configurations_;
  QVector<QMap<QString, double>> parameterValueSets_;

  // Results
  QVector<EKFSweepResult> results_;

  // Components
  EKFSimulationEngine* simulationEngine_ = nullptr;
  EKFMetricsAnalyzer* metricsAnalyzer_ = nullptr;

  // Current simulation state
  Trajectory currentGroundTruth_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_EKF_PARAMETER_SWEEP_HPP
