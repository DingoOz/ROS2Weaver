#ifndef ROS_WEAVER_CORE_EKF_METRICS_ANALYZER_HPP
#define ROS_WEAVER_CORE_EKF_METRICS_ANALYZER_HPP

#include <QObject>
#include <QString>
#include <QVector>
#include <QProcess>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

/**
 * @brief Computes trajectory comparison metrics for EKF evaluation
 *
 * Supports two modes:
 * 1. Using external 'evo' package (evo_ape, evo_rpe) if available
 * 2. Direct C++ computation as fallback
 *
 * Metrics computed:
 * - ATE (Absolute Trajectory Error): RMSE, mean, median, std, min, max
 * - RPE (Relative Pose Error): RMSE, mean, median, std
 * - Yaw drift: degrees per meter traveled
 */
class EKFMetricsAnalyzer : public QObject {
  Q_OBJECT

public:
  explicit EKFMetricsAnalyzer(QObject* parent = nullptr);
  ~EKFMetricsAnalyzer() override;

  // Compute metrics between two trajectories
  EKFMetrics computeMetrics(const Trajectory& estimated,
                            const Trajectory& reference);

  // Compute metrics with raw odometry comparison (no ground truth)
  EKFMetrics computeRelativeMetrics(const Trajectory& estimated,
                                    const Trajectory& rawOdom);

  // Check if evo package is available
  bool isEvoAvailable() const;

  // Force use of direct computation even if evo is available
  void setUseDirectComputation(bool direct);
  bool useDirectComputation() const;

  // Align trajectories before comparison (Umeyama alignment)
  void setAlignTrajectories(bool align);
  bool alignTrajectories() const;

  // Maximum time difference for pose matching (seconds)
  void setMaxTimeDelta(double delta);
  double maxTimeDelta() const;

signals:
  void metricsComputed(const EKFMetrics& metrics);
  void errorOccurred(const QString& error);
  void progressChanged(int percent, const QString& message);

private:
  // Direct computation methods
  EKFMetrics computeMetricsDirect(const Trajectory& estimated,
                                  const Trajectory& reference);

  // Use evo package via subprocess
  EKFMetrics computeMetricsEvo(const Trajectory& estimated,
                               const Trajectory& reference);

  // Utility: Associate poses by timestamp
  QVector<std::pair<TrajectoryPose, TrajectoryPose>> associatePoses(
      const Trajectory& estimated,
      const Trajectory& reference) const;

  // Utility: Save trajectory to TUM format for evo
  bool saveTrajectoryTUM(const Trajectory& traj, const QString& path) const;

  // Utility: Parse evo JSON output
  EKFMetrics parseEvoJson(const QString& jsonPath) const;

  // Utility: Compute Umeyama alignment
  void umeyamaAlign(QVector<std::pair<TrajectoryPose, TrajectoryPose>>& poses) const;

  // Utility: Normalize angle to [-pi, pi]
  static double normalizeAngle(double angle);

  // Utility: Compute angular difference
  static double angleDiff(double a, double b);

  // Configuration
  bool useDirectComputation_ = false;
  bool alignTrajectories_ = true;
  double maxTimeDelta_ = 0.1;  // 100ms default

  // Cached evo availability
  mutable int evoAvailable_ = -1;  // -1 = unknown, 0 = no, 1 = yes
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_EKF_METRICS_ANALYZER_HPP
