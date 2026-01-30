#include "ros_weaver/core/ekf_metrics_analyzer.hpp"

#include <QFile>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTemporaryDir>
#include <QProcess>
#include <QDebug>

#include <cmath>
#include <algorithm>
#include <numeric>

namespace ros_weaver {

EKFMetricsAnalyzer::EKFMetricsAnalyzer(QObject* parent)
  : QObject(parent)
{
}

EKFMetricsAnalyzer::~EKFMetricsAnalyzer() = default;

EKFMetrics EKFMetricsAnalyzer::computeMetrics(const Trajectory& estimated,
                                               const Trajectory& reference) {
  if (estimated.poses.isEmpty()) {
    emit errorOccurred("Estimated trajectory is empty");
    return EKFMetrics();
  }

  if (reference.poses.isEmpty()) {
    emit errorOccurred("Reference trajectory is empty");
    return EKFMetrics();
  }

  emit progressChanged(0, "Computing metrics...");

  EKFMetrics metrics;
  metrics.hasGroundTruth = true;

  // Try evo if available and not forcing direct computation
  if (!useDirectComputation_ && isEvoAvailable()) {
    metrics = computeMetricsEvo(estimated, reference);
  } else {
    metrics = computeMetricsDirect(estimated, reference);
  }

  emit metricsComputed(metrics);
  return metrics;
}

EKFMetrics EKFMetricsAnalyzer::computeRelativeMetrics(const Trajectory& estimated,
                                                       const Trajectory& rawOdom) {
  // Without ground truth, compute relative improvement metrics
  EKFMetrics metrics;
  metrics.hasGroundTruth = false;

  if (estimated.poses.isEmpty() || rawOdom.poses.isEmpty()) {
    return metrics;
  }

  // Compute trajectory lengths
  metrics.trajectoryLength = estimated.length();

  // For relative metrics without ground truth, we can compute:
  // - Smoothness (jerk magnitude)
  // - Consistency with odometry

  auto pairs = associatePoses(estimated, rawOdom);
  if (pairs.isEmpty()) {
    return metrics;
  }

  // Compute position differences
  QVector<double> posDiffs;
  for (const auto& pair : pairs) {
    double dx = pair.first.x - pair.second.x;
    double dy = pair.first.y - pair.second.y;
    double dz = pair.first.z - pair.second.z;
    posDiffs.append(std::sqrt(dx*dx + dy*dy + dz*dz));
  }

  // ATE-like metrics (deviation from raw odometry)
  if (!posDiffs.isEmpty()) {
    double sum = 0.0, sumSq = 0.0;
    double minVal = posDiffs[0], maxVal = posDiffs[0];

    for (double d : posDiffs) {
      sum += d;
      sumSq += d * d;
      minVal = std::min(minVal, d);
      maxVal = std::max(maxVal, d);
    }

    int n = posDiffs.size();
    metrics.ateMean = sum / n;
    metrics.ateRmse = std::sqrt(sumSq / n);
    metrics.ateMin = minVal;
    metrics.ateMax = maxVal;

    // Median
    std::sort(posDiffs.begin(), posDiffs.end());
    metrics.ateMedian = posDiffs[n / 2];

    // Std dev
    double variance = (sumSq / n) - (metrics.ateMean * metrics.ateMean);
    metrics.ateStd = std::sqrt(std::max(0.0, variance));

    metrics.poseCount = n;
  }

  emit metricsComputed(metrics);
  return metrics;
}

bool EKFMetricsAnalyzer::isEvoAvailable() const {
  if (evoAvailable_ < 0) {
    QProcess process;
    process.start("evo_ape", {"--version"});
    process.waitForFinished(3000);
    evoAvailable_ = (process.exitCode() == 0) ? 1 : 0;
  }
  return evoAvailable_ == 1;
}

void EKFMetricsAnalyzer::setUseDirectComputation(bool direct) {
  useDirectComputation_ = direct;
}

bool EKFMetricsAnalyzer::useDirectComputation() const {
  return useDirectComputation_;
}

void EKFMetricsAnalyzer::setAlignTrajectories(bool align) {
  alignTrajectories_ = align;
}

bool EKFMetricsAnalyzer::alignTrajectories() const {
  return alignTrajectories_;
}

void EKFMetricsAnalyzer::setMaxTimeDelta(double delta) {
  maxTimeDelta_ = delta;
}

double EKFMetricsAnalyzer::maxTimeDelta() const {
  return maxTimeDelta_;
}

EKFMetrics EKFMetricsAnalyzer::computeMetricsDirect(const Trajectory& estimated,
                                                     const Trajectory& reference) {
  EKFMetrics metrics;
  metrics.hasGroundTruth = true;
  metrics.trajectoryLength = reference.length();

  // Associate poses by timestamp
  auto pairs = associatePoses(estimated, reference);

  if (pairs.isEmpty()) {
    emit errorOccurred("No matching poses found between trajectories");
    return metrics;
  }

  emit progressChanged(30, "Aligning trajectories...");

  // Optionally align trajectories
  if (alignTrajectories_) {
    umeyamaAlign(pairs);
  }

  emit progressChanged(50, "Computing ATE...");

  // Compute ATE (Absolute Trajectory Error)
  QVector<double> posErrors;
  QVector<double> yawErrors;

  for (const auto& pair : pairs) {
    double dx = pair.first.x - pair.second.x;
    double dy = pair.first.y - pair.second.y;
    double dz = pair.first.z - pair.second.z;
    double posError = std::sqrt(dx*dx + dy*dy + dz*dz);
    posErrors.append(posError);

    double yawError = std::abs(angleDiff(pair.first.yaw, pair.second.yaw));
    yawErrors.append(yawError);
  }

  // ATE statistics
  double sum = 0.0, sumSq = 0.0;
  double minVal = posErrors[0], maxVal = posErrors[0];

  for (double e : posErrors) {
    sum += e;
    sumSq += e * e;
    minVal = std::min(minVal, e);
    maxVal = std::max(maxVal, e);
  }

  int n = posErrors.size();
  metrics.ateMean = sum / n;
  metrics.ateRmse = std::sqrt(sumSq / n);
  metrics.ateMin = minVal;
  metrics.ateMax = maxVal;

  std::sort(posErrors.begin(), posErrors.end());
  metrics.ateMedian = posErrors[n / 2];

  double variance = (sumSq / n) - (metrics.ateMean * metrics.ateMean);
  metrics.ateStd = std::sqrt(std::max(0.0, variance));

  emit progressChanged(70, "Computing RPE...");

  // Compute RPE (Relative Pose Error)
  QVector<double> rpeErrors;
  for (int i = 1; i < pairs.size(); ++i) {
    // Relative motion in estimated
    double est_dx = pairs[i].first.x - pairs[i-1].first.x;
    double est_dy = pairs[i].first.y - pairs[i-1].first.y;
    (void)angleDiff(pairs[i].first.yaw, pairs[i-1].first.yaw);  // est_dyaw for future use

    // Relative motion in reference
    double ref_dx = pairs[i].second.x - pairs[i-1].second.x;
    double ref_dy = pairs[i].second.y - pairs[i-1].second.y;
    (void)angleDiff(pairs[i].second.yaw, pairs[i-1].second.yaw);  // ref_dyaw for future use

    // RPE
    double dx = est_dx - ref_dx;
    double dy = est_dy - ref_dy;
    double rpe = std::sqrt(dx*dx + dy*dy);
    rpeErrors.append(rpe);
  }

  if (!rpeErrors.isEmpty()) {
    double rpeSum = 0.0, rpeSumSq = 0.0;
    for (double e : rpeErrors) {
      rpeSum += e;
      rpeSumSq += e * e;
    }

    int rpeN = rpeErrors.size();
    metrics.rpeMean = rpeSum / rpeN;
    metrics.rpeRmse = std::sqrt(rpeSumSq / rpeN);

    std::sort(rpeErrors.begin(), rpeErrors.end());
    metrics.rpeMedian = rpeErrors[rpeN / 2];

    double rpeVariance = (rpeSumSq / rpeN) - (metrics.rpeMean * metrics.rpeMean);
    metrics.rpeStd = std::sqrt(std::max(0.0, rpeVariance));
  }

  emit progressChanged(90, "Computing yaw drift...");

  // Compute yaw drift (degrees per meter)
  if (metrics.trajectoryLength > 0.0 && !yawErrors.empty()) {
    double totalYawError = 0.0;
    for (double e : yawErrors) {
      totalYawError += e;
    }
    double avgYawError = totalYawError / yawErrors.size();
    metrics.yawDrift = (avgYawError * 180.0 / M_PI) / metrics.trajectoryLength;
  }

  metrics.poseCount = n;

  emit progressChanged(100, "Metrics computed");

  return metrics;
}

EKFMetrics EKFMetricsAnalyzer::computeMetricsEvo(const Trajectory& estimated,
                                                  const Trajectory& reference) {
  EKFMetrics metrics;
  metrics.hasGroundTruth = true;

  // Create temp directory for trajectory files
  QTemporaryDir tempDir;
  if (!tempDir.isValid()) {
    emit errorOccurred("Failed to create temp directory for evo");
    return computeMetricsDirect(estimated, reference);
  }

  QString estPath = tempDir.path() + "/estimated.tum";
  QString refPath = tempDir.path() + "/reference.tum";
  QString outputPath = tempDir.path() + "/results.json";

  emit progressChanged(10, "Saving trajectories...");

  if (!saveTrajectoryTUM(estimated, estPath) ||
      !saveTrajectoryTUM(reference, refPath)) {
    return computeMetricsDirect(estimated, reference);
  }

  emit progressChanged(30, "Running evo_ape...");

  // Run evo_ape
  QProcess evoProcess;
  QStringList args;
  args << "tum" << refPath << estPath
       << "--align" << "--correct_scale"
       << "--save_results" << outputPath;

  evoProcess.start("evo_ape", args);

  if (!evoProcess.waitForFinished(30000)) {
    qWarning() << "evo_ape timed out, falling back to direct computation";
    return computeMetricsDirect(estimated, reference);
  }

  if (evoProcess.exitCode() != 0) {
    qWarning() << "evo_ape failed:" << evoProcess.readAllStandardError();
    return computeMetricsDirect(estimated, reference);
  }

  emit progressChanged(70, "Parsing results...");

  // Parse results
  metrics = parseEvoJson(outputPath);
  metrics.trajectoryLength = reference.length();
  metrics.hasGroundTruth = true;

  // Run evo_rpe for RPE metrics
  emit progressChanged(80, "Running evo_rpe...");

  QString rpeOutputPath = tempDir.path() + "/rpe_results.json";
  QStringList rpeArgs;
  rpeArgs << "tum" << refPath << estPath
          << "--align"
          << "--save_results" << rpeOutputPath;

  QProcess rpeProcess;
  rpeProcess.start("evo_rpe", rpeArgs);

  if (rpeProcess.waitForFinished(30000) && rpeProcess.exitCode() == 0) {
    EKFMetrics rpeMetrics = parseEvoJson(rpeOutputPath);
    metrics.rpeRmse = rpeMetrics.ateRmse;  // evo_rpe outputs to same fields
    metrics.rpeMean = rpeMetrics.ateMean;
    metrics.rpeMedian = rpeMetrics.ateMedian;
    metrics.rpeStd = rpeMetrics.ateStd;
  }

  emit progressChanged(100, "Metrics computed");

  return metrics;
}

QVector<std::pair<TrajectoryPose, TrajectoryPose>> EKFMetricsAnalyzer::associatePoses(
    const Trajectory& estimated,
    const Trajectory& reference) const {

  QVector<std::pair<TrajectoryPose, TrajectoryPose>> pairs;

  // For each estimated pose, find closest reference pose by timestamp
  for (const auto& estPose : estimated.poses) {
    double minDelta = maxTimeDelta_;
    int bestIdx = -1;

    for (int i = 0; i < reference.poses.size(); ++i) {
      double delta = std::abs(estPose.timestamp - reference.poses[i].timestamp);
      if (delta < minDelta) {
        minDelta = delta;
        bestIdx = i;
      }
    }

    if (bestIdx >= 0) {
      pairs.append({estPose, reference.poses[bestIdx]});
    }
  }

  return pairs;
}

bool EKFMetricsAnalyzer::saveTrajectoryTUM(const Trajectory& traj, const QString& path) const {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    emit const_cast<EKFMetricsAnalyzer*>(this)->errorOccurred(
        QString("Failed to save trajectory: %1").arg(path));
    return false;
  }

  QTextStream stream(&file);
  stream.setRealNumberPrecision(9);

  // TUM format: timestamp x y z qx qy qz qw
  for (const auto& pose : traj.poses) {
    // Convert euler to quaternion
    double cy = std::cos(pose.yaw * 0.5);
    double sy = std::sin(pose.yaw * 0.5);
    double cp = std::cos(pose.pitch * 0.5);
    double sp = std::sin(pose.pitch * 0.5);
    double cr = std::cos(pose.roll * 0.5);
    double sr = std::sin(pose.roll * 0.5);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    stream << pose.timestamp << " "
           << pose.x << " " << pose.y << " " << pose.z << " "
           << qx << " " << qy << " " << qz << " " << qw << "\n";
  }

  file.close();
  return true;
}

EKFMetrics EKFMetricsAnalyzer::parseEvoJson(const QString& jsonPath) const {
  EKFMetrics metrics;

  QFile file(jsonPath);
  if (!file.open(QIODevice::ReadOnly)) {
    return metrics;
  }

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  QJsonObject root = doc.object();

  if (root.contains("rmse")) {
    metrics.ateRmse = root["rmse"].toDouble();
  }
  if (root.contains("mean")) {
    metrics.ateMean = root["mean"].toDouble();
  }
  if (root.contains("median")) {
    metrics.ateMedian = root["median"].toDouble();
  }
  if (root.contains("std")) {
    metrics.ateStd = root["std"].toDouble();
  }
  if (root.contains("min")) {
    metrics.ateMin = root["min"].toDouble();
  }
  if (root.contains("max")) {
    metrics.ateMax = root["max"].toDouble();
  }

  return metrics;
}

void EKFMetricsAnalyzer::umeyamaAlign(
    QVector<std::pair<TrajectoryPose, TrajectoryPose>>& pairs) const {

  if (pairs.size() < 3) {
    return;  // Need at least 3 points for meaningful alignment
  }

  // Compute centroids
  double estCx = 0, estCy = 0, estCz = 0;
  double refCx = 0, refCy = 0, refCz = 0;

  for (const auto& pair : pairs) {
    estCx += pair.first.x;
    estCy += pair.first.y;
    estCz += pair.first.z;
    refCx += pair.second.x;
    refCy += pair.second.y;
    refCz += pair.second.z;
  }

  int n = pairs.size();
  estCx /= n; estCy /= n; estCz /= n;
  refCx /= n; refCy /= n; refCz /= n;

  // For 2D alignment (common case), compute rotation and translation
  // Using SVD-based Umeyama alignment (simplified 2D version)

  double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;

  for (const auto& pair : pairs) {
    double ex = pair.first.x - estCx;
    double ey = pair.first.y - estCy;
    double rx = pair.second.x - refCx;
    double ry = pair.second.y - refCy;

    Sxx += ex * rx;
    Sxy += ex * ry;
    Syx += ey * rx;
    Syy += ey * ry;
  }

  // Compute rotation angle
  double theta = std::atan2(Sxy - Syx, Sxx + Syy);
  double cosT = std::cos(theta);
  double sinT = std::sin(theta);

  // Apply transformation to estimated poses
  for (auto& pair : pairs) {
    double x = pair.first.x - estCx;
    double y = pair.first.y - estCy;

    pair.first.x = cosT * x - sinT * y + refCx;
    pair.first.y = sinT * x + cosT * y + refCy;
    pair.first.yaw = normalizeAngle(pair.first.yaw + theta);
  }
}

double EKFMetricsAnalyzer::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double EKFMetricsAnalyzer::angleDiff(double a, double b) {
  return normalizeAngle(a - b);
}

}  // namespace ros_weaver
