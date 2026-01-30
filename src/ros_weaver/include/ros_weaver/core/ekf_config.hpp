#ifndef ROS_WEAVER_CORE_EKF_CONFIG_HPP
#define ROS_WEAVER_CORE_EKF_CONFIG_HPP

#include <QString>
#include <QStringList>
#include <QVector>
#include <QMap>
#include <QVariant>
#include <QColor>

#include <array>
#include <optional>
#include <cmath>

namespace ros_weaver {

/**
 * @brief State indices for the 15-state EKF
 *
 * The robot_localization EKF uses a 15-dimensional state vector:
 * [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, x_ddot, y_ddot, z_ddot]
 */
enum class EKFStateIndex {
  X = 0,
  Y = 1,
  Z = 2,
  Roll = 3,
  Pitch = 4,
  Yaw = 5,
  VX = 6,
  VY = 7,
  VZ = 8,
  VRoll = 9,
  VPitch = 10,
  VYaw = 11,
  AX = 12,
  AY = 13,
  AZ = 14
};

constexpr int EKF_STATE_SIZE = 15;

/**
 * @brief Get human-readable label for a state index
 */
inline QString stateIndexLabel(int index) {
  static const QStringList labels = {
    "X", "Y", "Z", "Roll", "Pitch", "Yaw",
    "VX", "VY", "VZ", "VRoll", "VPitch", "VYaw",
    "AX", "AY", "AZ"
  };
  if (index >= 0 && index < labels.size()) {
    return labels[index];
  }
  return QString("State%1").arg(index);
}

/**
 * @brief Sensor configuration for a single sensor input
 *
 * Represents configuration for odom, imu, pose, or twist sensors.
 */
struct EKFSensorConfig {
  QString topic;              // ROS topic name
  bool enabled = false;       // Whether this sensor is used

  // Which states to fuse from this sensor (15-element boolean array)
  // Order: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
  std::array<bool, EKF_STATE_SIZE> config = {};

  // Differential mode (for odom/pose: integrate velocities instead of absolute poses)
  bool differential = false;

  // Relative mode (transforms are relative to first measurement)
  bool relative = false;

  // Queue size
  int queueSize = 10;

  // Rejection threshold (Mahalanobis distance)
  double rejectionThreshold = 0.0;  // 0 = no rejection

  // Remove gravitational acceleration (for IMU)
  bool removeGravitationalAcceleration = true;

  EKFSensorConfig() {
    config.fill(false);
  }
};

/**
 * @brief Complete EKF configuration matching robot_localization parameters
 */
struct EKFConfig {
  // Node configuration
  QString name = "ekf_filter_node";
  double frequency = 30.0;

  // Frame IDs
  QString mapFrame = "map";
  QString odomFrame = "odom";
  QString baseFrame = "base_link";
  QString worldFrame = "odom";  // Which frame to publish (odom or map)

  // Transform settings
  bool publishTf = true;
  double transformTimeout = 0.0;
  double transformTimeOffset = 0.0;

  // 2D mode (constrains filter to planar motion)
  bool twoDMode = false;

  // Sensor configurations (up to 4 of each type supported)
  QVector<EKFSensorConfig> odomSensors;
  QVector<EKFSensorConfig> imuSensors;
  QVector<EKFSensorConfig> poseSensors;
  QVector<EKFSensorConfig> twistSensors;

  // Process noise covariance (diagonal, 15 elements)
  // Default values from robot_localization
  std::array<double, EKF_STATE_SIZE> processNoiseCovariance = {
    0.05,   // X
    0.05,   // Y
    0.06,   // Z
    0.03,   // Roll
    0.03,   // Pitch
    0.06,   // Yaw
    0.025,  // VX
    0.025,  // VY
    0.04,   // VZ
    0.01,   // VRoll
    0.01,   // VPitch
    0.02,   // VYaw
    0.01,   // AX
    0.01,   // AY
    0.015   // AZ
  };

  // Initial estimate covariance (diagonal, 15 elements)
  std::array<double, EKF_STATE_SIZE> initialEstimateCovariance = {
    1e-9, 1e-9, 1e-9,      // Position
    1e-9, 1e-9, 1e-9,      // Orientation
    1e-9, 1e-9, 1e-9,      // Linear velocity
    1e-9, 1e-9, 1e-9,      // Angular velocity
    1e-9, 1e-9, 1e-9       // Linear acceleration
  };

  // Dynamic process noise (scale process noise based on velocity)
  bool dynamicProcessNoiseCovariance = false;

  // Gravitational acceleration magnitude
  double gravitationalAcceleration = 9.80665;

  // Permit corrected publication (for navsat transform)
  bool permitCorrectedPublication = false;

  // Prediction step control
  bool disablePrediction = false;

  // Use control input
  bool useControl = false;
  double controlTimeout = 0.2;
  std::array<double, 6> controlConfig = {false, false, false, false, false, false};
  std::array<double, 6> accelerationLimits = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> decelerationLimits = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> accelerationGains = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> decelerationGains = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Smoothing/lag settings
  bool smooth_lagged_data = false;
  int history_length = 0;

  // Reset service
  bool reset_on_time_jump = false;

  // Output topic configuration
  QString outputTopic = "odometry/filtered";

  // Debug mode
  bool printDiagnostics = false;

  // Validation
  bool isValid() const {
    // At least one sensor must be configured
    bool hasSensor = false;
    for (const auto& sensor : odomSensors) {
      if (sensor.enabled) hasSensor = true;
    }
    for (const auto& sensor : imuSensors) {
      if (sensor.enabled) hasSensor = true;
    }
    for (const auto& sensor : poseSensors) {
      if (sensor.enabled) hasSensor = true;
    }
    for (const auto& sensor : twistSensors) {
      if (sensor.enabled) hasSensor = true;
    }

    return hasSensor && frequency > 0.0;
  }

  // Create default config with odom + IMU
  static EKFConfig defaultConfig() {
    EKFConfig config;

    // Add default odometry sensor (fuse x, y, yaw)
    EKFSensorConfig odom;
    odom.topic = "/odom";
    odom.enabled = true;
    odom.config[static_cast<int>(EKFStateIndex::X)] = true;
    odom.config[static_cast<int>(EKFStateIndex::Y)] = true;
    odom.config[static_cast<int>(EKFStateIndex::Yaw)] = true;
    config.odomSensors.append(odom);

    // Add default IMU sensor (fuse roll, pitch, yaw velocity)
    EKFSensorConfig imu;
    imu.topic = "/imu/data";
    imu.enabled = true;
    imu.config[static_cast<int>(EKFStateIndex::Roll)] = true;
    imu.config[static_cast<int>(EKFStateIndex::Pitch)] = true;
    imu.config[static_cast<int>(EKFStateIndex::VYaw)] = true;
    config.imuSensors.append(imu);

    return config;
  }
};

/**
 * @brief Metrics from EKF trajectory evaluation
 */
struct EKFMetrics {
  // Absolute Trajectory Error (ATE)
  double ateRmse = 0.0;      // Root mean square error
  double ateMean = 0.0;      // Mean error
  double ateMedian = 0.0;    // Median error
  double ateStd = 0.0;       // Standard deviation
  double ateMin = 0.0;       // Minimum error
  double ateMax = 0.0;       // Maximum error

  // Relative Pose Error (RPE)
  double rpeRmse = 0.0;
  double rpeMean = 0.0;
  double rpeMedian = 0.0;
  double rpeStd = 0.0;

  // Yaw drift (degrees per meter traveled)
  double yawDrift = 0.0;

  // Total trajectory length
  double trajectoryLength = 0.0;

  // Number of poses compared
  int poseCount = 0;

  // Whether ground truth was available
  bool hasGroundTruth = false;

  // Comparison configuration
  QString configName;
  QString bagPath;
};

/**
 * @brief A single pose in a trajectory
 */
struct TrajectoryPose {
  double timestamp = 0.0;  // Seconds since bag start
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

/**
 * @brief A trajectory (sequence of poses)
 */
struct Trajectory {
  QString name;
  QString topic;
  QVector<TrajectoryPose> poses;
  QColor color;

  double length() const {
    double len = 0.0;
    for (int i = 1; i < poses.size(); ++i) {
      double dx = poses[i].x - poses[i-1].x;
      double dy = poses[i].y - poses[i-1].y;
      double dz = poses[i].z - poses[i-1].z;
      len += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return len;
  }
};

/**
 * @brief Parameter sweep configuration
 */
struct EKFSweepParameter {
  QString name;           // Parameter path (e.g., "processNoiseCovariance.0")
  double minValue = 0.0;
  double maxValue = 1.0;
  int steps = 3;          // Number of values to try
  bool logScale = false;  // Use logarithmic spacing

  QVector<double> values() const {
    QVector<double> result;
    for (int i = 0; i < steps; ++i) {
      double t = (steps > 1) ? static_cast<double>(i) / (steps - 1) : 0.0;
      double value;
      if (logScale && minValue > 0.0 && maxValue > 0.0) {
        double logMin = std::log10(minValue);
        double logMax = std::log10(maxValue);
        value = std::pow(10.0, logMin + t * (logMax - logMin));
      } else {
        value = minValue + t * (maxValue - minValue);
      }
      result.append(value);
    }
    return result;
  }
};

/**
 * @brief Result of a parameter sweep run
 */
struct EKFSweepResult {
  EKFConfig config;
  EKFMetrics metrics;
  int sweepIndex = 0;
  QMap<QString, double> parameterValues;  // Which parameter values were used
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::EKFConfig)
Q_DECLARE_METATYPE(ros_weaver::EKFSensorConfig)
Q_DECLARE_METATYPE(ros_weaver::EKFMetrics)
Q_DECLARE_METATYPE(ros_weaver::Trajectory)
Q_DECLARE_METATYPE(ros_weaver::TrajectoryPose)
Q_DECLARE_METATYPE(ros_weaver::EKFSweepParameter)
Q_DECLARE_METATYPE(ros_weaver::EKFSweepResult)

#endif  // ROS_WEAVER_CORE_EKF_CONFIG_HPP
