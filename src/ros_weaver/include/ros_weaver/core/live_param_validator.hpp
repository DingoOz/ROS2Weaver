#ifndef ROS_WEAVER_CORE_LIVE_PARAM_VALIDATOR_HPP
#define ROS_WEAVER_CORE_LIVE_PARAM_VALIDATOR_HPP

#include <QObject>
#include <QString>
#include <QVariant>
#include <QMap>
#include <QList>
#include <QTimer>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

// Represents a single parameter difference
struct ParamDifference {
  QString nodeName;
  QString paramName;
  QVariant projectValue;    // Value in the project
  QVariant liveValue;       // Value on the live node
  QString paramType;        // Parameter type
  bool existsInProject = false;
  bool existsInLive = false;

  enum class Status {
    Match,          // Values are equal
    Mismatch,       // Different values
    OnlyInProject,  // Exists in project but not live
    OnlyInLive,     // Exists in live but not project
    TypeMismatch    // Same name but different types
  };
  Status status = Status::Match;
};

// Result of validating parameters for a node
struct NodeParamValidation {
  QString nodeName;
  bool nodeExists = false;
  QList<ParamDifference> differences;
  int matchCount = 0;
  int mismatchCount = 0;
  QString errorMessage;
};

// Overall validation result
struct ParamValidationResult {
  QList<NodeParamValidation> nodeResults;
  int totalMatches = 0;
  int totalMismatches = 0;
  int nodesChecked = 0;
  int nodesNotFound = 0;
  bool validationComplete = false;
  QString validationTime;
};

// Live parameter validator - compares project params with running nodes
class LiveParamValidator : public QObject {
  Q_OBJECT

public:
  static LiveParamValidator& instance();

  // Prevent copying
  LiveParamValidator(const LiveParamValidator&) = delete;
  LiveParamValidator& operator=(const LiveParamValidator&) = delete;

  // Validate all parameters in project against live nodes
  ParamValidationResult validateAll(const class Project& project);

  // Validate parameters for a specific node
  NodeParamValidation validateNode(const QString& nodeName,
                                   const QList<struct BlockParamData>& projectParams);

  // Get live parameters from a running node
  QMap<QString, QVariant> getLiveParameters(const QString& nodeName);

  // Push a parameter value to a live node
  bool setLiveParameter(const QString& nodeName, const QString& paramName,
                        const QVariant& value);

  // Sync project parameters to live nodes
  int syncToLive(const class Project& project);

  // Sync live parameters back to project
  int syncFromLive(class Project& project);

  // Enable/disable continuous monitoring
  void setMonitoringEnabled(bool enabled);
  bool isMonitoringEnabled() const { return monitoringEnabled_; }

  // Set monitoring interval (in milliseconds)
  void setMonitoringInterval(int intervalMs);
  int monitoringInterval() const { return monitoringIntervalMs_; }

signals:
  void validationStarted();
  void validationProgress(int percent, const QString& currentNode);
  void validationCompleted(const ParamValidationResult& result);
  void parameterMismatchDetected(const ParamDifference& diff);
  void liveParameterChanged(const QString& nodeName, const QString& paramName,
                            const QVariant& newValue);

private slots:
  void onMonitoringTimer();

private:
  LiveParamValidator();
  ~LiveParamValidator() override;

  // Convert ROS parameter to QVariant
  QVariant rclcppParamToQVariant(const rclcpp::Parameter& param);

  // Convert QVariant to ROS parameter
  rclcpp::ParameterValue qVariantToRclcppParam(const QVariant& value,
                                                const QString& type);

  // Compare two parameter values
  bool valuesEqual(const QVariant& a, const QVariant& b);

  // ROS2 node for parameter operations
  std::shared_ptr<rclcpp::Node> paramNode_;

  // Monitoring
  bool monitoringEnabled_;
  int monitoringIntervalMs_;
  QTimer* monitoringTimer_;
  QMap<QString, QMap<QString, QVariant>> lastKnownParams_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LIVE_PARAM_VALIDATOR_HPP
