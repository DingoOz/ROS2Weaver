#include "ros_weaver/core/live_param_validator.hpp"
#include "ros_weaver/core/project.hpp"

#include <QElapsedTimer>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

LiveParamValidator& LiveParamValidator::instance() {
  static LiveParamValidator instance;
  return instance;
}

LiveParamValidator::LiveParamValidator()
    : QObject(nullptr)
    , monitoringEnabled_(false)
    , monitoringIntervalMs_(5000)
    , monitoringTimer_(new QTimer(this))
{
  connect(monitoringTimer_, &QTimer::timeout,
          this, &LiveParamValidator::onMonitoringTimer);

  // Create a ROS2 node for parameter operations
  if (rclcpp::ok()) {
    try {
      paramNode_ = std::make_shared<rclcpp::Node>("ros_weaver_param_validator");
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                  "Could not create param validator node: %s", e.what());
    }
  }
}

LiveParamValidator::~LiveParamValidator() {
  monitoringTimer_->stop();
  paramNode_.reset();
}

ParamValidationResult LiveParamValidator::validateAll(const Project& project) {
  ParamValidationResult result;
  QElapsedTimer timer;
  timer.start();

  emit validationStarted();

  const auto& blocks = project.blocks();
  int totalBlocks = blocks.size();
  int currentBlock = 0;

  for (const BlockData& block : blocks) {
    if (block.parameters.isEmpty()) {
      currentBlock++;
      continue;
    }

    emit validationProgress(100 * currentBlock / totalBlocks,
                            QString("Validating %1...").arg(block.name));

    NodeParamValidation nodeResult = validateNode(block.name, block.parameters);
    result.nodeResults.append(nodeResult);

    result.totalMatches += nodeResult.matchCount;
    result.totalMismatches += nodeResult.mismatchCount;
    result.nodesChecked++;

    if (!nodeResult.nodeExists) {
      result.nodesNotFound++;
    }

    currentBlock++;
  }

  result.validationComplete = true;
  result.validationTime = QString("%1 ms").arg(timer.elapsed());

  emit validationProgress(100, "Validation complete");
  emit validationCompleted(result);

  return result;
}

NodeParamValidation LiveParamValidator::validateNode(
    const QString& nodeName,
    const QList<BlockParamData>& projectParams) {

  NodeParamValidation result;
  result.nodeName = nodeName;

  // Get live parameters
  QMap<QString, QVariant> liveParams = getLiveParameters(nodeName);
  result.nodeExists = !liveParams.isEmpty();

  if (!result.nodeExists) {
    result.errorMessage = QString("Node '%1' not found or not responding").arg(nodeName);

    // Mark all project params as "only in project"
    for (const BlockParamData& param : projectParams) {
      ParamDifference diff;
      diff.nodeName = nodeName;
      diff.paramName = param.name;
      diff.projectValue = param.currentValue;
      diff.paramType = param.type;
      diff.existsInProject = true;
      diff.existsInLive = false;
      diff.status = ParamDifference::Status::OnlyInProject;
      result.differences.append(diff);
      result.mismatchCount++;
    }
    return result;
  }

  // Build set of project param names
  QSet<QString> projectParamNames;
  for (const BlockParamData& param : projectParams) {
    projectParamNames.insert(param.name);
  }

  // Check each project parameter against live
  for (const BlockParamData& param : projectParams) {
    ParamDifference diff;
    diff.nodeName = nodeName;
    diff.paramName = param.name;
    diff.projectValue = param.currentValue;
    diff.paramType = param.type;
    diff.existsInProject = true;

    if (liveParams.contains(param.name)) {
      diff.existsInLive = true;
      diff.liveValue = liveParams[param.name];

      if (valuesEqual(param.currentValue, diff.liveValue)) {
        diff.status = ParamDifference::Status::Match;
        result.matchCount++;
      } else {
        diff.status = ParamDifference::Status::Mismatch;
        result.mismatchCount++;
        emit parameterMismatchDetected(diff);
      }
    } else {
      diff.existsInLive = false;
      diff.status = ParamDifference::Status::OnlyInProject;
      result.mismatchCount++;
    }

    result.differences.append(diff);
  }

  // Check for live parameters not in project
  for (auto it = liveParams.begin(); it != liveParams.end(); ++it) {
    if (!projectParamNames.contains(it.key())) {
      ParamDifference diff;
      diff.nodeName = nodeName;
      diff.paramName = it.key();
      diff.liveValue = it.value();
      diff.existsInProject = false;
      diff.existsInLive = true;
      diff.status = ParamDifference::Status::OnlyInLive;
      result.differences.append(diff);
      // Don't count as mismatch - it's informational
    }
  }

  return result;
}

QMap<QString, QVariant> LiveParamValidator::getLiveParameters(const QString& nodeName) {
  QMap<QString, QVariant> params;

  if (!paramNode_ || !rclcpp::ok()) {
    return params;
  }

  try {
    // Create a parameter client for the target node
    QString fullNodeName = nodeName;
    if (!fullNodeName.startsWith('/')) {
      fullNodeName = "/" + fullNodeName;
    }

    auto paramClient = std::make_shared<rclcpp::SyncParametersClient>(
        paramNode_, fullNodeName.toStdString());

    // Wait for service with timeout
    if (!paramClient->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_DEBUG(rclcpp::get_logger("ros_weaver"),
                   "Parameter service not available for node: %s",
                   fullNodeName.toStdString().c_str());
      return params;
    }

    // Get list of parameters
    auto paramNames = paramClient->list_parameters({}, 10);

    // Get parameter values
    auto paramValues = paramClient->get_parameters(paramNames.names);

    for (const auto& param : paramValues) {
      params[QString::fromStdString(param.get_name())] = rclcppParamToQVariant(param);
    }

  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Error getting parameters from %s: %s",
                nodeName.toStdString().c_str(), e.what());
  }

  return params;
}

bool LiveParamValidator::setLiveParameter(const QString& nodeName,
                                           const QString& paramName,
                                           const QVariant& value) {
  if (!paramNode_ || !rclcpp::ok()) {
    return false;
  }

  try {
    QString fullNodeName = nodeName;
    if (!fullNodeName.startsWith('/')) {
      fullNodeName = "/" + fullNodeName;
    }

    auto paramClient = std::make_shared<rclcpp::SyncParametersClient>(
        paramNode_, fullNodeName.toStdString());

    if (!paramClient->wait_for_service(std::chrono::seconds(1))) {
      return false;
    }

    // Determine parameter type and convert
    rclcpp::ParameterValue paramValue = qVariantToRclcppParam(value, "");

    std::vector<rclcpp::Parameter> params;
    params.emplace_back(paramName.toStdString(), paramValue);

    auto results = paramClient->set_parameters(params);

    if (!results.empty() && results[0].successful) {
      emit liveParameterChanged(nodeName, paramName, value);
      return true;
    }

  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Error setting parameter %s on %s: %s",
                paramName.toStdString().c_str(),
                nodeName.toStdString().c_str(), e.what());
  }

  return false;
}

int LiveParamValidator::syncToLive(const Project& project) {
  int synced = 0;

  for (const BlockData& block : project.blocks()) {
    for (const BlockParamData& param : block.parameters) {
      if (setLiveParameter(block.name, param.name, param.currentValue)) {
        synced++;
      }
    }
  }

  return synced;
}

int LiveParamValidator::syncFromLive(Project& project) {
  int synced = 0;

  for (BlockData& block : project.blocks()) {
    QMap<QString, QVariant> liveParams = getLiveParameters(block.name);

    for (BlockParamData& param : block.parameters) {
      if (liveParams.contains(param.name)) {
        param.currentValue = liveParams[param.name];
        synced++;
      }
    }
  }

  return synced;
}

void LiveParamValidator::setMonitoringEnabled(bool enabled) {
  monitoringEnabled_ = enabled;
  if (enabled) {
    monitoringTimer_->start(monitoringIntervalMs_);
  } else {
    monitoringTimer_->stop();
  }
}

void LiveParamValidator::setMonitoringInterval(int intervalMs) {
  monitoringIntervalMs_ = qMax(1000, intervalMs);  // Minimum 1 second
  if (monitoringTimer_->isActive()) {
    monitoringTimer_->setInterval(monitoringIntervalMs_);
  }
}

void LiveParamValidator::onMonitoringTimer() {
  // Check for parameter changes on monitored nodes
  for (auto nodeIt = lastKnownParams_.begin(); nodeIt != lastKnownParams_.end(); ++nodeIt) {
    QString nodeName = nodeIt.key();
    QMap<QString, QVariant> currentParams = getLiveParameters(nodeName);

    for (auto paramIt = currentParams.begin(); paramIt != currentParams.end(); ++paramIt) {
      QString paramName = paramIt.key();
      QVariant newValue = paramIt.value();

      if (nodeIt.value().contains(paramName)) {
        QVariant oldValue = nodeIt.value()[paramName];
        if (!valuesEqual(oldValue, newValue)) {
          emit liveParameterChanged(nodeName, paramName, newValue);
        }
      }
    }

    // Update last known values
    lastKnownParams_[nodeName] = currentParams;
  }
}

QVariant LiveParamValidator::rclcppParamToQVariant(const rclcpp::Parameter& param) {
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return QVariant(param.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return QVariant(static_cast<qlonglong>(param.as_int()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return QVariant(param.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return QVariant(QString::fromStdString(param.as_string()));
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
      QByteArray bytes;
      for (uint8_t b : param.as_byte_array()) {
        bytes.append(static_cast<char>(b));
      }
      return QVariant(bytes);
    }
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      QVariantList list;
      for (bool b : param.as_bool_array()) {
        list.append(b);
      }
      return QVariant(list);
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      QVariantList list;
      for (int64_t i : param.as_integer_array()) {
        list.append(static_cast<qlonglong>(i));
      }
      return QVariant(list);
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      QVariantList list;
      for (double d : param.as_double_array()) {
        list.append(d);
      }
      return QVariant(list);
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      QStringList list;
      for (const std::string& s : param.as_string_array()) {
        list.append(QString::fromStdString(s));
      }
      return QVariant(list);
    }
    default:
      return QVariant();
  }
}

rclcpp::ParameterValue LiveParamValidator::qVariantToRclcppParam(
    const QVariant& value, const QString& /*type*/) {

  switch (static_cast<QMetaType::Type>(value.type())) {
    case QMetaType::Bool:
      return rclcpp::ParameterValue(value.toBool());
    case QMetaType::Int:
    case QMetaType::LongLong:
      return rclcpp::ParameterValue(static_cast<int64_t>(value.toLongLong()));
    case QMetaType::Double:
    case QMetaType::Float:
      return rclcpp::ParameterValue(value.toDouble());
    case QMetaType::QString:
      return rclcpp::ParameterValue(value.toString().toStdString());
    case QMetaType::QStringList: {
      std::vector<std::string> vec;
      for (const QString& s : value.toStringList()) {
        vec.push_back(s.toStdString());
      }
      return rclcpp::ParameterValue(vec);
    }
    case QMetaType::QVariantList: {
      QVariantList list = value.toList();
      if (list.isEmpty()) {
        return rclcpp::ParameterValue(std::vector<std::string>());
      }
      // Check first element to determine array type
      QMetaType::Type elemType = static_cast<QMetaType::Type>(list[0].type());
      if (elemType == QMetaType::Bool) {
        std::vector<bool> vec;
        for (const QVariant& v : list) {
          vec.push_back(v.toBool());
        }
        return rclcpp::ParameterValue(vec);
      } else if (elemType == QMetaType::Int ||
                 elemType == QMetaType::LongLong) {
        std::vector<int64_t> vec;
        for (const QVariant& v : list) {
          vec.push_back(v.toLongLong());
        }
        return rclcpp::ParameterValue(vec);
      } else if (elemType == QMetaType::Double ||
                 elemType == QMetaType::Float) {
        std::vector<double> vec;
        for (const QVariant& v : list) {
          vec.push_back(v.toDouble());
        }
        return rclcpp::ParameterValue(vec);
      } else {
        std::vector<std::string> vec;
        for (const QVariant& v : list) {
          vec.push_back(v.toString().toStdString());
        }
        return rclcpp::ParameterValue(vec);
      }
    }
    default:
      return rclcpp::ParameterValue(value.toString().toStdString());
  }
}

bool LiveParamValidator::valuesEqual(const QVariant& a, const QVariant& b) {
  // Handle numeric comparisons with tolerance
  QMetaType::Type aType = static_cast<QMetaType::Type>(a.type());
  QMetaType::Type bType = static_cast<QMetaType::Type>(b.type());
  if ((aType == QMetaType::Double || aType == QMetaType::Float) &&
      (bType == QMetaType::Double || bType == QMetaType::Float)) {
    return qAbs(a.toDouble() - b.toDouble()) < 1e-9;
  }

  return a == b;
}

}  // namespace ros_weaver
