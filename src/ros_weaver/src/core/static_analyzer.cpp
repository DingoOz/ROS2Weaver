#include "ros_weaver/core/static_analyzer.hpp"
#include "ros_weaver/core/project.hpp"

#include <QElapsedTimer>
#include <QSet>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

StaticAnalyzer& StaticAnalyzer::instance() {
  static StaticAnalyzer instance;
  return instance;
}

StaticAnalyzer::StaticAnalyzer() : QObject(nullptr) {
}

AnalysisResult StaticAnalyzer::analyze(const Project& project) {
  AnalysisResult result;
  QElapsedTimer timer;
  timer.start();

  emit analysisStarted();

  int totalSteps = 0;
  if (rules_.checkTypeMismatches) totalSteps++;
  if (rules_.checkUnusedPubsSubs) totalSteps++;
  if (rules_.checkQoSCompatibility) totalSteps++;
  if (rules_.checkCyclicDependencies) totalSteps++;
  if (rules_.checkNamingConventions) totalSteps++;
  if (rules_.checkDeprecatedFeatures) totalSteps++;
  if (rules_.checkSecurityIssues) totalSteps++;
  totalSteps++;  // Missing dependencies always checked

  int currentStep = 0;

  // Run analysis passes
  if (rules_.checkTypeMismatches) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking type compatibility..."));
    checkTypeMismatches(project, result);
    currentStep++;
  }

  if (rules_.checkUnusedPubsSubs) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking for unused pub/subs..."));
    checkUnusedPublishersSubscribers(project, result);
    currentStep++;
  }

  if (rules_.checkQoSCompatibility) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking QoS compatibility..."));
    checkQoSCompatibility(project, result);
    currentStep++;
  }

  if (rules_.checkCyclicDependencies) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking for cyclic dependencies..."));
    checkCyclicDependencies(project, result);
    currentStep++;
  }

  if (rules_.checkNamingConventions) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking naming conventions..."));
    checkNamingConventions(project, result);
    currentStep++;
  }

  if (rules_.checkDeprecatedFeatures) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking for deprecated features..."));
    checkDeprecatedFeatures(project, result);
    currentStep++;
  }

  if (rules_.checkSecurityIssues) {
    emit analysisProgress(100 * currentStep / totalSteps, tr("Checking security issues..."));
    checkSecurityIssues(project, result);
    currentStep++;
  }

  emit analysisProgress(100 * currentStep / totalSteps, tr("Checking missing dependencies..."));
  checkMissingDependencies(project, result);

  result.analysisComplete = true;
  qint64 elapsed = timer.elapsed();
  result.analysisDuration = QString("%1 ms").arg(elapsed);

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Static analysis complete: %d errors, %d warnings, %d info in %lld ms",
              result.errorCount, result.warningCount, result.infoCount, elapsed);

  emit analysisProgress(100, tr("Analysis complete"));
  emit analysisCompleted(result);

  return result;
}

void StaticAnalyzer::checkTypeMismatches(const Project& project, AnalysisResult& result) {
  // Check each connection for type compatibility
  for (const ConnectionData& conn : project.connections()) {
    // Find source and target blocks
    const BlockData* sourceBlock = nullptr;
    const BlockData* targetBlock = nullptr;

    for (const BlockData& block : project.blocks()) {
      if (block.id == conn.sourceBlockId) sourceBlock = &block;
      if (block.id == conn.targetBlockId) targetBlock = &block;
    }

    if (!sourceBlock || !targetBlock) continue;

    // Get source and target pin types
    QString sourceType, targetType;
    if (conn.sourcePinIndex >= 0 && conn.sourcePinIndex < sourceBlock->outputPins.size()) {
      sourceType = sourceBlock->outputPins[conn.sourcePinIndex].messageType;
    }
    if (conn.targetPinIndex >= 0 && conn.targetPinIndex < targetBlock->inputPins.size()) {
      targetType = targetBlock->inputPins[conn.targetPinIndex].messageType;
    }

    // Check compatibility
    if (!sourceType.isEmpty() && !targetType.isEmpty() &&
        !areTypesCompatible(sourceType, targetType)) {
      AnalysisIssue issue;
      issue.id = QUuid::createUuid();
      issue.severity = IssueSeverity::Error;
      issue.category = IssueCategory::TypeMismatch;
      issue.title = tr("Type mismatch");
      issue.description = tr("Connection between '%1' and '%2' has incompatible types: "
                             "'%3' vs '%4'")
                              .arg(sourceBlock->name, targetBlock->name, sourceType, targetType);
      issue.suggestion = tr("Ensure both ends use the same message type, or add a type converter");
      issue.affectedConnectionId = conn.id;
      addIssue(result, issue);
    }
  }
}

void StaticAnalyzer::checkUnusedPublishersSubscribers(const Project& project, AnalysisResult& result) {
  // Build maps of connected pins
  QSet<QPair<QUuid, int>> connectedOutputPins;
  QSet<QPair<QUuid, int>> connectedInputPins;

  for (const ConnectionData& conn : project.connections()) {
    connectedOutputPins.insert({conn.sourceBlockId, conn.sourcePinIndex});
    connectedInputPins.insert({conn.targetBlockId, conn.targetPinIndex});
  }

  // Check each block for unconnected pins
  for (const BlockData& block : project.blocks()) {
    // Check output pins (publishers)
    for (int i = 0; i < block.outputPins.size(); ++i) {
      if (!connectedOutputPins.contains({block.id, i})) {
        AnalysisIssue issue;
        issue.id = QUuid::createUuid();
        issue.severity = IssueSeverity::Warning;
        issue.category = IssueCategory::UnusedPublisher;
        issue.title = tr("Unused publisher");
        issue.description = tr("Output '%1' on node '%2' has no subscribers")
                                .arg(block.outputPins[i].name, block.name);
        issue.suggestion = tr("Connect this output to a subscriber or remove if unused");
        issue.affectedBlockId = block.id;
        issue.affectedElement = block.outputPins[i].name;
        addIssue(result, issue);
      }
    }

    // Check input pins (subscribers)
    for (int i = 0; i < block.inputPins.size(); ++i) {
      if (!connectedInputPins.contains({block.id, i})) {
        AnalysisIssue issue;
        issue.id = QUuid::createUuid();
        issue.severity = IssueSeverity::Warning;
        issue.category = IssueCategory::UnusedSubscriber;
        issue.title = tr("Unused subscriber");
        issue.description = tr("Input '%1' on node '%2' has no publishers")
                                .arg(block.inputPins[i].name, block.name);
        issue.suggestion = tr("Connect a publisher to this input or remove if unused");
        issue.affectedBlockId = block.id;
        issue.affectedElement = block.inputPins[i].name;
        addIssue(result, issue);
      }
    }
  }
}

void StaticAnalyzer::checkQoSCompatibility(const Project& project, AnalysisResult& result) {
  // QoS checking would require storing QoS settings on connections/pins
  // For now, this is a placeholder that could check common incompatibilities

  // Example: Check for sensor data topics that might need best-effort QoS
  for (const BlockData& block : project.blocks()) {
    for (const PinData& pin : block.outputPins) {
      if (pin.messageType.contains("sensor_msgs") ||
          pin.messageType.contains("Image") ||
          pin.messageType.contains("PointCloud")) {

        if (rules_.enableHints) {
          AnalysisIssue issue;
          issue.id = QUuid::createUuid();
          issue.severity = IssueSeverity::Hint;
          issue.category = IssueCategory::QoSIncompatible;
          issue.title = tr("QoS consideration");
          issue.description = tr("Sensor topic '%1' on '%2' may benefit from best-effort QoS")
                                  .arg(pin.name, block.name);
          issue.suggestion = tr("Consider using BEST_EFFORT reliability for high-frequency sensor data");
          issue.affectedBlockId = block.id;
          issue.affectedElement = pin.name;
          addIssue(result, issue);
        }
      }
    }
  }
}

void StaticAnalyzer::checkCyclicDependencies(const Project& project, AnalysisResult& result) {
  // Build adjacency list from connections
  QMap<QUuid, QList<QUuid>> adjacency;

  for (const ConnectionData& conn : project.connections()) {
    adjacency[conn.sourceBlockId].append(conn.targetBlockId);
  }

  // DFS to detect cycles
  QSet<QUuid> visited;
  QSet<QUuid> recursionStack;

  for (const BlockData& block : project.blocks()) {
    if (detectCycle(block.id, project, visited, recursionStack)) {
      AnalysisIssue issue;
      issue.id = QUuid::createUuid();
      issue.severity = IssueSeverity::Warning;
      issue.category = IssueCategory::CyclicDependency;
      issue.title = tr("Cyclic dependency detected");
      issue.description = tr("Node '%1' is part of a cyclic data flow").arg(block.name);
      issue.suggestion = tr("Review data flow for potential feedback loops or deadlocks");
      issue.affectedBlockId = block.id;
      addIssue(result, issue);
    }
    visited.clear();
    recursionStack.clear();
  }
}

void StaticAnalyzer::checkNamingConventions(const Project& project, AnalysisResult& result) {
  QRegularExpression validName("^[a-z][a-z0-9_]*$");

  for (const BlockData& block : project.blocks()) {
    QString name = block.name;
    // Remove namespace prefix for checking
    if (name.contains('/')) {
      name = name.mid(name.lastIndexOf('/') + 1);
    }

    if (!validName.match(name).hasMatch() && rules_.enableHints) {
      AnalysisIssue issue;
      issue.id = QUuid::createUuid();
      issue.severity = IssueSeverity::Hint;
      issue.category = IssueCategory::NamingConvention;
      issue.title = tr("Naming convention");
      issue.description = tr("Node name '%1' doesn't follow ROS2 naming conventions")
                              .arg(block.name);
      issue.suggestion = tr("Use lowercase with underscores (snake_case)");
      issue.affectedBlockId = block.id;
      addIssue(result, issue);
    }
  }
}

void StaticAnalyzer::checkDeprecatedFeatures(const Project& project, AnalysisResult& result) {
  // Check for known deprecated message types or patterns
  QStringList deprecatedTypes = {
      "std_msgs/Empty",  // Often better to use a service
  };

  for (const BlockData& block : project.blocks()) {
    for (const PinData& pin : block.inputPins) {
      for (const QString& deprecatedType : deprecatedTypes) {
        if (pin.messageType == deprecatedType && rules_.enableHints) {
          AnalysisIssue issue;
          issue.id = QUuid::createUuid();
          issue.severity = IssueSeverity::Hint;
          issue.category = IssueCategory::Deprecated;
          issue.title = tr("Consider alternative");
          issue.description = tr("Pin '%1' uses '%2' which might have better alternatives")
                                  .arg(pin.name, pin.messageType);
          issue.suggestion = tr("Consider using a service for trigger-style communication");
          issue.affectedBlockId = block.id;
          issue.affectedElement = pin.name;
          addIssue(result, issue);
        }
      }
    }
  }
}

void StaticAnalyzer::checkSecurityIssues(const Project& project, AnalysisResult& result) {
  // Check for potentially sensitive topic names without encryption hints
  QStringList sensitivePatterns = {"password", "auth", "key", "secret", "token", "credential"};

  for (const BlockData& block : project.blocks()) {
    for (const PinData& pin : block.inputPins) {
      QString lowerName = pin.name.toLower();
      for (const QString& pattern : sensitivePatterns) {
        if (lowerName.contains(pattern)) {
          AnalysisIssue issue;
          issue.id = QUuid::createUuid();
          issue.severity = IssueSeverity::Warning;
          issue.category = IssueCategory::Security;
          issue.title = tr("Security consideration");
          issue.description = tr("Topic '%1' on '%2' may contain sensitive data")
                                  .arg(pin.name, block.name);
          issue.suggestion = tr("Consider enabling ROS2 security (SROS2) for sensitive communications");
          issue.affectedBlockId = block.id;
          issue.affectedElement = pin.name;
          addIssue(result, issue);
        }
      }
    }
  }
}

void StaticAnalyzer::checkMissingDependencies(const Project& project, AnalysisResult& result) {
  // Check that all referenced nodes exist
  QSet<QUuid> blockIds;
  for (const BlockData& block : project.blocks()) {
    blockIds.insert(block.id);
  }

  for (const ConnectionData& conn : project.connections()) {
    if (!blockIds.contains(conn.sourceBlockId)) {
      AnalysisIssue issue;
      issue.id = QUuid::createUuid();
      issue.severity = IssueSeverity::Error;
      issue.category = IssueCategory::MissingDependency;
      issue.title = tr("Missing source node");
      issue.description = tr("Connection references a source node that doesn't exist");
      issue.affectedConnectionId = conn.id;
      addIssue(result, issue);
    }
    if (!blockIds.contains(conn.targetBlockId)) {
      AnalysisIssue issue;
      issue.id = QUuid::createUuid();
      issue.severity = IssueSeverity::Error;
      issue.category = IssueCategory::MissingDependency;
      issue.title = tr("Missing target node");
      issue.description = tr("Connection references a target node that doesn't exist");
      issue.affectedConnectionId = conn.id;
      addIssue(result, issue);
    }
  }
}

void StaticAnalyzer::addIssue(AnalysisResult& result, const AnalysisIssue& issue) {
  result.issues.append(issue);

  switch (issue.severity) {
    case IssueSeverity::Error:
      result.errorCount++;
      break;
    case IssueSeverity::Warning:
      result.warningCount++;
      break;
    case IssueSeverity::Info:
      result.infoCount++;
      break;
    case IssueSeverity::Hint:
      result.hintCount++;
      break;
  }

  emit issueFound(issue);
}

bool StaticAnalyzer::areTypesCompatible(const QString& pubType, const QString& subType) {
  // Exact match
  if (pubType == subType) return true;

  // Allow common compatible types
  // e.g., geometry_msgs/msg/Twist and geometry_msgs/Twist (with/without msg)
  QString normPub = pubType;
  QString normSub = subType;

  normPub.replace("/msg/", "/");
  normSub.replace("/msg/", "/");

  return normPub == normSub;
}

bool StaticAnalyzer::detectCycle(const QUuid& startBlock, const Project& project,
                                  QSet<QUuid>& visited, QSet<QUuid>& recursionStack) {
  visited.insert(startBlock);
  recursionStack.insert(startBlock);

  // Get all connections from this block
  for (const ConnectionData& conn : project.connections()) {
    if (conn.sourceBlockId == startBlock) {
      QUuid nextBlock = conn.targetBlockId;

      if (!visited.contains(nextBlock)) {
        if (detectCycle(nextBlock, project, visited, recursionStack)) {
          return true;
        }
      } else if (recursionStack.contains(nextBlock)) {
        return true;  // Cycle detected
      }
    }
  }

  recursionStack.remove(startBlock);
  return false;
}

bool StaticAnalyzer::autoFix(const AnalysisIssue& /*issue*/, Project& /*project*/) {
  // Auto-fix implementations would go here
  // For now, return false (no auto-fix available)
  return false;
}

QString StaticAnalyzer::severityToString(IssueSeverity severity) {
  switch (severity) {
    case IssueSeverity::Error: return tr("Error");
    case IssueSeverity::Warning: return tr("Warning");
    case IssueSeverity::Info: return tr("Info");
    case IssueSeverity::Hint: return tr("Hint");
  }
  return tr("Unknown");
}

QString StaticAnalyzer::categoryToString(IssueCategory category) {
  switch (category) {
    case IssueCategory::TypeMismatch: return tr("Type Mismatch");
    case IssueCategory::UnusedPublisher: return tr("Unused Publisher");
    case IssueCategory::UnusedSubscriber: return tr("Unused Subscriber");
    case IssueCategory::QoSIncompatible: return tr("QoS Incompatibility");
    case IssueCategory::CyclicDependency: return tr("Cyclic Dependency");
    case IssueCategory::MissingDependency: return tr("Missing Dependency");
    case IssueCategory::InvalidParameter: return tr("Invalid Parameter");
    case IssueCategory::NamingConvention: return tr("Naming Convention");
    case IssueCategory::Performance: return tr("Performance");
    case IssueCategory::Security: return tr("Security");
    case IssueCategory::Deprecated: return tr("Deprecated");
  }
  return tr("Unknown");
}

QString StaticAnalyzer::severityIcon(IssueSeverity severity) {
  switch (severity) {
    case IssueSeverity::Error: return "⛔";
    case IssueSeverity::Warning: return "⚠️";
    case IssueSeverity::Info: return "ℹ️";
    case IssueSeverity::Hint: return "💡";
  }
  return "?";
}

QString StaticAnalyzer::categoryIcon(IssueCategory category) {
  switch (category) {
    case IssueCategory::TypeMismatch: return "🔀";
    case IssueCategory::UnusedPublisher: return "📤";
    case IssueCategory::UnusedSubscriber: return "📥";
    case IssueCategory::QoSIncompatible: return "⚙️";
    case IssueCategory::CyclicDependency: return "🔄";
    case IssueCategory::MissingDependency: return "❓";
    case IssueCategory::InvalidParameter: return "📋";
    case IssueCategory::NamingConvention: return "📝";
    case IssueCategory::Performance: return "⚡";
    case IssueCategory::Security: return "🔒";
    case IssueCategory::Deprecated: return "📜";
  }
  return "?";
}

}  // namespace ros_weaver
