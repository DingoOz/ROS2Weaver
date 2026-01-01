#include "ros_weaver/core/architecture_optimizer.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include "ros_weaver/core/project.hpp"

#include <QJsonDocument>
#include <QJsonArray>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace ros_weaver {

ArchitectureOptimizer& ArchitectureOptimizer::instance() {
  static ArchitectureOptimizer instance;
  return instance;
}

ArchitectureOptimizer::ArchitectureOptimizer()
    : QObject(nullptr)
    , recommendationCounter_(0)
{
}

bool ArchitectureOptimizer::isAIAvailable() const {
  return OllamaManager::instance().isOllamaRunning();
}

QString ArchitectureOptimizer::categoryToString(OptimizationCategory category) {
  switch (category) {
    case OptimizationCategory::Performance: return "Performance";
    case OptimizationCategory::Reliability: return "Reliability";
    case OptimizationCategory::Modularity: return "Modularity";
    case OptimizationCategory::ResourceUsage: return "Resource Usage";
    case OptimizationCategory::Latency: return "Latency";
    case OptimizationCategory::Scalability: return "Scalability";
    case OptimizationCategory::BestPractices: return "Best Practices";
    default: return "Unknown";
  }
}

QString ArchitectureOptimizer::priorityToString(OptimizationPriority priority) {
  switch (priority) {
    case OptimizationPriority::Critical: return "Critical";
    case OptimizationPriority::High: return "High";
    case OptimizationPriority::Medium: return "Medium";
    case OptimizationPriority::Low: return "Low";
    case OptimizationPriority::Informational: return "Informational";
    default: return "Unknown";
  }
}

OptimizationResult ArchitectureOptimizer::analyze(const Project& project) {
  emit analysisStarted();

  OptimizationResult result;
  result.success = true;

  emit analysisProgress(10, tr("Calculating metrics..."));

  // Calculate metrics
  result.metrics = calculateMetrics(project);

  emit analysisProgress(30, tr("Analyzing performance..."));
  auto perfRecs = analyzePerformance(project, result.metrics);
  result.recommendations.append(perfRecs);

  emit analysisProgress(45, tr("Analyzing reliability..."));
  auto reliabilityRecs = analyzeReliability(project, result.metrics);
  result.recommendations.append(reliabilityRecs);

  emit analysisProgress(60, tr("Analyzing modularity..."));
  auto modularityRecs = analyzeModularity(project, result.metrics);
  result.recommendations.append(modularityRecs);

  emit analysisProgress(75, tr("Analyzing best practices..."));
  auto bpRecs = analyzeBestPractices(project, result.metrics);
  result.recommendations.append(bpRecs);

  emit analysisProgress(85, tr("Analyzing resource usage..."));
  auto resourceRecs = analyzeResourceUsage(project, result.metrics);
  result.recommendations.append(resourceRecs);

  emit analysisProgress(95, tr("Calculating score..."));

  // Calculate overall score
  result.overallScore = calculateOverallScore(result.metrics, result.recommendations);

  // Generate summary
  int criticalCount = 0, highCount = 0, mediumCount = 0;
  for (const auto& rec : result.recommendations) {
    if (rec.priority == OptimizationPriority::Critical) criticalCount++;
    else if (rec.priority == OptimizationPriority::High) highCount++;
    else if (rec.priority == OptimizationPriority::Medium) mediumCount++;
  }

  result.summary = tr("Architecture Score: %1/100. Found %2 recommendations "
                      "(%3 critical, %4 high priority, %5 medium priority).")
      .arg(result.overallScore, 0, 'f', 1)
      .arg(result.recommendations.size())
      .arg(criticalCount)
      .arg(highCount)
      .arg(mediumCount);

  emit analysisProgress(100, tr("Analysis complete"));
  emit analysisCompleted(result);

  return result;
}

OptimizationResult ArchitectureOptimizer::analyzeWithAI(const Project& project) {
  // First run standard analysis
  OptimizationResult result = analyze(project);

  if (!isAIAvailable()) {
    return result;  // Return standard analysis if AI not available
  }

  // Build AI prompt
  QString prompt = buildAIPrompt(project, result.metrics);

  // TODO: Actually send to AI and get response
  // For now, just return the standard analysis
  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "AI-enhanced analysis requested (AI integration pending)");

  return result;
}

ArchitectureMetrics ArchitectureOptimizer::calculateMetrics(const Project& project) {
  ArchitectureMetrics metrics;

  const auto& blocks = project.blocks();
  const auto& connections = project.connections();
  const auto& groups = project.nodeGroups();

  metrics.nodeCount = blocks.size();
  metrics.connectionCount = connections.size();
  metrics.groupCount = groups.size();

  if (metrics.nodeCount == 0) {
    return metrics;
  }

  // Calculate fan-in and fan-out for each node
  QMap<QUuid, int> fanIn;
  QMap<QUuid, int> fanOut;

  for (const BlockData& block : blocks) {
    fanIn[block.id] = 0;
    fanOut[block.id] = 0;
  }

  for (const ConnectionData& conn : connections) {
    fanOut[conn.sourceBlockId]++;
    fanIn[conn.targetBlockId]++;
  }

  // Calculate max fan-in/out and isolated nodes
  int totalConnectivity = 0;
  for (const BlockData& block : blocks) {
    int nodeIn = fanIn[block.id];
    int nodeOut = fanOut[block.id];

    metrics.maxFanIn = qMax(metrics.maxFanIn, nodeIn);
    metrics.maxFanOut = qMax(metrics.maxFanOut, nodeOut);

    if (nodeIn == 0 && nodeOut == 0) {
      metrics.isolatedNodes++;
    }

    if (nodeIn >= 3 && nodeOut >= 3) {
      metrics.bottleneckNodes++;
    }

    totalConnectivity += nodeIn + nodeOut;
  }

  metrics.averageConnectivity = static_cast<double>(totalConnectivity) / metrics.nodeCount;

  // Calculate modularity score based on grouping
  if (metrics.nodeCount > 0) {
    int groupedNodes = 0;
    for (const NodeGroupData& group : groups) {
      groupedNodes += group.containedNodeIds.size();
    }
    // Avoid double-counting nodes in multiple groups
    groupedNodes = qMin(groupedNodes, metrics.nodeCount);
    metrics.modularityScore = (static_cast<double>(groupedNodes) / metrics.nodeCount) * 100.0;
  }

  // Calculate reliability score (based on redundancy and fault tolerance indicators)
  metrics.reliabilityScore = 50.0;  // Base score
  if (metrics.bottleneckNodes == 0) {
    metrics.reliabilityScore += 20.0;  // No single points of failure
  }
  if (metrics.groupCount > 0) {
    metrics.reliabilityScore += 15.0;  // Good organization
  }
  if (metrics.isolatedNodes == 0) {
    metrics.reliabilityScore += 15.0;  // No orphaned nodes
  }
  metrics.reliabilityScore = qMin(100.0, metrics.reliabilityScore);

  // Estimate latency based on graph depth
  // (simplified: assume sequential pipeline)
  metrics.estimatedLatency = metrics.nodeCount * 5.0;  // 5ms per node estimate

  return metrics;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::analyzePerformance(
    const Project& project, const ArchitectureMetrics& metrics) {

  QList<OptimizationRecommendation> recs;
  Q_UNUSED(project);

  // Check for high fan-out nodes (potential performance bottleneck)
  if (metrics.maxFanOut > 5) {
    OptimizationRecommendation rec;
    rec.id = QString("PERF-%1").arg(++recommendationCounter_);
    rec.title = tr("High fan-out detected");
    rec.description = tr("One or more nodes have %1 outgoing connections. "
                         "This can cause performance issues if the publisher "
                         "needs to serialize the same message multiple times.")
        .arg(metrics.maxFanOut);
    rec.rationale = tr("Each subscriber typically causes the publisher to "
                       "serialize the message. Consider using intra-process "
                       "communication or a relay node.");
    rec.category = OptimizationCategory::Performance;
    rec.priority = metrics.maxFanOut > 10 ? OptimizationPriority::High
                                           : OptimizationPriority::Medium;
    rec.suggestedChange = tr("Consider adding a relay node or using shared memory transport.");
    rec.estimatedImprovement = 15.0;
    recs.append(rec);
  }

  // Check for bottleneck nodes
  if (metrics.bottleneckNodes > 0) {
    OptimizationRecommendation rec;
    rec.id = QString("PERF-%1").arg(++recommendationCounter_);
    rec.title = tr("Potential bottleneck nodes");
    rec.description = tr("Found %1 node(s) with high fan-in AND fan-out. "
                         "These nodes may become processing bottlenecks.")
        .arg(metrics.bottleneckNodes);
    rec.rationale = tr("Nodes that receive many inputs and produce many outputs "
                       "can become CPU-bound and cause latency spikes.");
    rec.category = OptimizationCategory::Performance;
    rec.priority = OptimizationPriority::High;
    rec.suggestedChange = tr("Consider splitting this node into multiple specialized nodes, "
                             "or using component composition for zero-copy communication.");
    rec.estimatedImprovement = 25.0;
    recs.append(rec);
  }

  // Check estimated latency
  if (metrics.estimatedLatency > 50.0) {
    OptimizationRecommendation rec;
    rec.id = QString("PERF-%1").arg(++recommendationCounter_);
    rec.title = tr("High pipeline latency");
    rec.description = tr("Estimated pipeline latency is %1ms, which may not "
                         "meet real-time requirements.")
        .arg(metrics.estimatedLatency, 0, 'f', 1);
    rec.rationale = tr("Long processing chains add latency. Consider parallelizing "
                       "independent processing paths.");
    rec.category = OptimizationCategory::Latency;
    rec.priority = metrics.estimatedLatency > 100.0 ? OptimizationPriority::Critical
                                                      : OptimizationPriority::High;
    rec.suggestedChange = tr("Review the data flow and consider parallelizing independent branches.");
    rec.estimatedImprovement = 30.0;
    recs.append(rec);
  }

  return recs;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::analyzeReliability(
    const Project& project, const ArchitectureMetrics& metrics) {

  QList<OptimizationRecommendation> recs;

  // Check for single points of failure
  if (metrics.bottleneckNodes > 0 && metrics.nodeCount > 5) {
    OptimizationRecommendation rec;
    rec.id = QString("REL-%1").arg(++recommendationCounter_);
    rec.title = tr("Single points of failure");
    rec.description = tr("The architecture has %1 bottleneck node(s) that, "
                         "if they fail, could bring down the entire system.")
        .arg(metrics.bottleneckNodes);
    rec.rationale = tr("Critical nodes should have redundancy or graceful degradation.");
    rec.category = OptimizationCategory::Reliability;
    rec.priority = OptimizationPriority::High;
    rec.suggestedChange = tr("Consider adding redundant nodes or implementing health monitoring.");
    recs.append(rec);
  }

  // Check for lack of lifecycle management
  bool hasLifecycleNode = false;
  for (const BlockData& block : project.blocks()) {
    if (block.name.contains("lifecycle", Qt::CaseInsensitive)) {
      hasLifecycleNode = true;
      break;
    }
  }

  if (!hasLifecycleNode && metrics.nodeCount > 3) {
    OptimizationRecommendation rec;
    rec.id = QString("REL-%1").arg(++recommendationCounter_);
    rec.title = tr("No lifecycle management detected");
    rec.description = tr("The architecture does not appear to have lifecycle management. "
                         "ROS2 lifecycle nodes provide better control over node states.");
    rec.rationale = tr("Lifecycle nodes allow coordinated startup/shutdown and "
                       "better error handling.");
    rec.category = OptimizationCategory::Reliability;
    rec.priority = OptimizationPriority::Medium;
    rec.suggestedChange = tr("Consider using nav2_lifecycle_manager or implementing "
                             "lifecycle-aware nodes.");
    rec.codeExample = "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
                      "-p node_names:=[\"node1\", \"node2\"]";
    recs.append(rec);
  }

  // Check for isolated nodes
  if (metrics.isolatedNodes > 0) {
    OptimizationRecommendation rec;
    rec.id = QString("REL-%1").arg(++recommendationCounter_);
    rec.title = tr("Isolated nodes detected");
    rec.description = tr("Found %1 node(s) with no connections. These may be "
                         "misconfigured or no longer needed.")
        .arg(metrics.isolatedNodes);
    rec.rationale = tr("Isolated nodes consume resources without contributing to the system.");
    rec.category = OptimizationCategory::Reliability;
    rec.priority = OptimizationPriority::Low;
    rec.suggestedChange = tr("Review isolated nodes and either connect them or remove them.");
    recs.append(rec);
  }

  return recs;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::analyzeModularity(
    const Project& project, const ArchitectureMetrics& metrics) {

  QList<OptimizationRecommendation> recs;
  Q_UNUSED(project);

  // Check for lack of grouping
  if (metrics.nodeCount > 5 && metrics.groupCount == 0) {
    OptimizationRecommendation rec;
    rec.id = QString("MOD-%1").arg(++recommendationCounter_);
    rec.title = tr("No node grouping");
    rec.description = tr("The architecture has %1 nodes but no groups. "
                         "Grouping related nodes improves maintainability.")
        .arg(metrics.nodeCount);
    rec.rationale = tr("Logical grouping helps with understanding, debugging, "
                       "and modular deployment.");
    rec.category = OptimizationCategory::Modularity;
    rec.priority = OptimizationPriority::Medium;
    rec.suggestedChange = tr("Group related nodes by function (e.g., Sensors, Planning, Control).");
    recs.append(rec);
  }

  // Check for low modularity score
  if (metrics.modularityScore < 50.0 && metrics.nodeCount > 5) {
    OptimizationRecommendation rec;
    rec.id = QString("MOD-%1").arg(++recommendationCounter_);
    rec.title = tr("Low modularity score");
    rec.description = tr("Only %1% of nodes are organized into groups. "
                         "Consider improving organization.")
        .arg(metrics.modularityScore, 0, 'f', 0);
    rec.rationale = tr("Well-organized architectures are easier to maintain and extend.");
    rec.category = OptimizationCategory::Modularity;
    rec.priority = OptimizationPriority::Low;
    rec.suggestedChange = tr("Add more logical groupings for ungrouped nodes.");
    recs.append(rec);
  }

  // Check for tightly coupled nodes
  if (metrics.averageConnectivity > 4.0 && metrics.nodeCount > 5) {
    OptimizationRecommendation rec;
    rec.id = QString("MOD-%1").arg(++recommendationCounter_);
    rec.title = tr("High coupling detected");
    rec.description = tr("Average connectivity of %1 suggests tight coupling between nodes.")
        .arg(metrics.averageConnectivity, 0, 'f', 1);
    rec.rationale = tr("Highly coupled architectures are harder to modify and test.");
    rec.category = OptimizationCategory::Modularity;
    rec.priority = OptimizationPriority::Medium;
    rec.suggestedChange = tr("Consider introducing abstraction layers or using "
                             "action servers to reduce direct coupling.");
    recs.append(rec);
  }

  return recs;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::analyzeBestPractices(
    const Project& project, const ArchitectureMetrics& metrics) {

  QList<OptimizationRecommendation> recs;

  // Check for consistent naming
  bool hasInconsistentNaming = false;
  QSet<QString> namingStyles;

  for (const BlockData& block : project.blocks()) {
    if (block.name.contains("_")) {
      namingStyles.insert("snake_case");
    } else if (block.name.contains("-")) {
      namingStyles.insert("kebab-case");
    } else if (block.name[0].isUpper()) {
      namingStyles.insert("PascalCase");
    } else {
      namingStyles.insert("camelCase");
    }
  }

  if (namingStyles.size() > 1) {
    hasInconsistentNaming = true;
  }

  if (hasInconsistentNaming) {
    OptimizationRecommendation rec;
    rec.id = QString("BP-%1").arg(++recommendationCounter_);
    rec.title = tr("Inconsistent naming conventions");
    rec.description = tr("Multiple naming styles detected: %1. ROS2 recommends snake_case.")
        .arg(QStringList(namingStyles.values()).join(", "));
    rec.rationale = tr("Consistent naming improves code readability and follows ROS2 conventions.");
    rec.category = OptimizationCategory::BestPractices;
    rec.priority = OptimizationPriority::Low;
    rec.suggestedChange = tr("Use snake_case for node names (e.g., my_node instead of myNode).");
    recs.append(rec);
  }

  // Check for robot_state_publisher
  bool hasRSP = false;
  for (const BlockData& block : project.blocks()) {
    if (block.name.contains("robot_state_publisher") ||
        block.name.contains("rsp")) {
      hasRSP = true;
      break;
    }
  }

  if (!hasRSP && metrics.nodeCount > 3) {
    OptimizationRecommendation rec;
    rec.id = QString("BP-%1").arg(++recommendationCounter_);
    rec.title = tr("Missing robot_state_publisher");
    rec.description = tr("No robot_state_publisher detected. This node is essential "
                         "for publishing TF transforms from robot URDF.");
    rec.rationale = tr("Most ROS2 systems need robot_state_publisher for TF tree.");
    rec.category = OptimizationCategory::BestPractices;
    rec.priority = OptimizationPriority::Medium;
    rec.suggestedChange = tr("Add robot_state_publisher node to publish robot transforms.");
    recs.append(rec);
  }

  // Check for QoS considerations
  if (metrics.connectionCount > 5) {
    OptimizationRecommendation rec;
    rec.id = QString("BP-%1").arg(++recommendationCounter_);
    rec.title = tr("Review QoS settings");
    rec.description = tr("With %1 connections, ensure QoS profiles are appropriately configured.")
        .arg(metrics.connectionCount);
    rec.rationale = tr("Mismatched QoS settings are a common source of communication issues.");
    rec.category = OptimizationCategory::BestPractices;
    rec.priority = OptimizationPriority::Informational;
    rec.suggestedChange = tr("Use SensorDataQoS for sensor topics, ServicesQoS for services, "
                             "and consider BEST_EFFORT for high-frequency low-criticality data.");
    recs.append(rec);
  }

  return recs;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::analyzeResourceUsage(
    const Project& project, const ArchitectureMetrics& metrics) {

  QList<OptimizationRecommendation> recs;
  Q_UNUSED(project);

  // Check for large number of nodes
  if (metrics.nodeCount > 20) {
    OptimizationRecommendation rec;
    rec.id = QString("RES-%1").arg(++recommendationCounter_);
    rec.title = tr("Large number of nodes");
    rec.description = tr("Architecture has %1 nodes. Consider using component "
                         "composition for related nodes.")
        .arg(metrics.nodeCount);
    rec.rationale = tr("Each separate node process has overhead. Components share "
                       "a process and can use zero-copy communication.");
    rec.category = OptimizationCategory::ResourceUsage;
    rec.priority = OptimizationPriority::Medium;
    rec.suggestedChange = tr("Consider using ros2_component_container for related nodes.");
    rec.codeExample = "ros2 run rclcpp_components component_container\n"
                      "ros2 component load /ComponentManager package_name component::Class";
    rec.estimatedImprovement = 20.0;
    recs.append(rec);
  }

  // Check for message type diversity
  if (metrics.messageTypeUsage.size() > 10) {
    OptimizationRecommendation rec;
    rec.id = QString("RES-%1").arg(++recommendationCounter_);
    rec.title = tr("High message type diversity");
    rec.description = tr("Architecture uses %1 different message types. "
                         "Consider standardizing where possible.")
        .arg(metrics.messageTypeUsage.size());
    rec.rationale = tr("Each message type has serialization overhead. Reusing "
                       "standard messages improves interoperability.");
    rec.category = OptimizationCategory::ResourceUsage;
    rec.priority = OptimizationPriority::Informational;
    rec.suggestedChange = tr("Review if custom messages can be replaced with standard ROS2 messages.");
    recs.append(rec);
  }

  return recs;
}

double ArchitectureOptimizer::calculateOverallScore(
    const ArchitectureMetrics& metrics,
    const QList<OptimizationRecommendation>& recs) {

  double score = 100.0;

  // Deduct for issues
  for (const auto& rec : recs) {
    switch (rec.priority) {
      case OptimizationPriority::Critical:
        score -= 15.0;
        break;
      case OptimizationPriority::High:
        score -= 8.0;
        break;
      case OptimizationPriority::Medium:
        score -= 4.0;
        break;
      case OptimizationPriority::Low:
        score -= 2.0;
        break;
      case OptimizationPriority::Informational:
        score -= 0.5;
        break;
    }
  }

  // Bonus for good practices
  if (metrics.modularityScore > 75.0) {
    score += 5.0;
  }
  if (metrics.isolatedNodes == 0) {
    score += 3.0;
  }
  if (metrics.bottleneckNodes == 0) {
    score += 5.0;
  }

  return qBound(0.0, score, 100.0);
}

QList<OptimizationRecommendation> ArchitectureOptimizer::filterByCategory(
    const QList<OptimizationRecommendation>& recs,
    OptimizationCategory category) const {

  QList<OptimizationRecommendation> filtered;
  for (const auto& rec : recs) {
    if (rec.category == category) {
      filtered.append(rec);
    }
  }
  return filtered;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::filterByPriority(
    const QList<OptimizationRecommendation>& recs,
    OptimizationPriority minPriority) const {

  QList<OptimizationRecommendation> filtered;
  for (const auto& rec : recs) {
    if (static_cast<int>(rec.priority) <= static_cast<int>(minPriority)) {
      filtered.append(rec);
    }
  }
  return filtered;
}

bool ArchitectureOptimizer::applyRecommendation(const OptimizationRecommendation& rec,
                                                 Project& project) {
  Q_UNUSED(project);

  if (!rec.canAutoApply) {
    return false;
  }

  // TODO: Implement auto-apply for specific recommendation types
  // For now, return false as no auto-apply is implemented

  emit recommendationApplied(rec.id);
  return false;
}

QString ArchitectureOptimizer::buildAIPrompt(const Project& project,
                                              const ArchitectureMetrics& metrics) const {
  QString prompt = QString(
      "You are a ROS2 robotics architecture expert. Analyze this architecture:\n\n"
      "## Metrics\n"
      "- Nodes: %1\n"
      "- Connections: %2\n"
      "- Groups: %3\n"
      "- Max Fan-Out: %4\n"
      "- Max Fan-In: %5\n"
      "- Isolated Nodes: %6\n"
      "- Bottleneck Nodes: %7\n"
      "- Modularity Score: %8%%\n\n"
      "## Nodes\n"
  ).arg(metrics.nodeCount)
   .arg(metrics.connectionCount)
   .arg(metrics.groupCount)
   .arg(metrics.maxFanOut)
   .arg(metrics.maxFanIn)
   .arg(metrics.isolatedNodes)
   .arg(metrics.bottleneckNodes)
   .arg(metrics.modularityScore, 0, 'f', 1);

  for (const BlockData& block : project.blocks()) {
    prompt += QString("- %1\n").arg(block.name);
  }

  prompt += "\n## Provide recommendations as JSON:\n"
            "```json\n"
            "{\n"
            "  \"recommendations\": [\n"
            "    {\n"
            "      \"title\": \"Issue title\",\n"
            "      \"description\": \"Detailed description\",\n"
            "      \"category\": \"Performance|Reliability|Modularity|ResourceUsage\",\n"
            "      \"priority\": \"Critical|High|Medium|Low|Informational\",\n"
            "      \"affected_nodes\": [\"node1\", \"node2\"],\n"
            "      \"suggestion\": \"How to fix\"\n"
            "    }\n"
            "  ]\n"
            "}\n"
            "```";

  return prompt;
}

QList<OptimizationRecommendation> ArchitectureOptimizer::parseAIRecommendations(
    const QString& response) {

  QList<OptimizationRecommendation> recs;

  static QRegularExpression jsonPattern(R"(\{[\s\S]*"recommendations"[\s\S]*\})");
  QRegularExpressionMatch match = jsonPattern.match(response);

  if (!match.hasMatch()) {
    return recs;
  }

  QJsonDocument doc = QJsonDocument::fromJson(match.captured(0).toUtf8());
  if (!doc.isObject()) {
    return recs;
  }

  QJsonArray recsArray = doc.object()["recommendations"].toArray();
  for (const QJsonValue& recVal : recsArray) {
    QJsonObject recObj = recVal.toObject();

    OptimizationRecommendation rec;
    rec.id = QString("AI-%1").arg(++recommendationCounter_);
    rec.title = recObj["title"].toString();
    rec.description = recObj["description"].toString();
    rec.suggestedChange = recObj["suggestion"].toString();

    QString catStr = recObj["category"].toString();
    if (catStr == "Performance") rec.category = OptimizationCategory::Performance;
    else if (catStr == "Reliability") rec.category = OptimizationCategory::Reliability;
    else if (catStr == "Modularity") rec.category = OptimizationCategory::Modularity;
    else if (catStr == "ResourceUsage") rec.category = OptimizationCategory::ResourceUsage;
    else rec.category = OptimizationCategory::BestPractices;

    QString priStr = recObj["priority"].toString();
    if (priStr == "Critical") rec.priority = OptimizationPriority::Critical;
    else if (priStr == "High") rec.priority = OptimizationPriority::High;
    else if (priStr == "Medium") rec.priority = OptimizationPriority::Medium;
    else if (priStr == "Low") rec.priority = OptimizationPriority::Low;
    else rec.priority = OptimizationPriority::Informational;

    QJsonArray affectedArray = recObj["affected_nodes"].toArray();
    for (const QJsonValue& nodeVal : affectedArray) {
      rec.affectedNodes.append(nodeVal.toString());
    }

    recs.append(rec);
  }

  return recs;
}

}  // namespace ros_weaver
