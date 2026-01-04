#include <gtest/gtest.h>
#include <QString>
#include <QMap>
#include <QList>

// Test architecture optimizer structures directly without importing the full class
// to avoid complex dependencies

namespace ros_weaver {

// Categories of optimization recommendations
enum class OptimizationCategory {
  Performance,
  Reliability,
  Modularity,
  ResourceUsage,
  Latency,
  Scalability,
  BestPractices
};

// Priority/impact level of recommendation
enum class OptimizationPriority {
  Critical,
  High,
  Medium,
  Low,
  Informational
};

// A single optimization recommendation
struct OptimizationRecommendation {
  QString id;
  QString title;
  QString description;
  QString rationale;
  OptimizationCategory category;
  OptimizationPriority priority;
  QStringList affectedNodes;
  QString suggestedChange;
  QString codeExample;
  double estimatedImprovement = 0.0;
  bool canAutoApply = false;
};

// Analysis metrics for an architecture
struct ArchitectureMetrics {
  int nodeCount = 0;
  int connectionCount = 0;
  int groupCount = 0;
  int maxFanOut = 0;
  int maxFanIn = 0;
  int isolatedNodes = 0;
  int bottleneckNodes = 0;
  double averageConnectivity = 0.0;
  double modularityScore = 0.0;
  double reliabilityScore = 0.0;
  double estimatedLatency = 0.0;
  QMap<QString, int> messageTypeUsage;
  QMap<QString, int> qosProfiles;
};

// Result of architecture optimization analysis
struct OptimizationResult {
  bool success = false;
  QString errorMessage;
  ArchitectureMetrics metrics;
  QList<OptimizationRecommendation> recommendations;
  double overallScore = 0.0;
  QString summary;
};

// Helper functions for testing
QString categoryToString(OptimizationCategory category) {
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

QString priorityToString(OptimizationPriority priority) {
  switch (priority) {
    case OptimizationPriority::Critical: return "Critical";
    case OptimizationPriority::High: return "High";
    case OptimizationPriority::Medium: return "Medium";
    case OptimizationPriority::Low: return "Low";
    case OptimizationPriority::Informational: return "Informational";
    default: return "Unknown";
  }
}

}  // namespace ros_weaver

using namespace ros_weaver;

class ArchitectureOptimizerStructuresTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

// Test category to string conversion
TEST_F(ArchitectureOptimizerStructuresTest, CategoryToString) {
  EXPECT_EQ(categoryToString(OptimizationCategory::Performance), "Performance");
  EXPECT_EQ(categoryToString(OptimizationCategory::Reliability), "Reliability");
  EXPECT_EQ(categoryToString(OptimizationCategory::Modularity), "Modularity");
  EXPECT_EQ(categoryToString(OptimizationCategory::ResourceUsage), "Resource Usage");
  EXPECT_EQ(categoryToString(OptimizationCategory::Latency), "Latency");
  EXPECT_EQ(categoryToString(OptimizationCategory::Scalability), "Scalability");
  EXPECT_EQ(categoryToString(OptimizationCategory::BestPractices), "Best Practices");
}

// Test priority to string conversion
TEST_F(ArchitectureOptimizerStructuresTest, PriorityToString) {
  EXPECT_EQ(priorityToString(OptimizationPriority::Critical), "Critical");
  EXPECT_EQ(priorityToString(OptimizationPriority::High), "High");
  EXPECT_EQ(priorityToString(OptimizationPriority::Medium), "Medium");
  EXPECT_EQ(priorityToString(OptimizationPriority::Low), "Low");
  EXPECT_EQ(priorityToString(OptimizationPriority::Informational), "Informational");
}

// Test ArchitectureMetrics initialization
TEST_F(ArchitectureOptimizerStructuresTest, MetricsInitialization) {
  ArchitectureMetrics metrics;

  EXPECT_EQ(metrics.nodeCount, 0);
  EXPECT_EQ(metrics.connectionCount, 0);
  EXPECT_EQ(metrics.groupCount, 0);
  EXPECT_EQ(metrics.maxFanOut, 0);
  EXPECT_EQ(metrics.maxFanIn, 0);
  EXPECT_EQ(metrics.isolatedNodes, 0);
  EXPECT_EQ(metrics.bottleneckNodes, 0);
  EXPECT_DOUBLE_EQ(metrics.averageConnectivity, 0.0);
  EXPECT_DOUBLE_EQ(metrics.modularityScore, 0.0);
  EXPECT_DOUBLE_EQ(metrics.reliabilityScore, 0.0);
  EXPECT_DOUBLE_EQ(metrics.estimatedLatency, 0.0);
}

// Test metrics with values
TEST_F(ArchitectureOptimizerStructuresTest, MetricsWithValues) {
  ArchitectureMetrics metrics;
  metrics.nodeCount = 10;
  metrics.connectionCount = 15;
  metrics.groupCount = 3;
  metrics.maxFanOut = 5;
  metrics.maxFanIn = 4;
  metrics.isolatedNodes = 1;
  metrics.bottleneckNodes = 2;
  metrics.averageConnectivity = 1.5;
  metrics.modularityScore = 75.0;
  metrics.reliabilityScore = 80.0;
  metrics.estimatedLatency = 15.5;

  EXPECT_EQ(metrics.nodeCount, 10);
  EXPECT_EQ(metrics.connectionCount, 15);
  EXPECT_DOUBLE_EQ(metrics.modularityScore, 75.0);
}

// Test message type tracking
TEST_F(ArchitectureOptimizerStructuresTest, MessageTypeTracking) {
  ArchitectureMetrics metrics;
  metrics.messageTypeUsage["sensor_msgs/LaserScan"] = 3;
  metrics.messageTypeUsage["nav_msgs/Odometry"] = 5;
  metrics.messageTypeUsage["geometry_msgs/Twist"] = 2;

  EXPECT_EQ(metrics.messageTypeUsage.size(), 3);
  EXPECT_EQ(metrics.messageTypeUsage["nav_msgs/Odometry"], 5);
}

// Test QoS profile tracking
TEST_F(ArchitectureOptimizerStructuresTest, QoSProfileTracking) {
  ArchitectureMetrics metrics;
  metrics.qosProfiles["reliable"] = 8;
  metrics.qosProfiles["best_effort"] = 4;
  metrics.qosProfiles["sensor_data"] = 3;

  EXPECT_EQ(metrics.qosProfiles.size(), 3);
  EXPECT_EQ(metrics.qosProfiles["reliable"], 8);
}

// Test OptimizationRecommendation creation
TEST_F(ArchitectureOptimizerStructuresTest, RecommendationCreation) {
  OptimizationRecommendation rec;
  rec.id = "PERF-001";
  rec.title = "Reduce message frequency";
  rec.description = "The /scan topic is publishing at 100Hz but subscribers only need 10Hz";
  rec.rationale = "High frequency topics consume more CPU and network bandwidth";
  rec.category = OptimizationCategory::Performance;
  rec.priority = OptimizationPriority::High;
  rec.affectedNodes << "/lidar_driver" << "/slam_toolbox";
  rec.suggestedChange = "Add a throttle node or reduce publish rate";
  rec.estimatedImprovement = 30.0;
  rec.canAutoApply = false;

  EXPECT_EQ(rec.id, "PERF-001");
  EXPECT_EQ(rec.category, OptimizationCategory::Performance);
  EXPECT_EQ(rec.priority, OptimizationPriority::High);
  EXPECT_EQ(rec.affectedNodes.size(), 2);
  EXPECT_DOUBLE_EQ(rec.estimatedImprovement, 30.0);
}

// Test OptimizationResult initialization
TEST_F(ArchitectureOptimizerStructuresTest, ResultInitialization) {
  OptimizationResult result;

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.errorMessage.isEmpty());
  EXPECT_TRUE(result.recommendations.isEmpty());
  EXPECT_DOUBLE_EQ(result.overallScore, 0.0);
}

// Test successful optimization result
TEST_F(ArchitectureOptimizerStructuresTest, SuccessfulResult) {
  OptimizationResult result;
  result.success = true;
  result.metrics.nodeCount = 5;
  result.metrics.connectionCount = 4;
  result.overallScore = 85.0;
  result.summary = "Architecture is well structured with minor optimization opportunities";

  OptimizationRecommendation rec;
  rec.id = "INFO-001";
  rec.title = "Consider adding monitoring";
  rec.category = OptimizationCategory::BestPractices;
  rec.priority = OptimizationPriority::Informational;
  result.recommendations.append(rec);

  EXPECT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.overallScore, 85.0);
  EXPECT_EQ(result.recommendations.size(), 1);
}

// Test filter by category logic
TEST_F(ArchitectureOptimizerStructuresTest, FilterByCategory) {
  QList<OptimizationRecommendation> recommendations;

  OptimizationRecommendation perf1;
  perf1.id = "P1";
  perf1.category = OptimizationCategory::Performance;
  recommendations.append(perf1);

  OptimizationRecommendation rel1;
  rel1.id = "R1";
  rel1.category = OptimizationCategory::Reliability;
  recommendations.append(rel1);

  OptimizationRecommendation perf2;
  perf2.id = "P2";
  perf2.category = OptimizationCategory::Performance;
  recommendations.append(perf2);

  // Filter by Performance category
  QList<OptimizationRecommendation> filtered;
  for (const auto& rec : recommendations) {
    if (rec.category == OptimizationCategory::Performance) {
      filtered.append(rec);
    }
  }

  EXPECT_EQ(filtered.size(), 2);
}

// Test filter by priority logic
TEST_F(ArchitectureOptimizerStructuresTest, FilterByPriority) {
  QList<OptimizationRecommendation> recommendations;

  OptimizationRecommendation critical;
  critical.id = "C1";
  critical.priority = OptimizationPriority::Critical;
  recommendations.append(critical);

  OptimizationRecommendation low;
  low.id = "L1";
  low.priority = OptimizationPriority::Low;
  recommendations.append(low);

  OptimizationRecommendation high;
  high.id = "H1";
  high.priority = OptimizationPriority::High;
  recommendations.append(high);

  // Filter by High priority and above
  QList<OptimizationRecommendation> filtered;
  for (const auto& rec : recommendations) {
    if (rec.priority <= OptimizationPriority::High) {  // Critical, High
      filtered.append(rec);
    }
  }

  EXPECT_EQ(filtered.size(), 2);
}

// Test overall score calculation
TEST_F(ArchitectureOptimizerStructuresTest, OverallScoreCalculation) {
  ArchitectureMetrics metrics;
  metrics.modularityScore = 80.0;
  metrics.reliabilityScore = 70.0;

  // Simple weighted average
  double overall = (metrics.modularityScore * 0.5) + (metrics.reliabilityScore * 0.5);

  EXPECT_DOUBLE_EQ(overall, 75.0);
}

// Test bottleneck detection logic
TEST_F(ArchitectureOptimizerStructuresTest, BottleneckDetection) {
  const int HIGH_FAN_THRESHOLD = 5;

  ArchitectureMetrics metrics;
  metrics.maxFanIn = 8;
  metrics.maxFanOut = 6;

  bool hasHighFanIn = metrics.maxFanIn > HIGH_FAN_THRESHOLD;
  bool hasHighFanOut = metrics.maxFanOut > HIGH_FAN_THRESHOLD;
  bool hasPotentialBottleneck = hasHighFanIn && hasHighFanOut;

  EXPECT_TRUE(hasHighFanIn);
  EXPECT_TRUE(hasHighFanOut);
  EXPECT_TRUE(hasPotentialBottleneck);
}

// Test isolated node detection
TEST_F(ArchitectureOptimizerStructuresTest, IsolatedNodeDetection) {
  ArchitectureMetrics metrics;
  metrics.nodeCount = 10;
  metrics.isolatedNodes = 2;

  double isolationRate = static_cast<double>(metrics.isolatedNodes) / metrics.nodeCount;

  EXPECT_DOUBLE_EQ(isolationRate, 0.2);  // 20% isolated
}

// Test average connectivity calculation
TEST_F(ArchitectureOptimizerStructuresTest, AverageConnectivityCalculation) {
  ArchitectureMetrics metrics;
  metrics.nodeCount = 5;
  metrics.connectionCount = 10;

  // Average connections per node
  metrics.averageConnectivity = static_cast<double>(metrics.connectionCount * 2) / metrics.nodeCount;

  EXPECT_DOUBLE_EQ(metrics.averageConnectivity, 4.0);
}

// Test recommendation with code example
TEST_F(ArchitectureOptimizerStructuresTest, RecommendationWithCodeExample) {
  OptimizationRecommendation rec;
  rec.id = "QOS-001";
  rec.title = "Use reliable QoS for critical topics";
  rec.category = OptimizationCategory::Reliability;
  rec.codeExample = R"(
rclcpp::QoS qos(10);
qos.reliable();
publisher_ = node->create_publisher<std_msgs::msg::String>("/critical_topic", qos);
)";

  EXPECT_FALSE(rec.codeExample.isEmpty());
  EXPECT_TRUE(rec.codeExample.contains("reliable"));
}

// Test auto-apply capability
TEST_F(ArchitectureOptimizerStructuresTest, AutoApplyCapability) {
  OptimizationRecommendation autoApplicable;
  autoApplicable.id = "AUTO-001";
  autoApplicable.canAutoApply = true;
  autoApplicable.suggestedChange = "Rename topic from /data to /sensor/data";

  OptimizationRecommendation manual;
  manual.id = "MANUAL-001";
  manual.canAutoApply = false;
  manual.suggestedChange = "Refactor node architecture to reduce coupling";

  EXPECT_TRUE(autoApplicable.canAutoApply);
  EXPECT_FALSE(manual.canAutoApply);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
