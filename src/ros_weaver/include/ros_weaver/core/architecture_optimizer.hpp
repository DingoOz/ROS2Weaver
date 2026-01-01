#ifndef ROS_WEAVER_CORE_ARCHITECTURE_OPTIMIZER_HPP
#define ROS_WEAVER_CORE_ARCHITECTURE_OPTIMIZER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>

namespace ros_weaver {

class Project;

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
  double estimatedImprovement = 0.0;  // Percentage improvement estimate
  bool canAutoApply = false;
};

// Analysis metrics for an architecture
struct ArchitectureMetrics {
  int nodeCount = 0;
  int connectionCount = 0;
  int groupCount = 0;
  int maxFanOut = 0;           // Max outgoing connections from a node
  int maxFanIn = 0;            // Max incoming connections to a node
  int isolatedNodes = 0;       // Nodes with no connections
  int bottleneckNodes = 0;     // Nodes with both high fan-in and fan-out
  double averageConnectivity = 0.0;
  double modularityScore = 0.0;   // 0-100 based on grouping
  double reliabilityScore = 0.0;  // 0-100 based on redundancy
  double estimatedLatency = 0.0;  // Estimated pipeline latency
  QMap<QString, int> messageTypeUsage;  // Message type -> count
  QMap<QString, int> qosProfiles;       // QoS profile -> count
};

// Result of architecture optimization analysis
struct OptimizationResult {
  bool success = false;
  QString errorMessage;
  ArchitectureMetrics metrics;
  QList<OptimizationRecommendation> recommendations;
  double overallScore = 0.0;  // 0-100 overall architecture score
  QString summary;
};

// Analyzes and optimizes ROS2 architectures
class ArchitectureOptimizer : public QObject {
  Q_OBJECT

public:
  static ArchitectureOptimizer& instance();

  // Prevent copying
  ArchitectureOptimizer(const ArchitectureOptimizer&) = delete;
  ArchitectureOptimizer& operator=(const ArchitectureOptimizer&) = delete;

  // Analyze a project and generate recommendations
  OptimizationResult analyze(const Project& project);

  // Get AI-enhanced analysis (if available)
  OptimizationResult analyzeWithAI(const Project& project);

  // Apply a specific recommendation (if auto-applicable)
  bool applyRecommendation(const OptimizationRecommendation& rec, Project& project);

  // Filter recommendations by category
  QList<OptimizationRecommendation> filterByCategory(
      const QList<OptimizationRecommendation>& recs,
      OptimizationCategory category) const;

  // Filter recommendations by priority
  QList<OptimizationRecommendation> filterByPriority(
      const QList<OptimizationRecommendation>& recs,
      OptimizationPriority minPriority) const;

  // Get category display name
  static QString categoryToString(OptimizationCategory category);

  // Get priority display name
  static QString priorityToString(OptimizationPriority priority);

  // Check if AI backend is available
  bool isAIAvailable() const;

signals:
  void analysisStarted();
  void analysisProgress(int percent, const QString& status);
  void analysisCompleted(const OptimizationResult& result);
  void recommendationApplied(const QString& id);

private:
  ArchitectureOptimizer();
  ~ArchitectureOptimizer() override = default;

  // Calculate architecture metrics
  ArchitectureMetrics calculateMetrics(const Project& project);

  // Generate performance recommendations
  QList<OptimizationRecommendation> analyzePerformance(const Project& project,
                                                        const ArchitectureMetrics& metrics);

  // Generate reliability recommendations
  QList<OptimizationRecommendation> analyzeReliability(const Project& project,
                                                        const ArchitectureMetrics& metrics);

  // Generate modularity recommendations
  QList<OptimizationRecommendation> analyzeModularity(const Project& project,
                                                       const ArchitectureMetrics& metrics);

  // Generate best practices recommendations
  QList<OptimizationRecommendation> analyzeBestPractices(const Project& project,
                                                          const ArchitectureMetrics& metrics);

  // Generate resource usage recommendations
  QList<OptimizationRecommendation> analyzeResourceUsage(const Project& project,
                                                          const ArchitectureMetrics& metrics);

  // Calculate overall architecture score
  double calculateOverallScore(const ArchitectureMetrics& metrics,
                               const QList<OptimizationRecommendation>& recs);

  // Build AI analysis prompt
  QString buildAIPrompt(const Project& project, const ArchitectureMetrics& metrics) const;

  // Parse AI response
  QList<OptimizationRecommendation> parseAIRecommendations(const QString& response);

  int recommendationCounter_ = 0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ARCHITECTURE_OPTIMIZER_HPP
