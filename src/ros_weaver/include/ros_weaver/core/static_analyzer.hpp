#ifndef ROS_WEAVER_CORE_STATIC_ANALYZER_HPP
#define ROS_WEAVER_CORE_STATIC_ANALYZER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QUuid>

namespace ros_weaver {

class Project;
class PackageBlock;
class ConnectionLine;

// Severity level for analysis issues
enum class IssueSeverity {
  Error,      // Critical issues that will cause runtime failures
  Warning,    // Potential issues that may cause problems
  Info,       // Informational messages, suggestions
  Hint        // Minor suggestions for improvement
};

// Category of the issue
enum class IssueCategory {
  TypeMismatch,       // Incompatible message types
  UnusedPublisher,    // Publisher with no subscribers
  UnusedSubscriber,   // Subscriber with no publishers
  QoSIncompatible,    // Incompatible QoS settings
  CyclicDependency,   // Circular data flow detected
  MissingDependency,  // Required node or topic not found
  InvalidParameter,   // Parameter value out of range or invalid type
  NamingConvention,   // Non-standard naming
  Performance,        // Potential performance issues
  Security,           // Security-related concerns
  Deprecated          // Using deprecated features
};

// A single analysis issue
struct AnalysisIssue {
  QUuid id;                    // Unique issue identifier
  IssueSeverity severity;
  IssueCategory category;
  QString title;               // Short description
  QString description;         // Detailed description
  QString suggestion;          // Suggested fix
  QUuid affectedBlockId;       // Block with the issue (if applicable)
  QUuid affectedConnectionId;  // Connection with the issue (if applicable)
  QString affectedElement;     // Name of affected element (e.g., pin name)
  int lineNumber = -1;         // Source line if applicable
  bool canAutoFix = false;     // True if automatic fix is available

  bool operator==(const AnalysisIssue& other) const {
    return id == other.id;
  }
};

// Analysis results for a project
struct AnalysisResult {
  QList<AnalysisIssue> issues;
  int errorCount = 0;
  int warningCount = 0;
  int infoCount = 0;
  int hintCount = 0;
  bool analysisComplete = false;
  QString analysisDuration;  // Human-readable duration
};

// Analysis rules configuration
struct AnalysisRules {
  bool checkTypeMismatches = true;
  bool checkUnusedPubsSubs = true;
  bool checkQoSCompatibility = true;
  bool checkCyclicDependencies = true;
  bool checkNamingConventions = false;
  bool checkDeprecatedFeatures = true;
  bool checkSecurityIssues = true;
  bool enableHints = true;
};

// Static analyzer for ROS2 architecture validation
class StaticAnalyzer : public QObject {
  Q_OBJECT

public:
  static StaticAnalyzer& instance();

  // Prevent copying
  StaticAnalyzer(const StaticAnalyzer&) = delete;
  StaticAnalyzer& operator=(const StaticAnalyzer&) = delete;

  // Analyze a project
  AnalysisResult analyze(const Project& project);

  // Analyze specific elements
  QList<AnalysisIssue> analyzeBlock(const PackageBlock* block, const Project& project);
  QList<AnalysisIssue> analyzeConnection(const ConnectionLine* conn, const Project& project);

  // Get/set analysis rules
  AnalysisRules rules() const { return rules_; }
  void setRules(const AnalysisRules& rules) { rules_ = rules; }

  // Try to auto-fix an issue
  bool autoFix(const AnalysisIssue& issue, Project& project);

  // Get human-readable descriptions
  static QString severityToString(IssueSeverity severity);
  static QString categoryToString(IssueCategory category);
  static QString severityIcon(IssueSeverity severity);
  static QString categoryIcon(IssueCategory category);

signals:
  void analysisStarted();
  void analysisProgress(int percent, const QString& currentItem);
  void analysisCompleted(const AnalysisResult& result);
  void issueFound(const AnalysisIssue& issue);

private:
  StaticAnalyzer();
  ~StaticAnalyzer() override = default;

  // Analysis passes
  void checkTypeMismatches(const Project& project, AnalysisResult& result);
  void checkUnusedPublishersSubscribers(const Project& project, AnalysisResult& result);
  void checkQoSCompatibility(const Project& project, AnalysisResult& result);
  void checkCyclicDependencies(const Project& project, AnalysisResult& result);
  void checkNamingConventions(const Project& project, AnalysisResult& result);
  void checkDeprecatedFeatures(const Project& project, AnalysisResult& result);
  void checkSecurityIssues(const Project& project, AnalysisResult& result);
  void checkMissingDependencies(const Project& project, AnalysisResult& result);

  // Helper functions
  void addIssue(AnalysisResult& result, const AnalysisIssue& issue);
  bool areTypesCompatible(const QString& pubType, const QString& subType);
  bool detectCycle(const QUuid& startBlock, const Project& project,
                   QSet<QUuid>& visited, QSet<QUuid>& recursionStack);

  AnalysisRules rules_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_STATIC_ANALYZER_HPP
