#ifndef ROS_WEAVER_CANVAS_MAPPER_HPP
#define ROS_WEAVER_CANVAS_MAPPER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QUuid>

#include "ros_weaver/core/system_discovery.hpp"

namespace ros_weaver {

class Project;
struct BlockData;

// Confidence level for node matching
enum class MatchConfidence {
  High,    // Exact name match + matching topics
  Medium,  // Fuzzy name match or topic-only match
  Low,     // Partial topic overlap
  None     // No match found
};

// Mapping of a single topic/pin
struct TopicMapping {
  QString canvasPinName;
  QString canvasTopicName;
  QString ros2TopicName;
  QString messageType;
  bool typeMatches = false;
  bool isActive = false;  // Has publishers/subscribers
  bool isPublisher = true;  // true = output pin, false = input pin
};

// Result of mapping a canvas block to a ROS2 node
struct BlockMappingResult {
  QUuid canvasBlockId;
  QString canvasName;
  QString ros2NodeName;       // Empty if not matched
  QString ros2Namespace;
  MatchConfidence confidence = MatchConfidence::None;
  QList<TopicMapping> topicMappings;
  int matchedTopics = 0;
  int totalCanvasTopics = 0;
  QString matchReason;        // Human-readable explanation
};

// Summary of the overall mapping
struct MappingSummary {
  int totalCanvasBlocks = 0;
  int matchedBlocks = 0;
  int highConfidenceMatches = 0;
  int mediumConfidenceMatches = 0;
  int lowConfidenceMatches = 0;
  int totalCanvasTopics = 0;
  int matchedTopics = 0;
  int activeTopics = 0;
  QStringList unmatchedRos2Nodes;  // Nodes in system not on canvas
  QStringList unmatchedRos2Topics; // Topics in system not on canvas
  qint64 timestamp = 0;
};

// Complete mapping results
struct MappingResults {
  QList<BlockMappingResult> blockMappings;
  MappingSummary summary;
};

class CanvasMapper : public QObject {
  Q_OBJECT

public:
  explicit CanvasMapper(QObject* parent = nullptr);
  ~CanvasMapper() override;

  // Configuration
  bool ignoreNamespace() const { return ignoreNamespace_; }
  bool caseSensitive() const { return caseSensitive_; }
  double fuzzyThreshold() const { return fuzzyThreshold_; }

  // Get the last mapping results
  const MappingResults& lastResults() const { return lastResults_; }

public slots:
  void setIgnoreNamespace(bool ignore);
  void setCaseSensitive(bool sensitive);
  void setFuzzyThreshold(double threshold);

  // Perform mapping
  void mapCanvasToSystem(const Project& project, const SystemGraph& systemGraph);

signals:
  void mappingStarted();
  void mappingProgress(int percent, const QString& message);
  void mappingCompleted(const MappingResults& results);

private:
  // Matching algorithms
  MatchConfidence matchBlockToNode(
    const BlockData& block,
    const DiscoveredNode& node,
    QString& matchReason
  );

  // Name matching helpers
  bool exactNameMatch(const QString& canvasName, const QString& ros2Name);
  bool fuzzyNameMatch(const QString& canvasName, const QString& ros2Name, double& score);
  double calculateStringSimilarity(const QString& s1, const QString& s2);

  // Topic matching
  QList<TopicMapping> matchTopics(
    const BlockData& block,
    const DiscoveredNode& node,
    const SystemGraph& systemGraph
  );

  // Find best matching node for a block
  const DiscoveredNode* findBestMatch(
    const BlockData& block,
    const QList<DiscoveredNode>& nodes,
    MatchConfidence& confidence,
    QString& matchReason
  );

  // Configuration
  bool ignoreNamespace_ = true;
  bool caseSensitive_ = false;
  double fuzzyThreshold_ = 0.7;

  // Results
  MappingResults lastResults_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CANVAS_MAPPER_HPP
