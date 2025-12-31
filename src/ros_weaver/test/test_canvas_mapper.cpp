#include <gtest/gtest.h>
#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/core/project.hpp"
#include <QApplication>
#include <QSignalSpy>

using namespace ros_weaver;

// ============================================================================
// MatchConfidence Enum Tests
// ============================================================================

TEST(MatchConfidenceTest, EnumValues) {
  // Verify enum ordering - High is best (lowest value), None is worst (highest value)
  EXPECT_LT(static_cast<int>(MatchConfidence::High),
            static_cast<int>(MatchConfidence::Medium));
  EXPECT_LT(static_cast<int>(MatchConfidence::Medium),
            static_cast<int>(MatchConfidence::Low));
  EXPECT_LT(static_cast<int>(MatchConfidence::Low),
            static_cast<int>(MatchConfidence::None));
}

TEST(MatchConfidenceTest, SpecificValues) {
  EXPECT_EQ(static_cast<int>(MatchConfidence::High), 0);
  EXPECT_EQ(static_cast<int>(MatchConfidence::Medium), 1);
  EXPECT_EQ(static_cast<int>(MatchConfidence::Low), 2);
  EXPECT_EQ(static_cast<int>(MatchConfidence::None), 3);
}

// ============================================================================
// TopicMapping Struct Tests
// ============================================================================

TEST(TopicMappingTest, DefaultValues) {
  TopicMapping tm;

  EXPECT_TRUE(tm.canvasPinName.isEmpty());
  EXPECT_TRUE(tm.canvasTopicName.isEmpty());
  EXPECT_TRUE(tm.ros2TopicName.isEmpty());
  EXPECT_TRUE(tm.messageType.isEmpty());
  EXPECT_FALSE(tm.typeMatches);
  EXPECT_FALSE(tm.isActive);
  EXPECT_TRUE(tm.isPublisher);  // Default is publisher (output)
}

TEST(TopicMappingTest, SetValues) {
  TopicMapping tm;
  tm.canvasPinName = "cmd_vel_out";
  tm.canvasTopicName = "/cmd_vel";
  tm.ros2TopicName = "/robot/cmd_vel";
  tm.messageType = "geometry_msgs/msg/Twist";
  tm.typeMatches = true;
  tm.isActive = true;
  tm.isPublisher = true;

  EXPECT_EQ(tm.canvasPinName, "cmd_vel_out");
  EXPECT_EQ(tm.canvasTopicName, "/cmd_vel");
  EXPECT_EQ(tm.ros2TopicName, "/robot/cmd_vel");
  EXPECT_EQ(tm.messageType, "geometry_msgs/msg/Twist");
  EXPECT_TRUE(tm.typeMatches);
  EXPECT_TRUE(tm.isActive);
  EXPECT_TRUE(tm.isPublisher);
}

TEST(TopicMappingTest, SubscriberMapping) {
  TopicMapping tm;
  tm.canvasPinName = "odom_in";
  tm.isPublisher = false;

  EXPECT_FALSE(tm.isPublisher);
}

// ============================================================================
// BlockMappingResult Struct Tests
// ============================================================================

TEST(BlockMappingResultTest, DefaultValues) {
  BlockMappingResult result;

  EXPECT_TRUE(result.canvasBlockId.isNull());
  EXPECT_TRUE(result.canvasName.isEmpty());
  EXPECT_TRUE(result.ros2NodeName.isEmpty());
  EXPECT_TRUE(result.ros2Namespace.isEmpty());
  EXPECT_EQ(result.confidence, MatchConfidence::None);
  EXPECT_TRUE(result.topicMappings.isEmpty());
  EXPECT_EQ(result.matchedTopics, 0);
  EXPECT_EQ(result.totalCanvasTopics, 0);
  EXPECT_TRUE(result.matchReason.isEmpty());
}

TEST(BlockMappingResultTest, SetValues) {
  BlockMappingResult result;
  result.canvasBlockId = QUuid::createUuid();
  result.canvasName = "teleop_node";
  result.ros2NodeName = "/teleop/teleop_twist_keyboard";
  result.ros2Namespace = "/teleop";
  result.confidence = MatchConfidence::High;
  result.matchedTopics = 2;
  result.totalCanvasTopics = 3;
  result.matchReason = "Exact name match";

  EXPECT_FALSE(result.canvasBlockId.isNull());
  EXPECT_EQ(result.canvasName, "teleop_node");
  EXPECT_EQ(result.ros2NodeName, "/teleop/teleop_twist_keyboard");
  EXPECT_EQ(result.ros2Namespace, "/teleop");
  EXPECT_EQ(result.confidence, MatchConfidence::High);
  EXPECT_EQ(result.matchedTopics, 2);
  EXPECT_EQ(result.totalCanvasTopics, 3);
  EXPECT_EQ(result.matchReason, "Exact name match");
}

TEST(BlockMappingResultTest, AddTopicMappings) {
  BlockMappingResult result;

  TopicMapping tm1;
  tm1.canvasPinName = "cmd_vel";
  tm1.isPublisher = true;

  TopicMapping tm2;
  tm2.canvasPinName = "odom";
  tm2.isPublisher = false;

  result.topicMappings.append(tm1);
  result.topicMappings.append(tm2);

  EXPECT_EQ(result.topicMappings.size(), 2);
  EXPECT_TRUE(result.topicMappings[0].isPublisher);
  EXPECT_FALSE(result.topicMappings[1].isPublisher);
}

// ============================================================================
// MappingSummary Struct Tests
// ============================================================================

TEST(MappingSummaryTest, DefaultValues) {
  MappingSummary summary;

  EXPECT_EQ(summary.totalCanvasBlocks, 0);
  EXPECT_EQ(summary.matchedBlocks, 0);
  EXPECT_EQ(summary.highConfidenceMatches, 0);
  EXPECT_EQ(summary.mediumConfidenceMatches, 0);
  EXPECT_EQ(summary.lowConfidenceMatches, 0);
  EXPECT_EQ(summary.totalCanvasTopics, 0);
  EXPECT_EQ(summary.matchedTopics, 0);
  EXPECT_EQ(summary.activeTopics, 0);
  EXPECT_TRUE(summary.unmatchedRos2Nodes.isEmpty());
  EXPECT_TRUE(summary.unmatchedRos2Topics.isEmpty());
  EXPECT_EQ(summary.timestamp, 0);
}

TEST(MappingSummaryTest, SetValues) {
  MappingSummary summary;
  summary.totalCanvasBlocks = 10;
  summary.matchedBlocks = 8;
  summary.highConfidenceMatches = 5;
  summary.mediumConfidenceMatches = 2;
  summary.lowConfidenceMatches = 1;
  summary.totalCanvasTopics = 25;
  summary.matchedTopics = 20;
  summary.activeTopics = 15;
  summary.unmatchedRos2Nodes << "/unmatched_node1" << "/unmatched_node2";
  summary.unmatchedRos2Topics << "/unmatched_topic";
  summary.timestamp = 1234567890;

  EXPECT_EQ(summary.totalCanvasBlocks, 10);
  EXPECT_EQ(summary.matchedBlocks, 8);
  EXPECT_EQ(summary.highConfidenceMatches, 5);
  EXPECT_EQ(summary.mediumConfidenceMatches, 2);
  EXPECT_EQ(summary.lowConfidenceMatches, 1);
  EXPECT_EQ(summary.totalCanvasTopics, 25);
  EXPECT_EQ(summary.matchedTopics, 20);
  EXPECT_EQ(summary.activeTopics, 15);
  EXPECT_EQ(summary.unmatchedRos2Nodes.size(), 2);
  EXPECT_EQ(summary.unmatchedRos2Topics.size(), 1);
  EXPECT_EQ(summary.timestamp, 1234567890);
}

TEST(MappingSummaryTest, MatchRatioCalculation) {
  MappingSummary summary;
  summary.totalCanvasBlocks = 10;
  summary.matchedBlocks = 8;

  double matchRatio = summary.totalCanvasBlocks > 0
    ? static_cast<double>(summary.matchedBlocks) / summary.totalCanvasBlocks
    : 0.0;

  EXPECT_DOUBLE_EQ(matchRatio, 0.8);
}

// ============================================================================
// MappingResults Struct Tests
// ============================================================================

TEST(MappingResultsTest, DefaultValues) {
  MappingResults results;

  EXPECT_TRUE(results.blockMappings.isEmpty());
  EXPECT_EQ(results.summary.totalCanvasBlocks, 0);
}

TEST(MappingResultsTest, AddBlockMappings) {
  MappingResults results;

  BlockMappingResult block1;
  block1.canvasName = "node1";
  block1.confidence = MatchConfidence::High;

  BlockMappingResult block2;
  block2.canvasName = "node2";
  block2.confidence = MatchConfidence::Medium;

  results.blockMappings.append(block1);
  results.blockMappings.append(block2);
  results.summary.totalCanvasBlocks = 2;
  results.summary.matchedBlocks = 2;

  EXPECT_EQ(results.blockMappings.size(), 2);
  EXPECT_EQ(results.blockMappings[0].canvasName, "node1");
  EXPECT_EQ(results.blockMappings[1].canvasName, "node2");
}

// ============================================================================
// CanvasMapper Configuration Tests
// ============================================================================

class CanvasMapperTest : public ::testing::Test {
protected:
  void SetUp() override {
    mapper_ = std::make_unique<CanvasMapper>();
  }

  void TearDown() override {
    mapper_.reset();
  }

  std::unique_ptr<CanvasMapper> mapper_;
};

TEST_F(CanvasMapperTest, DefaultConfiguration) {
  EXPECT_TRUE(mapper_->ignoreNamespace());
  EXPECT_FALSE(mapper_->caseSensitive());
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 0.7);
}

TEST_F(CanvasMapperTest, SetIgnoreNamespace) {
  mapper_->setIgnoreNamespace(false);
  EXPECT_FALSE(mapper_->ignoreNamespace());

  mapper_->setIgnoreNamespace(true);
  EXPECT_TRUE(mapper_->ignoreNamespace());
}

TEST_F(CanvasMapperTest, SetCaseSensitive) {
  mapper_->setCaseSensitive(true);
  EXPECT_TRUE(mapper_->caseSensitive());

  mapper_->setCaseSensitive(false);
  EXPECT_FALSE(mapper_->caseSensitive());
}

TEST_F(CanvasMapperTest, SetFuzzyThreshold) {
  mapper_->setFuzzyThreshold(0.8);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 0.8);

  mapper_->setFuzzyThreshold(0.5);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 0.5);
}

TEST_F(CanvasMapperTest, FuzzyThresholdClampedLow) {
  mapper_->setFuzzyThreshold(-0.5);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 0.0);
}

TEST_F(CanvasMapperTest, FuzzyThresholdClampedHigh) {
  mapper_->setFuzzyThreshold(1.5);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 1.0);
}

TEST_F(CanvasMapperTest, FuzzyThresholdBoundaryValues) {
  mapper_->setFuzzyThreshold(0.0);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 0.0);

  mapper_->setFuzzyThreshold(1.0);
  EXPECT_DOUBLE_EQ(mapper_->fuzzyThreshold(), 1.0);
}

TEST_F(CanvasMapperTest, LastResultsInitiallyEmpty) {
  const MappingResults& results = mapper_->lastResults();
  EXPECT_TRUE(results.blockMappings.isEmpty());
  EXPECT_EQ(results.summary.totalCanvasBlocks, 0);
}

TEST_F(CanvasMapperTest, MappingStartedSignal) {
  QSignalSpy spy(mapper_.get(), &CanvasMapper::mappingStarted);

  Project project;
  SystemGraph graph;
  mapper_->mapCanvasToSystem(project, graph);

  EXPECT_EQ(spy.count(), 1);
}

TEST_F(CanvasMapperTest, MappingCompletedSignal) {
  QSignalSpy spy(mapper_.get(), &CanvasMapper::mappingCompleted);

  Project project;
  SystemGraph graph;
  mapper_->mapCanvasToSystem(project, graph);

  EXPECT_EQ(spy.count(), 1);
}

TEST_F(CanvasMapperTest, MappingProgressSignals) {
  QSignalSpy spy(mapper_.get(), &CanvasMapper::mappingProgress);

  Project project;
  SystemGraph graph;
  mapper_->mapCanvasToSystem(project, graph);

  // Should emit progress signals during mapping
  EXPECT_GE(spy.count(), 1);
}

TEST_F(CanvasMapperTest, EmptyProjectMapping) {
  Project project;
  SystemGraph graph;

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_TRUE(results.blockMappings.isEmpty());
  EXPECT_EQ(results.summary.totalCanvasBlocks, 0);
  EXPECT_EQ(results.summary.matchedBlocks, 0);
  EXPECT_GT(results.summary.timestamp, 0);
}

TEST_F(CanvasMapperTest, MappingWithNoSystemNodes) {
  Project project;

  // Add a block to the project
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "test_node";
  project.addBlock(block);

  SystemGraph graph;  // Empty system graph

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.blockMappings.size(), 1);
  EXPECT_EQ(results.summary.totalCanvasBlocks, 1);
  EXPECT_EQ(results.summary.matchedBlocks, 0);
  EXPECT_EQ(results.blockMappings[0].confidence, MatchConfidence::None);
}

TEST_F(CanvasMapperTest, MappingWithExactMatch) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "teleop_node";
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "teleop_node";
  node.fullName = "/teleop_node";
  node.namespacePath = "/";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedBlocks, 1);
  EXPECT_EQ(results.blockMappings[0].confidence, MatchConfidence::High);
}

TEST_F(CanvasMapperTest, MappingIgnoresNamespaceByDefault) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "camera_driver";
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "camera_driver";
  node.fullName = "/sensors/camera/camera_driver";
  node.namespacePath = "/sensors/camera";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedBlocks, 1);
  EXPECT_EQ(results.blockMappings[0].confidence, MatchConfidence::High);
}

TEST_F(CanvasMapperTest, MappingWithNamespaceNotIgnored) {
  mapper_->setIgnoreNamespace(false);

  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "camera_driver";
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "camera_driver";
  node.fullName = "/sensors/camera/camera_driver";
  node.namespacePath = "/sensors/camera";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  // Without ignoring namespace, "camera_driver" won't match "/sensors/camera/camera_driver"
  // but it should still match the node.name
  EXPECT_EQ(results.summary.matchedBlocks, 1);
}

TEST_F(CanvasMapperTest, CaseInsensitiveMatchByDefault) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "TELEOP_NODE";
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "teleop_node";
  node.fullName = "/teleop_node";
  node.namespacePath = "/";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedBlocks, 1);
  EXPECT_EQ(results.blockMappings[0].confidence, MatchConfidence::High);
}

TEST_F(CanvasMapperTest, CaseSensitiveMatchWhenEnabled) {
  mapper_->setCaseSensitive(true);

  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "TELEOP_NODE";
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "teleop_node";
  node.fullName = "/teleop_node";
  node.namespacePath = "/";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  // Should not match with different case when case-sensitive
  EXPECT_NE(results.blockMappings[0].confidence, MatchConfidence::High);
}

TEST_F(CanvasMapperTest, UnmatchedRos2NodesTracked) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "node_a";
  project.addBlock(block);

  SystemGraph graph;

  DiscoveredNode node1;
  node1.name = "node_a";
  node1.fullName = "/node_a";
  graph.nodes.append(node1);

  DiscoveredNode node2;
  node2.name = "node_b";
  node2.fullName = "/node_b";
  graph.nodes.append(node2);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.unmatchedRos2Nodes.size(), 1);
  EXPECT_TRUE(results.summary.unmatchedRos2Nodes.contains("/node_b"));
}

TEST_F(CanvasMapperTest, UnmatchedRos2TopicsTracked) {
  Project project;
  SystemGraph graph;

  DiscoveredTopic topic;
  topic.name = "/orphan_topic";
  topic.type = "std_msgs/msg/String";
  graph.topics.append(topic);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.unmatchedRos2Topics.size(), 1);
  EXPECT_TRUE(results.summary.unmatchedRos2Topics.contains("/orphan_topic"));
}

TEST_F(CanvasMapperTest, MultipleBlocksMapping) {
  Project project;

  BlockData block1;
  block1.id = QUuid::createUuid();
  block1.name = "node_a";

  BlockData block2;
  block2.id = QUuid::createUuid();
  block2.name = "node_b";

  BlockData block3;
  block3.id = QUuid::createUuid();
  block3.name = "unmatched_node";

  project.addBlock(block1);
  project.addBlock(block2);
  project.addBlock(block3);

  SystemGraph graph;

  DiscoveredNode node1;
  node1.name = "node_a";
  node1.fullName = "/node_a";
  graph.nodes.append(node1);

  DiscoveredNode node2;
  node2.name = "node_b";
  node2.fullName = "/node_b";
  graph.nodes.append(node2);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.totalCanvasBlocks, 3);
  EXPECT_EQ(results.summary.matchedBlocks, 2);
  EXPECT_EQ(results.summary.highConfidenceMatches, 2);
}

TEST_F(CanvasMapperTest, SuffixStrippingMatch) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "camera";  // Without suffix
  project.addBlock(block);

  SystemGraph graph;
  DiscoveredNode node;
  node.name = "camera_node";  // With _node suffix
  node.fullName = "/camera_node";
  node.namespacePath = "/";
  graph.nodes.append(node);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedBlocks, 1);
  EXPECT_EQ(results.blockMappings[0].confidence, MatchConfidence::Medium);
  EXPECT_TRUE(results.blockMappings[0].matchReason.contains("suffix"));
}

// ============================================================================
// Topic Matching Tests
// ============================================================================

TEST_F(CanvasMapperTest, TopicMatchingWithOutputPins) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "teleop";

  PinData outputPin;
  outputPin.name = "cmd_vel";
  outputPin.type = "output";
  outputPin.messageType = "geometry_msgs/msg/Twist";
  block.outputPins.append(outputPin);

  project.addBlock(block);

  SystemGraph graph;

  DiscoveredNode node;
  node.name = "teleop";
  node.fullName = "/teleop";
  node.publishers << "/cmd_vel";
  graph.nodes.append(node);

  graph.topicTypes["/cmd_vel"] = "geometry_msgs/msg/Twist";

  DiscoveredTopic topic;
  topic.name = "/cmd_vel";
  topic.type = "geometry_msgs/msg/Twist";
  topic.subscriberCount = 1;
  graph.topics.append(topic);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedTopics, 1);
  EXPECT_EQ(results.summary.activeTopics, 1);

  ASSERT_EQ(results.blockMappings[0].topicMappings.size(), 1);
  const TopicMapping& tm = results.blockMappings[0].topicMappings[0];
  EXPECT_EQ(tm.canvasPinName, "cmd_vel");
  EXPECT_EQ(tm.ros2TopicName, "/cmd_vel");
  EXPECT_TRUE(tm.typeMatches);
  EXPECT_TRUE(tm.isActive);
  EXPECT_TRUE(tm.isPublisher);
}

TEST_F(CanvasMapperTest, TopicMatchingWithInputPins) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "controller";

  PinData inputPin;
  inputPin.name = "odom";
  inputPin.type = "input";
  inputPin.messageType = "nav_msgs/msg/Odometry";
  block.inputPins.append(inputPin);

  project.addBlock(block);

  SystemGraph graph;

  DiscoveredNode node;
  node.name = "controller";
  node.fullName = "/controller";
  node.subscribers << "/odom";
  graph.nodes.append(node);

  graph.topicTypes["/odom"] = "nav_msgs/msg/Odometry";

  DiscoveredTopic topic;
  topic.name = "/odom";
  topic.type = "nav_msgs/msg/Odometry";
  topic.publisherCount = 1;
  graph.topics.append(topic);

  mapper_->mapCanvasToSystem(project, graph);

  const MappingResults& results = mapper_->lastResults();
  EXPECT_EQ(results.summary.matchedTopics, 1);

  ASSERT_EQ(results.blockMappings[0].topicMappings.size(), 1);
  const TopicMapping& tm = results.blockMappings[0].topicMappings[0];
  EXPECT_EQ(tm.canvasPinName, "odom");
  EXPECT_FALSE(tm.isPublisher);  // Input pin = subscriber
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
