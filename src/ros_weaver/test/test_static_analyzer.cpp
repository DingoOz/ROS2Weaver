#include <gtest/gtest.h>
#include "ros_weaver/core/static_analyzer.hpp"
#include "ros_weaver/core/project.hpp"

using namespace ros_weaver;

class StaticAnalyzerTest : public ::testing::Test {
protected:
  void SetUp() override {
    project_.metadata().name = "TestProject";
  }

  void addBlock(const QString& name, const QList<PinData>& inputs,
                const QList<PinData>& outputs) {
    BlockData block;
    block.id = QUuid::createUuid();
    block.name = name;
    block.position = QPointF(blockCount_ * 200, 0);
    block.inputPins = inputs;
    block.outputPins = outputs;
    project_.addBlock(block);
    blockIds_[name] = block.id;
    blockCount_++;
  }

  void addConnection(const QString& sourceName, int sourcePin,
                     const QString& targetName, int targetPin) {
    ConnectionData conn;
    conn.id = QUuid::createUuid();
    conn.sourceBlockId = blockIds_[sourceName];
    conn.sourcePinIndex = sourcePin;
    conn.targetBlockId = blockIds_[targetName];
    conn.targetPinIndex = targetPin;
    project_.addConnection(conn);
  }

  PinData createTopicPin(const QString& name, const QString& msgType, bool isOutput) {
    PinData pin;
    pin.name = name;
    pin.type = isOutput ? "output" : "input";
    pin.dataType = "topic";
    pin.messageType = msgType;
    return pin;
  }

  Project project_;
  QMap<QString, QUuid> blockIds_;
  int blockCount_ = 0;
};

// Test severity and category string conversions
TEST_F(StaticAnalyzerTest, SeverityToString) {
  EXPECT_EQ(StaticAnalyzer::severityToString(IssueSeverity::Error), "Error");
  EXPECT_EQ(StaticAnalyzer::severityToString(IssueSeverity::Warning), "Warning");
  EXPECT_EQ(StaticAnalyzer::severityToString(IssueSeverity::Info), "Info");
  EXPECT_EQ(StaticAnalyzer::severityToString(IssueSeverity::Hint), "Hint");
}

TEST_F(StaticAnalyzerTest, CategoryToString) {
  EXPECT_EQ(StaticAnalyzer::categoryToString(IssueCategory::TypeMismatch), "Type Mismatch");
  EXPECT_EQ(StaticAnalyzer::categoryToString(IssueCategory::UnusedPublisher), "Unused Publisher");
  EXPECT_EQ(StaticAnalyzer::categoryToString(IssueCategory::UnusedSubscriber), "Unused Subscriber");
  EXPECT_EQ(StaticAnalyzer::categoryToString(IssueCategory::QoSIncompatible), "QoS Incompatibility");
  EXPECT_EQ(StaticAnalyzer::categoryToString(IssueCategory::CyclicDependency), "Cyclic Dependency");
}

TEST_F(StaticAnalyzerTest, SeverityIcons) {
  EXPECT_FALSE(StaticAnalyzer::severityIcon(IssueSeverity::Error).isEmpty());
  EXPECT_FALSE(StaticAnalyzer::severityIcon(IssueSeverity::Warning).isEmpty());
  EXPECT_FALSE(StaticAnalyzer::severityIcon(IssueSeverity::Info).isEmpty());
  EXPECT_FALSE(StaticAnalyzer::severityIcon(IssueSeverity::Hint).isEmpty());
}

TEST_F(StaticAnalyzerTest, CategoryIcons) {
  EXPECT_FALSE(StaticAnalyzer::categoryIcon(IssueCategory::TypeMismatch).isEmpty());
  EXPECT_FALSE(StaticAnalyzer::categoryIcon(IssueCategory::UnusedPublisher).isEmpty());
}

// Test analyzing an empty project
TEST_F(StaticAnalyzerTest, EmptyProjectNoIssues) {
  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  EXPECT_TRUE(result.analysisComplete);
  EXPECT_EQ(result.errorCount, 0);
  EXPECT_EQ(result.warningCount, 0);
  EXPECT_TRUE(result.issues.isEmpty());
}

// Test detecting type mismatches
TEST_F(StaticAnalyzerTest, DetectTypeMismatch) {
  // Publisher with LaserScan
  addBlock("lidar_driver", {}, {createTopicPin("/scan", "sensor_msgs/LaserScan", true)});

  // Subscriber expecting PointCloud2 (wrong type!)
  addBlock("slam_node", {createTopicPin("/scan", "sensor_msgs/PointCloud2", false)}, {});

  addConnection("lidar_driver", 0, "slam_node", 0);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  EXPECT_TRUE(result.analysisComplete);
  EXPECT_GT(result.errorCount, 0);

  // Should find a type mismatch issue
  bool foundTypeMismatch = false;
  for (const auto& issue : result.issues) {
    if (issue.category == IssueCategory::TypeMismatch) {
      foundTypeMismatch = true;
      EXPECT_EQ(issue.severity, IssueSeverity::Error);
      break;
    }
  }
  EXPECT_TRUE(foundTypeMismatch);
}

// Test compatible types don't produce errors
TEST_F(StaticAnalyzerTest, CompatibleTypesNoError) {
  // Publisher with LaserScan
  addBlock("lidar_driver", {}, {createTopicPin("/scan", "sensor_msgs/LaserScan", true)});

  // Subscriber expecting same type
  addBlock("slam_node", {createTopicPin("/scan", "sensor_msgs/LaserScan", false)}, {});

  addConnection("lidar_driver", 0, "slam_node", 0);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  // Should not find type mismatch
  bool foundTypeMismatch = false;
  for (const auto& issue : result.issues) {
    if (issue.category == IssueCategory::TypeMismatch) {
      foundTypeMismatch = true;
      break;
    }
  }
  EXPECT_FALSE(foundTypeMismatch);
}

// Test detecting unused publishers
TEST_F(StaticAnalyzerTest, DetectUnusedPublisher) {
  // Publisher with no subscriber
  addBlock("lonely_publisher", {}, {createTopicPin("/orphan_topic", "std_msgs/String", true)});

  AnalysisRules rules;
  rules.checkUnusedPubsSubs = true;
  StaticAnalyzer::instance().setRules(rules);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  bool foundUnusedPub = false;
  for (const auto& issue : result.issues) {
    if (issue.category == IssueCategory::UnusedPublisher) {
      foundUnusedPub = true;
      EXPECT_EQ(issue.severity, IssueSeverity::Warning);
      break;
    }
  }
  EXPECT_TRUE(foundUnusedPub);
}

// Test detecting unused subscribers
TEST_F(StaticAnalyzerTest, DetectUnusedSubscriber) {
  // Subscriber with no publisher
  addBlock("lonely_subscriber", {createTopicPin("/orphan_topic", "std_msgs/String", false)}, {});

  AnalysisRules rules;
  rules.checkUnusedPubsSubs = true;
  StaticAnalyzer::instance().setRules(rules);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  bool foundUnusedSub = false;
  for (const auto& issue : result.issues) {
    if (issue.category == IssueCategory::UnusedSubscriber) {
      foundUnusedSub = true;
      EXPECT_EQ(issue.severity, IssueSeverity::Warning);
      break;
    }
  }
  EXPECT_TRUE(foundUnusedSub);
}

// Test analysis with rules disabled
TEST_F(StaticAnalyzerTest, RulesCanBeDisabled) {
  // Create mismatch scenario
  addBlock("pub", {}, {createTopicPin("/topic", "std_msgs/String", true)});
  addBlock("sub", {createTopicPin("/topic", "std_msgs/Int32", false)}, {});
  addConnection("pub", 0, "sub", 0);

  AnalysisRules rules;
  rules.checkTypeMismatches = false;  // Disable type mismatch checking
  StaticAnalyzer::instance().setRules(rules);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  // Should NOT find type mismatch since rule is disabled
  bool foundTypeMismatch = false;
  for (const auto& issue : result.issues) {
    if (issue.category == IssueCategory::TypeMismatch) {
      foundTypeMismatch = true;
      break;
    }
  }
  EXPECT_FALSE(foundTypeMismatch);

  // Re-enable for other tests
  rules.checkTypeMismatches = true;
  StaticAnalyzer::instance().setRules(rules);
}

// Test issue counting
TEST_F(StaticAnalyzerTest, IssueCountingAccurate) {
  // Create multiple issues
  addBlock("pub1", {}, {createTopicPin("/topic1", "std_msgs/String", true)});
  addBlock("pub2", {}, {createTopicPin("/topic2", "std_msgs/Int32", true)});
  addBlock("sub", {createTopicPin("/topic1", "std_msgs/Bool", false)}, {});  // Type mismatch
  addConnection("pub1", 0, "sub", 0);
  // pub2 has unused publisher

  AnalysisRules rules;
  rules.checkTypeMismatches = true;
  rules.checkUnusedPubsSubs = true;
  StaticAnalyzer::instance().setRules(rules);

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  int totalCount = result.errorCount + result.warningCount +
                   result.infoCount + result.hintCount;
  EXPECT_EQ(totalCount, result.issues.size());
}

// Test analysis duration is set
TEST_F(StaticAnalyzerTest, AnalysisDurationSet) {
  addBlock("node", {}, {});

  AnalysisResult result = StaticAnalyzer::instance().analyze(project_);

  EXPECT_FALSE(result.analysisDuration.isEmpty());
}

// Test rules getter/setter
TEST_F(StaticAnalyzerTest, RulesGetterSetter) {
  AnalysisRules rules;
  rules.checkTypeMismatches = false;
  rules.checkUnusedPubsSubs = false;
  rules.enableHints = false;

  StaticAnalyzer::instance().setRules(rules);

  AnalysisRules retrieved = StaticAnalyzer::instance().rules();
  EXPECT_EQ(retrieved.checkTypeMismatches, false);
  EXPECT_EQ(retrieved.checkUnusedPubsSubs, false);
  EXPECT_EQ(retrieved.enableHints, false);

  // Reset to default for other tests
  AnalysisRules defaultRules;
  StaticAnalyzer::instance().setRules(defaultRules);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
