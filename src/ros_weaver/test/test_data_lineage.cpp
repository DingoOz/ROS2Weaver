#include <gtest/gtest.h>
#include "ros_weaver/core/data_lineage.hpp"
#include <QJsonDocument>

using namespace ros_weaver;

// LineageNode Tests
TEST(LineageNodeTest, SourceTypeName) {
  LineageNode node;

  node.sourceType = LineageSourceType::Unknown;
  EXPECT_EQ(node.sourceTypeName(), "Unknown");

  node.sourceType = LineageSourceType::ProjectFile;
  EXPECT_EQ(node.sourceTypeName(), "Project File");

  node.sourceType = LineageSourceType::YamlFile;
  EXPECT_EQ(node.sourceTypeName(), "YAML Configuration");

  node.sourceType = LineageSourceType::SourceCodeFile;
  EXPECT_EQ(node.sourceTypeName(), "Source Code");

  node.sourceType = LineageSourceType::RosTopic;
  EXPECT_EQ(node.sourceTypeName(), "ROS Topic");

  node.sourceType = LineageSourceType::RosParameter;
  EXPECT_EQ(node.sourceTypeName(), "ROS Parameter");

  node.sourceType = LineageSourceType::Generated;
  EXPECT_EQ(node.sourceTypeName(), "Generated Code");

  node.sourceType = LineageSourceType::UserInput;
  EXPECT_EQ(node.sourceTypeName(), "User Input");

  node.sourceType = LineageSourceType::SystemDiscovery;
  EXPECT_EQ(node.sourceTypeName(), "System Discovery");
}

TEST(LineageNodeTest, SerializationRoundTrip) {
  LineageNode node;
  node.sourceType = LineageSourceType::YamlFile;
  node.sourcePath = "/path/to/config.yaml";
  node.lineNumber = 42;
  node.columnNumber = 10;
  node.description = "Navigation parameters";
  node.dataKey = "max_velocity";
  node.dataValue = 1.5;
  node.timestamp = "2024-01-01T12:00:00";

  QJsonObject json = node.toJson();
  LineageNode restored = LineageNode::fromJson(json);

  EXPECT_EQ(restored.sourceType, LineageSourceType::YamlFile);
  EXPECT_EQ(restored.sourcePath, "/path/to/config.yaml");
  EXPECT_EQ(restored.lineNumber, 42);
  EXPECT_EQ(restored.columnNumber, 10);
  EXPECT_EQ(restored.description, "Navigation parameters");
  EXPECT_EQ(restored.dataKey, "max_velocity");
  EXPECT_DOUBLE_EQ(restored.dataValue.toDouble(), 1.5);
  EXPECT_EQ(restored.timestamp, "2024-01-01T12:00:00");
}

TEST(LineageNodeTest, IsEditableForNonExistentFile) {
  LineageNode node;
  node.sourceType = LineageSourceType::YamlFile;
  node.sourcePath = "/non/existent/path.yaml";

  // Non-existent file should not be editable
  EXPECT_FALSE(node.isEditable());
}

TEST(LineageNodeTest, IsEditableForNonEditableTypes) {
  LineageNode node;
  node.sourcePath = "/some/path";

  node.sourceType = LineageSourceType::RosTopic;
  EXPECT_FALSE(node.isEditable());

  node.sourceType = LineageSourceType::RosParameter;
  EXPECT_FALSE(node.isEditable());

  node.sourceType = LineageSourceType::UserInput;
  EXPECT_FALSE(node.isEditable());

  node.sourceType = LineageSourceType::SystemDiscovery;
  EXPECT_FALSE(node.isEditable());

  node.sourceType = LineageSourceType::Unknown;
  EXPECT_FALSE(node.isEditable());
}

// DataLineage Tests
TEST(DataLineageTest, DefaultConstructor) {
  DataLineage lineage;

  EXPECT_FALSE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::Unknown);
}

TEST(DataLineageTest, ConstructorWithPrimarySource) {
  LineageNode node;
  node.sourceType = LineageSourceType::ProjectFile;
  node.sourcePath = "/path/to/project.rwp";

  DataLineage lineage(node);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
}

TEST(DataLineageTest, SetDataDescription) {
  DataLineage lineage;
  lineage.setDataDescription("Robot velocity parameter");

  EXPECT_EQ(lineage.dataDescription(), "Robot velocity parameter");
}

TEST(DataLineageTest, IntermediateChain) {
  LineageNode primary;
  primary.sourceType = LineageSourceType::Generated;
  primary.sourcePath = "/generated/code.cpp";

  LineageNode intermediate;
  intermediate.sourceType = LineageSourceType::SourceCodeFile;
  intermediate.sourcePath = "/templates/template.cpp";

  DataLineage lineage(primary);
  lineage.addIntermediateSource(intermediate);

  EXPECT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_EQ(lineage.intermediateChain()[0].sourcePath, "/templates/template.cpp");
}

TEST(DataLineageTest, SerializationRoundTrip) {
  LineageNode primary;
  primary.sourceType = LineageSourceType::YamlFile;
  primary.sourcePath = "/config/params.yaml";
  primary.lineNumber = 10;
  primary.dataKey = "speed";

  LineageNode intermediate;
  intermediate.sourceType = LineageSourceType::ProjectFile;
  intermediate.sourcePath = "/project.rwp";

  DataLineage lineage(primary);
  lineage.setDataDescription("Speed configuration");
  lineage.addIntermediateSource(intermediate);

  QJsonObject json = lineage.toJson();
  DataLineage restored = DataLineage::fromJson(json);

  EXPECT_TRUE(restored.isKnown());
  EXPECT_EQ(restored.dataDescription(), "Speed configuration");
  EXPECT_EQ(restored.primarySource().sourceType, LineageSourceType::YamlFile);
  EXPECT_EQ(restored.primarySource().sourcePath, "/config/params.yaml");
  ASSERT_EQ(restored.intermediateChain().size(), 1);
  EXPECT_EQ(restored.intermediateChain()[0].sourceType, LineageSourceType::ProjectFile);
}

TEST(DataLineageTest, ToDisplayString) {
  LineageNode primary;
  primary.sourceType = LineageSourceType::YamlFile;
  primary.sourcePath = "/config/nav.yaml";
  primary.lineNumber = 15;
  primary.dataKey = "max_speed";
  primary.description = "Maximum robot speed";

  DataLineage lineage(primary);
  lineage.setDataDescription("Speed limit");

  QString display = lineage.toDisplayString();

  EXPECT_TRUE(display.contains("Speed limit"));
  EXPECT_TRUE(display.contains("YAML Configuration"));
  EXPECT_TRUE(display.contains("/config/nav.yaml"));
  EXPECT_TRUE(display.contains("15"));
  EXPECT_TRUE(display.contains("max_speed"));
}

// Factory Method Tests
TEST(DataLineageTest, FromProjectFile) {
  DataLineage lineage = DataLineage::fromProjectFile("/path/project.rwp", "node_name", 25);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/path/project.rwp");
  EXPECT_EQ(lineage.primarySource().lineNumber, 25);
  EXPECT_EQ(lineage.primarySource().dataKey, "node_name");
}

TEST(DataLineageTest, FromYamlFile) {
  DataLineage lineage = DataLineage::fromYamlFile("/config/params.yaml", "velocity", 42);

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::YamlFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/config/params.yaml");
  EXPECT_EQ(lineage.primarySource().dataKey, "velocity");
  EXPECT_EQ(lineage.primarySource().lineNumber, 42);
}

TEST(DataLineageTest, FromSourceFile) {
  DataLineage lineage = DataLineage::fromSourceFile("/src/node.cpp", 100, "Callback function");

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::SourceCodeFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/src/node.cpp");
  EXPECT_EQ(lineage.primarySource().lineNumber, 100);
  EXPECT_EQ(lineage.primarySource().description, "Callback function");
}

TEST(DataLineageTest, FromRosTopic) {
  DataLineage lineage = DataLineage::fromRosTopic("/cmd_vel", "geometry_msgs/msg/Twist");

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::RosTopic);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/cmd_vel");
  EXPECT_EQ(lineage.primarySource().dataKey, "geometry_msgs/msg/Twist");
  EXPECT_FALSE(lineage.primarySource().timestamp.isEmpty());
}

TEST(DataLineageTest, FromGenerated) {
  DataLineage lineage = DataLineage::fromGenerated("/output/node.cpp", "/templates/base.cpp");

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::Generated);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/output/node.cpp");

  // Should have template as intermediate source
  ASSERT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_EQ(lineage.intermediateChain()[0].sourceType, LineageSourceType::SourceCodeFile);
  EXPECT_EQ(lineage.intermediateChain()[0].sourcePath, "/templates/base.cpp");
}

TEST(DataLineageTest, FromGeneratedWithoutTemplate) {
  DataLineage lineage = DataLineage::fromGenerated("/output/node.cpp", "");

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::Generated);
  EXPECT_EQ(lineage.intermediateChain().size(), 0);
}

TEST(DataLineageTest, FromUserInput) {
  DataLineage lineage = DataLineage::fromUserInput("ParamDashboard", "velocity_limit");

  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::UserInput);
  EXPECT_EQ(lineage.primarySource().dataKey, "velocity_limit");
  EXPECT_TRUE(lineage.primarySource().description.contains("ParamDashboard"));
  EXPECT_FALSE(lineage.primarySource().timestamp.isEmpty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
