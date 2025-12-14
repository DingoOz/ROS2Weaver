#include <gtest/gtest.h>
#include "ros_weaver/core/project.hpp"
#include <QJsonDocument>

using namespace ros_weaver;

class ProjectTest : public ::testing::Test {
protected:
  void SetUp() override {
    project_ = Project();
  }

  Project project_;
};

// PinData Tests
TEST(PinDataTest, SerializationRoundTrip) {
  PinData pin;
  pin.name = "cmd_vel";
  pin.type = "output";
  pin.dataType = "topic";
  pin.messageType = "geometry_msgs/msg/Twist";

  QJsonObject json = pin.toJson();
  PinData restored = PinData::fromJson(json);

  EXPECT_EQ(restored.name, pin.name);
  EXPECT_EQ(restored.type, pin.type);
  EXPECT_EQ(restored.dataType, pin.dataType);
  EXPECT_EQ(restored.messageType, pin.messageType);
}

// BlockParamData Tests
TEST(BlockParamDataTest, StringParamSerialization) {
  BlockParamData param;
  param.name = "frame_id";
  param.type = "string";
  param.defaultValue = "base_link";
  param.currentValue = "odom";
  param.description = "Reference frame";
  param.group = "general";

  QJsonObject json = param.toJson();
  BlockParamData restored = BlockParamData::fromJson(json);

  EXPECT_EQ(restored.name, param.name);
  EXPECT_EQ(restored.type, param.type);
  EXPECT_EQ(restored.defaultValue.toString(), "base_link");
  EXPECT_EQ(restored.currentValue.toString(), "odom");
  EXPECT_EQ(restored.description, param.description);
}

TEST(BlockParamDataTest, IntParamWithMinMaxSerialization) {
  BlockParamData param;
  param.name = "queue_size";
  param.type = "int";
  param.defaultValue = 10;
  param.currentValue = 20;
  param.minValue = 1;
  param.maxValue = 100;

  QJsonObject json = param.toJson();
  BlockParamData restored = BlockParamData::fromJson(json);

  EXPECT_EQ(restored.currentValue.toInt(), 20);
  EXPECT_EQ(restored.minValue.toInt(), 1);
  EXPECT_EQ(restored.maxValue.toInt(), 100);
}

TEST(BlockParamDataTest, BoolParamSerialization) {
  BlockParamData param;
  param.name = "use_sim_time";
  param.type = "bool";
  param.defaultValue = false;
  param.currentValue = true;

  QJsonObject json = param.toJson();
  BlockParamData restored = BlockParamData::fromJson(json);

  EXPECT_EQ(restored.defaultValue.toBool(), false);
  EXPECT_EQ(restored.currentValue.toBool(), true);
}

// BlockData Tests
TEST(BlockDataTest, SerializationRoundTrip) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "teleop_twist";
  block.position = QPointF(100.0, 200.0);

  PinData outPin;
  outPin.name = "cmd_vel";
  outPin.type = "output";
  outPin.dataType = "topic";
  outPin.messageType = "geometry_msgs/msg/Twist";
  block.outputPins.append(outPin);

  QJsonObject json = block.toJson();
  BlockData restored = BlockData::fromJson(json);

  EXPECT_EQ(restored.id, block.id);
  EXPECT_EQ(restored.name, block.name);
  EXPECT_DOUBLE_EQ(restored.position.x(), 100.0);
  EXPECT_DOUBLE_EQ(restored.position.y(), 200.0);
  ASSERT_EQ(restored.outputPins.size(), 1);
  EXPECT_EQ(restored.outputPins[0].name, "cmd_vel");
}

// ConnectionData Tests
TEST(ConnectionDataTest, SerializationRoundTrip) {
  ConnectionData conn;
  conn.id = QUuid::createUuid();
  conn.sourceBlockId = QUuid::createUuid();
  conn.sourcePinIndex = 0;
  conn.targetBlockId = QUuid::createUuid();
  conn.targetPinIndex = 1;

  QJsonObject json = conn.toJson();
  ConnectionData restored = ConnectionData::fromJson(json);

  EXPECT_EQ(restored.id, conn.id);
  EXPECT_EQ(restored.sourceBlockId, conn.sourceBlockId);
  EXPECT_EQ(restored.sourcePinIndex, 0);
  EXPECT_EQ(restored.targetBlockId, conn.targetBlockId);
  EXPECT_EQ(restored.targetPinIndex, 1);
}

// NodeGroupData Tests
TEST(NodeGroupDataTest, SerializationRoundTrip) {
  NodeGroupData group;
  group.id = QUuid::createUuid();
  group.title = "Navigation Stack";
  group.position = QPointF(50.0, 50.0);
  group.size = QSizeF(300.0, 200.0);
  group.color = QColor(100, 150, 200, 128);
  group.containedNodeIds.append(QUuid::createUuid());
  group.containedNodeIds.append(QUuid::createUuid());

  QJsonObject json = group.toJson();
  NodeGroupData restored = NodeGroupData::fromJson(json);

  EXPECT_EQ(restored.id, group.id);
  EXPECT_EQ(restored.title, "Navigation Stack");
  EXPECT_DOUBLE_EQ(restored.size.width(), 300.0);
  EXPECT_EQ(restored.color.red(), 100);
  EXPECT_EQ(restored.color.alpha(), 128);
  EXPECT_EQ(restored.containedNodeIds.size(), 2);
}

// ProjectMetadata Tests
TEST(ProjectMetadataTest, SerializationRoundTrip) {
  ProjectMetadata meta;
  meta.name = "Test Project";
  meta.description = "A test project";
  meta.author = "Test Author";
  meta.version = "1.0";
  meta.rosDistro = "jazzy";

  QJsonObject json = meta.toJson();
  ProjectMetadata restored = ProjectMetadata::fromJson(json);

  EXPECT_EQ(restored.name, "Test Project");
  EXPECT_EQ(restored.description, "A test project");
  EXPECT_EQ(restored.author, "Test Author");
  EXPECT_EQ(restored.rosDistro, "jazzy");
}

// YamlFileInfo Tests
TEST(YamlFileInfoTest, SerializationRoundTrip) {
  YamlFileInfo info;
  info.filePath = "/path/to/params.yaml";
  info.nodeNames.append("slam_toolbox");
  info.nodeNames.append("amcl");

  QJsonObject json = info.toJson();
  YamlFileInfo restored = YamlFileInfo::fromJson(json);

  EXPECT_EQ(restored.filePath, "/path/to/params.yaml");
  ASSERT_EQ(restored.nodeNames.size(), 2);
  EXPECT_EQ(restored.nodeNames[0], "slam_toolbox");
  EXPECT_EQ(restored.nodeNames[1], "amcl");
}

// Project Tests
TEST_F(ProjectTest, AddAndRemoveBlock) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "test_node";

  project_.addBlock(block);
  EXPECT_EQ(project_.blocks().size(), 1);
  EXPECT_TRUE(project_.hasUnsavedChanges());

  project_.removeBlock(block.id);
  EXPECT_EQ(project_.blocks().size(), 0);
}

TEST_F(ProjectTest, AddAndRemoveConnection) {
  ConnectionData conn;
  conn.id = QUuid::createUuid();
  conn.sourceBlockId = QUuid::createUuid();
  conn.targetBlockId = QUuid::createUuid();

  project_.addConnection(conn);
  EXPECT_EQ(project_.connections().size(), 1);

  project_.removeConnection(conn.id);
  EXPECT_EQ(project_.connections().size(), 0);
}

TEST_F(ProjectTest, AddAndRemoveNodeGroup) {
  NodeGroupData group;
  group.id = QUuid::createUuid();
  group.title = "Test Group";

  project_.addNodeGroup(group);
  EXPECT_EQ(project_.nodeGroups().size(), 1);

  project_.removeNodeGroup(group.id);
  EXPECT_EQ(project_.nodeGroups().size(), 0);
}

TEST_F(ProjectTest, YamlFileManagement) {
  YamlFileInfo yaml1;
  yaml1.filePath = "/path/to/nav.yaml";
  yaml1.nodeNames.append("slam_toolbox");

  YamlFileInfo yaml2;
  yaml2.filePath = "/path/to/sensors.yaml";
  yaml2.nodeNames.append("lidar_node");

  project_.addYamlFile(yaml1);
  project_.addYamlFile(yaml2);
  EXPECT_EQ(project_.yamlFiles().size(), 2);

  // Duplicate should not be added
  project_.addYamlFile(yaml1);
  EXPECT_EQ(project_.yamlFiles().size(), 2);

  // Find YAML for node
  QString found = project_.findYamlFileForNode("slam_toolbox");
  EXPECT_EQ(found, "/path/to/nav.yaml");

  QString notFound = project_.findYamlFileForNode("unknown_node");
  EXPECT_TRUE(notFound.isEmpty());

  project_.removeYamlFile("/path/to/nav.yaml");
  EXPECT_EQ(project_.yamlFiles().size(), 1);
}

TEST_F(ProjectTest, ClearProject) {
  BlockData block;
  block.id = QUuid::createUuid();
  project_.addBlock(block);

  ConnectionData conn;
  conn.id = QUuid::createUuid();
  project_.addConnection(conn);

  project_.clear();

  EXPECT_EQ(project_.blocks().size(), 0);
  EXPECT_EQ(project_.connections().size(), 0);
  EXPECT_FALSE(project_.hasUnsavedChanges());
}

TEST_F(ProjectTest, FullSerializationRoundTrip) {
  // Setup metadata
  project_.metadata().name = "Complete Test";
  project_.metadata().rosDistro = "jazzy";

  // Add a block with parameters
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "nav_node";
  block.position = QPointF(100, 200);

  BlockParamData param;
  param.name = "update_rate";
  param.type = "double";
  param.currentValue = 10.0;
  block.parameters.append(param);

  PinData pin;
  pin.name = "odom";
  pin.type = "input";
  block.inputPins.append(pin);

  project_.addBlock(block);

  // Add a node group
  NodeGroupData group;
  group.id = QUuid::createUuid();
  group.title = "Navigation";
  group.containedNodeIds.append(block.id);
  project_.addNodeGroup(group);

  // Serialize and restore
  QJsonObject json = project_.toJson();
  Project restored = Project::fromJson(json);

  // Verify
  EXPECT_EQ(restored.metadata().name, "Complete Test");
  EXPECT_EQ(restored.metadata().rosDistro, "jazzy");
  ASSERT_EQ(restored.blocks().size(), 1);
  EXPECT_EQ(restored.blocks()[0].name, "nav_node");
  ASSERT_EQ(restored.blocks()[0].parameters.size(), 1);
  EXPECT_EQ(restored.blocks()[0].parameters[0].name, "update_rate");
  ASSERT_EQ(restored.nodeGroups().size(), 1);
  EXPECT_EQ(restored.nodeGroups()[0].title, "Navigation");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
