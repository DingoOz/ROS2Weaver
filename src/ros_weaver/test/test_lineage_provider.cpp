#include <gtest/gtest.h>
#include "ros_weaver/core/lineage_provider.hpp"
#include "ros_weaver/core/project.hpp"
#include <QTemporaryFile>
#include <QTextStream>

using namespace ros_weaver;

class LineageProviderTest : public ::testing::Test {
protected:
  void SetUp() override {
    provider_ = new LineageProvider();
  }

  void TearDown() override {
    delete provider_;
  }

  LineageProvider* provider_;
};

// ============================================================================
// Project File Path Tests
// ============================================================================

TEST_F(LineageProviderTest, SetProjectFilePath) {
  provider_->setProjectFilePath("/path/to/project.rwp");

  // Verify by checking lineage includes the path
  DataLineage lineage = provider_->getMetadataLineage("name");
  EXPECT_EQ(lineage.primarySource().sourcePath, "/path/to/project.rwp");
}

// ============================================================================
// Parameter Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetParameterLineage_FromYaml) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getParameterLineage(
      "nav_node", "max_speed", "/config/params.yaml");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::YamlFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/config/params.yaml");
  EXPECT_EQ(lineage.primarySource().dataKey, "max_speed");
  EXPECT_TRUE(lineage.dataDescription().contains("max_speed"));
}

TEST_F(LineageProviderTest, GetParameterLineage_FromBlock) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getParameterLineage(
      "nav_node", "max_speed", "block");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/project/test.rwp");
  EXPECT_TRUE(lineage.primarySource().dataKey.contains("nav_node"));
  EXPECT_TRUE(lineage.primarySource().dataKey.contains("max_speed"));
}

TEST_F(LineageProviderTest, GetParameterLineage_UserInput) {
  DataLineage lineage = provider_->getParameterLineage(
      "nav_node", "max_speed", "");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::UserInput);
  EXPECT_EQ(lineage.primarySource().dataKey, "max_speed");
}

TEST_F(LineageProviderTest, GetParameterLineage_YamlWithProjectIntermediate) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getParameterLineage(
      "nav_node", "max_speed", "/config/params.yaml");

  // Should have project file as intermediate source
  EXPECT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_EQ(lineage.intermediateChain()[0].sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.intermediateChain()[0].sourcePath, "/project/test.rwp");
}

// ============================================================================
// Block Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetBlockLineage_Basic) {
  provider_->setProjectFilePath("/project/test.rwp");

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "teleop_node";

  DataLineage lineage = provider_->getBlockLineage(block);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/project/test.rwp");
  EXPECT_TRUE(lineage.primarySource().dataKey.contains(block.id.toString()));
  EXPECT_TRUE(lineage.dataDescription().contains("teleop_node"));
}

// ============================================================================
// Connection Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetConnectionLineage_LiveData) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getConnectionLineage(
      "/cmd_vel", "geometry_msgs/msg/Twist", true);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::RosTopic);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/cmd_vel");
  EXPECT_EQ(lineage.primarySource().dataKey, "geometry_msgs/msg/Twist");
  EXPECT_TRUE(lineage.primarySource().description.contains("Live"));
}

TEST_F(LineageProviderTest, GetConnectionLineage_ProjectData) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getConnectionLineage(
      "/cmd_vel", "geometry_msgs/msg/Twist", false);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/project/test.rwp");
  EXPECT_EQ(lineage.primarySource().dataKey, "/cmd_vel");
}

TEST_F(LineageProviderTest, GetConnectionLineage_LiveDataWithIntermediate) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getConnectionLineage(
      "/cmd_vel", "geometry_msgs/msg/Twist", true);

  // Should have project file as intermediate source
  EXPECT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_EQ(lineage.intermediateChain()[0].sourceType, LineageSourceType::ProjectFile);
}

// ============================================================================
// ROS Topic Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetRosTopicLineage) {
  DataLineage lineage = provider_->getRosTopicLineage(
      "/scan", "sensor_msgs/msg/LaserScan");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::RosTopic);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/scan");
  EXPECT_EQ(lineage.primarySource().dataKey, "sensor_msgs/msg/LaserScan");
  EXPECT_TRUE(lineage.primarySource().description.contains("ROS2"));
}

// ============================================================================
// TF Frame Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetTfFrameLineage_WithParent) {
  DataLineage lineage = provider_->getTfFrameLineage("base_link", "odom");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::SystemDiscovery);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/tf");
  EXPECT_EQ(lineage.primarySource().dataKey, "base_link");
  EXPECT_TRUE(lineage.primarySource().description.contains("odom"));
  EXPECT_TRUE(lineage.primarySource().description.contains("base_link"));
}

TEST_F(LineageProviderTest, GetTfFrameLineage_WithoutParent) {
  DataLineage lineage = provider_->getTfFrameLineage("base_link");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::SystemDiscovery);
  EXPECT_EQ(lineage.primarySource().dataKey, "base_link");
  EXPECT_TRUE(lineage.primarySource().description.contains("TF tree"));
}

// ============================================================================
// Generated Code Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetGeneratedCodeLineage_Basic) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getGeneratedCodeLineage(
      "/output/my_node.cpp", "my_node");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::Generated);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/output/my_node.cpp");
  EXPECT_TRUE(lineage.primarySource().description.contains("ROS Weaver"));
}

TEST_F(LineageProviderTest, GetGeneratedCodeLineage_WithProjectIntermediate) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getGeneratedCodeLineage(
      "/output/my_node.cpp", "my_node");

  EXPECT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_EQ(lineage.intermediateChain()[0].sourceType, LineageSourceType::ProjectFile);
  EXPECT_TRUE(lineage.intermediateChain()[0].description.contains("my_node"));
}

TEST_F(LineageProviderTest, GetGeneratedCodeLineage_WithoutBlockName) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getGeneratedCodeLineage(
      "/output/launch.py", "");

  EXPECT_EQ(lineage.intermediateChain().size(), 1);
  EXPECT_TRUE(lineage.intermediateChain()[0].description.contains("project"));
}

// ============================================================================
// YAML Config Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetYamlConfigLineage_WithLineNumber) {
  DataLineage lineage = provider_->getYamlConfigLineage(
      "/config/params.yaml", "robot.max_speed", 42);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::YamlFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/config/params.yaml");
  EXPECT_EQ(lineage.primarySource().lineNumber, 42);
  EXPECT_EQ(lineage.primarySource().dataKey, "robot.max_speed");
}

TEST_F(LineageProviderTest, GetYamlConfigLineage_WithoutLineNumber) {
  DataLineage lineage = provider_->getYamlConfigLineage(
      "/config/params.yaml", "robot.max_speed", 0);

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::YamlFile);
  // Line number will be 0 if file doesn't exist or param not found
}

// ============================================================================
// Metadata Lineage Tests
// ============================================================================

TEST_F(LineageProviderTest, GetMetadataLineage) {
  provider_->setProjectFilePath("/project/test.rwp");

  DataLineage lineage = provider_->getMetadataLineage("description");

  EXPECT_TRUE(lineage.isKnown());
  EXPECT_EQ(lineage.primarySource().sourceType, LineageSourceType::ProjectFile);
  EXPECT_EQ(lineage.primarySource().sourcePath, "/project/test.rwp");
  EXPECT_EQ(lineage.primarySource().dataKey, "metadata.description");
  EXPECT_TRUE(lineage.primarySource().description.contains("metadata"));
}

// ============================================================================
// YAML Line Number Finding Tests
// ============================================================================

TEST(LineageProviderStaticTest, FindYamlLineNumber_FileNotFound) {
  int lineNumber = LineageProvider::findYamlLineNumber(
      "/non/existent/file.yaml", "param");

  EXPECT_EQ(lineNumber, 0);
}

TEST(LineageProviderStaticTest, FindYamlLineNumber_WithTempFile) {
  // Create a temporary YAML file
  QTemporaryFile tempFile;
  tempFile.setAutoRemove(true);
  if (tempFile.open()) {
    QTextStream stream(&tempFile);
    stream << "robot:\n";
    stream << "  name: turtlebot\n";
    stream << "  max_speed: 1.5\n";
    stream << "  use_sim: true\n";
    tempFile.close();

    int lineNumber = LineageProvider::findYamlLineNumber(
        tempFile.fileName(), "max_speed");

    EXPECT_EQ(lineNumber, 3);
  }
}

TEST(LineageProviderStaticTest, FindYamlLineNumber_NotFound) {
  QTemporaryFile tempFile;
  tempFile.setAutoRemove(true);
  if (tempFile.open()) {
    QTextStream stream(&tempFile);
    stream << "robot:\n";
    stream << "  name: turtlebot\n";
    tempFile.close();

    int lineNumber = LineageProvider::findYamlLineNumber(
        tempFile.fileName(), "nonexistent_param");

    EXPECT_EQ(lineNumber, 0);
  }
}

TEST(LineageProviderStaticTest, FindYamlLineNumber_NestedParam) {
  QTemporaryFile tempFile;
  tempFile.setAutoRemove(true);
  if (tempFile.open()) {
    QTextStream stream(&tempFile);
    stream << "robot:\n";
    stream << "  navigation:\n";
    stream << "    max_speed: 1.5\n";
    tempFile.close();

    // Should find based on last part of nested path
    int lineNumber = LineageProvider::findYamlLineNumber(
        tempFile.fileName(), "navigation.max_speed");

    EXPECT_EQ(lineNumber, 3);
  }
}

// ============================================================================
// Source Line Number Finding Tests
// ============================================================================

TEST(LineageProviderStaticTest, FindSourceLineNumber_FileNotFound) {
  int lineNumber = LineageProvider::findSourceLineNumber(
      "/non/existent/file.cpp", "pattern");

  EXPECT_EQ(lineNumber, 0);
}

TEST(LineageProviderStaticTest, FindSourceLineNumber_WithTempFile) {
  QTemporaryFile tempFile;
  tempFile.setAutoRemove(true);
  if (tempFile.open()) {
    QTextStream stream(&tempFile);
    stream << "#include <rclcpp/rclcpp.hpp>\n";
    stream << "\n";
    stream << "class MyNode : public rclcpp::Node {\n";
    stream << "public:\n";
    stream << "  void callback() {}\n";
    stream << "};\n";
    tempFile.close();

    int lineNumber = LineageProvider::findSourceLineNumber(
        tempFile.fileName(), "class MyNode");

    EXPECT_EQ(lineNumber, 3);
  }
}

TEST(LineageProviderStaticTest, FindSourceLineNumber_RegexPattern) {
  QTemporaryFile tempFile;
  tempFile.setAutoRemove(true);
  if (tempFile.open()) {
    QTextStream stream(&tempFile);
    stream << "void foo() {}\n";
    stream << "void bar() {}\n";
    stream << "void callback_handler() {}\n";
    tempFile.close();

    int lineNumber = LineageProvider::findSourceLineNumber(
        tempFile.fileName(), "void callback.*\\(\\)");

    EXPECT_EQ(lineNumber, 3);
  }
}

// ============================================================================
// Global Lineage Provider Tests
// ============================================================================

TEST(GlobalLineageProviderTest, SingletonInstance) {
  LineageProvider& instance1 = globalLineageProvider();
  LineageProvider& instance2 = globalLineageProvider();

  EXPECT_EQ(&instance1, &instance2);
}

TEST(GlobalLineageProviderTest, PersistentState) {
  globalLineageProvider().setProjectFilePath("/test/project.rwp");

  DataLineage lineage = globalLineageProvider().getMetadataLineage("name");
  EXPECT_EQ(lineage.primarySource().sourcePath, "/test/project.rwp");
}

// ============================================================================
// Data Description Tests
// ============================================================================

TEST_F(LineageProviderTest, ParameterLineage_HasCorrectDescription) {
  DataLineage lineage = provider_->getParameterLineage(
      "slam_node", "resolution", "/config/slam.yaml");

  EXPECT_TRUE(lineage.dataDescription().contains("Parameter"));
  EXPECT_TRUE(lineage.dataDescription().contains("slam_node"));
  EXPECT_TRUE(lineage.dataDescription().contains("resolution"));
}

TEST_F(LineageProviderTest, TopicLineage_HasCorrectDescription) {
  DataLineage lineage = provider_->getRosTopicLineage(
      "/odom", "nav_msgs/msg/Odometry");

  EXPECT_TRUE(lineage.dataDescription().contains("ROS Topic"));
  EXPECT_TRUE(lineage.dataDescription().contains("/odom"));
}

TEST_F(LineageProviderTest, TfFrameLineage_HasCorrectDescription) {
  DataLineage lineage = provider_->getTfFrameLineage("base_footprint", "odom");

  EXPECT_TRUE(lineage.dataDescription().contains("TF Frame"));
  EXPECT_TRUE(lineage.dataDescription().contains("base_footprint"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
