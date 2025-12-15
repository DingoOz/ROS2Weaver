#include <gtest/gtest.h>
#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"

using namespace ros_weaver;

// Test fixture for CodeGenerator
class CodeGeneratorTest : public ::testing::Test {
protected:
  void SetUp() override {
    generator_ = new CodeGenerator();
  }

  void TearDown() override {
    delete generator_;
  }

  CodeGenerator* generator_;
};

// ============================================================================
// GeneratorOptions Tests
// ============================================================================

TEST(GeneratorOptionsTest, DefaultValues) {
  GeneratorOptions options;
  EXPECT_TRUE(options.generateLaunchFile);
  EXPECT_FALSE(options.generateTests);
  EXPECT_TRUE(options.useCppStyle);
  EXPECT_EQ(options.rosDistro, "humble");
  EXPECT_EQ(options.license, "Apache-2.0");
}

// ============================================================================
// CMakeLists Generation Tests
// ============================================================================

TEST_F(CodeGeneratorTest, GenerateCMakeLists_BasicStructure) {
  Project project;
  GeneratorOptions options;
  options.packageName = "test_package";

  QString cmake = generator_->generateCMakeLists(project, options);

  EXPECT_TRUE(cmake.contains("cmake_minimum_required"));
  EXPECT_TRUE(cmake.contains("project(test_package)"));
  EXPECT_TRUE(cmake.contains("find_package(ament_cmake REQUIRED)"));
  EXPECT_TRUE(cmake.contains("find_package(rclcpp REQUIRED)"));
  EXPECT_TRUE(cmake.contains("ament_package()"));
}

TEST_F(CodeGeneratorTest, GenerateCMakeLists_WithBlocks) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "my_node";
  PinData outPin;
  outPin.name = "output";
  outPin.type = "output";
  outPin.dataType = "topic";
  outPin.messageType = "std_msgs/msg/String";
  block.outputPins.append(outPin);
  project.addBlock(block);

  GeneratorOptions options;
  options.packageName = "test_package";

  QString cmake = generator_->generateCMakeLists(project, options);

  EXPECT_TRUE(cmake.contains("find_package(std_msgs REQUIRED)"));
  EXPECT_TRUE(cmake.contains("add_executable(my_node_node"));
  EXPECT_TRUE(cmake.contains("ament_target_dependencies(my_node_node"));
}

TEST_F(CodeGeneratorTest, GenerateCMakeLists_WithTests) {
  Project project;
  GeneratorOptions options;
  options.packageName = "test_package";
  options.generateTests = true;

  QString cmake = generator_->generateCMakeLists(project, options);

  EXPECT_TRUE(cmake.contains("if(BUILD_TESTING)"));
  EXPECT_TRUE(cmake.contains("find_package(ament_lint_auto REQUIRED)"));
}

// ============================================================================
// package.xml Generation Tests
// ============================================================================

TEST_F(CodeGeneratorTest, GeneratePackageXml_BasicStructure) {
  Project project;
  project.metadata().description = "A test package";

  GeneratorOptions options;
  options.packageName = "test_package";
  options.maintainer = "Test User";
  options.maintainerEmail = "test@example.com";
  options.license = "MIT";

  QString packageXml = generator_->generatePackageXml(project, options);

  EXPECT_TRUE(packageXml.contains("<name>test_package</name>"));
  EXPECT_TRUE(packageXml.contains("<description>A test package</description>"));
  EXPECT_TRUE(packageXml.contains("<maintainer email=\"test@example.com\">Test User</maintainer>"));
  EXPECT_TRUE(packageXml.contains("<license>MIT</license>"));
  EXPECT_TRUE(packageXml.contains("<depend>rclcpp</depend>"));
  EXPECT_TRUE(packageXml.contains("<build_type>ament_cmake</build_type>"));
}

TEST_F(CodeGeneratorTest, GeneratePackageXml_WithMessageDependencies) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "test_node";
  PinData pin;
  pin.name = "cmd_vel";
  pin.type = "output";
  pin.dataType = "topic";
  pin.messageType = "geometry_msgs/msg/Twist";
  block.outputPins.append(pin);
  project.addBlock(block);

  GeneratorOptions options;
  options.packageName = "test_package";

  QString packageXml = generator_->generatePackageXml(project, options);

  EXPECT_TRUE(packageXml.contains("<depend>geometry_msgs</depend>"));
}

// ============================================================================
// Params YAML Generation Tests
// ============================================================================

TEST_F(CodeGeneratorTest, GenerateParamsYaml_BasicStructure) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "my_node";
  project.addBlock(block);

  QString yaml = generator_->generateParamsYaml(project);

  EXPECT_TRUE(yaml.contains("# Auto-generated parameters file"));
  EXPECT_TRUE(yaml.contains("my_node:"));
  EXPECT_TRUE(yaml.contains("ros__parameters:"));
  EXPECT_TRUE(yaml.contains("use_sim_time: false"));
}

TEST_F(CodeGeneratorTest, GenerateParamsYaml_WithParameters) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "nav_node";

  BlockParamData param;
  param.name = "max_speed";
  param.type = "double";
  param.defaultValue = 1.5;
  param.currentValue = 2.0;
  param.description = "Maximum robot speed";
  block.parameters.append(param);

  project.addBlock(block);

  QString yaml = generator_->generateParamsYaml(project);

  EXPECT_TRUE(yaml.contains("nav_node:"));
  EXPECT_TRUE(yaml.contains("# Maximum robot speed"));
  EXPECT_TRUE(yaml.contains("max_speed: 2"));
}

// ============================================================================
// Launch File Generation Tests
// ============================================================================

TEST_F(CodeGeneratorTest, GenerateLaunchFile_BasicStructure) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "test_node";
  project.addBlock(block);

  GeneratorOptions options;
  options.packageName = "test_package";

  QString launch = generator_->generateLaunchFile(project, options);

  EXPECT_TRUE(launch.contains("from launch import LaunchDescription"));
  EXPECT_TRUE(launch.contains("from launch_ros.actions import Node"));
  EXPECT_TRUE(launch.contains("def generate_launch_description()"));
  EXPECT_TRUE(launch.contains("get_package_share_directory('test_package')"));
  EXPECT_TRUE(launch.contains("use_sim_time"));
}

TEST_F(CodeGeneratorTest, GenerateLaunchFile_NodeConfiguration) {
  Project project;

  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "my_controller";
  project.addBlock(block);

  GeneratorOptions options;
  options.packageName = "robot_pkg";

  QString launch = generator_->generateLaunchFile(project, options);

  EXPECT_TRUE(launch.contains("package='robot_pkg'"));
  EXPECT_TRUE(launch.contains("executable='my_controller_node'"));
  EXPECT_TRUE(launch.contains("name='my_controller'"));
}

// ============================================================================
// Node Generation Tests
// ============================================================================

TEST_F(CodeGeneratorTest, GenerateNodeCpp_PublisherNode) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "talker";

  PinData outPin;
  outPin.name = "chatter";
  outPin.type = "output";
  outPin.dataType = "topic";
  outPin.messageType = "std_msgs/msg/String";
  block.outputPins.append(outPin);

  QString code = generator_->generateNodeCpp(block, {}, {});

  EXPECT_TRUE(code.contains("#include <rclcpp/rclcpp.hpp>"));
  EXPECT_TRUE(code.contains("#include <std_msgs/msg/string.hpp>"));
  EXPECT_TRUE(code.contains("class TalkerNode : public rclcpp::Node"));
  EXPECT_TRUE(code.contains("create_publisher<std_msgs::msg::String>"));
  EXPECT_TRUE(code.contains("create_wall_timer"));
  EXPECT_TRUE(code.contains("rclcpp::init(argc, argv)"));
}

TEST_F(CodeGeneratorTest, GenerateNodeCpp_SubscriberNode) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "listener";

  PinData inPin;
  inPin.name = "chatter";
  inPin.type = "input";
  inPin.dataType = "topic";
  inPin.messageType = "std_msgs/msg/String";
  block.inputPins.append(inPin);

  QString code = generator_->generateNodeCpp(block, {}, {});

  EXPECT_TRUE(code.contains("class ListenerNode : public rclcpp::Node"));
  EXPECT_TRUE(code.contains("create_subscription<std_msgs::msg::String>"));
  EXPECT_TRUE(code.contains("void on_chatter"));
  EXPECT_TRUE(code.contains("SharedPtr msg"));
}

TEST_F(CodeGeneratorTest, GenerateNodeCpp_WithParameters) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "param_node";

  QList<ParamDefinition> params;
  ParamDefinition param1;
  param1.name = "rate";
  param1.type = "double";
  param1.defaultValue = 10.0;
  params.append(param1);

  ParamDefinition param2;
  param2.name = "topic_name";
  param2.type = "string";
  param2.defaultValue = "output";
  params.append(param2);

  QString code = generator_->generateNodeCpp(block, {}, params);

  EXPECT_TRUE(code.contains("declare_parameter(\"rate\""));
  EXPECT_TRUE(code.contains("declare_parameter(\"topic_name\""));
}

TEST_F(CodeGeneratorTest, GenerateNodePython_PublisherNode) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "talker";

  PinData outPin;
  outPin.name = "chatter";
  outPin.type = "output";
  outPin.dataType = "topic";
  outPin.messageType = "std_msgs/msg/String";
  block.outputPins.append(outPin);

  QString code = generator_->generateNodePython(block, {}, {});

  EXPECT_TRUE(code.contains("#!/usr/bin/env python3"));
  EXPECT_TRUE(code.contains("import rclpy"));
  EXPECT_TRUE(code.contains("from rclpy.node import Node"));
  EXPECT_TRUE(code.contains("from std_msgs.msg import String"));
  EXPECT_TRUE(code.contains("class TalkerNode(Node)"));
  EXPECT_TRUE(code.contains("create_publisher"));
  EXPECT_TRUE(code.contains("create_timer"));
  EXPECT_TRUE(code.contains("def main"));
}

TEST_F(CodeGeneratorTest, GenerateNodePython_SubscriberNode) {
  BlockData block;
  block.id = QUuid::createUuid();
  block.name = "listener";

  PinData inPin;
  inPin.name = "chatter";
  inPin.type = "input";
  inPin.dataType = "topic";
  inPin.messageType = "std_msgs/msg/String";
  block.inputPins.append(inPin);

  QString code = generator_->generateNodePython(block, {}, {});

  EXPECT_TRUE(code.contains("class ListenerNode(Node)"));
  EXPECT_TRUE(code.contains("create_subscription"));
  EXPECT_TRUE(code.contains("def on_chatter"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
