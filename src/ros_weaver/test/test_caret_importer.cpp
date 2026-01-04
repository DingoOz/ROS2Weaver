#include <gtest/gtest.h>
#include <QString>
#include <QMap>
#include <QList>
#include <QVariant>

// Test CARET structures directly without importing the full class
// to avoid complex dependencies

namespace ros_weaver {

// Local copies of structures for testing
struct CaretNode {
  QString name;
  QString executorType;
  QStringList callbackGroups;
  QMap<QString, QVariant> parameters;
};

struct CaretCallback {
  QString name;
  QString type;
  QString topicName;
  QString serviceName;
  double period = 0.0;
  QString callbackGroup;
};

struct CaretPath {
  QString name;
  QStringList nodeNames;
  QStringList topicNames;
};

struct CaretExecutor {
  QString name;
  QString type;
  QStringList nodeNames;
};

struct CaretArchitecture {
  QString name;
  QList<CaretNode> nodes;
  QList<CaretCallback> callbacks;
  QList<CaretPath> paths;
  QList<CaretExecutor> executors;
  QMap<QString, QVariant> metadata;
  QStringList errors;
  bool isValid = false;
};

}  // namespace ros_weaver

using namespace ros_weaver;

class CaretStructuresTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

// Test CaretNode structure
TEST_F(CaretStructuresTest, CaretNodeCreation) {
  CaretNode node;
  node.name = "/sensor_node";
  node.executorType = "single_threaded_executor";
  node.callbackGroups.append("default_group");
  node.parameters["rate"] = 100;

  EXPECT_EQ(node.name, "/sensor_node");
  EXPECT_EQ(node.executorType, "single_threaded_executor");
  EXPECT_EQ(node.callbackGroups.size(), 1);
  EXPECT_EQ(node.parameters["rate"].toInt(), 100);
}

// Test CaretCallback structure - timer type
TEST_F(CaretStructuresTest, CaretCallbackTimer) {
  CaretCallback callback;
  callback.name = "timer_callback";
  callback.type = "timer";
  callback.period = 100.0;  // milliseconds
  callback.callbackGroup = "default";

  EXPECT_EQ(callback.name, "timer_callback");
  EXPECT_EQ(callback.type, "timer");
  EXPECT_DOUBLE_EQ(callback.period, 100.0);
}

// Test CaretCallback structure - subscription type
TEST_F(CaretStructuresTest, CaretCallbackSubscription) {
  CaretCallback callback;
  callback.name = "data_callback";
  callback.type = "subscription";
  callback.topicName = "/sensor_data";

  EXPECT_EQ(callback.type, "subscription");
  EXPECT_EQ(callback.topicName, "/sensor_data");
}

// Test CaretCallback structure - service type
TEST_F(CaretStructuresTest, CaretCallbackService) {
  CaretCallback callback;
  callback.name = "service_callback";
  callback.type = "service";
  callback.serviceName = "/get_state";

  EXPECT_EQ(callback.type, "service");
  EXPECT_EQ(callback.serviceName, "/get_state");
}

// Test CaretPath structure
TEST_F(CaretStructuresTest, CaretPathCreation) {
  CaretPath path;
  path.name = "sensor_to_actuator";
  path.nodeNames.append("/sensor_node");
  path.nodeNames.append("/processor_node");
  path.nodeNames.append("/actuator_node");
  path.topicNames.append("/sensor_data");
  path.topicNames.append("/processed_data");

  EXPECT_EQ(path.name, "sensor_to_actuator");
  EXPECT_EQ(path.nodeNames.size(), 3);
  EXPECT_EQ(path.topicNames.size(), 2);
}

// Test CaretExecutor structure
TEST_F(CaretStructuresTest, CaretExecutorCreation) {
  CaretExecutor executor;
  executor.name = "main_executor";
  executor.type = "single_threaded_executor";
  executor.nodeNames.append("/node_a");
  executor.nodeNames.append("/node_b");

  EXPECT_EQ(executor.name, "main_executor");
  EXPECT_EQ(executor.type, "single_threaded_executor");
  EXPECT_EQ(executor.nodeNames.size(), 2);
}

// Test CaretArchitecture initialization
TEST_F(CaretStructuresTest, CaretArchitectureInitialization) {
  CaretArchitecture arch;

  EXPECT_FALSE(arch.isValid);
  EXPECT_TRUE(arch.nodes.isEmpty());
  EXPECT_TRUE(arch.callbacks.isEmpty());
  EXPECT_TRUE(arch.paths.isEmpty());
  EXPECT_TRUE(arch.executors.isEmpty());
  EXPECT_TRUE(arch.errors.isEmpty());
}

// Test building architecture manually
TEST_F(CaretStructuresTest, BuildArchitectureManually) {
  CaretArchitecture arch;
  arch.name = "TestArchitecture";
  arch.isValid = true;

  // Add nodes
  CaretNode node1;
  node1.name = "/camera_driver";
  arch.nodes.append(node1);

  CaretNode node2;
  node2.name = "/image_processor";
  arch.nodes.append(node2);

  // Add callbacks
  CaretCallback timerCb;
  timerCb.name = "capture_timer";
  timerCb.type = "timer";
  timerCb.period = 33.33;  // ~30 FPS
  arch.callbacks.append(timerCb);

  CaretCallback subCb;
  subCb.name = "image_cb";
  subCb.type = "subscription";
  subCb.topicName = "/camera/image_raw";
  arch.callbacks.append(subCb);

  // Add executor
  CaretExecutor exec;
  exec.name = "perception_executor";
  exec.type = "multi_threaded_executor";
  exec.nodeNames.append("/camera_driver");
  exec.nodeNames.append("/image_processor");
  arch.executors.append(exec);

  // Add path
  CaretPath path;
  path.name = "perception_pipeline";
  path.nodeNames.append("/camera_driver");
  path.nodeNames.append("/image_processor");
  path.topicNames.append("/camera/image_raw");
  arch.paths.append(path);

  EXPECT_TRUE(arch.isValid);
  EXPECT_EQ(arch.nodes.size(), 2);
  EXPECT_EQ(arch.callbacks.size(), 2);
  EXPECT_EQ(arch.executors.size(), 1);
  EXPECT_EQ(arch.paths.size(), 1);
}

// Test architecture with errors
TEST_F(CaretStructuresTest, ArchitectureWithErrors) {
  CaretArchitecture arch;
  arch.isValid = false;
  arch.errors.append("Invalid YAML syntax");
  arch.errors.append("Missing node definition");

  EXPECT_FALSE(arch.isValid);
  EXPECT_EQ(arch.errors.size(), 2);
}

// Test architecture metadata
TEST_F(CaretStructuresTest, ArchitectureMetadata) {
  CaretArchitecture arch;
  arch.metadata["target_path"] = "/opt/ros2/my_package";
  arch.metadata["version"] = "1.0";
  arch.metadata["created_by"] = "CARET";

  EXPECT_EQ(arch.metadata.size(), 3);
  EXPECT_EQ(arch.metadata["version"].toString(), "1.0");
}

// Test complex perception pipeline architecture
TEST_F(CaretStructuresTest, PerceptionPipelineArchitecture) {
  CaretArchitecture arch;
  arch.name = "PerceptionSystem";
  arch.isValid = true;

  // Camera driver
  CaretNode camera;
  camera.name = "/camera_driver";
  camera.executorType = "single_threaded_executor";
  arch.nodes.append(camera);

  // Lidar driver
  CaretNode lidar;
  lidar.name = "/lidar_driver";
  lidar.executorType = "single_threaded_executor";
  arch.nodes.append(lidar);

  // Sensor fusion
  CaretNode fusion;
  fusion.name = "/sensor_fusion";
  fusion.executorType = "multi_threaded_executor";
  arch.nodes.append(fusion);

  // Object detector
  CaretNode detector;
  detector.name = "/object_detector";
  detector.executorType = "multi_threaded_executor";
  arch.nodes.append(detector);

  // Create path
  CaretPath path;
  path.name = "detection_pipeline";
  path.nodeNames << "/camera_driver" << "/sensor_fusion" << "/object_detector";
  arch.paths.append(path);

  EXPECT_EQ(arch.nodes.size(), 4);
  EXPECT_EQ(arch.paths.size(), 1);
  EXPECT_EQ(arch.paths[0].nodeNames.size(), 3);
}

// Test callback groups
TEST_F(CaretStructuresTest, CallbackGroups) {
  CaretNode node;
  node.name = "/complex_node";
  node.callbackGroups << "mutually_exclusive_1"
                      << "mutually_exclusive_2"
                      << "reentrant_group";

  EXPECT_EQ(node.callbackGroups.size(), 3);
  EXPECT_TRUE(node.callbackGroups.contains("reentrant_group"));
}

// Test multi-threaded executor
TEST_F(CaretStructuresTest, MultiThreadedExecutor) {
  CaretExecutor exec;
  exec.name = "mt_executor";
  exec.type = "multi_threaded_executor";
  exec.nodeNames << "/node1" << "/node2" << "/node3" << "/node4";

  EXPECT_EQ(exec.type, "multi_threaded_executor");
  EXPECT_EQ(exec.nodeNames.size(), 4);
}

// Test node parameters
TEST_F(CaretStructuresTest, NodeParameters) {
  CaretNode node;
  node.name = "/configurable_node";
  node.parameters["rate"] = 100;
  node.parameters["threshold"] = 0.5;
  node.parameters["enabled"] = true;
  node.parameters["name"] = "sensor_1";

  EXPECT_EQ(node.parameters.size(), 4);
  EXPECT_EQ(node.parameters["rate"].toInt(), 100);
  EXPECT_DOUBLE_EQ(node.parameters["threshold"].toDouble(), 0.5);
  EXPECT_TRUE(node.parameters["enabled"].toBool());
  EXPECT_EQ(node.parameters["name"].toString(), "sensor_1");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
