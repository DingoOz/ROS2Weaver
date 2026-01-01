#include <gtest/gtest.h>
#include "ros_weaver/core/graph_exporter.hpp"
#include "ros_weaver/core/project.hpp"

using namespace ros_weaver;

class GraphExporterTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a simple test project
    project_.metadata().name = "TestProject";

    // Add two blocks
    BlockData block1;
    block1.id = QUuid::createUuid();
    block1.name = "lidar_driver";
    block1.position = QPointF(0, 0);
    PinData outPin1;
    outPin1.name = "/scan";
    outPin1.type = "output";
    outPin1.dataType = "topic";
    outPin1.messageType = "sensor_msgs/LaserScan";
    block1.outputPins.append(outPin1);
    project_.addBlock(block1);

    BlockData block2;
    block2.id = QUuid::createUuid();
    block2.name = "slam_toolbox";
    block2.position = QPointF(200, 0);
    PinData inPin2;
    inPin2.name = "/scan";
    inPin2.type = "input";
    inPin2.dataType = "topic";
    inPin2.messageType = "sensor_msgs/LaserScan";
    block2.inputPins.append(inPin2);
    PinData outPin2;
    outPin2.name = "/map";
    outPin2.type = "output";
    outPin2.dataType = "topic";
    outPin2.messageType = "nav_msgs/OccupancyGrid";
    block2.outputPins.append(outPin2);
    project_.addBlock(block2);

    // Add a connection
    ConnectionData conn;
    conn.id = QUuid::createUuid();
    conn.sourceBlockId = block1.id;
    conn.sourcePinIndex = 0;
    conn.targetBlockId = block2.id;
    conn.targetPinIndex = 0;
    project_.addConnection(conn);

    // Store block IDs for tests
    block1Id_ = block1.id;
    block2Id_ = block2.id;
  }

  Project project_;
  QUuid block1Id_;
  QUuid block2Id_;
};

TEST_F(GraphExporterTest, ExportToPlantUML) {
  ExportOptions options;
  options.title = "Test Diagram";

  QString output = GraphExporter::instance().exportToPlantUML(project_, options);

  // Check header
  EXPECT_TRUE(output.contains("@startuml"));
  EXPECT_TRUE(output.contains("@enduml"));
  EXPECT_TRUE(output.contains("title Test Diagram"));

  // Check nodes
  EXPECT_TRUE(output.contains("lidar_driver"));
  EXPECT_TRUE(output.contains("slam_toolbox"));

  // Check connection
  EXPECT_TRUE(output.contains("lidar_driver --> slam_toolbox"));
}

TEST_F(GraphExporterTest, ExportToMermaid) {
  ExportOptions options;
  options.layoutDirection = "LR";

  QString output = GraphExporter::instance().exportToMermaid(project_, options);

  // Check header
  EXPECT_TRUE(output.contains("graph LR"));

  // Check nodes
  EXPECT_TRUE(output.contains("lidar_driver"));
  EXPECT_TRUE(output.contains("slam_toolbox"));

  // Check connection arrow
  EXPECT_TRUE(output.contains("-->"));
}

TEST_F(GraphExporterTest, ExportToGraphviz) {
  ExportOptions options;
  options.title = "ROS2 Architecture";

  QString output = GraphExporter::instance().exportToGraphviz(project_, options);

  // Check header
  EXPECT_TRUE(output.contains("digraph"));
  EXPECT_TRUE(output.contains("label=\"ROS2 Architecture\""));

  // Check nodes
  EXPECT_TRUE(output.contains("lidar_driver"));
  EXPECT_TRUE(output.contains("slam_toolbox"));

  // Check connection
  EXPECT_TRUE(output.contains("lidar_driver -> slam_toolbox"));

  // Check footer
  EXPECT_TRUE(output.endsWith("}\n"));
}

TEST_F(GraphExporterTest, ExportWithMessageTypes) {
  ExportOptions options;
  options.includeMessageTypes = true;

  QString plantUml = GraphExporter::instance().exportToPlantUML(project_, options);
  EXPECT_TRUE(plantUml.contains("LaserScan"));

  QString mermaid = GraphExporter::instance().exportToMermaid(project_, options);
  EXPECT_TRUE(mermaid.contains("/scan"));

  QString graphviz = GraphExporter::instance().exportToGraphviz(project_, options);
  EXPECT_TRUE(graphviz.contains("/scan"));
}

TEST_F(GraphExporterTest, ExportWithoutMessageTypes) {
  ExportOptions options;
  options.includeMessageTypes = false;

  QString plantUml = GraphExporter::instance().exportToPlantUML(project_, options);

  // Should still have node names but not topic labels on edges
  EXPECT_TRUE(plantUml.contains("lidar_driver"));
  // Edge should be simple without label
  EXPECT_TRUE(plantUml.contains("lidar_driver --> slam_toolbox\n"));
}

TEST_F(GraphExporterTest, FileExtensions) {
  EXPECT_EQ(GraphExporter::fileExtension(ExportFormat::PlantUML), "puml");
  EXPECT_EQ(GraphExporter::fileExtension(ExportFormat::Mermaid), "mmd");
  EXPECT_EQ(GraphExporter::fileExtension(ExportFormat::Graphviz), "dot");
}

TEST_F(GraphExporterTest, FormatNames) {
  EXPECT_EQ(GraphExporter::formatName(ExportFormat::PlantUML), "PlantUML");
  EXPECT_EQ(GraphExporter::formatName(ExportFormat::Mermaid), "Mermaid");
  EXPECT_EQ(GraphExporter::formatName(ExportFormat::Graphviz), "Graphviz DOT");
}

TEST_F(GraphExporterTest, FileFilters) {
  EXPECT_TRUE(GraphExporter::fileFilter(ExportFormat::PlantUML).contains("*.puml"));
  EXPECT_TRUE(GraphExporter::fileFilter(ExportFormat::Mermaid).contains("*.mmd"));
  EXPECT_TRUE(GraphExporter::fileFilter(ExportFormat::Graphviz).contains("*.dot"));
}

TEST_F(GraphExporterTest, ExportWithGroups) {
  // Add a group containing both blocks
  NodeGroupData group;
  group.id = QUuid::createUuid();
  group.title = "Perception";
  group.color = QColor(100, 150, 200);
  group.containedNodeIds.append(block1Id_);
  group.containedNodeIds.append(block2Id_);
  project_.addNodeGroup(group);

  ExportOptions options;
  options.includeGroups = true;

  QString plantUml = GraphExporter::instance().exportToPlantUML(project_, options);
  EXPECT_TRUE(plantUml.contains("package \"Perception\""));

  QString mermaid = GraphExporter::instance().exportToMermaid(project_, options);
  EXPECT_TRUE(mermaid.contains("subgraph"));
  EXPECT_TRUE(mermaid.contains("Perception"));

  QString graphviz = GraphExporter::instance().exportToGraphviz(project_, options);
  EXPECT_TRUE(graphviz.contains("subgraph cluster_"));
  EXPECT_TRUE(graphviz.contains("Perception"));
}

TEST_F(GraphExporterTest, EmptyProject) {
  Project emptyProject;

  QString plantUml = GraphExporter::instance().exportToPlantUML(emptyProject);
  EXPECT_TRUE(plantUml.contains("@startuml"));
  EXPECT_TRUE(plantUml.contains("@enduml"));

  QString mermaid = GraphExporter::instance().exportToMermaid(emptyProject);
  EXPECT_TRUE(mermaid.contains("graph TB"));

  QString graphviz = GraphExporter::instance().exportToGraphviz(emptyProject);
  EXPECT_TRUE(graphviz.contains("digraph"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
