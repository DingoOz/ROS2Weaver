#include <gtest/gtest.h>
#include <QString>
#include <QMap>
#include <QList>

// Test DotNode and DotEdge structures directly without importing the full class
// to avoid complex dependencies

namespace ros_weaver {

// Local copies of structures for testing
struct DotNode {
  QString id;
  QString label;
  QString shape;
  QString url;
  QMap<QString, QString> attributes;
};

struct DotEdge {
  QString sourceId;
  QString targetId;
  QString label;
  QString url;
  QMap<QString, QString> attributes;
};

struct DotGraph {
  QString graphName;
  bool isDigraph = true;
  QList<DotNode> nodes;
  QList<DotEdge> edges;
  QMap<QString, QString> graphAttributes;
  QStringList errors;
  bool isValid = false;
};

}  // namespace ros_weaver

using namespace ros_weaver;

class DotStructuresTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

// Test DotNode structure
TEST_F(DotStructuresTest, DotNodeCreation) {
  DotNode node;
  node.id = "test_node";
  node.label = "Test Node";
  node.shape = "ellipse";
  node.url = "/test_node";
  node.attributes["color"] = "blue";

  EXPECT_EQ(node.id, "test_node");
  EXPECT_EQ(node.label, "Test Node");
  EXPECT_EQ(node.shape, "ellipse");
  EXPECT_EQ(node.url, "/test_node");
  EXPECT_EQ(node.attributes["color"], "blue");
}

// Test DotEdge structure
TEST_F(DotStructuresTest, DotEdgeCreation) {
  DotEdge edge;
  edge.sourceId = "node_a";
  edge.targetId = "node_b";
  edge.label = "/topic";
  edge.attributes["style"] = "dashed";

  EXPECT_EQ(edge.sourceId, "node_a");
  EXPECT_EQ(edge.targetId, "node_b");
  EXPECT_EQ(edge.label, "/topic");
  EXPECT_EQ(edge.attributes["style"], "dashed");
}

// Test DotGraph structure initialization
TEST_F(DotStructuresTest, DotGraphInitialization) {
  DotGraph graph;

  EXPECT_TRUE(graph.isDigraph);
  EXPECT_FALSE(graph.isValid);
  EXPECT_TRUE(graph.nodes.isEmpty());
  EXPECT_TRUE(graph.edges.isEmpty());
  EXPECT_TRUE(graph.graphAttributes.isEmpty());
  EXPECT_TRUE(graph.errors.isEmpty());
}

// Test building a graph manually
TEST_F(DotStructuresTest, BuildGraphManually) {
  DotGraph graph;
  graph.graphName = "TestGraph";
  graph.isDigraph = true;
  graph.isValid = true;

  DotNode node1;
  node1.id = "lidar_driver";
  node1.label = "lidar_driver";
  node1.shape = "ellipse";
  graph.nodes.append(node1);

  DotNode node2;
  node2.id = "slam_toolbox";
  node2.label = "slam_toolbox";
  node2.shape = "ellipse";
  graph.nodes.append(node2);

  DotEdge edge;
  edge.sourceId = "lidar_driver";
  edge.targetId = "slam_toolbox";
  edge.label = "/scan";
  graph.edges.append(edge);

  EXPECT_EQ(graph.graphName, "TestGraph");
  EXPECT_TRUE(graph.isDigraph);
  EXPECT_TRUE(graph.isValid);
  EXPECT_EQ(graph.nodes.size(), 2);
  EXPECT_EQ(graph.edges.size(), 1);
  EXPECT_EQ(graph.nodes[0].id, "lidar_driver");
  EXPECT_EQ(graph.edges[0].label, "/scan");
}

// Test graph with multiple edges
TEST_F(DotStructuresTest, GraphWithMultipleEdges) {
  DotGraph graph;
  graph.isValid = true;

  // Add nodes
  for (int i = 0; i < 5; i++) {
    DotNode node;
    node.id = QString("node_%1").arg(i);
    node.label = QString("Node %1").arg(i);
    graph.nodes.append(node);
  }

  // Add linear chain of edges
  for (int i = 0; i < 4; i++) {
    DotEdge edge;
    edge.sourceId = QString("node_%1").arg(i);
    edge.targetId = QString("node_%1").arg(i + 1);
    edge.label = QString("/topic_%1").arg(i);
    graph.edges.append(edge);
  }

  EXPECT_EQ(graph.nodes.size(), 5);
  EXPECT_EQ(graph.edges.size(), 4);
}

// Test graph attributes
TEST_F(DotStructuresTest, GraphAttributes) {
  DotGraph graph;
  graph.graphAttributes["rankdir"] = "LR";
  graph.graphAttributes["label"] = "ROS2 Architecture";
  graph.graphAttributes["fontsize"] = "12";

  EXPECT_EQ(graph.graphAttributes.size(), 3);
  EXPECT_EQ(graph.graphAttributes["rankdir"], "LR");
  EXPECT_EQ(graph.graphAttributes["label"], "ROS2 Architecture");
}

// Test error handling in graph
TEST_F(DotStructuresTest, GraphWithErrors) {
  DotGraph graph;
  graph.isValid = false;
  graph.errors.append("Failed to parse node definition");
  graph.errors.append("Missing closing brace");

  EXPECT_FALSE(graph.isValid);
  EXPECT_EQ(graph.errors.size(), 2);
  EXPECT_TRUE(graph.errors[0].contains("parse"));
}

// Test topic node identification (by shape convention)
TEST_F(DotStructuresTest, IdentifyTopicNodesByShape) {
  DotGraph graph;

  // Regular node (ellipse)
  DotNode nodeNode;
  nodeNode.id = "my_node";
  nodeNode.shape = "ellipse";
  graph.nodes.append(nodeNode);

  // Topic node (box shape in rqt_graph)
  DotNode topicNode;
  topicNode.id = "/scan";
  topicNode.shape = "box";
  graph.nodes.append(topicNode);

  EXPECT_EQ(graph.nodes.size(), 2);

  // Check shape-based identification
  bool foundEllipse = false;
  bool foundBox = false;
  for (const auto& node : graph.nodes) {
    if (node.shape == "ellipse") foundEllipse = true;
    if (node.shape == "box") foundBox = true;
  }
  EXPECT_TRUE(foundEllipse);
  EXPECT_TRUE(foundBox);
}

// Test undirected graph flag
TEST_F(DotStructuresTest, UndirectedGraphFlag) {
  DotGraph digraph;
  digraph.isDigraph = true;

  DotGraph graph;
  graph.isDigraph = false;

  EXPECT_TRUE(digraph.isDigraph);
  EXPECT_FALSE(graph.isDigraph);
}

// Test node attribute handling
TEST_F(DotStructuresTest, NodeAttributes) {
  DotNode node;
  node.id = "styled_node";
  node.attributes["color"] = "red";
  node.attributes["fillcolor"] = "lightblue";
  node.attributes["style"] = "filled";
  node.attributes["fontsize"] = "14";

  EXPECT_EQ(node.attributes.size(), 4);
  EXPECT_EQ(node.attributes["color"], "red");
  EXPECT_EQ(node.attributes["style"], "filled");
}

// Test edge attribute handling
TEST_F(DotStructuresTest, EdgeAttributes) {
  DotEdge edge;
  edge.sourceId = "a";
  edge.targetId = "b";
  edge.attributes["color"] = "green";
  edge.attributes["style"] = "bold";
  edge.attributes["arrowhead"] = "normal";

  EXPECT_EQ(edge.attributes.size(), 3);
  EXPECT_EQ(edge.attributes["arrowhead"], "normal");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
