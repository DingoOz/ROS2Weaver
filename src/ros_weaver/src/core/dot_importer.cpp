#include "ros_weaver/core/dot_importer.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

DotImporter& DotImporter::instance() {
  static DotImporter instance;
  return instance;
}

DotImporter::DotImporter() : QObject(nullptr) {
}

DotGraph DotImporter::parseFile(const QString& filePath) {
  DotGraph graph;

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    lastError_ = QString("Could not open file: %1").arg(filePath);
    graph.errors.append(lastError_);
    return graph;
  }

  QTextStream in(&file);
  QString content = in.readAll();
  file.close();

  return parseString(content);
}

DotGraph DotImporter::parseString(const QString& content) {
  DotGraph graph;
  emit importStarted();

  // Clean up the content
  QString cleanContent = content;
  cleanContent.replace("\r\n", "\n");
  cleanContent.replace("\r", "\n");

  // Match graph/digraph header
  static QRegularExpression headerPattern(
      R"(^\s*(strict\s+)?(di)?graph\s+(\w+|\"[^\"]+\")?\s*\{)",
      QRegularExpression::MultilineOption | QRegularExpression::CaseInsensitiveOption);

  QRegularExpressionMatch headerMatch = headerPattern.match(cleanContent);
  if (!headerMatch.hasMatch()) {
    lastError_ = "Invalid DOT format: no graph/digraph header found";
    graph.errors.append(lastError_);
    emit importFailed(lastError_);
    return graph;
  }

  graph.isDigraph = headerMatch.captured(2).toLower() == "di";
  graph.graphName = unquoteString(headerMatch.captured(3));

  // Parse nodes - format: "node_id" [attr1=val1, attr2=val2];
  static QRegularExpression nodePattern(
      R"(\"([^\"]+)\"\s*\[([^\]]*)\]\s*;)",
      QRegularExpression::MultilineOption);

  QRegularExpressionMatchIterator nodeIter = nodePattern.globalMatch(cleanContent);
  while (nodeIter.hasNext()) {
    QRegularExpressionMatch match = nodeIter.next();
    DotNode node;
    node.id = match.captured(1);
    node.attributes = parseAttributeList(match.captured(2));
    node.label = unquoteString(node.attributes.value("label", node.id));
    node.shape = node.attributes.value("shape", "ellipse");
    node.url = unquoteString(node.attributes.value("URL", ""));

    // Parse position if available
    QString posStr = node.attributes.value("pos", "");
    if (!posStr.isEmpty()) {
      QStringList posParts = posStr.split(',');
      if (posParts.size() >= 2) {
        node.position = QPointF(posParts[0].toDouble(), posParts[1].toDouble());
      }
    }

    graph.nodes.append(node);
  }

  // Parse edges - format: "source" -> "target" [attr=val];
  static QRegularExpression edgePattern(
      R"(\"([^\"]+)\"\s*->\s*\"([^\"]+)\"\s*(?:\[([^\]]*)\])?\s*;)",
      QRegularExpression::MultilineOption);

  QRegularExpressionMatchIterator edgeIter = edgePattern.globalMatch(cleanContent);
  while (edgeIter.hasNext()) {
    QRegularExpressionMatch match = edgeIter.next();
    DotEdge edge;
    edge.sourceId = match.captured(1);
    edge.targetId = match.captured(2);
    if (match.lastCapturedIndex() >= 3) {
      edge.attributes = parseAttributeList(match.captured(3));
      edge.label = unquoteString(edge.attributes.value("label", ""));
      edge.url = unquoteString(edge.attributes.value("URL", ""));
    }
    graph.edges.append(edge);
  }

  graph.isValid = !graph.nodes.isEmpty();

  if (!graph.isValid) {
    lastError_ = "No nodes found in DOT file";
    graph.errors.append(lastError_);
    emit importFailed(lastError_);
  }

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Parsed DOT graph: %d nodes, %d edges",
              static_cast<int>(graph.nodes.size()),
              static_cast<int>(graph.edges.size()));

  return graph;
}

int DotImporter::importToCanvas(const DotGraph& graph, WeaverCanvas* canvas) {
  if (!canvas || !graph.isValid) {
    return 0;
  }

  emit importStarted();

  // Map DOT node IDs to canvas blocks
  QMap<QString, PackageBlock*> nodeToBlock;
  int nodesCreated = 0;
  int connectionsCreated = 0;

  // Calculate positions for nodes if not provided
  double xOffset = 100.0;
  double yOffset = 100.0;
  double xSpacing = 250.0;
  double ySpacing = 150.0;
  int row = 0;
  int col = 0;
  int nodesPerRow = 4;

  // First pass: identify actual ROS nodes (not topics)
  // In rqt_graph, topics are usually "box" shape and nodes are "ellipse"
  QList<DotNode> rosNodes;
  QSet<QString> topicNames;

  for (const DotNode& node : graph.nodes) {
    if (isTopicNode(node)) {
      topicNames.insert(node.id);
    } else {
      rosNodes.append(node);
    }
  }

  // Create blocks for ROS nodes
  for (const DotNode& node : rosNodes) {
    QPointF pos;
    if (!node.position.isNull()) {
      // Use DOT position (may need scaling)
      pos = node.position;
    } else {
      // Calculate grid position
      pos = QPointF(xOffset + col * xSpacing, yOffset + row * ySpacing);
      col++;
      if (col >= nodesPerRow) {
        col = 0;
        row++;
      }
    }

    // Extract node name from label
    QString nodeName = node.label;
    if (nodeName.isEmpty()) {
      nodeName = node.id;
    }
    // Clean up the name (remove namespace prefixes if too long)
    if (nodeName.startsWith('/')) {
      nodeName = nodeName.mid(1);
    }

    PackageBlock* block = canvas->addPackageBlock(nodeName, pos);
    if (block) {
      nodeToBlock[node.id] = block;
      nodesCreated++;
    }

    emit importProgress(50 * nodesCreated / rosNodes.size(),
                        QString("Creating node: %1").arg(nodeName));
  }

  // Second pass: analyze edges to determine input/output topics for each node
  QMap<QString, QSet<QString>> nodeOutputTopics;  // node -> topics it publishes
  QMap<QString, QSet<QString>> nodeInputTopics;   // node -> topics it subscribes

  for (const DotEdge& edge : graph.edges) {
    if (topicNames.contains(edge.targetId)) {
      // Node -> Topic (publishing)
      nodeOutputTopics[edge.sourceId].insert(edge.targetId);
    }
    if (topicNames.contains(edge.sourceId)) {
      // Topic -> Node (subscribing)
      nodeInputTopics[edge.targetId].insert(edge.sourceId);
    }
  }

  // Add pins to blocks based on topics
  for (auto it = nodeToBlock.begin(); it != nodeToBlock.end(); ++it) {
    const QString& nodeId = it.key();
    PackageBlock* block = it.value();

    // Add output pins for topics this node publishes
    for (const QString& topic : nodeOutputTopics.value(nodeId)) {
      QString topicName = topic;
      if (topicName.startsWith('/')) topicName = topicName.mid(1);
      // Get short name if too long
      int lastSlash = topicName.lastIndexOf('/');
      if (lastSlash > 0) {
        topicName = topicName.mid(lastSlash + 1);
      }
      block->addOutputPin(topicName, Pin::DataType::Topic, "");
    }

    // Add input pins for topics this node subscribes
    for (const QString& topic : nodeInputTopics.value(nodeId)) {
      QString topicName = topic;
      if (topicName.startsWith('/')) topicName = topicName.mid(1);
      int lastSlash = topicName.lastIndexOf('/');
      if (lastSlash > 0) {
        topicName = topicName.mid(lastSlash + 1);
      }
      block->addInputPin(topicName, Pin::DataType::Topic, "");
    }
  }

  // Third pass: create connections between nodes that share topics
  for (const DotEdge& edge : graph.edges) {
    // Skip if edge involves a topic (we handled those above)
    if (topicNames.contains(edge.sourceId) || topicNames.contains(edge.targetId)) {
      continue;
    }

    // Direct node-to-node edge
    PackageBlock* sourceBlock = nodeToBlock.value(edge.sourceId);
    PackageBlock* targetBlock = nodeToBlock.value(edge.targetId);

    if (sourceBlock && targetBlock) {
      // Try to find matching pins
      int sourcePin = sourceBlock->outputPins().isEmpty() ? -1 : 0;
      int targetPin = targetBlock->inputPins().isEmpty() ? -1 : 0;

      if (sourcePin >= 0 && targetPin >= 0) {
        canvas->createConnection(sourceBlock, sourcePin, targetBlock, targetPin);
        connectionsCreated++;
      }
    }
  }

  emit importProgress(100, "Import complete");
  emit importCompleted(nodesCreated, connectionsCreated);

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "DOT import complete: %d nodes, %d connections created",
              nodesCreated, connectionsCreated);

  return nodesCreated;
}

QMap<QString, QString> DotImporter::parseAttributeList(const QString& attrStr) {
  QMap<QString, QString> attrs;

  // Match key=value or key="value" pairs
  static QRegularExpression attrPattern(R"((\w+)\s*=\s*(\"[^\"]*\"|[\w.]+))");

  QRegularExpressionMatchIterator iter = attrPattern.globalMatch(attrStr);
  while (iter.hasNext()) {
    QRegularExpressionMatch match = iter.next();
    QString key = match.captured(1);
    QString value = unquoteString(match.captured(2));
    attrs[key] = value;
  }

  return attrs;
}

QString DotImporter::unquoteString(const QString& str) {
  QString result = str.trimmed();
  if (result.startsWith('"') && result.endsWith('"')) {
    result = result.mid(1, result.length() - 2);
  }
  // Handle escape sequences
  result.replace("\\\"", "\"");
  result.replace("\\n", "\n");
  result.replace("\\\\", "\\");
  return result;
}

bool DotImporter::isTopicNode(const DotNode& node) const {
  // rqt_graph typically uses "box" shape for topics
  // and topics usually start with /
  if (node.shape == "box") {
    return true;
  }
  // Check if it looks like a topic name
  if (node.label.startsWith('/') && !node.label.contains("node")) {
    // Check if it's connected to multiple nodes (topics usually are)
    return true;
  }
  return false;
}

bool DotImporter::isActionNode(const DotNode& node) const {
  // Actions typically have _action or /action in their name
  return node.label.contains("action", Qt::CaseInsensitive);
}

bool DotImporter::isServiceNode(const DotNode& node) const {
  // Services might have specific naming conventions
  return node.shape == "diamond" ||
         node.label.contains("service", Qt::CaseInsensitive);
}

}  // namespace ros_weaver
