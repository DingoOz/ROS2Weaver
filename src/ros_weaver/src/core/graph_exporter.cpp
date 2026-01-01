#include "ros_weaver/core/graph_exporter.hpp"
#include "ros_weaver/core/project.hpp"

#include <QFile>
#include <QTextStream>
#include <QRegularExpression>

namespace ros_weaver {

GraphExporter& GraphExporter::instance() {
  static GraphExporter instance;
  return instance;
}

GraphExporter::GraphExporter(QObject* parent)
    : QObject(parent) {
}

QString GraphExporter::exportToPlantUML(const Project& project, const ExportOptions& options) {
  QString output;
  QTextStream stream(&output);

  stream << generatePlantUMLHeader(options);

  // Generate groups first (as packages in PlantUML)
  if (options.includeGroups) {
    for (const auto& group : project.nodeGroups()) {
      stream << generatePlantUMLGroup(group, project, options);
    }
  }

  // Generate standalone nodes (not in groups)
  QSet<QUuid> groupedNodes;
  for (const auto& group : project.nodeGroups()) {
    for (const auto& nodeId : group.containedNodeIds) {
      groupedNodes.insert(nodeId);
    }
  }

  for (const auto& block : project.blocks()) {
    if (!options.includeGroups || !groupedNodes.contains(block.id)) {
      stream << generatePlantUMLNode(block, options);
    }
  }

  // Generate connections
  for (const auto& conn : project.connections()) {
    stream << generatePlantUMLConnection(conn, project, options);
  }

  stream << generatePlantUMLFooter();

  return output;
}

QString GraphExporter::exportToMermaid(const Project& project, const ExportOptions& options) {
  QString output;
  QTextStream stream(&output);

  stream << generateMermaidHeader(options);

  // Generate groups as subgraphs
  if (options.includeGroups) {
    for (const auto& group : project.nodeGroups()) {
      stream << generateMermaidGroup(group, project, options);
    }
  }

  // Generate standalone nodes
  QSet<QUuid> groupedNodes;
  for (const auto& group : project.nodeGroups()) {
    for (const auto& nodeId : group.containedNodeIds) {
      groupedNodes.insert(nodeId);
    }
  }

  for (const auto& block : project.blocks()) {
    if (!options.includeGroups || !groupedNodes.contains(block.id)) {
      stream << generateMermaidNode(block, options);
    }
  }

  // Generate connections
  for (const auto& conn : project.connections()) {
    stream << generateMermaidConnection(conn, project, options);
  }

  return output;
}

QString GraphExporter::exportToGraphviz(const Project& project, const ExportOptions& options) {
  QString output;
  QTextStream stream(&output);

  stream << generateGraphvizHeader(options);

  // Generate groups as subgraphs
  if (options.includeGroups) {
    for (const auto& group : project.nodeGroups()) {
      stream << generateGraphvizGroup(group, project, options);
    }
  }

  // Generate standalone nodes
  QSet<QUuid> groupedNodes;
  for (const auto& group : project.nodeGroups()) {
    for (const auto& nodeId : group.containedNodeIds) {
      groupedNodes.insert(nodeId);
    }
  }

  for (const auto& block : project.blocks()) {
    if (!options.includeGroups || !groupedNodes.contains(block.id)) {
      stream << generateGraphvizNode(block, options);
    }
  }

  // Generate connections
  for (const auto& conn : project.connections()) {
    stream << generateGraphvizConnection(conn, project, options);
  }

  stream << generateGraphvizFooter();

  return output;
}

bool GraphExporter::exportToFile(const Project& project, const QString& filePath,
                                  ExportFormat format, const ExportOptions& options) {
  QString content;
  switch (format) {
    case ExportFormat::PlantUML:
      content = exportToPlantUML(project, options);
      break;
    case ExportFormat::Mermaid:
      content = exportToMermaid(project, options);
      break;
    case ExportFormat::Graphviz:
      content = exportToGraphviz(project, options);
      break;
  }

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    emit exportError(QString("Failed to open file for writing: %1").arg(filePath));
    return false;
  }

  QTextStream stream(&file);
  stream << content;
  file.close();

  emit exportComplete(filePath);
  return true;
}

QString GraphExporter::fileExtension(ExportFormat format) {
  switch (format) {
    case ExportFormat::PlantUML:
      return "puml";
    case ExportFormat::Mermaid:
      return "mmd";
    case ExportFormat::Graphviz:
      return "dot";
  }
  return "txt";
}

QString GraphExporter::formatName(ExportFormat format) {
  switch (format) {
    case ExportFormat::PlantUML:
      return "PlantUML";
    case ExportFormat::Mermaid:
      return "Mermaid";
    case ExportFormat::Graphviz:
      return "Graphviz DOT";
  }
  return "Unknown";
}

QString GraphExporter::fileFilter(ExportFormat format) {
  switch (format) {
    case ExportFormat::PlantUML:
      return "PlantUML Files (*.puml *.plantuml)";
    case ExportFormat::Mermaid:
      return "Mermaid Files (*.mmd *.mermaid *.md)";
    case ExportFormat::Graphviz:
      return "Graphviz DOT Files (*.dot *.gv)";
  }
  return "All Files (*)";
}

// PlantUML generation
QString GraphExporter::generatePlantUMLHeader(const ExportOptions& options) const {
  QString header = "@startuml\n";
  if (!options.title.isEmpty()) {
    header += QString("title %1\n").arg(options.title);
  }

  // Set direction
  if (options.layoutDirection == "LR") {
    header += "left to right direction\n";
  }

  // Styling
  header += "\n' Node styling\n";
  header += "skinparam component {\n";
  header += "  BackgroundColor #E3F2FD\n";
  header += "  BorderColor #1976D2\n";
  header += "  FontColor #1565C0\n";
  header += "}\n";
  header += "skinparam package {\n";
  header += "  BackgroundColor #F5F5F5\n";
  header += "  BorderColor #9E9E9E\n";
  header += "}\n\n";

  return header;
}

QString GraphExporter::generatePlantUMLNode(const BlockData& block, const ExportOptions& options) const {
  QString nodeId = sanitizeId(block.name);
  QString label = block.name;

  if (options.includeMessageTypes && !block.outputPins.isEmpty()) {
    QString msgType = block.outputPins.first().messageType;
    if (!msgType.isEmpty()) {
      // Extract just the message name
      int lastSlash = msgType.lastIndexOf('/');
      if (lastSlash >= 0) {
        msgType = msgType.mid(lastSlash + 1);
      }
      label += QString("\\n<size:10>%1</size>").arg(msgType);
    }
  }

  return QString("component \"%1\" as %2\n").arg(label, nodeId);
}

QString GraphExporter::generatePlantUMLConnection(const ConnectionData& conn,
                                                   const Project& project,
                                                   const ExportOptions& options) const {
  QString sourceNode = getBlockName(conn.sourceBlockId, project);
  QString targetNode = getBlockName(conn.targetBlockId, project);

  if (sourceNode.isEmpty() || targetNode.isEmpty()) {
    return QString();
  }

  QString sourceId = sanitizeId(sourceNode);
  QString targetId = sanitizeId(targetNode);

  // Find the source block to get the pin label
  QString edgeLabel;
  if (options.includeMessageTypes) {
    for (const auto& block : project.blocks()) {
      if (block.id == conn.sourceBlockId && conn.sourcePinIndex < block.outputPins.size()) {
        QString pinName = block.outputPins[conn.sourcePinIndex].name;
        if (!pinName.isEmpty()) {
          edgeLabel = QString(" : %1").arg(pinName);
        }
        break;
      }
    }
  }

  return QString("%1 --> %2%3\n").arg(sourceId, targetId, edgeLabel);
}

QString GraphExporter::generatePlantUMLGroup(const NodeGroupData& group,
                                              const Project& project,
                                              const ExportOptions& options) const {
  QString output;
  QString groupId = sanitizeId(group.title);

  output += QString("package \"%1\" as %2 {\n").arg(group.title, groupId);

  for (const auto& nodeId : group.containedNodeIds) {
    for (const auto& block : project.blocks()) {
      if (block.id == nodeId) {
        output += "  " + generatePlantUMLNode(block, options);
        break;
      }
    }
  }

  output += "}\n\n";
  return output;
}

QString GraphExporter::generatePlantUMLFooter() const {
  return "@enduml\n";
}

// Mermaid generation
QString GraphExporter::generateMermaidHeader(const ExportOptions& options) const {
  QString direction = options.layoutDirection;
  if (direction.isEmpty()) {
    direction = "TB";
  }
  return QString("graph %1\n").arg(direction);
}

QString GraphExporter::generateMermaidNode(const BlockData& block, const ExportOptions& options) const {
  QString nodeId = sanitizeId(block.name);
  QString label = block.name;

  if (options.includeMessageTypes && !block.outputPins.isEmpty()) {
    QString msgType = block.outputPins.first().messageType;
    if (!msgType.isEmpty()) {
      int lastSlash = msgType.lastIndexOf('/');
      if (lastSlash >= 0) {
        msgType = msgType.mid(lastSlash + 1);
      }
      label += QString("<br/><small>%1</small>").arg(msgType);
    }
  }

  // Use rounded box for nodes
  return QString("    %1[\"%2\"]\n").arg(nodeId, escapeLabel(label));
}

QString GraphExporter::generateMermaidConnection(const ConnectionData& conn,
                                                  const Project& project,
                                                  const ExportOptions& options) const {
  QString sourceNode = getBlockName(conn.sourceBlockId, project);
  QString targetNode = getBlockName(conn.targetBlockId, project);

  if (sourceNode.isEmpty() || targetNode.isEmpty()) {
    return QString();
  }

  QString sourceId = sanitizeId(sourceNode);
  QString targetId = sanitizeId(targetNode);

  // Find edge label (topic name)
  QString edgeLabel;
  if (options.includeMessageTypes) {
    for (const auto& block : project.blocks()) {
      if (block.id == conn.sourceBlockId && conn.sourcePinIndex < block.outputPins.size()) {
        QString pinName = block.outputPins[conn.sourcePinIndex].name;
        if (!pinName.isEmpty()) {
          edgeLabel = QString("|%1|").arg(escapeLabel(pinName));
        }
        break;
      }
    }
  }

  return QString("    %1 -->%2 %3\n").arg(sourceId, edgeLabel, targetId);
}

QString GraphExporter::generateMermaidGroup(const NodeGroupData& group,
                                             const Project& project,
                                             const ExportOptions& options) const {
  QString output;
  QString groupId = sanitizeId(group.title);

  output += QString("    subgraph %1[\"%2\"]\n").arg(groupId, escapeLabel(group.title));

  for (const auto& nodeId : group.containedNodeIds) {
    for (const auto& block : project.blocks()) {
      if (block.id == nodeId) {
        output += "    " + generateMermaidNode(block, options);
        break;
      }
    }
  }

  output += "    end\n";
  return output;
}

// Graphviz generation
QString GraphExporter::generateGraphvizHeader(const ExportOptions& options) const {
  QString output = "digraph ROS2Architecture {\n";

  // Graph attributes
  output += "    // Graph settings\n";
  if (options.layoutDirection == "LR") {
    output += "    rankdir=LR;\n";
  } else if (options.layoutDirection == "BT") {
    output += "    rankdir=BT;\n";
  } else if (options.layoutDirection == "RL") {
    output += "    rankdir=RL;\n";
  } else {
    output += "    rankdir=TB;\n";
  }

  output += "    splines=ortho;\n";
  output += "    nodesep=0.8;\n";
  output += "    ranksep=1.0;\n";

  if (!options.title.isEmpty()) {
    output += QString("    label=\"%1\";\n").arg(escapeLabel(options.title));
    output += "    labelloc=t;\n";
    output += "    fontsize=16;\n";
  }

  // Node styling
  output += "\n    // Node styling\n";
  output += "    node [\n";
  output += "        shape=box,\n";
  output += "        style=\"rounded,filled\",\n";
  output += "        fillcolor=\"#E3F2FD\",\n";
  output += "        color=\"#1976D2\",\n";
  output += "        fontcolor=\"#1565C0\",\n";
  output += "        fontname=\"Arial\"\n";
  output += "    ];\n";

  // Edge styling
  output += "\n    // Edge styling\n";
  output += "    edge [\n";
  output += "        color=\"#64B5F6\",\n";
  output += "        fontsize=10,\n";
  output += "        fontname=\"Arial\"\n";
  output += "    ];\n\n";

  return output;
}

QString GraphExporter::generateGraphvizNode(const BlockData& block, const ExportOptions& options) const {
  QString nodeId = sanitizeId(block.name);
  QString label = block.name;

  if (options.includeMessageTypes && !block.outputPins.isEmpty()) {
    QString msgType = block.outputPins.first().messageType;
    if (!msgType.isEmpty()) {
      int lastSlash = msgType.lastIndexOf('/');
      if (lastSlash >= 0) {
        msgType = msgType.mid(lastSlash + 1);
      }
      label += QString("\\n<font point-size='10'>%1</font>").arg(msgType);
    }
  }

  return QString("    %1 [label=<%2>];\n").arg(nodeId, label);
}

QString GraphExporter::generateGraphvizConnection(const ConnectionData& conn,
                                                   const Project& project,
                                                   const ExportOptions& options) const {
  QString sourceNode = getBlockName(conn.sourceBlockId, project);
  QString targetNode = getBlockName(conn.targetBlockId, project);

  if (sourceNode.isEmpty() || targetNode.isEmpty()) {
    return QString();
  }

  QString sourceId = sanitizeId(sourceNode);
  QString targetId = sanitizeId(targetNode);

  // Find edge label
  QString labelAttr;
  if (options.includeMessageTypes) {
    for (const auto& block : project.blocks()) {
      if (block.id == conn.sourceBlockId && conn.sourcePinIndex < block.outputPins.size()) {
        QString pinName = block.outputPins[conn.sourcePinIndex].name;
        if (!pinName.isEmpty()) {
          labelAttr = QString(" [label=\"%1\"]").arg(escapeLabel(pinName));
        }
        break;
      }
    }
  }

  return QString("    %1 -> %2%3;\n").arg(sourceId, targetId, labelAttr);
}

QString GraphExporter::generateGraphvizGroup(const NodeGroupData& group,
                                              const Project& project,
                                              const ExportOptions& options) const {
  QString output;
  QString groupId = sanitizeId(group.title);

  output += QString("    subgraph cluster_%1 {\n").arg(groupId);
  output += QString("        label=\"%1\";\n").arg(escapeLabel(group.title));
  output += "        style=rounded;\n";
  output += QString("        bgcolor=\"%1\";\n").arg(group.color.name());

  for (const auto& nodeId : group.containedNodeIds) {
    for (const auto& block : project.blocks()) {
      if (block.id == nodeId) {
        output += "    " + generateGraphvizNode(block, options);
        break;
      }
    }
  }

  output += "    }\n\n";
  return output;
}

QString GraphExporter::generateGraphvizFooter() const {
  return "}\n";
}

// Utility functions
QString GraphExporter::sanitizeId(const QString& name) const {
  QString id = name;
  // Replace non-alphanumeric characters with underscores
  id.replace(QRegularExpression("[^a-zA-Z0-9_]"), "_");
  // Ensure it doesn't start with a number
  if (!id.isEmpty() && id[0].isDigit()) {
    id.prepend("node_");
  }
  if (id.isEmpty()) {
    id = "unnamed";
  }
  return id;
}

QString GraphExporter::escapeLabel(const QString& label) const {
  QString escaped = label;
  escaped.replace("\"", "\\\"");
  escaped.replace("\n", "\\n");
  return escaped;
}

QString GraphExporter::getBlockName(const QUuid& blockId, const Project& project) const {
  for (const auto& block : project.blocks()) {
    if (block.id == blockId) {
      return block.name;
    }
  }
  return QString();
}

QString GraphExporter::getPinLabel(const BlockData& block, int pinIndex, bool isOutput) const {
  const auto& pins = isOutput ? block.outputPins : block.inputPins;
  if (pinIndex >= 0 && pinIndex < pins.size()) {
    return pins[pinIndex].name;
  }
  return QString();
}

}  // namespace ros_weaver
