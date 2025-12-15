#include "ros_weaver/core/ai_context_provider.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/core/system_discovery.hpp"
#include <QJsonDocument>
#include <QJsonArray>

namespace ros_weaver {

AIContextProvider& AIContextProvider::instance() {
  static AIContextProvider instance;
  return instance;
}

AIContextProvider::AIContextProvider() : QObject(nullptr) {}

AIContextProvider::~AIContextProvider() = default;

void AIContextProvider::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void AIContextProvider::setSystemDiscovery(SystemDiscovery* discovery) {
  systemDiscovery_ = discovery;
}

QString AIContextProvider::getProjectContext() const {
  QStringList context;

  context << "=== ROS Weaver Project Context ===\n";

  if (!canvas_) {
    context << "No canvas available.\n";
    return context.join("\n");
  }

  // Count items
  int blockCount = 0;
  int connectionCount = canvas_->connections().size();
  int groupCount = canvas_->nodeGroups().size();

  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (dynamic_cast<PackageBlock*>(item)) {
      blockCount++;
    }
  }

  context << QString("Project contains: %1 blocks, %2 connections, %3 groups\n")
                 .arg(blockCount)
                 .arg(connectionCount)
                 .arg(groupCount);

  // List blocks
  if (blockCount > 0) {
    context << "\n## Blocks on Canvas:";
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        context << formatBlockInfo(block);
      }
    }
  }

  // List connections
  if (connectionCount > 0) {
    context << "\n## Connections:";
    for (ConnectionLine* conn : canvas_->connections()) {
      context << formatConnectionInfo(conn);
    }
  }

  // List groups
  if (groupCount > 0) {
    context << "\n## Groups:";
    for (NodeGroup* group : canvas_->nodeGroups()) {
      QStringList blockNames;
      for (PackageBlock* block : group->containedNodes()) {
        blockNames << block->packageName();
      }
      context << QString("- %1: contains [%2]")
                     .arg(group->title(), blockNames.join(", "));
    }
  }

  // ROS2 system state if available
  QString ros2State = getROS2SystemState();
  if (!ros2State.isEmpty()) {
    context << "\n" << ros2State;
  }

  return context.join("\n");
}

QJsonObject AIContextProvider::getProjectContextJson() const {
  QJsonObject context;

  if (!canvas_) {
    context["error"] = "No canvas available";
    return context;
  }

  context["blocks"] = getBlocksInfo();
  context["connections"] = getConnectionsInfo();
  context["groups"] = getGroupsInfo();

  return context;
}

QString AIContextProvider::getBlockContext(PackageBlock* block) const {
  if (!block) return QString();

  QStringList context;

  context << QString("## Block: %1").arg(block->packageName());
  context << QString("ID: %1").arg(block->id().toString());
  context << QString("Position: (%1, %2)").arg(block->pos().x()).arg(block->pos().y());

  // Input pins
  if (!block->inputPins().isEmpty()) {
    context << "\nInput Pins (Subscribers):";
    for (int i = 0; i < block->inputPins().size(); ++i) {
      const Pin& pin = block->inputPins()[i];
      int connCount = block->connectionsForPin(i, false).size();
      context << QString("  [%1] %2 (%3) - %4 connection(s)")
                     .arg(i)
                     .arg(pin.name, pin.messageType)
                     .arg(connCount);
    }
  }

  // Output pins
  if (!block->outputPins().isEmpty()) {
    context << "\nOutput Pins (Publishers):";
    for (int i = 0; i < block->outputPins().size(); ++i) {
      const Pin& pin = block->outputPins()[i];
      int connCount = block->connectionsForPin(i, true).size();
      context << QString("  [%1] %2 (%3) - %4 connection(s)")
                     .arg(i)
                     .arg(pin.name, pin.messageType)
                     .arg(connCount);
    }
  }

  // Parameters
  if (block->hasParameters()) {
    context << "\nParameters:";
    for (const BlockParamData& param : block->parameters()) {
      context << QString("  %1 = %2 (%3)")
                     .arg(param.name, param.currentValue.toString(), param.type);
    }
  }

  // Runtime status
  QString statusStr;
  switch (block->runtimeStatus()) {
    case BlockRuntimeStatus::Running:
      statusStr = "Running (matched with ROS2 system)";
      break;
    case BlockRuntimeStatus::PartialMatch:
      statusStr = "Partial match in ROS2 system";
      break;
    case BlockRuntimeStatus::NotFound:
      statusStr = "Not found in ROS2 system";
      break;
    default:
      statusStr = "Unknown (system not scanned)";
      break;
  }
  context << QString("\nRuntime Status: %1").arg(statusStr);

  if (!block->matchedNodeName().isEmpty()) {
    context << QString("Matched Node: %1").arg(block->matchedNodeName());
  }

  // Connected blocks
  QSet<QString> connectedBlocks;
  for (ConnectionLine* conn : block->connections()) {
    if (conn->sourceBlock() && conn->sourceBlock() != block) {
      connectedBlocks.insert(conn->sourceBlock()->packageName());
    }
    if (conn->targetBlock() && conn->targetBlock() != block) {
      connectedBlocks.insert(conn->targetBlock()->packageName());
    }
  }

  if (!connectedBlocks.isEmpty()) {
    context << QString("\nConnected to: %1").arg(connectedBlocks.values().join(", "));
  }

  return context.join("\n");
}

QString AIContextProvider::getConnectionContext(ConnectionLine* connection) const {
  if (!connection) return QString();

  QStringList context;

  QString sourceName = connection->sourceBlock()
                           ? connection->sourceBlock()->packageName()
                           : "Unknown";
  QString targetName = connection->targetBlock()
                           ? connection->targetBlock()->packageName()
                           : "Unknown";

  context << QString("## Connection: %1 -> %2").arg(sourceName, targetName);
  context << QString("Topic: %1").arg(connection->topicName());
  context << QString("Message Type: %1").arg(connection->messageType());
  context << QString("Source Pin Index: %1").arg(connection->sourcePinIndex());
  context << QString("Target Pin Index: %1").arg(connection->targetPinIndex());

  // Activity state
  QString activityStr;
  switch (connection->activityState()) {
    case TopicActivityState::Active:
      activityStr = "Active (messages flowing)";
      break;
    case TopicActivityState::Inactive:
      activityStr = "Inactive (no recent messages)";
      break;
    default:
      activityStr = "Unknown";
      break;
  }
  context << QString("Activity: %1").arg(activityStr);

  // Source block info
  if (connection->sourceBlock()) {
    context << QString("\nSource Block: %1").arg(sourceName);
    if (connection->sourcePinIndex() >= 0 &&
        connection->sourcePinIndex() < connection->sourceBlock()->outputPins().size()) {
      const Pin& pin = connection->sourceBlock()->outputPins()[connection->sourcePinIndex()];
      context << QString("  Output Pin: %1").arg(pin.name);
    }
  }

  // Target block info
  if (connection->targetBlock()) {
    context << QString("\nTarget Block: %1").arg(targetName);
    if (connection->targetPinIndex() >= 0 &&
        connection->targetPinIndex() < connection->targetBlock()->inputPins().size()) {
      const Pin& pin = connection->targetBlock()->inputPins()[connection->targetPinIndex()];
      context << QString("  Input Pin: %1").arg(pin.name);
    }
  }

  return context.join("\n");
}

QString AIContextProvider::getGroupContext(NodeGroup* group) const {
  if (!group) return QString();

  QStringList context;

  context << QString("## Group: %1").arg(group->title());

  QList<PackageBlock*> blocks = group->containedNodes();
  context << QString("Contains %1 block(s):").arg(blocks.size());

  for (PackageBlock* block : blocks) {
    context << QString("  - %1").arg(block->packageName());

    // Show connections within the group
    for (ConnectionLine* conn : block->connections()) {
      PackageBlock* other = nullptr;
      QString direction;

      if (conn->sourceBlock() == block && conn->targetBlock()) {
        other = conn->targetBlock();
        direction = "->";
      } else if (conn->targetBlock() == block && conn->sourceBlock()) {
        other = conn->sourceBlock();
        direction = "<-";
      }

      if (other && blocks.contains(other)) {
        context << QString("    %1 %2 (internal)")
                       .arg(direction, other->packageName());
      }
    }
  }

  // External connections
  QStringList externalConnections;
  for (PackageBlock* block : blocks) {
    for (ConnectionLine* conn : block->connections()) {
      PackageBlock* other = nullptr;
      QString direction;

      if (conn->sourceBlock() == block && conn->targetBlock()) {
        other = conn->targetBlock();
        direction = QString("%1 -> %2").arg(block->packageName(), other->packageName());
      } else if (conn->targetBlock() == block && conn->sourceBlock()) {
        other = conn->sourceBlock();
        direction = QString("%1 <- %2").arg(block->packageName(), other->packageName());
      }

      if (other && !blocks.contains(other)) {
        externalConnections << QString("  %1 via %2")
                                   .arg(direction, conn->topicName());
      }
    }
  }

  if (!externalConnections.isEmpty()) {
    context << "\nExternal Connections:";
    context << externalConnections;
  }

  return context.join("\n");
}

QString AIContextProvider::getPinContext(PackageBlock* block, int pinIndex, bool isOutput) const {
  if (!block) return QString();

  QStringList context;

  const QList<Pin>& pins = isOutput ? block->outputPins() : block->inputPins();

  if (pinIndex < 0 || pinIndex >= pins.size()) {
    return QString("Invalid pin index %1").arg(pinIndex);
  }

  const Pin& pin = pins[pinIndex];

  QString pinType = isOutput ? "Output Pin (Publisher)" : "Input Pin (Subscriber)";
  context << QString("## %1 on %2").arg(pinType, block->packageName());
  context << QString("Pin Name: %1").arg(pin.name);
  context << QString("Message Type: %1").arg(pin.messageType);
  context << QString("Pin Index: %1").arg(pinIndex);

  // Connections
  QList<ConnectionLine*> connections = block->connectionsForPin(pinIndex, isOutput);
  if (!connections.isEmpty()) {
    context << QString("\nConnections (%1):").arg(connections.size());
    for (ConnectionLine* conn : connections) {
      if (isOutput && conn->targetBlock()) {
        context << QString("  -> %1 (pin %2)")
                       .arg(conn->targetBlock()->packageName())
                       .arg(conn->targetPinIndex());
      } else if (!isOutput && conn->sourceBlock()) {
        context << QString("  <- %1 (pin %2)")
                       .arg(conn->sourceBlock()->packageName())
                       .arg(conn->sourcePinIndex());
      }
    }
  } else {
    context << "\nNo connections";
  }

  return context.join("\n");
}

QString AIContextProvider::getTopicContext(const QString& topicName) const {
  QStringList context;

  context << QString("## Topic: %1").arg(topicName);

  if (!canvas_) {
    context << "No canvas available for detailed information.";
    return context.join("\n");
  }

  // Find connections using this topic
  QStringList publishers;
  QStringList subscribers;
  QString messageType;

  for (ConnectionLine* conn : canvas_->connections()) {
    if (conn->topicName() == topicName) {
      messageType = conn->messageType();

      if (conn->sourceBlock()) {
        publishers << conn->sourceBlock()->packageName();
      }
      if (conn->targetBlock()) {
        subscribers << conn->targetBlock()->packageName();
      }
    }
  }

  if (!messageType.isEmpty()) {
    context << QString("Message Type: %1").arg(messageType);
  }

  if (!publishers.isEmpty()) {
    context << QString("Publishers: %1").arg(publishers.join(", "));
  }

  if (!subscribers.isEmpty()) {
    context << QString("Subscribers: %1").arg(subscribers.join(", "));
  }

  // ROS2 system info if available
  if (systemDiscovery_) {
    const SystemGraph& graph = systemDiscovery_->systemGraph();
    for (const DiscoveredTopic& topic : graph.topics) {
      if (topic.name == topicName) {
        context << QString("\nLive System Info:");
        context << QString("  Type: %1").arg(topic.type);
        context << QString("  Publisher count: %1").arg(topic.publisherCount);
        context << QString("  Subscriber count: %1").arg(topic.subscriberCount);
        break;
      }
    }
  }

  return context.join("\n");
}

QString AIContextProvider::getFrameContext(const QString& frameName) const {
  QStringList context;

  context << QString("## TF Frame: %1").arg(frameName);

  // TF frame information would come from the TF tree
  // This is a placeholder for integration with TFTreePanel

  context << "TF frame information would be displayed here.";
  context << "This includes:";
  context << "  - Parent frame";
  context << "  - Child frames";
  context << "  - Transform broadcaster node";
  context << "  - Update rate";

  return context.join("\n");
}

QJsonArray AIContextProvider::getAvailablePackages() const {
  QJsonArray packages;

  // Common ROS2 packages
  packages.append("publisher_node");
  packages.append("subscriber_node");
  packages.append("relay_node");
  packages.append("service_server");
  packages.append("service_client");
  packages.append("action_server");
  packages.append("action_client");
  packages.append("turtlesim_node");
  packages.append("turtle_teleop_key");
  packages.append("mimic");
  packages.append("nav2_bringup");
  packages.append("slam_toolbox");
  packages.append("amcl");

  return packages;
}

QJsonArray AIContextProvider::getBlocksInfo() const {
  QJsonArray blocks;

  if (!canvas_) return blocks;

  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      QJsonObject blockObj;
      blockObj["id"] = block->id().toString();
      blockObj["name"] = block->packageName();
      blockObj["x"] = block->pos().x();
      blockObj["y"] = block->pos().y();

      QJsonArray inputPins, outputPins;
      for (const Pin& pin : block->inputPins()) {
        inputPins.append(QJsonObject{
            {"name", pin.name},
            {"type", pin.messageType}
        });
      }
      for (const Pin& pin : block->outputPins()) {
        outputPins.append(QJsonObject{
            {"name", pin.name},
            {"type", pin.messageType}
        });
      }
      blockObj["input_pins"] = inputPins;
      blockObj["output_pins"] = outputPins;

      blocks.append(blockObj);
    }
  }

  return blocks;
}

QJsonArray AIContextProvider::getConnectionsInfo() const {
  QJsonArray connections;

  if (!canvas_) return connections;

  for (ConnectionLine* conn : canvas_->connections()) {
    QJsonObject connObj;
    if (conn->sourceBlock()) {
      connObj["source"] = conn->sourceBlock()->packageName();
      connObj["source_pin"] = conn->sourcePinIndex();
    }
    if (conn->targetBlock()) {
      connObj["target"] = conn->targetBlock()->packageName();
      connObj["target_pin"] = conn->targetPinIndex();
    }
    connObj["topic"] = conn->topicName();
    connObj["type"] = conn->messageType();
    connections.append(connObj);
  }

  return connections;
}

QJsonArray AIContextProvider::getGroupsInfo() const {
  QJsonArray groups;

  if (!canvas_) return groups;

  for (NodeGroup* group : canvas_->nodeGroups()) {
    QJsonObject groupObj;
    groupObj["title"] = group->title();

    QJsonArray members;
    for (PackageBlock* block : group->containedNodes()) {
      members.append(block->packageName());
    }
    groupObj["blocks"] = members;

    groups.append(groupObj);
  }

  return groups;
}

QString AIContextProvider::getROS2SystemState() const {
  QStringList state;

  if (!systemDiscovery_) {
    return QString();
  }

  const SystemGraph& graph = systemDiscovery_->systemGraph();

  if (graph.nodes.isEmpty() && graph.topics.isEmpty()) {
    return QString();  // No system scan performed
  }

  state << "## Live ROS2 System State";

  // Nodes
  if (!graph.nodes.isEmpty()) {
    state << QString("\nRunning Nodes (%1):").arg(graph.nodes.size());
    for (const DiscoveredNode& node : graph.nodes) {
      state << QString("  - %1 (ns: %2)")
                   .arg(node.name, node.namespacePath.isEmpty() ? "/" : node.namespacePath);
    }
  }

  // Topics
  if (!graph.topics.isEmpty()) {
    state << QString("\nActive Topics (%1):").arg(graph.topics.size());
    for (const DiscoveredTopic& topic : graph.topics) {
      state << QString("  - %1 [%2] (pub: %3, sub: %4)")
                   .arg(topic.name, topic.type)
                   .arg(topic.publisherCount)
                   .arg(topic.subscriberCount);
    }
  }

  return state.join("\n");
}

QString AIContextProvider::buildSystemPrompt() const {
  QStringList prompt;

  prompt << "You are an AI assistant integrated into ROS Weaver, a visual editor for ROS2 robotics projects.";
  prompt << "";
  prompt << "You have the ability to:";
  prompt << "1. View and understand the current project state (blocks, connections, parameters)";
  prompt << "2. Load example projects (turtlesim_teleop, turtlebot3_navigation)";
  prompt << "3. Add and remove blocks on the canvas";
  prompt << "4. Create and remove connections between blocks";
  prompt << "5. Set parameters on blocks";
  prompt << "6. Create visual groups to organize blocks";
  prompt << "";
  prompt << "When you want to perform an action, use the tool call format:";
  prompt << "<tool_call>";
  prompt << "{\"tool\": \"tool_name\", \"parameters\": {\"param1\": \"value1\"}}";
  prompt << "</tool_call>";
  prompt << "";
  prompt << "Available tools:";
  prompt << "- load_example: Load a pre-built example project";
  prompt << "- add_block: Add a new block to the canvas";
  prompt << "- remove_block: Remove a block from the canvas";
  prompt << "- set_parameter: Set a parameter on a block";
  prompt << "- create_connection: Connect two blocks";
  prompt << "- remove_connection: Remove a connection";
  prompt << "- create_group: Create a visual group";
  prompt << "- get_project_state: Get current project info (no permission needed)";
  prompt << "- get_block_info: Get details about a specific block (no permission needed)";
  prompt << "- list_available_packages: List packages you can add (no permission needed)";
  prompt << "";
  prompt << "IMPORTANT: Actions that modify the project will require user permission.";
  prompt << "The user can choose to approve individual actions or approve all actions for the session.";
  prompt << "";

  // Add current project context
  QString projectContext = getProjectContext();
  if (!projectContext.isEmpty()) {
    prompt << projectContext;
  }

  return prompt.join("\n");
}

QString AIContextProvider::buildElementPrompt(const QString& elementContext,
                                               const QString& userQuestion) const {
  QStringList prompt;

  prompt << "The user is asking about a specific element in their ROS Weaver project.";
  prompt << "";
  prompt << "Element Information:";
  prompt << elementContext;
  prompt << "";

  if (!userQuestion.isEmpty()) {
    prompt << "User's question: " << userQuestion;
  } else {
    prompt << "Please provide helpful information about this element, including:";
    prompt << "- What it does in the context of ROS2";
    prompt << "- How it connects to other elements";
    prompt << "- Any suggestions for configuration or improvement";
  }

  return prompt.join("\n");
}

QString AIContextProvider::formatBlockInfo(PackageBlock* block) const {
  if (!block) return QString();

  QString info = QString("- %1 at (%2, %3)")
                     .arg(block->packageName())
                     .arg(static_cast<int>(block->pos().x()))
                     .arg(static_cast<int>(block->pos().y()));

  QStringList details;
  if (!block->inputPins().isEmpty()) {
    details << QString("%1 inputs").arg(block->inputPins().size());
  }
  if (!block->outputPins().isEmpty()) {
    details << QString("%1 outputs").arg(block->outputPins().size());
  }
  if (!block->parameters().isEmpty()) {
    details << QString("%1 params").arg(block->parameters().size());
  }

  if (!details.isEmpty()) {
    info += QString(" [%1]").arg(details.join(", "));
  }

  return info;
}

QString AIContextProvider::formatConnectionInfo(ConnectionLine* conn) const {
  if (!conn) return QString();

  QString source = conn->sourceBlock() ? conn->sourceBlock()->packageName() : "?";
  QString target = conn->targetBlock() ? conn->targetBlock()->packageName() : "?";

  return QString("- %1 -> %2 via %3 (%4)")
             .arg(source, target, conn->topicName(), conn->messageType());
}

QString AIContextProvider::formatPinInfo(const QString& pinName, const QString& type,
                                          const QString& messageType, bool isInput) const {
  QString direction = isInput ? "input" : "output";
  return QString("%1 (%2, %3): %4").arg(pinName, direction, type, messageType);
}

}  // namespace ros_weaver
