#include "ros_weaver/core/ai_tool_manager.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/core/project.hpp"
#include <QJsonDocument>
#include <QRegularExpression>
#include <QDateTime>
#include <QDebug>

namespace ros_weaver {

AIToolManager& AIToolManager::instance() {
  static AIToolManager instance;
  return instance;
}

AIToolManager::AIToolManager() : QObject(nullptr) {}

AIToolManager::~AIToolManager() = default;

void AIToolManager::registerTools(WeaverCanvas* canvas) {
  canvas_ = canvas;

  // Register all available tools
  registerLoadExampleTool();
  registerAddBlockTool();
  registerRemoveBlockTool();
  registerSetParameterTool();
  registerCreateConnectionTool();
  registerRemoveConnectionTool();
  registerCreateGroupTool();
  registerGetProjectStateTool();
  registerGetBlockInfoTool();
  registerListAvailablePackagesTool();
}

void AIToolManager::registerLoadExampleTool() {
  AITool tool;
  tool.name = "load_example";
  tool.description = "Load a pre-built example project. Available examples: "
                     "'turtlesim_teleop' (basic turtle control), "
                     "'turtlebot3_navigation' (TurtleBot3 SLAM/navigation stack).";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"example_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the example to load"},
              {"enum", QJsonArray{"turtlesim_teleop", "turtlebot3_navigation"}}
          }}
      }},
      {"required", QJsonArray{"example_name"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;
    QString exampleName = params["example_name"].toString();

    if (exampleName.isEmpty()) {
      result.success = false;
      result.message = "Example name is required";
      return result;
    }

    // Store undo data (current project state)
    AIAction action;
    action.toolName = "load_example";
    action.parameters = params;
    action.description = QString("Load example: %1").arg(exampleName);
    action.timestamp = QDateTime::currentDateTime();

    // Store current state for undo
    Project currentProject;
    if (canvas_) {
      canvas_->exportToProject(currentProject);
      action.undoData["previousProject"] = currentProject.toJson();
    }

    // Emit signal to trigger example loading (MainWindow handles this)
    result.success = true;
    result.message = QString("Loading example '%1'. The MainWindow will handle the actual loading.").arg(exampleName);
    result.data["example_name"] = exampleName;
    result.data["action"] = "load_example";
    result.undoDescription = QString("Loaded example: %1").arg(exampleName);

    action.approved = true;
    undoStack_.push(action);
    emit undoStackChanged(undoStack_.size());

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerAddBlockTool() {
  AITool tool;
  tool.name = "add_block";
  tool.description = "Add a new package block to the canvas at the specified position.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"package_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the package/node to add"}
          }},
          {"x", QJsonObject{
              {"type", "number"},
              {"description", "X position on canvas (default: 100)"}
          }},
          {"y", QJsonObject{
              {"type", "number"},
              {"description", "Y position on canvas (default: 100)"}
          }}
      }},
      {"required", QJsonArray{"package_name"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString packageName = params["package_name"].toString();
    double x = params.contains("x") ? params["x"].toDouble() : 100.0;
    double y = params.contains("y") ? params["y"].toDouble() : 100.0;

    if (packageName.isEmpty()) {
      result.success = false;
      result.message = "Package name is required";
      return result;
    }

    PackageBlock* block = canvas_->addPackageBlock(packageName, QPointF(x, y));

    if (block) {
      AIAction action;
      action.toolName = "add_block";
      action.parameters = params;
      action.description = QString("Add block: %1").arg(packageName);
      action.timestamp = QDateTime::currentDateTime();
      action.undoData["block_id"] = block->id().toString();
      action.approved = true;
      undoStack_.push(action);
      emit undoStackChanged(undoStack_.size());

      result.success = true;
      result.message = QString("Added block '%1' at position (%2, %3)")
                           .arg(packageName)
                           .arg(x)
                           .arg(y);
      result.data["block_id"] = block->id().toString();
      result.undoDescription = QString("Added block: %1").arg(packageName);
    } else {
      result.success = false;
      result.message = "Failed to add block";
    }

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerRemoveBlockTool() {
  AITool tool;
  tool.name = "remove_block";
  tool.description = "Remove a package block from the canvas by its ID or name.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"block_id", QJsonObject{
              {"type", "string"},
              {"description", "UUID of the block to remove"}
          }},
          {"block_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the block to remove (if ID not provided)"}
          }}
      }}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString blockId = params["block_id"].toString();
    QString blockName = params["block_name"].toString();

    // Find the block
    PackageBlock* targetBlock = nullptr;
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        if (!blockId.isEmpty() && block->id().toString() == blockId) {
          targetBlock = block;
          break;
        }
        if (!blockName.isEmpty() && block->packageName() == blockName) {
          targetBlock = block;
          break;
        }
      }
    }

    if (!targetBlock) {
      result.success = false;
      result.message = "Block not found";
      return result;
    }

    // Store undo data
    AIAction action;
    action.toolName = "remove_block";
    action.parameters = params;
    action.description = QString("Remove block: %1").arg(targetBlock->packageName());
    action.timestamp = QDateTime::currentDateTime();

    // Store block data for undo
    BlockData blockData;
    blockData.id = targetBlock->id();
    blockData.name = targetBlock->packageName();
    blockData.position = targetBlock->pos();
    for (const Pin& pin : targetBlock->inputPins()) {
      blockData.inputPins.append({pin.name, pin.messageType});
    }
    for (const Pin& pin : targetBlock->outputPins()) {
      blockData.outputPins.append({pin.name, pin.messageType});
    }
    blockData.parameters = targetBlock->parameters();

    QJsonObject blockJson;
    blockJson["id"] = blockData.id.toString();
    blockJson["name"] = blockData.name;
    blockJson["x"] = blockData.position.x();
    blockJson["y"] = blockData.position.y();
    action.undoData["block"] = blockJson;

    canvas_->removePackageBlock(targetBlock);

    action.approved = true;
    undoStack_.push(action);
    emit undoStackChanged(undoStack_.size());

    result.success = true;
    result.message = QString("Removed block '%1'").arg(blockData.name);
    result.undoDescription = QString("Removed block: %1").arg(blockData.name);

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerSetParameterTool() {
  AITool tool;
  tool.name = "set_parameter";
  tool.description = "Set a parameter value on a package block.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"block_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the block to modify"}
          }},
          {"parameter_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the parameter to set"}
          }},
          {"parameter_value", QJsonObject{
              {"type", "string"},
              {"description", "New value for the parameter"}
          }}
      }},
      {"required", QJsonArray{"block_name", "parameter_name", "parameter_value"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString blockName = params["block_name"].toString();
    QString paramName = params["parameter_name"].toString();
    QString paramValue = params["parameter_value"].toString();

    // Find the block
    PackageBlock* targetBlock = nullptr;
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        if (block->packageName() == blockName) {
          targetBlock = block;
          break;
        }
      }
    }

    if (!targetBlock) {
      result.success = false;
      result.message = QString("Block '%1' not found").arg(blockName);
      return result;
    }

    // Get current parameters
    QList<BlockParamData> parameters = targetBlock->parameters();

    // Find and update the parameter
    bool found = false;
    QString oldValue;
    for (int i = 0; i < parameters.size(); ++i) {
      if (parameters[i].name == paramName) {
        oldValue = parameters[i].currentValue.toString();
        parameters[i].currentValue = paramValue;
        found = true;
        break;
      }
    }

    if (!found) {
      // Add new parameter if not found
      BlockParamData newParam;
      newParam.name = paramName;
      newParam.currentValue = paramValue;
      newParam.type = "string";
      parameters.append(newParam);
      oldValue = "";
    }

    targetBlock->setParameters(parameters);

    // Store undo data
    AIAction action;
    action.toolName = "set_parameter";
    action.parameters = params;
    action.description = QString("Set %1.%2 = %3").arg(blockName, paramName, paramValue);
    action.timestamp = QDateTime::currentDateTime();
    action.undoData["block_name"] = blockName;
    action.undoData["parameter_name"] = paramName;
    action.undoData["old_value"] = oldValue;
    action.undoData["was_new"] = !found;
    action.approved = true;
    undoStack_.push(action);
    emit undoStackChanged(undoStack_.size());

    result.success = true;
    result.message = QString("Set parameter '%1' on block '%2' to '%3'")
                         .arg(paramName, blockName, paramValue);
    result.undoDescription = action.description;

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerCreateConnectionTool() {
  AITool tool;
  tool.name = "create_connection";
  tool.description = "Create a connection between two blocks by connecting their pins.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"source_block", QJsonObject{
              {"type", "string"},
              {"description", "Name of the source block"}
          }},
          {"source_pin", QJsonObject{
              {"type", "integer"},
              {"description", "Index of the output pin on source block (0-based)"}
          }},
          {"target_block", QJsonObject{
              {"type", "string"},
              {"description", "Name of the target block"}
          }},
          {"target_pin", QJsonObject{
              {"type", "integer"},
              {"description", "Index of the input pin on target block (0-based)"}
          }}
      }},
      {"required", QJsonArray{"source_block", "source_pin", "target_block", "target_pin"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString sourceBlockName = params["source_block"].toString();
    int sourcePin = params["source_pin"].toInt();
    QString targetBlockName = params["target_block"].toString();
    int targetPin = params["target_pin"].toInt();

    // Find blocks
    PackageBlock* sourceBlock = nullptr;
    PackageBlock* targetBlock = nullptr;

    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        if (block->packageName() == sourceBlockName) sourceBlock = block;
        if (block->packageName() == targetBlockName) targetBlock = block;
      }
    }

    if (!sourceBlock) {
      result.success = false;
      result.message = QString("Source block '%1' not found").arg(sourceBlockName);
      return result;
    }

    if (!targetBlock) {
      result.success = false;
      result.message = QString("Target block '%1' not found").arg(targetBlockName);
      return result;
    }

    if (sourcePin < 0 || sourcePin >= sourceBlock->outputPins().size()) {
      result.success = false;
      result.message = QString("Invalid source pin index %1").arg(sourcePin);
      return result;
    }

    if (targetPin < 0 || targetPin >= targetBlock->inputPins().size()) {
      result.success = false;
      result.message = QString("Invalid target pin index %1").arg(targetPin);
      return result;
    }

    ConnectionLine* conn = canvas_->createConnection(sourceBlock, sourcePin,
                                                      targetBlock, targetPin);

    if (conn) {
      AIAction action;
      action.toolName = "create_connection";
      action.parameters = params;
      action.description = QString("Connect %1[%2] -> %3[%4]")
                               .arg(sourceBlockName)
                               .arg(sourcePin)
                               .arg(targetBlockName)
                               .arg(targetPin);
      action.timestamp = QDateTime::currentDateTime();
      action.undoData["source_block"] = sourceBlockName;
      action.undoData["target_block"] = targetBlockName;
      action.approved = true;
      undoStack_.push(action);
      emit undoStackChanged(undoStack_.size());

      result.success = true;
      result.message = QString("Created connection from %1 to %2")
                           .arg(sourceBlockName, targetBlockName);
      result.undoDescription = action.description;
    } else {
      result.success = false;
      result.message = "Failed to create connection";
    }

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerRemoveConnectionTool() {
  AITool tool;
  tool.name = "remove_connection";
  tool.description = "Remove a connection between two blocks.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"source_block", QJsonObject{
              {"type", "string"},
              {"description", "Name of the source block"}
          }},
          {"target_block", QJsonObject{
              {"type", "string"},
              {"description", "Name of the target block"}
          }},
          {"topic_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the topic (optional, to identify specific connection)"}
          }}
      }},
      {"required", QJsonArray{"source_block", "target_block"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString sourceBlockName = params["source_block"].toString();
    QString targetBlockName = params["target_block"].toString();

    // Find the connection
    ConnectionLine* targetConn = nullptr;
    for (ConnectionLine* conn : canvas_->connections()) {
      if (conn->sourceBlock() && conn->targetBlock() &&
          conn->sourceBlock()->packageName() == sourceBlockName &&
          conn->targetBlock()->packageName() == targetBlockName) {
        targetConn = conn;
        break;
      }
    }

    if (!targetConn) {
      result.success = false;
      result.message = QString("Connection from '%1' to '%2' not found")
                           .arg(sourceBlockName, targetBlockName);
      return result;
    }

    // Store undo data
    AIAction action;
    action.toolName = "remove_connection";
    action.parameters = params;
    action.description = QString("Remove connection: %1 -> %2")
                             .arg(sourceBlockName, targetBlockName);
    action.timestamp = QDateTime::currentDateTime();
    action.undoData["source_block"] = sourceBlockName;
    action.undoData["source_pin"] = targetConn->sourcePinIndex();
    action.undoData["target_block"] = targetBlockName;
    action.undoData["target_pin"] = targetConn->targetPinIndex();

    canvas_->removeConnection(targetConn);

    action.approved = true;
    undoStack_.push(action);
    emit undoStackChanged(undoStack_.size());

    result.success = true;
    result.message = QString("Removed connection from '%1' to '%2'")
                         .arg(sourceBlockName, targetBlockName);
    result.undoDescription = action.description;

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerCreateGroupTool() {
  AITool tool;
  tool.name = "create_group";
  tool.description = "Create a visual group containing specified blocks.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"title", QJsonObject{
              {"type", "string"},
              {"description", "Title for the group"}
          }},
          {"block_names", QJsonObject{
              {"type", "array"},
              {"items", QJsonObject{{"type", "string"}}},
              {"description", "List of block names to include in the group"}
          }},
          {"color", QJsonObject{
              {"type", "string"},
              {"description", "Color for the group (blue, green, red, orange, purple)"},
              {"enum", QJsonArray{"blue", "green", "red", "orange", "purple"}}
          }}
      }},
      {"required", QJsonArray{"title"}}
  };
  tool.requiresPermission = true;

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString title = params["title"].toString();
    QJsonArray blockNames = params["block_names"].toArray();
    QString colorName = params.contains("color") ? params["color"].toString() : "blue";

    // Select the specified blocks
    canvas_->scene()->clearSelection();

    if (!blockNames.isEmpty()) {
      for (const QJsonValue& nameVal : blockNames) {
        QString blockName = nameVal.toString();
        for (QGraphicsItem* item : canvas_->scene()->items()) {
          if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
            if (block->packageName() == blockName) {
              block->setSelected(true);
              break;
            }
          }
        }
      }
    }

    NodeGroup* group = canvas_->createGroupFromSelection(title);

    if (group) {
      // Set color
      QColor color(80, 120, 180, 180);  // Default blue
      if (colorName == "green") color = QColor(80, 160, 100, 180);
      else if (colorName == "red") color = QColor(180, 80, 80, 180);
      else if (colorName == "orange") color = QColor(200, 140, 60, 180);
      else if (colorName == "purple") color = QColor(140, 80, 180, 180);
      group->setColor(color);

      AIAction action;
      action.toolName = "create_group";
      action.parameters = params;
      action.description = QString("Create group: %1").arg(title);
      action.timestamp = QDateTime::currentDateTime();
      action.approved = true;
      undoStack_.push(action);
      emit undoStackChanged(undoStack_.size());

      result.success = true;
      result.message = QString("Created group '%1'").arg(title);
      result.undoDescription = action.description;
    } else {
      result.success = false;
      result.message = "Failed to create group (no blocks selected?)";
    }

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerGetProjectStateTool() {
  AITool tool;
  tool.name = "get_project_state";
  tool.description = "Get the current state of the project including all blocks, "
                     "connections, and groups on the canvas.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{}}
  };
  tool.requiresPermission = false;  // Read-only, no permission needed

  tool.execute = [this](const QJsonObject& /*params*/) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QJsonObject projectState;
    QJsonArray blocksArray;
    QJsonArray connectionsArray;
    QJsonArray groupsArray;

    // Collect blocks
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        QJsonObject blockObj;
        blockObj["id"] = block->id().toString();
        blockObj["name"] = block->packageName();
        blockObj["x"] = block->pos().x();
        blockObj["y"] = block->pos().y();

        QJsonArray inputPins, outputPins;
        for (const Pin& pin : block->inputPins()) {
          QJsonObject pinObj;
          pinObj["name"] = pin.name;
          pinObj["type"] = pin.messageType;
          inputPins.append(pinObj);
        }
        for (const Pin& pin : block->outputPins()) {
          QJsonObject pinObj;
          pinObj["name"] = pin.name;
          pinObj["type"] = pin.messageType;
          outputPins.append(pinObj);
        }
        blockObj["input_pins"] = inputPins;
        blockObj["output_pins"] = outputPins;

        // Parameters
        QJsonArray paramsArray;
        for (const BlockParamData& param : block->parameters()) {
          QJsonObject paramObj;
          paramObj["name"] = param.name;
          paramObj["value"] = param.currentValue.toString();
          paramObj["type"] = param.type;
          paramsArray.append(paramObj);
        }
        blockObj["parameters"] = paramsArray;

        blocksArray.append(blockObj);
      }
    }

    // Collect connections
    for (ConnectionLine* conn : canvas_->connections()) {
      QJsonObject connObj;
      if (conn->sourceBlock()) {
        connObj["source_block"] = conn->sourceBlock()->packageName();
        connObj["source_pin"] = conn->sourcePinIndex();
      }
      if (conn->targetBlock()) {
        connObj["target_block"] = conn->targetBlock()->packageName();
        connObj["target_pin"] = conn->targetPinIndex();
      }
      connObj["topic_name"] = conn->topicName();
      connObj["message_type"] = conn->messageType();
      connectionsArray.append(connObj);
    }

    // Collect groups
    for (NodeGroup* group : canvas_->nodeGroups()) {
      QJsonObject groupObj;
      groupObj["title"] = group->title();
      QJsonArray memberBlocks;
      for (PackageBlock* block : group->containedNodes()) {
        memberBlocks.append(block->packageName());
      }
      groupObj["blocks"] = memberBlocks;
      groupsArray.append(groupObj);
    }

    projectState["blocks"] = blocksArray;
    projectState["connections"] = connectionsArray;
    projectState["groups"] = groupsArray;
    projectState["block_count"] = blocksArray.size();
    projectState["connection_count"] = connectionsArray.size();
    projectState["group_count"] = groupsArray.size();

    result.success = true;
    result.message = QString("Project has %1 blocks, %2 connections, %3 groups")
                         .arg(blocksArray.size())
                         .arg(connectionsArray.size())
                         .arg(groupsArray.size());
    result.data = projectState;

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerGetBlockInfoTool() {
  AITool tool;
  tool.name = "get_block_info";
  tool.description = "Get detailed information about a specific block.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{
          {"block_name", QJsonObject{
              {"type", "string"},
              {"description", "Name of the block to get info about"}
          }}
      }},
      {"required", QJsonArray{"block_name"}}
  };
  tool.requiresPermission = false;  // Read-only

  tool.execute = [this](const QJsonObject& params) -> AIToolResult {
    AIToolResult result;

    if (!canvas_) {
      result.success = false;
      result.message = "Canvas not available";
      return result;
    }

    QString blockName = params["block_name"].toString();

    // Find the block
    PackageBlock* targetBlock = nullptr;
    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
        if (block->packageName() == blockName) {
          targetBlock = block;
          break;
        }
      }
    }

    if (!targetBlock) {
      result.success = false;
      result.message = QString("Block '%1' not found").arg(blockName);
      return result;
    }

    QJsonObject blockInfo;
    blockInfo["id"] = targetBlock->id().toString();
    blockInfo["name"] = targetBlock->packageName();
    blockInfo["x"] = targetBlock->pos().x();
    blockInfo["y"] = targetBlock->pos().y();
    blockInfo["expanded"] = targetBlock->isExpanded();

    // Pins
    QJsonArray inputPins, outputPins;
    for (int i = 0; i < targetBlock->inputPins().size(); ++i) {
      const Pin& pin = targetBlock->inputPins()[i];
      QJsonObject pinObj;
      pinObj["index"] = i;
      pinObj["name"] = pin.name;
      pinObj["type"] = pin.messageType;
      pinObj["connections"] = targetBlock->connectionsForPin(i, false).size();
      inputPins.append(pinObj);
    }
    for (int i = 0; i < targetBlock->outputPins().size(); ++i) {
      const Pin& pin = targetBlock->outputPins()[i];
      QJsonObject pinObj;
      pinObj["index"] = i;
      pinObj["name"] = pin.name;
      pinObj["type"] = pin.messageType;
      pinObj["connections"] = targetBlock->connectionsForPin(i, true).size();
      outputPins.append(pinObj);
    }
    blockInfo["input_pins"] = inputPins;
    blockInfo["output_pins"] = outputPins;

    // Parameters
    QJsonArray paramsArray;
    for (const BlockParamData& param : targetBlock->parameters()) {
      QJsonObject paramObj;
      paramObj["name"] = param.name;
      paramObj["value"] = param.currentValue.toString();
      paramObj["type"] = param.type;
      paramsArray.append(paramObj);
    }
    blockInfo["parameters"] = paramsArray;

    // Runtime status
    QString statusStr;
    switch (targetBlock->runtimeStatus()) {
      case BlockRuntimeStatus::Running: statusStr = "running"; break;
      case BlockRuntimeStatus::PartialMatch: statusStr = "partial_match"; break;
      case BlockRuntimeStatus::NotFound: statusStr = "not_found"; break;
      default: statusStr = "unknown"; break;
    }
    blockInfo["runtime_status"] = statusStr;
    blockInfo["matched_node"] = targetBlock->matchedNodeName();

    result.success = true;
    result.message = QString("Block '%1' info retrieved").arg(blockName);
    result.data = blockInfo;

    return result;
  };

  tools_[tool.name] = tool;
}

void AIToolManager::registerListAvailablePackagesTool() {
  AITool tool;
  tool.name = "list_available_packages";
  tool.description = "List commonly available ROS2 packages that can be added to the canvas.";
  tool.parametersSchema = QJsonObject{
      {"type", "object"},
      {"properties", QJsonObject{}}
  };
  tool.requiresPermission = false;  // Read-only

  tool.execute = [](const QJsonObject& /*params*/) -> AIToolResult {
    AIToolResult result;

    // List of common ROS2 packages
    QJsonArray packages;

    // Basic nodes
    packages.append(QJsonObject{{"name", "publisher_node"}, {"category", "basic"}, {"description", "Simple topic publisher"}});
    packages.append(QJsonObject{{"name", "subscriber_node"}, {"category", "basic"}, {"description", "Simple topic subscriber"}});
    packages.append(QJsonObject{{"name", "relay_node"}, {"category", "basic"}, {"description", "Relays messages between topics"}});

    // Services
    packages.append(QJsonObject{{"name", "service_server"}, {"category", "services"}, {"description", "ROS2 service server"}});
    packages.append(QJsonObject{{"name", "service_client"}, {"category", "services"}, {"description", "ROS2 service client"}});

    // Actions
    packages.append(QJsonObject{{"name", "action_server"}, {"category", "actions"}, {"description", "ROS2 action server"}});
    packages.append(QJsonObject{{"name", "action_client"}, {"category", "actions"}, {"description", "ROS2 action client"}});

    // Turtlesim
    packages.append(QJsonObject{{"name", "turtlesim_node"}, {"category", "turtlesim"}, {"description", "Turtlesim simulation node"}});
    packages.append(QJsonObject{{"name", "turtle_teleop_key"}, {"category", "turtlesim"}, {"description", "Keyboard teleop for turtle"}});
    packages.append(QJsonObject{{"name", "mimic"}, {"category", "turtlesim"}, {"description", "Mimic turtle movements"}});

    // Navigation
    packages.append(QJsonObject{{"name", "nav2_bringup"}, {"category", "navigation"}, {"description", "Nav2 navigation stack"}});
    packages.append(QJsonObject{{"name", "slam_toolbox"}, {"category", "navigation"}, {"description", "SLAM mapping"}});
    packages.append(QJsonObject{{"name", "amcl"}, {"category", "navigation"}, {"description", "Adaptive Monte Carlo Localization"}});

    // Perception
    packages.append(QJsonObject{{"name", "image_transport"}, {"category", "perception"}, {"description", "Image transport"}});
    packages.append(QJsonObject{{"name", "depth_image_proc"}, {"category", "perception"}, {"description", "Depth image processing"}});
    packages.append(QJsonObject{{"name", "pointcloud_to_laserscan"}, {"category", "perception"}, {"description", "Convert pointcloud to laser scan"}});

    result.success = true;
    result.message = QString("Found %1 available packages").arg(packages.size());
    result.data["packages"] = packages;

    return result;
  };

  tools_[tool.name] = tool;
}

QString AIToolManager::getToolsDescription() const {
  QStringList descriptions;

  descriptions << "## Available Tools\n";
  descriptions << "You can use the following tools by outputting a tool call in this format:\n";
  descriptions << "```\n<tool_call>\n{\"tool\": \"tool_name\", \"parameters\": {\"param1\": \"value1\"}}\n</tool_call>\n```\n";

  for (const AITool& tool : tools_) {
    descriptions << QString("### %1\n%2\n").arg(tool.name, tool.description);

    QJsonDocument schemaDoc(tool.parametersSchema);
    descriptions << "Parameters:\n```json\n" + schemaDoc.toJson(QJsonDocument::Indented) + "```\n";
  }

  return descriptions.join("\n");
}

QJsonArray AIToolManager::getToolsSchema() const {
  QJsonArray toolsArray;

  for (const AITool& tool : tools_) {
    QJsonObject toolObj;
    toolObj["name"] = tool.name;
    toolObj["description"] = tool.description;
    toolObj["parameters"] = tool.parametersSchema;
    toolObj["requires_permission"] = tool.requiresPermission;
    toolsArray.append(toolObj);
  }

  return toolsArray;
}

QList<QPair<QString, QJsonObject>> AIToolManager::parseToolCalls(const QString& response) const {
  QList<QPair<QString, QJsonObject>> toolCalls;

  // Look for tool calls in format: <tool_call>{"tool": "name", "parameters": {...}}</tool_call>
  QRegularExpression toolCallRe("<tool_call>\\s*(.+?)\\s*</tool_call>",
                                  QRegularExpression::DotMatchesEverythingOption);

  QRegularExpressionMatchIterator it = toolCallRe.globalMatch(response);
  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    QString jsonStr = match.captured(1).trimmed();

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(jsonStr.toUtf8(), &error);

    if (error.error == QJsonParseError::NoError && doc.isObject()) {
      QJsonObject obj = doc.object();
      QString toolName = obj["tool"].toString();
      QJsonObject params = obj["parameters"].toObject();

      if (!toolName.isEmpty() && tools_.contains(toolName)) {
        toolCalls.append({toolName, params});
      }
    }
  }

  return toolCalls;
}

AIToolResult AIToolManager::executeTool(const QString& toolName, const QJsonObject& params) {
  AIToolResult result;

  if (!tools_.contains(toolName)) {
    result.success = false;
    result.message = QString("Unknown tool: %1").arg(toolName);
    return result;
  }

  const AITool& tool = tools_[toolName];

  // Check if permission is required and not auto-approved
  if (tool.requiresPermission && !autoApproveSession_) {
    // Store pending execution and emit signal for permission dialog
    pendingToolName_ = toolName;
    pendingParams_ = params;

    emit permissionRequired(toolName, tool.description, params);

    // Return pending result - actual execution happens after permission granted
    result.success = false;
    result.message = "Waiting for user permission";
    result.data["pending"] = true;
    return result;
  }

  // Execute the tool
  result = tool.execute(params);
  emit toolExecuted(toolName, result.success, result.message);

  return result;
}

QStringList AIToolManager::availableTools() const {
  return tools_.keys();
}

const AITool* AIToolManager::getTool(const QString& name) const {
  auto it = tools_.find(name);
  if (it != tools_.end()) {
    return &it.value();
  }
  return nullptr;
}

QString AIToolManager::lastActionDescription() const {
  if (undoStack_.isEmpty()) {
    return QString();
  }
  return undoStack_.top().description;
}

void AIToolManager::undoLastAction() {
  if (undoStack_.isEmpty()) {
    return;
  }

  AIAction action = undoStack_.pop();

  // Handle undo based on tool type
  if (action.toolName == "add_block") {
    // Remove the added block
    QString blockId = action.undoData["block_id"].toString();
    if (canvas_) {
      for (QGraphicsItem* item : canvas_->scene()->items()) {
        if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
          if (block->id().toString() == blockId) {
            canvas_->removePackageBlock(block);
            break;
          }
        }
      }
    }
  } else if (action.toolName == "remove_block") {
    // Re-add the removed block
    QJsonObject blockJson = action.undoData["block"].toObject();
    if (canvas_) {
      QString name = blockJson["name"].toString();
      double x = blockJson["x"].toDouble();
      double y = blockJson["y"].toDouble();
      canvas_->addPackageBlock(name, QPointF(x, y));
    }
  } else if (action.toolName == "set_parameter") {
    // Restore previous parameter value
    QString blockName = action.undoData["block_name"].toString();
    QString paramName = action.undoData["parameter_name"].toString();
    QString oldValue = action.undoData["old_value"].toString();
    bool wasNew = action.undoData["was_new"].toBool();

    if (canvas_) {
      for (QGraphicsItem* item : canvas_->scene()->items()) {
        if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
          if (block->packageName() == blockName) {
            QList<BlockParamData> params = block->parameters();
            if (wasNew) {
              // Remove the newly added parameter
              for (int i = 0; i < params.size(); ++i) {
                if (params[i].name == paramName) {
                  params.removeAt(i);
                  break;
                }
              }
            } else {
              // Restore old value
              for (int i = 0; i < params.size(); ++i) {
                if (params[i].name == paramName) {
                  params[i].currentValue = oldValue;
                  break;
                }
              }
            }
            block->setParameters(params);
            break;
          }
        }
      }
    }
  } else if (action.toolName == "create_connection") {
    // Remove the created connection
    QString sourceBlock = action.parameters["source_block"].toString();
    QString targetBlock = action.parameters["target_block"].toString();

    if (canvas_) {
      for (ConnectionLine* conn : canvas_->connections()) {
        if (conn->sourceBlock() && conn->targetBlock() &&
            conn->sourceBlock()->packageName() == sourceBlock &&
            conn->targetBlock()->packageName() == targetBlock) {
          canvas_->removeConnection(conn);
          break;
        }
      }
    }
  } else if (action.toolName == "remove_connection") {
    // Recreate the removed connection
    QString sourceBlockName = action.undoData["source_block"].toString();
    int sourcePin = action.undoData["source_pin"].toInt();
    QString targetBlockName = action.undoData["target_block"].toString();
    int targetPin = action.undoData["target_pin"].toInt();

    if (canvas_) {
      PackageBlock* sourceBlock = nullptr;
      PackageBlock* targetBlock = nullptr;

      for (QGraphicsItem* item : canvas_->scene()->items()) {
        if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
          if (block->packageName() == sourceBlockName) sourceBlock = block;
          if (block->packageName() == targetBlockName) targetBlock = block;
        }
      }

      if (sourceBlock && targetBlock) {
        canvas_->createConnection(sourceBlock, sourcePin, targetBlock, targetPin);
      }
    }
  }
  // Note: load_example and create_group undo not fully implemented for brevity

  emit actionUndone(action.description);
  emit undoStackChanged(undoStack_.size());
}

void AIToolManager::clearUndoStack() {
  undoStack_.clear();
  emit undoStackChanged(0);
}

void AIToolManager::onPermissionGranted(const QString& toolName, const QJsonObject& params,
                                         bool approveAll) {
  if (approveAll) {
    autoApproveSession_ = true;
    emit autoApproveChanged(true);
  }

  // Execute the pending tool
  if (tools_.contains(toolName)) {
    AIToolResult result = tools_[toolName].execute(params);
    emit toolExecuted(toolName, result.success, result.message);
  }

  pendingToolName_.clear();
  pendingParams_ = QJsonObject();
}

void AIToolManager::onPermissionDenied(const QString& toolName) {
  emit toolExecuted(toolName, false, "Permission denied by user");
  pendingToolName_.clear();
  pendingParams_ = QJsonObject();
}

}  // namespace ros_weaver
