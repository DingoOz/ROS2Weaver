#include "ros_weaver/core/project.hpp"
#include <QDateTime>
#include <QFileInfo>

namespace ros_weaver {

// PinData serialization
QJsonObject PinData::toJson() const {
  QJsonObject json;
  json["name"] = name;
  json["type"] = type;
  json["dataType"] = dataType;
  json["messageType"] = messageType;
  return json;
}

PinData PinData::fromJson(const QJsonObject& json) {
  PinData pin;
  pin.name = json["name"].toString();
  pin.type = json["type"].toString();
  pin.dataType = json["dataType"].toString();
  pin.messageType = json["messageType"].toString();
  return pin;
}

// BlockParamData serialization
QJsonObject BlockParamData::toJson() const {
  QJsonObject json;
  json["name"] = name;
  json["type"] = type;
  json["description"] = description;
  json["group"] = group;

  // Handle QVariant serialization based on type
  if (type == "string") {
    json["defaultValue"] = defaultValue.toString();
    json["currentValue"] = currentValue.toString();
  } else if (type == "int") {
    json["defaultValue"] = defaultValue.toInt();
    json["currentValue"] = currentValue.toInt();
    if (minValue.isValid()) json["minValue"] = minValue.toInt();
    if (maxValue.isValid()) json["maxValue"] = maxValue.toInt();
  } else if (type == "double") {
    json["defaultValue"] = defaultValue.toDouble();
    json["currentValue"] = currentValue.toDouble();
    if (minValue.isValid()) json["minValue"] = minValue.toDouble();
    if (maxValue.isValid()) json["maxValue"] = maxValue.toDouble();
  } else if (type == "bool") {
    json["defaultValue"] = defaultValue.toBool();
    json["currentValue"] = currentValue.toBool();
  } else if (type == "array") {
    QJsonArray defaultArr, currentArr;
    for (const QString& s : defaultValue.toStringList()) defaultArr.append(s);
    for (const QString& s : currentValue.toStringList()) currentArr.append(s);
    json["defaultValue"] = defaultArr;
    json["currentValue"] = currentArr;
  }

  if (!enumValues.isEmpty()) {
    QJsonArray enumArr;
    for (const QString& s : enumValues) enumArr.append(s);
    json["enumValues"] = enumArr;
  }

  return json;
}

BlockParamData BlockParamData::fromJson(const QJsonObject& json) {
  BlockParamData param;
  param.name = json["name"].toString();
  param.type = json["type"].toString();
  param.description = json["description"].toString();
  param.group = json["group"].toString();

  // Handle QVariant deserialization based on type
  if (param.type == "string") {
    param.defaultValue = json["defaultValue"].toString();
    param.currentValue = json["currentValue"].toString();
  } else if (param.type == "int") {
    param.defaultValue = json["defaultValue"].toInt();
    param.currentValue = json["currentValue"].toInt();
    if (json.contains("minValue")) param.minValue = json["minValue"].toInt();
    if (json.contains("maxValue")) param.maxValue = json["maxValue"].toInt();
  } else if (param.type == "double") {
    param.defaultValue = json["defaultValue"].toDouble();
    param.currentValue = json["currentValue"].toDouble();
    if (json.contains("minValue")) param.minValue = json["minValue"].toDouble();
    if (json.contains("maxValue")) param.maxValue = json["maxValue"].toDouble();
  } else if (param.type == "bool") {
    param.defaultValue = json["defaultValue"].toBool();
    param.currentValue = json["currentValue"].toBool();
  } else if (param.type == "array") {
    QStringList defaultList, currentList;
    for (const auto& val : json["defaultValue"].toArray()) defaultList.append(val.toString());
    for (const auto& val : json["currentValue"].toArray()) currentList.append(val.toString());
    param.defaultValue = defaultList;
    param.currentValue = currentList;
  }

  if (json.contains("enumValues")) {
    for (const auto& val : json["enumValues"].toArray()) {
      param.enumValues.append(val.toString());
    }
  }

  return param;
}

// BlockData serialization
QJsonObject BlockData::toJson() const {
  QJsonObject json;
  json["id"] = id.toString();
  json["name"] = name;

  QJsonObject posJson;
  posJson["x"] = position.x();
  posJson["y"] = position.y();
  json["position"] = posJson;

  QJsonArray inputsArray;
  for (const auto& pin : inputPins) {
    inputsArray.append(pin.toJson());
  }
  json["inputPins"] = inputsArray;

  QJsonArray outputsArray;
  for (const auto& pin : outputPins) {
    outputsArray.append(pin.toJson());
  }
  json["outputPins"] = outputsArray;

  // Parameters
  if (!parameters.isEmpty()) {
    QJsonArray paramsArray;
    for (const auto& param : parameters) {
      paramsArray.append(param.toJson());
    }
    json["parameters"] = paramsArray;
  }

  // Preferred YAML source
  if (!preferredYamlSource.isEmpty()) {
    json["preferredYamlSource"] = preferredYamlSource;
  }

  return json;
}

BlockData BlockData::fromJson(const QJsonObject& json) {
  BlockData block;
  block.id = QUuid::fromString(json["id"].toString());
  block.name = json["name"].toString();

  QJsonObject posJson = json["position"].toObject();
  block.position = QPointF(posJson["x"].toDouble(), posJson["y"].toDouble());

  QJsonArray inputsArray = json["inputPins"].toArray();
  for (const auto& pinVal : inputsArray) {
    block.inputPins.append(PinData::fromJson(pinVal.toObject()));
  }

  QJsonArray outputsArray = json["outputPins"].toArray();
  for (const auto& pinVal : outputsArray) {
    block.outputPins.append(PinData::fromJson(pinVal.toObject()));
  }

  // Parameters
  if (json.contains("parameters")) {
    QJsonArray paramsArray = json["parameters"].toArray();
    for (const auto& paramVal : paramsArray) {
      block.parameters.append(BlockParamData::fromJson(paramVal.toObject()));
    }
  }

  // Preferred YAML source
  if (json.contains("preferredYamlSource")) {
    block.preferredYamlSource = json["preferredYamlSource"].toString();
  }

  return block;
}

// ConnectionData serialization
QJsonObject ConnectionData::toJson() const {
  QJsonObject json;
  json["id"] = id.toString();
  json["sourceBlockId"] = sourceBlockId.toString();
  json["sourcePinIndex"] = sourcePinIndex;
  json["targetBlockId"] = targetBlockId.toString();
  json["targetPinIndex"] = targetPinIndex;
  return json;
}

ConnectionData ConnectionData::fromJson(const QJsonObject& json) {
  ConnectionData conn;
  conn.id = QUuid::fromString(json["id"].toString());
  conn.sourceBlockId = QUuid::fromString(json["sourceBlockId"].toString());
  conn.sourcePinIndex = json["sourcePinIndex"].toInt();
  conn.targetBlockId = QUuid::fromString(json["targetBlockId"].toString());
  conn.targetPinIndex = json["targetPinIndex"].toInt();
  return conn;
}

// NodeGroupData serialization
QJsonObject NodeGroupData::toJson() const {
  QJsonObject json;
  json["id"] = id.toString();
  json["title"] = title;

  QJsonObject posJson;
  posJson["x"] = position.x();
  posJson["y"] = position.y();
  json["position"] = posJson;

  QJsonObject sizeJson;
  sizeJson["width"] = size.width();
  sizeJson["height"] = size.height();
  json["size"] = sizeJson;

  QJsonObject colorJson;
  colorJson["r"] = color.red();
  colorJson["g"] = color.green();
  colorJson["b"] = color.blue();
  colorJson["a"] = color.alpha();
  json["color"] = colorJson;

  QJsonArray nodeIdsArray;
  for (const QUuid& nodeId : containedNodeIds) {
    nodeIdsArray.append(nodeId.toString());
  }
  json["containedNodeIds"] = nodeIdsArray;

  return json;
}

NodeGroupData NodeGroupData::fromJson(const QJsonObject& json) {
  NodeGroupData group;
  group.id = QUuid::fromString(json["id"].toString());
  group.title = json["title"].toString();

  QJsonObject posJson = json["position"].toObject();
  group.position = QPointF(posJson["x"].toDouble(), posJson["y"].toDouble());

  QJsonObject sizeJson = json["size"].toObject();
  group.size = QSizeF(sizeJson["width"].toDouble(), sizeJson["height"].toDouble());

  QJsonObject colorJson = json["color"].toObject();
  group.color = QColor(colorJson["r"].toInt(), colorJson["g"].toInt(),
                       colorJson["b"].toInt(), colorJson["a"].toInt());

  QJsonArray nodeIdsArray = json["containedNodeIds"].toArray();
  for (const auto& nodeIdVal : nodeIdsArray) {
    group.containedNodeIds.append(QUuid::fromString(nodeIdVal.toString()));
  }

  return group;
}

// ProjectMetadata serialization
QJsonObject ProjectMetadata::toJson() const {
  QJsonObject json;
  json["name"] = name;
  json["description"] = description;
  json["author"] = author;
  json["version"] = version;
  json["rosDistro"] = rosDistro;
  json["createdDate"] = createdDate;
  json["modifiedDate"] = modifiedDate;
  return json;
}

ProjectMetadata ProjectMetadata::fromJson(const QJsonObject& json) {
  ProjectMetadata meta;
  meta.name = json["name"].toString();
  meta.description = json["description"].toString();
  meta.author = json["author"].toString();
  meta.version = json["version"].toString();
  meta.rosDistro = json["rosDistro"].toString();
  meta.createdDate = json["createdDate"].toString();
  meta.modifiedDate = json["modifiedDate"].toString();
  return meta;
}

// YamlFileInfo serialization
QJsonObject YamlFileInfo::toJson() const {
  QJsonObject json;
  json["path"] = filePath;

  QJsonArray nodeNamesArray;
  for (const QString& nodeName : nodeNames) {
    nodeNamesArray.append(nodeName);
  }
  json["nodeNames"] = nodeNamesArray;

  return json;
}

YamlFileInfo YamlFileInfo::fromJson(const QJsonObject& json) {
  YamlFileInfo info;
  info.filePath = json["path"].toString();

  QJsonArray nodeNamesArray = json["nodeNames"].toArray();
  for (const auto& val : nodeNamesArray) {
    info.nodeNames.append(val.toString());
  }

  return info;
}

// CanvasViewState serialization
QJsonObject CanvasViewState::toJson() const {
  QJsonObject json;

  QJsonObject centerJson;
  centerJson["x"] = viewCenter.x();
  centerJson["y"] = viewCenter.y();
  json["viewCenter"] = centerJson;

  json["zoomLevel"] = zoomLevel;

  if (!expandedGroups.isEmpty()) {
    QJsonArray groupsArray;
    for (const QUuid& groupId : expandedGroups) {
      groupsArray.append(groupId.toString());
    }
    json["expandedGroups"] = groupsArray;
  }

  return json;
}

CanvasViewState CanvasViewState::fromJson(const QJsonObject& json) {
  CanvasViewState state;

  if (json.contains("viewCenter")) {
    QJsonObject centerJson = json["viewCenter"].toObject();
    state.viewCenter = QPointF(centerJson["x"].toDouble(), centerJson["y"].toDouble());
  }

  state.zoomLevel = json["zoomLevel"].toDouble(1.0);

  if (json.contains("expandedGroups")) {
    QJsonArray groupsArray = json["expandedGroups"].toArray();
    for (const auto& val : groupsArray) {
      state.expandedGroups.append(QUuid::fromString(val.toString()));
    }
  }

  return state;
}

// LayoutPreset serialization
QJsonObject LayoutPreset::toJson() const {
  QJsonObject json;
  json["name"] = name;
  json["dockState"] = QString::fromLatin1(dockState.toBase64());
  json["windowGeometry"] = QString::fromLatin1(windowGeometry.toBase64());
  return json;
}

LayoutPreset LayoutPreset::fromJson(const QJsonObject& json) {
  LayoutPreset preset;
  preset.name = json["name"].toString();
  preset.dockState = QByteArray::fromBase64(json["dockState"].toString().toLatin1());
  preset.windowGeometry = QByteArray::fromBase64(json["windowGeometry"].toString().toLatin1());
  return preset;
}

// CustomMetadata serialization
QJsonObject CustomMetadata::toJson() const {
  QJsonObject json;
  for (auto it = data.constBegin(); it != data.constEnd(); ++it) {
    QJsonValue value;
    const QVariant& var = it.value();

    if (var.type() == QVariant::String) {
      value = var.toString();
    } else if (var.type() == QVariant::Int) {
      value = var.toInt();
    } else if (var.type() == QVariant::Double) {
      value = var.toDouble();
    } else if (var.type() == QVariant::Bool) {
      value = var.toBool();
    } else if (var.type() == QVariant::StringList) {
      QJsonArray arr;
      for (const QString& s : var.toStringList()) {
        arr.append(s);
      }
      value = arr;
    } else {
      value = var.toString();  // Fallback to string
    }

    json[it.key()] = value;
  }
  return json;
}

CustomMetadata CustomMetadata::fromJson(const QJsonObject& json) {
  CustomMetadata meta;
  for (auto it = json.constBegin(); it != json.constEnd(); ++it) {
    const QJsonValue& val = it.value();

    if (val.isString()) {
      meta.data[it.key()] = val.toString();
    } else if (val.isDouble()) {
      // Check if it's an integer
      double d = val.toDouble();
      if (d == static_cast<int>(d)) {
        meta.data[it.key()] = static_cast<int>(d);
      } else {
        meta.data[it.key()] = d;
      }
    } else if (val.isBool()) {
      meta.data[it.key()] = val.toBool();
    } else if (val.isArray()) {
      QStringList list;
      for (const auto& arrVal : val.toArray()) {
        list.append(arrVal.toString());
      }
      meta.data[it.key()] = list;
    }
  }
  return meta;
}

// CanvasData serialization
QJsonObject CanvasData::toJson() const {
  QJsonObject json;
  json["id"] = id.toString();
  json["name"] = name;
  json["description"] = description;
  json["isActive"] = isActive;

  // Blocks
  QJsonArray blocksArray;
  for (const auto& block : blocks) {
    blocksArray.append(block.toJson());
  }
  json["blocks"] = blocksArray;

  // Connections
  QJsonArray connectionsArray;
  for (const auto& conn : connections) {
    connectionsArray.append(conn.toJson());
  }
  json["connections"] = connectionsArray;

  // Groups
  QJsonArray groupsArray;
  for (const auto& group : groups) {
    groupsArray.append(group.toJson());
  }
  json["groups"] = groupsArray;

  // View state
  json["viewState"] = viewState.toJson();

  return json;
}

CanvasData CanvasData::fromJson(const QJsonObject& json) {
  CanvasData canvas;
  canvas.id = QUuid::fromString(json["id"].toString());
  canvas.name = json["name"].toString();
  canvas.description = json["description"].toString();
  canvas.isActive = json["isActive"].toBool();

  // Blocks
  QJsonArray blocksArray = json["blocks"].toArray();
  for (const auto& blockVal : blocksArray) {
    canvas.blocks.append(BlockData::fromJson(blockVal.toObject()));
  }

  // Connections
  QJsonArray connectionsArray = json["connections"].toArray();
  for (const auto& connVal : connectionsArray) {
    canvas.connections.append(ConnectionData::fromJson(connVal.toObject()));
  }

  // Groups
  QJsonArray groupsArray = json["groups"].toArray();
  for (const auto& groupVal : groupsArray) {
    canvas.groups.append(NodeGroupData::fromJson(groupVal.toObject()));
  }

  // View state
  if (json.contains("viewState")) {
    canvas.viewState = CanvasViewState::fromJson(json["viewState"].toObject());
  }

  return canvas;
}

// Project implementation
Project::Project()
  : activeCanvasIndex_(0)
  , hasUnsavedChanges_(false)
{
  metadata_.version = "1.0";
  metadata_.createdDate = QDateTime::currentDateTime().toString(Qt::ISODate);
  metadata_.modifiedDate = metadata_.createdDate;
}

void Project::addBlock(const BlockData& block) {
  blocks_.append(block);
  hasUnsavedChanges_ = true;
}

void Project::removeBlock(const QUuid& id) {
  for (int i = 0; i < blocks_.size(); ++i) {
    if (blocks_[i].id == id) {
      blocks_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

void Project::addConnection(const ConnectionData& connection) {
  connections_.append(connection);
  hasUnsavedChanges_ = true;
}

void Project::removeConnection(const QUuid& id) {
  for (int i = 0; i < connections_.size(); ++i) {
    if (connections_[i].id == id) {
      connections_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

void Project::addNodeGroup(const NodeGroupData& group) {
  nodeGroups_.append(group);
  hasUnsavedChanges_ = true;
}

void Project::removeNodeGroup(const QUuid& id) {
  for (int i = 0; i < nodeGroups_.size(); ++i) {
    if (nodeGroups_[i].id == id) {
      nodeGroups_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

void Project::addYamlFile(const YamlFileInfo& yamlFile) {
  // Check if already exists
  for (const auto& existing : yamlFiles_) {
    if (existing.filePath == yamlFile.filePath) {
      return;
    }
  }
  yamlFiles_.append(yamlFile);
  hasUnsavedChanges_ = true;
}

void Project::removeYamlFile(const QString& filePath) {
  for (int i = 0; i < yamlFiles_.size(); ++i) {
    if (yamlFiles_[i].filePath == filePath) {
      yamlFiles_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

QString Project::findYamlFileForNode(const QString& nodeName) const {
  for (const auto& yamlFile : yamlFiles_) {
    if (yamlFile.nodeNames.contains(nodeName)) {
      return yamlFile.filePath;
    }
  }
  return QString();
}

void Project::addLayoutPreset(const LayoutPreset& preset) {
  // Replace if exists with same name
  for (int i = 0; i < layoutPresets_.size(); ++i) {
    if (layoutPresets_[i].name == preset.name) {
      layoutPresets_[i] = preset;
      hasUnsavedChanges_ = true;
      return;
    }
  }
  layoutPresets_.append(preset);
  hasUnsavedChanges_ = true;
}

void Project::removeLayoutPreset(const QString& name) {
  for (int i = 0; i < layoutPresets_.size(); ++i) {
    if (layoutPresets_[i].name == name) {
      layoutPresets_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

LayoutPreset* Project::findLayoutPreset(const QString& name) {
  for (int i = 0; i < layoutPresets_.size(); ++i) {
    if (layoutPresets_[i].name == name) {
      return &layoutPresets_[i];
    }
  }
  return nullptr;
}

void Project::setCustomMetadata(const QString& key, const QVariant& value) {
  customMetadata_.data[key] = value;
  hasUnsavedChanges_ = true;
}

QVariant Project::getCustomMetadata(const QString& key, const QVariant& defaultValue) const {
  return customMetadata_.data.value(key, defaultValue);
}

// Multi-canvas support
void Project::addCanvas(const CanvasData& canvas) {
  canvases_.append(canvas);
  hasUnsavedChanges_ = true;
}

void Project::removeCanvas(const QUuid& id) {
  for (int i = 0; i < canvases_.size(); ++i) {
    if (canvases_[i].id == id) {
      canvases_.removeAt(i);
      hasUnsavedChanges_ = true;
      break;
    }
  }
}

void Project::clearCanvases() {
  canvases_.clear();
}

void Project::clearBlocks() {
  blocks_.clear();
}

void Project::clearConnections() {
  connections_.clear();
}

void Project::clearNodeGroups() {
  nodeGroups_.clear();
}

void Project::clear() {
  blocks_.clear();
  connections_.clear();
  nodeGroups_.clear();
  yamlFiles_.clear();
  viewState_ = CanvasViewState();
  layoutPresets_.clear();
  customMetadata_.data.clear();
  canvases_.clear();
  activeCanvasIndex_ = 0;
  metadata_ = ProjectMetadata();
  metadata_.version = "1.0";
  metadata_.createdDate = QDateTime::currentDateTime().toString(Qt::ISODate);
  metadata_.modifiedDate = metadata_.createdDate;
  filePath_.clear();
  hasUnsavedChanges_ = false;
}

QJsonObject Project::toJson() const {
  QJsonObject json;

  // File format version
  json["formatVersion"] = "1.0";
  json["application"] = "ROS Weaver";

  // Metadata
  json["metadata"] = metadata_.toJson();

  // Blocks
  QJsonArray blocksArray;
  for (const auto& block : blocks_) {
    blocksArray.append(block.toJson());
  }
  json["blocks"] = blocksArray;

  // Connections
  QJsonArray connectionsArray;
  for (const auto& conn : connections_) {
    connectionsArray.append(conn.toJson());
  }
  json["connections"] = connectionsArray;

  // Node Groups
  QJsonArray nodeGroupsArray;
  for (const auto& group : nodeGroups_) {
    nodeGroupsArray.append(group.toJson());
  }
  json["nodeGroups"] = nodeGroupsArray;

  // YAML Files
  if (!yamlFiles_.isEmpty()) {
    QJsonArray yamlFilesArray;
    for (const auto& yamlFile : yamlFiles_) {
      yamlFilesArray.append(yamlFile.toJson());
    }
    json["yamlFiles"] = yamlFilesArray;
  }

  // Canvas View State
  json["viewState"] = viewState_.toJson();

  // Layout Presets
  if (!layoutPresets_.isEmpty()) {
    QJsonArray presetsArray;
    for (const auto& preset : layoutPresets_) {
      presetsArray.append(preset.toJson());
    }
    json["layoutPresets"] = presetsArray;
  }

  // Custom Metadata
  if (!customMetadata_.data.isEmpty()) {
    json["customMetadata"] = customMetadata_.toJson();
  }

  // Multi-Canvas support
  if (!canvases_.isEmpty()) {
    QJsonArray canvasesArray;
    for (const auto& canvas : canvases_) {
      canvasesArray.append(canvas.toJson());
    }
    json["canvases"] = canvasesArray;
    json["activeCanvasIndex"] = activeCanvasIndex_;
  }

  return json;
}

Project Project::fromJson(const QJsonObject& json) {
  Project project;

  // Metadata
  if (json.contains("metadata")) {
    project.metadata_ = ProjectMetadata::fromJson(json["metadata"].toObject());
  }

  // Blocks
  QJsonArray blocksArray = json["blocks"].toArray();
  for (const auto& blockVal : blocksArray) {
    project.blocks_.append(BlockData::fromJson(blockVal.toObject()));
  }

  // Connections
  QJsonArray connectionsArray = json["connections"].toArray();
  for (const auto& connVal : connectionsArray) {
    project.connections_.append(ConnectionData::fromJson(connVal.toObject()));
  }

  // Node Groups
  QJsonArray nodeGroupsArray = json["nodeGroups"].toArray();
  for (const auto& groupVal : nodeGroupsArray) {
    project.nodeGroups_.append(NodeGroupData::fromJson(groupVal.toObject()));
  }

  // YAML Files
  if (json.contains("yamlFiles")) {
    QJsonArray yamlFilesArray = json["yamlFiles"].toArray();
    for (const auto& yamlVal : yamlFilesArray) {
      project.yamlFiles_.append(YamlFileInfo::fromJson(yamlVal.toObject()));
    }
  }

  // Canvas View State
  if (json.contains("viewState")) {
    project.viewState_ = CanvasViewState::fromJson(json["viewState"].toObject());
  }

  // Layout Presets
  if (json.contains("layoutPresets")) {
    QJsonArray presetsArray = json["layoutPresets"].toArray();
    for (const auto& presetVal : presetsArray) {
      project.layoutPresets_.append(LayoutPreset::fromJson(presetVal.toObject()));
    }
  }

  // Custom Metadata
  if (json.contains("customMetadata")) {
    project.customMetadata_ = CustomMetadata::fromJson(json["customMetadata"].toObject());
  }

  // Multi-Canvas support
  if (json.contains("canvases")) {
    QJsonArray canvasesArray = json["canvases"].toArray();
    for (const auto& canvasVal : canvasesArray) {
      project.canvases_.append(CanvasData::fromJson(canvasVal.toObject()));
    }
    project.activeCanvasIndex_ = json["activeCanvasIndex"].toInt(0);
  }

  project.hasUnsavedChanges_ = false;
  return project;
}

bool Project::saveToFile(const QString& filePath) const {
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  // Update modified date
  ProjectMetadata& mutableMeta = const_cast<ProjectMetadata&>(metadata_);
  mutableMeta.modifiedDate = QDateTime::currentDateTime().toString(Qt::ISODate);

  QJsonDocument doc(toJson());
  file.write(doc.toJson(QJsonDocument::Indented));
  file.close();

  return true;
}

Project Project::loadFromFile(const QString& filePath, QString* errorMsg) {
  Project project;

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    if (errorMsg) {
      *errorMsg = QString("Could not open file: %1").arg(filePath);
    }
    return project;
  }

  QByteArray data = file.readAll();
  file.close();

  QJsonParseError parseError;
  QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);

  if (parseError.error != QJsonParseError::NoError) {
    if (errorMsg) {
      *errorMsg = QString("JSON parse error: %1").arg(parseError.errorString());
    }
    return project;
  }

  if (!doc.isObject()) {
    if (errorMsg) {
      *errorMsg = "Invalid project file format";
    }
    return project;
  }

  project = fromJson(doc.object());
  project.filePath_ = filePath;
  project.hasUnsavedChanges_ = false;

  return project;
}

}  // namespace ros_weaver
