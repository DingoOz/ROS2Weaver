#include "ros_weaver/core/data_lineage.hpp"
#include <QProcess>
#include <QFileInfo>
#include <QDateTime>
#include <QJsonArray>

namespace ros_weaver {

// LineageNode implementation

bool LineageNode::isEditable() const {
  switch (sourceType) {
    case LineageSourceType::ProjectFile:
    case LineageSourceType::YamlFile:
    case LineageSourceType::SourceCodeFile:
    case LineageSourceType::Generated:
      return !sourcePath.isEmpty() && QFileInfo::exists(sourcePath);
    default:
      return false;
  }
}

QString LineageNode::sourceTypeName() const {
  switch (sourceType) {
    case LineageSourceType::Unknown:
      return "Unknown";
    case LineageSourceType::ProjectFile:
      return "Project File";
    case LineageSourceType::YamlFile:
      return "YAML Configuration";
    case LineageSourceType::SourceCodeFile:
      return "Source Code";
    case LineageSourceType::RosTopic:
      return "ROS Topic";
    case LineageSourceType::RosParameter:
      return "ROS Parameter";
    case LineageSourceType::Generated:
      return "Generated Code";
    case LineageSourceType::UserInput:
      return "User Input";
    case LineageSourceType::SystemDiscovery:
      return "System Discovery";
  }
  return "Unknown";
}

QJsonObject LineageNode::toJson() const {
  QJsonObject json;
  json["sourceType"] = static_cast<int>(sourceType);
  json["sourcePath"] = sourcePath;
  json["lineNumber"] = lineNumber;
  json["columnNumber"] = columnNumber;
  json["description"] = description;
  json["dataKey"] = dataKey;
  json["timestamp"] = timestamp;
  if (dataValue.isValid()) {
    json["dataValue"] = QJsonValue::fromVariant(dataValue);
  }
  return json;
}

LineageNode LineageNode::fromJson(const QJsonObject& json) {
  LineageNode node;
  node.sourceType = static_cast<LineageSourceType>(json["sourceType"].toInt());
  node.sourcePath = json["sourcePath"].toString();
  node.lineNumber = json["lineNumber"].toInt();
  node.columnNumber = json["columnNumber"].toInt();
  node.description = json["description"].toString();
  node.dataKey = json["dataKey"].toString();
  node.timestamp = json["timestamp"].toString();
  if (json.contains("dataValue")) {
    node.dataValue = json["dataValue"].toVariant();
  }
  return node;
}

// DataLineage implementation

DataLineage::DataLineage() {
  primarySource_.sourceType = LineageSourceType::Unknown;
}

DataLineage::DataLineage(const LineageNode& primarySource)
    : primarySource_(primarySource) {}

void DataLineage::setPrimarySource(const LineageNode& source) {
  primarySource_ = source;
}

void DataLineage::addIntermediateSource(const LineageNode& source) {
  intermediateChain_.append(source);
}

bool DataLineage::isKnown() const {
  return primarySource_.sourceType != LineageSourceType::Unknown;
}

bool DataLineage::hasEditableSource() const {
  if (primarySource_.isEditable()) {
    return true;
  }
  for (const auto& node : intermediateChain_) {
    if (node.isEditable()) {
      return true;
    }
  }
  return false;
}

LineageNode DataLineage::firstEditableSource() const {
  if (primarySource_.isEditable()) {
    return primarySource_;
  }
  for (const auto& node : intermediateChain_) {
    if (node.isEditable()) {
      return node;
    }
  }
  return LineageNode();
}

QString DataLineage::toDisplayString() const {
  QStringList lines;

  if (!dataDescription_.isEmpty()) {
    lines << QString("Data: %1").arg(dataDescription_);
    lines << "";
  }

  lines << "=== Primary Source ===";
  lines << QString("Type: %1").arg(primarySource_.sourceTypeName());
  if (!primarySource_.sourcePath.isEmpty()) {
    lines << QString("Location: %1").arg(primarySource_.sourcePath);
    if (primarySource_.lineNumber > 0) {
      lines << QString("Line: %1").arg(primarySource_.lineNumber);
    }
  }
  if (!primarySource_.dataKey.isEmpty()) {
    lines << QString("Key: %1").arg(primarySource_.dataKey);
  }
  if (!primarySource_.description.isEmpty()) {
    lines << QString("Details: %1").arg(primarySource_.description);
  }

  if (!intermediateChain_.isEmpty()) {
    lines << "";
    lines << "=== Intermediate Sources ===";
    for (int i = 0; i < intermediateChain_.size(); ++i) {
      const auto& node = intermediateChain_[i];
      lines << QString("[%1] %2").arg(i + 1).arg(node.sourceTypeName());
      if (!node.sourcePath.isEmpty()) {
        lines << QString("    Location: %1").arg(node.sourcePath);
      }
      if (!node.description.isEmpty()) {
        lines << QString("    Details: %1").arg(node.description);
      }
    }
  }

  return lines.join("\n");
}

QJsonObject DataLineage::toJson() const {
  QJsonObject json;
  json["primarySource"] = primarySource_.toJson();
  json["dataDescription"] = dataDescription_;

  QJsonArray chainArray;
  for (const auto& node : intermediateChain_) {
    chainArray.append(node.toJson());
  }
  json["intermediateChain"] = chainArray;

  return json;
}

DataLineage DataLineage::fromJson(const QJsonObject& json) {
  DataLineage lineage;
  lineage.primarySource_ = LineageNode::fromJson(json["primarySource"].toObject());
  lineage.dataDescription_ = json["dataDescription"].toString();

  QJsonArray chainArray = json["intermediateChain"].toArray();
  for (const auto& nodeVal : chainArray) {
    lineage.intermediateChain_.append(LineageNode::fromJson(nodeVal.toObject()));
  }

  return lineage;
}

// Factory methods

DataLineage DataLineage::fromProjectFile(const QString& projectPath,
                                          const QString& dataKey,
                                          int lineNumber) {
  LineageNode node;
  node.sourceType = LineageSourceType::ProjectFile;
  node.sourcePath = projectPath;
  node.lineNumber = lineNumber;
  node.dataKey = dataKey;
  node.description = QString("Stored in project file");
  return DataLineage(node);
}

DataLineage DataLineage::fromYamlFile(const QString& yamlPath,
                                       const QString& paramName,
                                       int lineNumber) {
  LineageNode node;
  node.sourceType = LineageSourceType::YamlFile;
  node.sourcePath = yamlPath;
  node.lineNumber = lineNumber;
  node.dataKey = paramName;
  node.description = QString("YAML parameter configuration");
  return DataLineage(node);
}

DataLineage DataLineage::fromSourceFile(const QString& filePath,
                                         int lineNumber,
                                         const QString& description) {
  LineageNode node;
  node.sourceType = LineageSourceType::SourceCodeFile;
  node.sourcePath = filePath;
  node.lineNumber = lineNumber;
  node.description = description;
  return DataLineage(node);
}

DataLineage DataLineage::fromRosTopic(const QString& topicName,
                                       const QString& messageType) {
  LineageNode node;
  node.sourceType = LineageSourceType::RosTopic;
  node.sourcePath = topicName;
  node.dataKey = messageType;
  node.description = QString("Live data from ROS2 topic");
  node.timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);
  return DataLineage(node);
}

DataLineage DataLineage::fromGenerated(const QString& generatedPath,
                                        const QString& templateSource) {
  LineageNode primary;
  primary.sourceType = LineageSourceType::Generated;
  primary.sourcePath = generatedPath;
  primary.description = QString("Generated by ROS Weaver");

  DataLineage lineage(primary);

  if (!templateSource.isEmpty()) {
    LineageNode templateNode;
    templateNode.sourceType = LineageSourceType::SourceCodeFile;
    templateNode.sourcePath = templateSource;
    templateNode.description = "Template used for generation";
    lineage.addIntermediateSource(templateNode);
  }

  return lineage;
}

DataLineage DataLineage::fromUserInput(const QString& widgetName,
                                        const QString& fieldName) {
  LineageNode node;
  node.sourceType = LineageSourceType::UserInput;
  node.description = QString("Entered in %1").arg(widgetName);
  node.dataKey = fieldName;
  node.timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);
  return DataLineage(node);
}

// VsCodeIntegration implementation

bool VsCodeIntegration::openFile(const QString& filePath, int lineNumber, int column) {
  QString command = vsCodeCommand();
  if (command.isEmpty()) {
    return false;
  }

  QStringList args;
  args << "--goto";

  QString location = filePath;
  if (lineNumber > 0) {
    location += QString(":%1").arg(lineNumber);
    if (column > 0) {
      location += QString(":%1").arg(column);
    }
  }
  args << location;

  return QProcess::startDetached(command, args);
}

bool VsCodeIntegration::isAvailable() {
  return !vsCodeCommand().isEmpty();
}

QString VsCodeIntegration::vsCodeCommand() {
  // Check for VS Code variants in order of preference
  QStringList candidates = {"code", "code-insiders", "codium"};

  for (const QString& cmd : candidates) {
    QProcess process;
    process.start("which", QStringList() << cmd);
    process.waitForFinished(1000);
    if (process.exitCode() == 0) {
      return cmd;
    }
  }

  return QString();
}

}  // namespace ros_weaver
