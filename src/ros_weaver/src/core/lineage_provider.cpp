#include "ros_weaver/core/lineage_provider.hpp"
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QFileInfo>

namespace ros_weaver {

// Singleton instance
LineageProvider& globalLineageProvider() {
  static LineageProvider instance;
  return instance;
}

LineageProvider::LineageProvider()
    : project_(nullptr) {}

void LineageProvider::setProject(const Project* project) {
  project_ = project;
  if (project_) {
    projectFilePath_ = project_->filePath();
  }
}

void LineageProvider::setProjectFilePath(const QString& path) {
  projectFilePath_ = path;
}

DataLineage LineageProvider::getParameterLineage(const QString& blockName,
                                                   const QString& paramName,
                                                   const QString& yamlSourcePath) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Parameter: %1.%2").arg(blockName, paramName));

  if (!yamlSourcePath.isEmpty() && yamlSourcePath != "block") {
    // Parameter comes from a YAML file
    int lineNumber = findYamlLineNumber(yamlSourcePath, paramName);

    LineageNode primary;
    primary.sourceType = LineageSourceType::YamlFile;
    primary.sourcePath = yamlSourcePath;
    primary.lineNumber = lineNumber;
    primary.dataKey = paramName;
    primary.description = QString("YAML configuration for %1").arg(blockName);
    lineage.setPrimarySource(primary);

    // Add project file as intermediate source if available
    if (!projectFilePath_.isEmpty()) {
      LineageNode projectNode;
      projectNode.sourceType = LineageSourceType::ProjectFile;
      projectNode.sourcePath = projectFilePath_;
      projectNode.description = "YAML file referenced in project";
      lineage.addIntermediateSource(projectNode);
    }
  } else if (yamlSourcePath == "block") {
    // Parameter is stored directly in the block (project file)
    LineageNode primary;
    primary.sourceType = LineageSourceType::ProjectFile;
    primary.sourcePath = projectFilePath_;
    primary.dataKey = QString("%1.parameters.%2").arg(blockName, paramName);
    primary.description = "Block parameter stored in project";
    lineage.setPrimarySource(primary);
  } else {
    // Auto-detect mode or unknown source
    LineageNode primary;
    primary.sourceType = LineageSourceType::UserInput;
    primary.dataKey = paramName;
    primary.description = QString("Parameter for %1 (auto-detected or user-defined)").arg(blockName);
    lineage.setPrimarySource(primary);
  }

  return lineage;
}

DataLineage LineageProvider::getBlockLineage(const BlockData& block) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Block: %1").arg(block.name));

  LineageNode primary;
  primary.sourceType = LineageSourceType::ProjectFile;
  primary.sourcePath = projectFilePath_;
  primary.dataKey = QString("blocks[%1]").arg(block.id.toString());
  primary.description = QString("Block definition for '%1'").arg(block.name);
  lineage.setPrimarySource(primary);

  return lineage;
}

DataLineage LineageProvider::getConnectionLineage(const QString& topicName,
                                                    const QString& messageType,
                                                    bool isLiveData) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Topic: %1").arg(topicName));

  if (isLiveData) {
    // Live data from ROS2 topic
    LineageNode primary;
    primary.sourceType = LineageSourceType::RosTopic;
    primary.sourcePath = topicName;
    primary.dataKey = messageType;
    primary.description = "Live data from running ROS2 system";
    lineage.setPrimarySource(primary);

    // Add project file as intermediate source for connection definition
    if (!projectFilePath_.isEmpty()) {
      LineageNode projectNode;
      projectNode.sourceType = LineageSourceType::ProjectFile;
      projectNode.sourcePath = projectFilePath_;
      projectNode.description = "Connection defined in project";
      lineage.addIntermediateSource(projectNode);
    }
  } else {
    // Connection definition from project
    LineageNode primary;
    primary.sourceType = LineageSourceType::ProjectFile;
    primary.sourcePath = projectFilePath_;
    primary.dataKey = topicName;
    primary.description = QString("Connection with message type %1").arg(messageType);
    lineage.setPrimarySource(primary);
  }

  return lineage;
}

DataLineage LineageProvider::getRosTopicLineage(const QString& topicName,
                                                  const QString& messageType) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("ROS Topic: %1").arg(topicName));

  LineageNode primary;
  primary.sourceType = LineageSourceType::RosTopic;
  primary.sourcePath = topicName;
  primary.dataKey = messageType;
  primary.description = "Discovered from running ROS2 system";
  lineage.setPrimarySource(primary);

  return lineage;
}

DataLineage LineageProvider::getTfFrameLineage(const QString& frameName,
                                                 const QString& parentFrame) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("TF Frame: %1").arg(frameName));

  LineageNode primary;
  primary.sourceType = LineageSourceType::SystemDiscovery;
  primary.sourcePath = "/tf";
  primary.dataKey = frameName;
  if (!parentFrame.isEmpty()) {
    primary.description = QString("Transform from %1 to %2").arg(parentFrame, frameName);
  } else {
    primary.description = "Transform frame from TF tree";
  }
  lineage.setPrimarySource(primary);

  return lineage;
}

DataLineage LineageProvider::getGeneratedCodeLineage(const QString& generatedFilePath,
                                                       const QString& blockName) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Generated: %1").arg(QFileInfo(generatedFilePath).fileName()));

  LineageNode primary;
  primary.sourceType = LineageSourceType::Generated;
  primary.sourcePath = generatedFilePath;
  primary.description = "Generated by ROS Weaver code generator";
  lineage.setPrimarySource(primary);

  // Add project file as source
  if (!projectFilePath_.isEmpty()) {
    LineageNode projectNode;
    projectNode.sourceType = LineageSourceType::ProjectFile;
    projectNode.sourcePath = projectFilePath_;
    if (!blockName.isEmpty()) {
      projectNode.description = QString("Generated from block '%1'").arg(blockName);
    } else {
      projectNode.description = "Generated from project";
    }
    lineage.addIntermediateSource(projectNode);
  }

  return lineage;
}

DataLineage LineageProvider::getYamlConfigLineage(const QString& yamlPath,
                                                    const QString& paramPath,
                                                    int lineNumber) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Config: %1").arg(paramPath));

  LineageNode primary;
  primary.sourceType = LineageSourceType::YamlFile;
  primary.sourcePath = yamlPath;
  primary.lineNumber = lineNumber > 0 ? lineNumber : findYamlLineNumber(yamlPath, paramPath);
  primary.dataKey = paramPath;
  primary.description = "YAML configuration parameter";
  lineage.setPrimarySource(primary);

  return lineage;
}

DataLineage LineageProvider::getMetadataLineage(const QString& fieldName) const {
  DataLineage lineage;
  lineage.setDataDescription(QString("Project: %1").arg(fieldName));

  LineageNode primary;
  primary.sourceType = LineageSourceType::ProjectFile;
  primary.sourcePath = projectFilePath_;
  primary.dataKey = QString("metadata.%1").arg(fieldName);
  primary.description = "Project metadata field";
  lineage.setPrimarySource(primary);

  return lineage;
}

int LineageProvider::findYamlLineNumber(const QString& yamlPath, const QString& paramName) {
  QFile file(yamlPath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return 0;
  }

  QTextStream in(&file);
  int lineNumber = 0;

  // Handle nested parameter names (e.g., "node.param.subparam")
  QStringList parts = paramName.split('.');
  QString searchKey = parts.isEmpty() ? paramName : parts.last();

  // Create regex to match the parameter key
  QRegularExpression keyRegex(
      QString("^\\s*%1\\s*:").arg(QRegularExpression::escape(searchKey)));

  while (!in.atEnd()) {
    lineNumber++;
    QString line = in.readLine();

    if (keyRegex.match(line).hasMatch()) {
      file.close();
      return lineNumber;
    }
  }

  file.close();
  return 0;  // Not found
}

int LineageProvider::findSourceLineNumber(const QString& filePath, const QString& pattern) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return 0;
  }

  QTextStream in(&file);
  int lineNumber = 0;

  QRegularExpression regex(pattern);

  while (!in.atEnd()) {
    lineNumber++;
    QString line = in.readLine();

    if (regex.match(line).hasMatch()) {
      file.close();
      return lineNumber;
    }
  }

  file.close();
  return 0;  // Not found
}

}  // namespace ros_weaver
