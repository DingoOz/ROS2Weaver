#ifndef ROS_WEAVER_CORE_LINEAGE_PROVIDER_HPP
#define ROS_WEAVER_CORE_LINEAGE_PROVIDER_HPP

#include "ros_weaver/core/data_lineage.hpp"
#include "ros_weaver/core/project.hpp"
#include <QString>
#include <QMap>
#include <functional>

namespace ros_weaver {

// Forward declarations
class PackageBlock;
class ConnectionLine;

// Provides lineage information for various data elements in ROS Weaver
class LineageProvider {
public:
  LineageProvider();
  ~LineageProvider() = default;

  // Set the current project (for file path tracking)
  void setProject(const Project* project);
  void setProjectFilePath(const QString& path);

  // Get lineage for a block parameter
  DataLineage getParameterLineage(const QString& blockName,
                                   const QString& paramName,
                                   const QString& yamlSourcePath = QString()) const;

  // Get lineage for a block (node) definition
  DataLineage getBlockLineage(const BlockData& block) const;

  // Get lineage for a connection/topic
  DataLineage getConnectionLineage(const QString& topicName,
                                    const QString& messageType,
                                    bool isLiveData = false) const;

  // Get lineage for a topic discovered from the ROS2 system
  DataLineage getRosTopicLineage(const QString& topicName,
                                  const QString& messageType) const;

  // Get lineage for a TF frame
  DataLineage getTfFrameLineage(const QString& frameName,
                                 const QString& parentFrame = QString()) const;

  // Get lineage for generated code
  DataLineage getGeneratedCodeLineage(const QString& generatedFilePath,
                                       const QString& blockName = QString()) const;

  // Get lineage for YAML configuration
  DataLineage getYamlConfigLineage(const QString& yamlPath,
                                    const QString& paramPath,
                                    int lineNumber = 0) const;

  // Get lineage for a project metadata field
  DataLineage getMetadataLineage(const QString& fieldName) const;

  // Utility: Find line number in YAML file for a parameter
  static int findYamlLineNumber(const QString& yamlPath, const QString& paramName);

  // Utility: Find line number in source file for a pattern
  static int findSourceLineNumber(const QString& filePath, const QString& pattern);

private:
  const Project* project_;
  QString projectFilePath_;
};

// Singleton instance for global access
LineageProvider& globalLineageProvider();

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LINEAGE_PROVIDER_HPP
