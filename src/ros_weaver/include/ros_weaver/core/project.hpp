#ifndef ROS_WEAVER_CORE_PROJECT_HPP
#define ROS_WEAVER_CORE_PROJECT_HPP

#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QList>
#include <QPointF>
#include <QSizeF>
#include <QColor>
#include <QUuid>

namespace ros_weaver {

// Serializable pin data
struct PinData {
  QString name;
  QString type;       // "input" or "output"
  QString dataType;   // "topic", "service", "action", "parameter"
  QString messageType;

  QJsonObject toJson() const;
  static PinData fromJson(const QJsonObject& json);
};

// Serializable block data
struct BlockData {
  QUuid id;
  QString name;
  QPointF position;
  QList<PinData> inputPins;
  QList<PinData> outputPins;

  QJsonObject toJson() const;
  static BlockData fromJson(const QJsonObject& json);
};

// Serializable connection data
struct ConnectionData {
  QUuid id;
  QUuid sourceBlockId;
  int sourcePinIndex;
  QUuid targetBlockId;
  int targetPinIndex;

  QJsonObject toJson() const;
  static ConnectionData fromJson(const QJsonObject& json);
};

// Serializable node group data
struct NodeGroupData {
  QUuid id;
  QString title;
  QPointF position;
  QSizeF size;
  QColor color;
  QList<QUuid> containedNodeIds;

  QJsonObject toJson() const;
  static NodeGroupData fromJson(const QJsonObject& json);
};

// Project metadata
struct ProjectMetadata {
  QString name;
  QString description;
  QString author;
  QString version;
  QString rosDistro;
  QString createdDate;
  QString modifiedDate;

  QJsonObject toJson() const;
  static ProjectMetadata fromJson(const QJsonObject& json);
};

// Main project class
class Project {
public:
  Project();
  ~Project() = default;

  // Metadata
  ProjectMetadata& metadata() { return metadata_; }
  const ProjectMetadata& metadata() const { return metadata_; }

  // Blocks
  void addBlock(const BlockData& block);
  void removeBlock(const QUuid& id);
  const QList<BlockData>& blocks() const { return blocks_; }
  QList<BlockData>& blocks() { return blocks_; }

  // Connections
  void addConnection(const ConnectionData& connection);
  void removeConnection(const QUuid& id);
  const QList<ConnectionData>& connections() const { return connections_; }
  QList<ConnectionData>& connections() { return connections_; }

  // Node Groups
  void addNodeGroup(const NodeGroupData& group);
  void removeNodeGroup(const QUuid& id);
  const QList<NodeGroupData>& nodeGroups() const { return nodeGroups_; }
  QList<NodeGroupData>& nodeGroups() { return nodeGroups_; }

  // Clear all data
  void clear();

  // Serialization
  QJsonObject toJson() const;
  static Project fromJson(const QJsonObject& json);

  // File operations
  bool saveToFile(const QString& filePath) const;
  static Project loadFromFile(const QString& filePath, QString* errorMsg = nullptr);

  // File path tracking
  QString filePath() const { return filePath_; }
  void setFilePath(const QString& path) { filePath_ = path; }
  bool hasUnsavedChanges() const { return hasUnsavedChanges_; }
  void setHasUnsavedChanges(bool value) { hasUnsavedChanges_ = value; }

private:
  ProjectMetadata metadata_;
  QList<BlockData> blocks_;
  QList<ConnectionData> connections_;
  QList<NodeGroupData> nodeGroups_;
  QString filePath_;
  bool hasUnsavedChanges_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_PROJECT_HPP
