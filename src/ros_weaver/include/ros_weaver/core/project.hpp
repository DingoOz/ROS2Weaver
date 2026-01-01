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
#include <QByteArray>
#include <QMap>
#include <QVariant>

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

// Serializable parameter data for blocks
struct BlockParamData {
  QString name;
  QString type;           // "string", "int", "double", "bool", "array"
  QVariant defaultValue;
  QVariant currentValue;
  QString description;
  QVariant minValue;
  QVariant maxValue;
  QStringList enumValues;
  QString group;

  QJsonObject toJson() const;
  static BlockParamData fromJson(const QJsonObject& json);
};

// Serializable block data
struct BlockData {
  QUuid id;
  QString name;
  QPointF position;
  QList<PinData> inputPins;
  QList<PinData> outputPins;
  QList<BlockParamData> parameters;  // Parameters for this block
  QString preferredYamlSource;       // Empty = auto, "block" = block params, path = YAML file

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

// YAML file information for project-associated parameter files
struct YamlFileInfo {
  QString filePath;         // Path to YAML file (relative to project or absolute)
  QStringList nodeNames;    // Top-level keys in the YAML (e.g., "slam_toolbox", "amcl")

  QJsonObject toJson() const;
  static YamlFileInfo fromJson(const QJsonObject& json);
};

// Canvas view state for restoring zoom and pan position
struct CanvasViewState {
  QPointF viewCenter;           // Center point of the view
  double zoomLevel = 1.0;       // Current zoom level
  QList<QUuid> expandedGroups;  // Groups that are expanded in the view

  QJsonObject toJson() const;
  static CanvasViewState fromJson(const QJsonObject& json);
};

// Layout preset for saving/restoring dock widget arrangements
struct LayoutPreset {
  QString name;               // User-friendly name for the preset
  QByteArray dockState;       // QMainWindow::saveState() result
  QByteArray windowGeometry;  // QWidget::saveGeometry() result

  QJsonObject toJson() const;
  static LayoutPreset fromJson(const QJsonObject& json);
};

// Custom metadata for extensibility
struct CustomMetadata {
  QMap<QString, QVariant> data;

  QJsonObject toJson() const;
  static CustomMetadata fromJson(const QJsonObject& json);
};

// Canvas data for multi-canvas support
struct CanvasData {
  QUuid id;
  QString name;
  QString description;
  bool isActive = false;
  QList<BlockData> blocks;
  QList<ConnectionData> connections;
  QList<NodeGroupData> groups;
  CanvasViewState viewState;

  QJsonObject toJson() const;
  static CanvasData fromJson(const QJsonObject& json);
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

  // YAML Files
  void addYamlFile(const YamlFileInfo& yamlFile);
  void removeYamlFile(const QString& filePath);
  const QList<YamlFileInfo>& yamlFiles() const { return yamlFiles_; }
  QList<YamlFileInfo>& yamlFiles() { return yamlFiles_; }
  QString findYamlFileForNode(const QString& nodeName) const;

  // Canvas View State
  CanvasViewState& viewState() { return viewState_; }
  const CanvasViewState& viewState() const { return viewState_; }
  void setViewState(const CanvasViewState& state) { viewState_ = state; }

  // Layout Presets
  void addLayoutPreset(const LayoutPreset& preset);
  void removeLayoutPreset(const QString& name);
  const QList<LayoutPreset>& layoutPresets() const { return layoutPresets_; }
  QList<LayoutPreset>& layoutPresets() { return layoutPresets_; }
  LayoutPreset* findLayoutPreset(const QString& name);

  // Custom Metadata (for extensibility)
  CustomMetadata& customMetadata() { return customMetadata_; }
  const CustomMetadata& customMetadata() const { return customMetadata_; }
  void setCustomMetadata(const QString& key, const QVariant& value);
  QVariant getCustomMetadata(const QString& key, const QVariant& defaultValue = QVariant()) const;

  // Multi-Canvas support
  void addCanvas(const CanvasData& canvas);
  void removeCanvas(const QUuid& id);
  const QList<CanvasData>& canvases() const { return canvases_; }
  QList<CanvasData>& canvases() { return canvases_; }
  void clearCanvases();
  int activeCanvasIndex() const { return activeCanvasIndex_; }
  void setActiveCanvasIndex(int index) { activeCanvasIndex_ = index; }

  // Clear operations
  void clearBlocks();
  void clearConnections();
  void clearNodeGroups();

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
  QList<YamlFileInfo> yamlFiles_;
  CanvasViewState viewState_;
  QList<LayoutPreset> layoutPresets_;
  CustomMetadata customMetadata_;
  QList<CanvasData> canvases_;
  int activeCanvasIndex_ = 0;
  QString filePath_;
  bool hasUnsavedChanges_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_PROJECT_HPP
