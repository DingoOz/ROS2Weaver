#ifndef ROS_WEAVER_CORE_PARAMETER_PRESET_HPP
#define ROS_WEAVER_CORE_PARAMETER_PRESET_HPP

#include <QString>
#include <QVariant>
#include <QMap>
#include <QList>
#include <QDateTime>
#include <QUuid>

namespace ros_weaver {

/**
 * @brief A single parameter override within a preset
 */
struct ParameterOverride {
  QString parameterName;       // Full parameter path (e.g., "node_name.param")
  QVariant value;              // Override value
  bool enabled = true;         // Whether this override is active
  QString description;         // Optional description

  bool operator==(const ParameterOverride& other) const {
    return parameterName == other.parameterName && value == other.value;
  }
};

/**
 * @brief Node-specific parameter overrides
 */
struct NodeParameterSet {
  QString nodeName;                        // Node identifier
  QList<ParameterOverride> overrides;      // Parameter overrides for this node

  bool hasOverride(const QString& paramName) const;
  QVariant getOverrideValue(const QString& paramName) const;
};

/**
 * @brief A complete parameter preset configuration
 */
struct ParameterPreset {
  QUuid id;
  QString name;
  QString description;
  QString category;                          // e.g., "simulation", "debug", "production"
  QDateTime createdAt;
  QDateTime modifiedAt;

  // Global parameter overrides (applied to all nodes)
  QList<ParameterOverride> globalOverrides;

  // Per-node parameter overrides
  QList<NodeParameterSet> nodeOverrides;

  // Color for visual identification
  QString color = "#4CAF50";

  // Keyboard shortcut for quick access
  QString shortcut;

  // Factory methods
  static ParameterPreset createDefault();
  static ParameterPreset createSimulation();
  static ParameterPreset createRealRobot();
  static ParameterPreset createDebug();

  // Utility methods
  bool hasOverrideFor(const QString& nodeName, const QString& paramName) const;
  QVariant getOverrideValue(const QString& nodeName, const QString& paramName) const;
  void setOverride(const QString& nodeName, const QString& paramName, const QVariant& value);
  void removeOverride(const QString& nodeName, const QString& paramName);

  // Comparison for diff
  QStringList diffWith(const ParameterPreset& other) const;

  // Serialization
  QVariantMap toVariantMap() const;
  static ParameterPreset fromVariantMap(const QVariantMap& map);
  QString toYaml() const;
  static ParameterPreset fromYaml(const QString& yaml);

  bool operator==(const ParameterPreset& other) const;
};

/**
 * @brief Manager for parameter presets
 */
class PresetManager : public QObject {
  Q_OBJECT

public:
  static PresetManager& instance();

  // Preset management
  QList<ParameterPreset> presets() const { return presets_; }
  void addPreset(const ParameterPreset& preset);
  void updatePreset(const ParameterPreset& preset);
  void removePreset(const QUuid& id);
  ParameterPreset* findPreset(const QUuid& id);
  ParameterPreset* findPresetByName(const QString& name);

  // Active preset
  ParameterPreset* activePreset() { return activePreset_; }
  void setActivePreset(const QUuid& id);
  void clearActivePreset();

  // File operations
  bool savePreset(const ParameterPreset& preset, const QString& filePath);
  ParameterPreset loadPreset(const QString& filePath);
  bool saveAllPresets(const QString& dirPath);
  void loadPresetsFromDir(const QString& dirPath);

  // Built-in presets
  void createBuiltInPresets();

signals:
  void presetAdded(const ParameterPreset& preset);
  void presetUpdated(const ParameterPreset& preset);
  void presetRemoved(const QUuid& id);
  void activePresetChanged(ParameterPreset* preset);

private:
  PresetManager(QObject* parent = nullptr);

  QList<ParameterPreset> presets_;
  ParameterPreset* activePreset_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_PARAMETER_PRESET_HPP
