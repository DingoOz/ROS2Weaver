#include "ros_weaver/core/parameter_preset.hpp"

#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace ros_weaver {

// NodeParameterSet implementation

bool NodeParameterSet::hasOverride(const QString& paramName) const {
  for (const auto& override : overrides) {
    if (override.parameterName == paramName && override.enabled) {
      return true;
    }
  }
  return false;
}

QVariant NodeParameterSet::getOverrideValue(const QString& paramName) const {
  for (const auto& override : overrides) {
    if (override.parameterName == paramName && override.enabled) {
      return override.value;
    }
  }
  return QVariant();
}

// ParameterPreset implementation

ParameterPreset ParameterPreset::createDefault() {
  ParameterPreset preset;
  preset.id = QUuid::createUuid();
  preset.name = "Default";
  preset.description = "Default parameter configuration";
  preset.category = "default";
  preset.color = "#808080";
  preset.createdAt = QDateTime::currentDateTime();
  preset.modifiedAt = preset.createdAt;
  return preset;
}

ParameterPreset ParameterPreset::createSimulation() {
  ParameterPreset preset;
  preset.id = QUuid::createUuid();
  preset.name = "Simulation";
  preset.description = "Parameters optimized for simulation environment";
  preset.category = "simulation";
  preset.color = "#2196F3";
  preset.shortcut = "Ctrl+1";
  preset.createdAt = QDateTime::currentDateTime();
  preset.modifiedAt = preset.createdAt;

  // Add common simulation overrides
  ParameterOverride simTime;
  simTime.parameterName = "use_sim_time";
  simTime.value = true;
  simTime.description = "Use simulation time from /clock topic";
  preset.globalOverrides.append(simTime);

  return preset;
}

ParameterPreset ParameterPreset::createRealRobot() {
  ParameterPreset preset;
  preset.id = QUuid::createUuid();
  preset.name = "Real Robot";
  preset.description = "Parameters for real hardware deployment";
  preset.category = "production";
  preset.color = "#4CAF50";
  preset.shortcut = "Ctrl+2";
  preset.createdAt = QDateTime::currentDateTime();
  preset.modifiedAt = preset.createdAt;

  // Add common real robot overrides
  ParameterOverride simTime;
  simTime.parameterName = "use_sim_time";
  simTime.value = false;
  simTime.description = "Use real system time";
  preset.globalOverrides.append(simTime);

  return preset;
}

ParameterPreset ParameterPreset::createDebug() {
  ParameterPreset preset;
  preset.id = QUuid::createUuid();
  preset.name = "Debug";
  preset.description = "Parameters for debugging with verbose logging";
  preset.category = "debug";
  preset.color = "#FF9800";
  preset.shortcut = "Ctrl+3";
  preset.createdAt = QDateTime::currentDateTime();
  preset.modifiedAt = preset.createdAt;

  // Add common debug overrides
  ParameterOverride logLevel;
  logLevel.parameterName = "log_level";
  logLevel.value = "debug";
  logLevel.description = "Enable debug logging";
  preset.globalOverrides.append(logLevel);

  return preset;
}

bool ParameterPreset::hasOverrideFor(const QString& nodeName, const QString& paramName) const {
  // Check global overrides
  for (const auto& override : globalOverrides) {
    if (override.parameterName == paramName && override.enabled) {
      return true;
    }
  }

  // Check node-specific overrides
  for (const auto& nodeSet : nodeOverrides) {
    if (nodeSet.nodeName == nodeName) {
      return nodeSet.hasOverride(paramName);
    }
  }

  return false;
}

QVariant ParameterPreset::getOverrideValue(const QString& nodeName, const QString& paramName) const {
  // Node-specific overrides take precedence
  for (const auto& nodeSet : nodeOverrides) {
    if (nodeSet.nodeName == nodeName && nodeSet.hasOverride(paramName)) {
      return nodeSet.getOverrideValue(paramName);
    }
  }

  // Fall back to global overrides
  for (const auto& override : globalOverrides) {
    if (override.parameterName == paramName && override.enabled) {
      return override.value;
    }
  }

  return QVariant();
}

void ParameterPreset::setOverride(const QString& nodeName, const QString& paramName, const QVariant& value) {
  modifiedAt = QDateTime::currentDateTime();

  if (nodeName.isEmpty()) {
    // Global override
    for (auto& override : globalOverrides) {
      if (override.parameterName == paramName) {
        override.value = value;
        override.enabled = true;
        return;
      }
    }
    ParameterOverride newOverride;
    newOverride.parameterName = paramName;
    newOverride.value = value;
    globalOverrides.append(newOverride);
  } else {
    // Node-specific override
    for (auto& nodeSet : nodeOverrides) {
      if (nodeSet.nodeName == nodeName) {
        for (auto& override : nodeSet.overrides) {
          if (override.parameterName == paramName) {
            override.value = value;
            override.enabled = true;
            return;
          }
        }
        ParameterOverride newOverride;
        newOverride.parameterName = paramName;
        newOverride.value = value;
        nodeSet.overrides.append(newOverride);
        return;
      }
    }
    // Create new node set
    NodeParameterSet newNodeSet;
    newNodeSet.nodeName = nodeName;
    ParameterOverride newOverride;
    newOverride.parameterName = paramName;
    newOverride.value = value;
    newNodeSet.overrides.append(newOverride);
    nodeOverrides.append(newNodeSet);
  }
}

void ParameterPreset::removeOverride(const QString& nodeName, const QString& paramName) {
  modifiedAt = QDateTime::currentDateTime();

  if (nodeName.isEmpty()) {
    for (int i = 0; i < globalOverrides.size(); ++i) {
      if (globalOverrides[i].parameterName == paramName) {
        globalOverrides.removeAt(i);
        return;
      }
    }
  } else {
    for (auto& nodeSet : nodeOverrides) {
      if (nodeSet.nodeName == nodeName) {
        for (int i = 0; i < nodeSet.overrides.size(); ++i) {
          if (nodeSet.overrides[i].parameterName == paramName) {
            nodeSet.overrides.removeAt(i);
            return;
          }
        }
      }
    }
  }
}

QStringList ParameterPreset::diffWith(const ParameterPreset& other) const {
  QStringList differences;

  // Compare global overrides
  for (const auto& override : globalOverrides) {
    bool found = false;
    for (const auto& otherOverride : other.globalOverrides) {
      if (override.parameterName == otherOverride.parameterName) {
        found = true;
        if (override.value != otherOverride.value) {
          differences << QString("Global '%1': %2 -> %3")
            .arg(override.parameterName)
            .arg(override.value.toString())
            .arg(otherOverride.value.toString());
        }
        break;
      }
    }
    if (!found) {
      differences << QString("Global '%1': %2 (only in %3)")
        .arg(override.parameterName)
        .arg(override.value.toString())
        .arg(name);
    }
  }

  // Check other's globals not in this
  for (const auto& otherOverride : other.globalOverrides) {
    bool found = false;
    for (const auto& override : globalOverrides) {
      if (override.parameterName == otherOverride.parameterName) {
        found = true;
        break;
      }
    }
    if (!found) {
      differences << QString("Global '%1': %2 (only in %3)")
        .arg(otherOverride.parameterName)
        .arg(otherOverride.value.toString())
        .arg(other.name);
    }
  }

  return differences;
}

QVariantMap ParameterPreset::toVariantMap() const {
  QVariantMap map;
  map["id"] = id.toString();
  map["name"] = name;
  map["description"] = description;
  map["category"] = category;
  map["color"] = color;
  map["shortcut"] = shortcut;
  map["createdAt"] = createdAt.toString(Qt::ISODate);
  map["modifiedAt"] = modifiedAt.toString(Qt::ISODate);

  // Global overrides
  QVariantList globalList;
  for (const auto& override : globalOverrides) {
    QVariantMap ovMap;
    ovMap["parameterName"] = override.parameterName;
    ovMap["value"] = override.value;
    ovMap["enabled"] = override.enabled;
    ovMap["description"] = override.description;
    globalList.append(ovMap);
  }
  map["globalOverrides"] = globalList;

  // Node overrides
  QVariantList nodeList;
  for (const auto& nodeSet : nodeOverrides) {
    QVariantMap nodeMap;
    nodeMap["nodeName"] = nodeSet.nodeName;
    QVariantList overrideList;
    for (const auto& override : nodeSet.overrides) {
      QVariantMap ovMap;
      ovMap["parameterName"] = override.parameterName;
      ovMap["value"] = override.value;
      ovMap["enabled"] = override.enabled;
      ovMap["description"] = override.description;
      overrideList.append(ovMap);
    }
    nodeMap["overrides"] = overrideList;
    nodeList.append(nodeMap);
  }
  map["nodeOverrides"] = nodeList;

  return map;
}

ParameterPreset ParameterPreset::fromVariantMap(const QVariantMap& map) {
  ParameterPreset preset;
  preset.id = QUuid(map.value("id").toString());
  preset.name = map.value("name").toString();
  preset.description = map.value("description").toString();
  preset.category = map.value("category").toString();
  preset.color = map.value("color", "#4CAF50").toString();
  preset.shortcut = map.value("shortcut").toString();
  preset.createdAt = QDateTime::fromString(map.value("createdAt").toString(), Qt::ISODate);
  preset.modifiedAt = QDateTime::fromString(map.value("modifiedAt").toString(), Qt::ISODate);

  // Global overrides
  QVariantList globalList = map.value("globalOverrides").toList();
  for (const QVariant& v : globalList) {
    QVariantMap ovMap = v.toMap();
    ParameterOverride override;
    override.parameterName = ovMap.value("parameterName").toString();
    override.value = ovMap.value("value");
    override.enabled = ovMap.value("enabled", true).toBool();
    override.description = ovMap.value("description").toString();
    preset.globalOverrides.append(override);
  }

  // Node overrides
  QVariantList nodeList = map.value("nodeOverrides").toList();
  for (const QVariant& v : nodeList) {
    QVariantMap nodeMap = v.toMap();
    NodeParameterSet nodeSet;
    nodeSet.nodeName = nodeMap.value("nodeName").toString();
    QVariantList overrideList = nodeMap.value("overrides").toList();
    for (const QVariant& ov : overrideList) {
      QVariantMap ovMap = ov.toMap();
      ParameterOverride override;
      override.parameterName = ovMap.value("parameterName").toString();
      override.value = ovMap.value("value");
      override.enabled = ovMap.value("enabled", true).toBool();
      override.description = ovMap.value("description").toString();
      nodeSet.overrides.append(override);
    }
    preset.nodeOverrides.append(nodeSet);
  }

  return preset;
}

QString ParameterPreset::toYaml() const {
  QString yaml;
  QTextStream stream(&yaml);

  stream << "# Parameter Preset: " << name << "\n";
  stream << "# " << description << "\n";
  stream << "# Category: " << category << "\n\n";

  // Global overrides as /**
  if (!globalOverrides.isEmpty()) {
    stream << "/**:\n";
    stream << "  ros__parameters:\n";
    for (const auto& override : globalOverrides) {
      if (override.enabled) {
        stream << "    " << override.parameterName << ": ";
        if (override.value.type() == QVariant::String) {
          stream << "\"" << override.value.toString() << "\"";
        } else if (override.value.type() == QVariant::Bool) {
          stream << (override.value.toBool() ? "true" : "false");
        } else {
          stream << override.value.toString();
        }
        stream << "\n";
      }
    }
    stream << "\n";
  }

  // Node-specific overrides
  for (const auto& nodeSet : nodeOverrides) {
    if (nodeSet.overrides.isEmpty()) continue;

    stream << nodeSet.nodeName << ":\n";
    stream << "  ros__parameters:\n";
    for (const auto& override : nodeSet.overrides) {
      if (override.enabled) {
        stream << "    " << override.parameterName << ": ";
        if (override.value.type() == QVariant::String) {
          stream << "\"" << override.value.toString() << "\"";
        } else if (override.value.type() == QVariant::Bool) {
          stream << (override.value.toBool() ? "true" : "false");
        } else {
          stream << override.value.toString();
        }
        stream << "\n";
      }
    }
    stream << "\n";
  }

  return yaml;
}

ParameterPreset ParameterPreset::fromYaml(const QString& yaml) {
  // Basic YAML parsing - in production would use yaml-cpp
  ParameterPreset preset;
  preset.id = QUuid::createUuid();
  preset.name = "Imported Preset";
  preset.createdAt = QDateTime::currentDateTime();
  preset.modifiedAt = preset.createdAt;

  // Parse comments for metadata
  QStringList lines = yaml.split('\n');
  for (const QString& line : lines) {
    if (line.startsWith("# Parameter Preset:")) {
      preset.name = line.mid(19).trimmed();
    } else if (line.startsWith("# Category:")) {
      preset.category = line.mid(11).trimmed();
    }
  }

  return preset;
}

bool ParameterPreset::operator==(const ParameterPreset& other) const {
  return id == other.id;
}

// PresetManager implementation

PresetManager& PresetManager::instance() {
  static PresetManager instance;
  return instance;
}

PresetManager::PresetManager(QObject* parent)
  : QObject(parent)
{
}

void PresetManager::addPreset(const ParameterPreset& preset) {
  presets_.append(preset);
  emit presetAdded(preset);
}

void PresetManager::updatePreset(const ParameterPreset& preset) {
  for (int i = 0; i < presets_.size(); ++i) {
    if (presets_[i].id == preset.id) {
      presets_[i] = preset;
      emit presetUpdated(preset);
      return;
    }
  }
}

void PresetManager::removePreset(const QUuid& id) {
  for (int i = 0; i < presets_.size(); ++i) {
    if (presets_[i].id == id) {
      if (activePreset_ && activePreset_->id == id) {
        clearActivePreset();
      }
      presets_.removeAt(i);
      emit presetRemoved(id);
      return;
    }
  }
}

ParameterPreset* PresetManager::findPreset(const QUuid& id) {
  for (int i = 0; i < presets_.size(); ++i) {
    if (presets_[i].id == id) {
      return &presets_[i];
    }
  }
  return nullptr;
}

ParameterPreset* PresetManager::findPresetByName(const QString& name) {
  for (int i = 0; i < presets_.size(); ++i) {
    if (presets_[i].name == name) {
      return &presets_[i];
    }
  }
  return nullptr;
}

void PresetManager::setActivePreset(const QUuid& id) {
  activePreset_ = findPreset(id);
  emit activePresetChanged(activePreset_);
}

void PresetManager::clearActivePreset() {
  activePreset_ = nullptr;
  emit activePresetChanged(nullptr);
}

bool PresetManager::savePreset(const ParameterPreset& preset, const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QJsonDocument doc(QJsonObject::fromVariantMap(preset.toVariantMap()));
  file.write(doc.toJson(QJsonDocument::Indented));
  file.close();
  return true;
}

ParameterPreset PresetManager::loadPreset(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return ParameterPreset();
  }

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  file.close();

  return ParameterPreset::fromVariantMap(doc.object().toVariantMap());
}

bool PresetManager::saveAllPresets(const QString& dirPath) {
  QDir dir(dirPath);
  if (!dir.exists()) {
    dir.mkpath(".");
  }

  for (const auto& preset : presets_) {
    QString fileName = preset.name.toLower().replace(' ', '_') + ".json";
    savePreset(preset, dir.filePath(fileName));
  }

  return true;
}

void PresetManager::loadPresetsFromDir(const QString& dirPath) {
  QDir dir(dirPath);
  QStringList filters;
  filters << "*.json";
  QStringList files = dir.entryList(filters, QDir::Files);

  for (const QString& file : files) {
    ParameterPreset preset = loadPreset(dir.filePath(file));
    if (!preset.id.isNull()) {
      addPreset(preset);
    }
  }
}

void PresetManager::createBuiltInPresets() {
  // Only create if no presets exist
  if (presets_.isEmpty()) {
    addPreset(ParameterPreset::createSimulation());
    addPreset(ParameterPreset::createRealRobot());
    addPreset(ParameterPreset::createDebug());
  }
}

}  // namespace ros_weaver
