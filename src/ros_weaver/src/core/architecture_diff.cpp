#include "ros_weaver/core/architecture_diff.hpp"
#include "ros_weaver/core/project.hpp"

#include <QSet>
#include <QFile>

namespace ros_weaver {

// =============================================================================
// DiffItem
// =============================================================================

QColor DiffItem::getTypeColor() const {
  switch (type) {
    case DiffType::NodeAdded:
    case DiffType::ConnectionAdded:
    case DiffType::ParameterAdded:
    case DiffType::GroupAdded:
      return QColor(76, 175, 80);  // Green

    case DiffType::NodeRemoved:
    case DiffType::ConnectionRemoved:
    case DiffType::ParameterRemoved:
    case DiffType::GroupRemoved:
      return QColor(244, 67, 54);  // Red

    case DiffType::NodeModified:
    case DiffType::ConnectionModified:
    case DiffType::ParameterChanged:
    case DiffType::TopicChanged:
    case DiffType::GroupModified:
      return QColor(255, 152, 0);  // Orange
  }
  return QColor(128, 128, 128);
}

QString DiffItem::getTypeIcon() const {
  switch (type) {
    case DiffType::NodeAdded:
    case DiffType::ConnectionAdded:
    case DiffType::ParameterAdded:
    case DiffType::GroupAdded:
      return QString::fromUtf8("\xE2\x9E\x95");  // Plus

    case DiffType::NodeRemoved:
    case DiffType::ConnectionRemoved:
    case DiffType::ParameterRemoved:
    case DiffType::GroupRemoved:
      return QString::fromUtf8("\xE2\x9E\x96");  // Minus

    case DiffType::NodeModified:
    case DiffType::ConnectionModified:
    case DiffType::ParameterChanged:
    case DiffType::TopicChanged:
    case DiffType::GroupModified:
      return QString::fromUtf8("\xE2\x9C\x8F");  // Pencil
  }
  return "?";
}

QString DiffItem::getTypeLabel() const {
  switch (type) {
    case DiffType::NodeAdded: return "Node Added";
    case DiffType::NodeRemoved: return "Node Removed";
    case DiffType::NodeModified: return "Node Modified";
    case DiffType::ConnectionAdded: return "Connection Added";
    case DiffType::ConnectionRemoved: return "Connection Removed";
    case DiffType::ConnectionModified: return "Connection Modified";
    case DiffType::ParameterAdded: return "Parameter Added";
    case DiffType::ParameterRemoved: return "Parameter Removed";
    case DiffType::ParameterChanged: return "Parameter Changed";
    case DiffType::TopicChanged: return "Topic Changed";
    case DiffType::GroupAdded: return "Group Added";
    case DiffType::GroupRemoved: return "Group Removed";
    case DiffType::GroupModified: return "Group Modified";
  }
  return "Unknown";
}

QString DiffItem::getSeverityIcon() const {
  switch (severity) {
    case DiffSeverity::Info:
      return QString::fromUtf8("\xE2\x84\xB9");  // Info
    case DiffSeverity::Warning:
      return QString::fromUtf8("\xE2\x9A\xA0");  // Warning
    case DiffSeverity::Critical:
      return QString::fromUtf8("\xE2\x9D\x97");  // Exclamation
  }
  return "?";
}

// =============================================================================
// DiffResult
// =============================================================================

int DiffResult::addedCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.type == DiffType::NodeAdded ||
        item.type == DiffType::ConnectionAdded ||
        item.type == DiffType::ParameterAdded ||
        item.type == DiffType::GroupAdded) {
      ++count;
    }
  }
  return count;
}

int DiffResult::removedCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.type == DiffType::NodeRemoved ||
        item.type == DiffType::ConnectionRemoved ||
        item.type == DiffType::ParameterRemoved ||
        item.type == DiffType::GroupRemoved) {
      ++count;
    }
  }
  return count;
}

int DiffResult::modifiedCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.type == DiffType::NodeModified ||
        item.type == DiffType::ConnectionModified ||
        item.type == DiffType::ParameterChanged ||
        item.type == DiffType::TopicChanged ||
        item.type == DiffType::GroupModified) {
      ++count;
    }
  }
  return count;
}

int DiffResult::infoCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.severity == DiffSeverity::Info) ++count;
  }
  return count;
}

int DiffResult::warningCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.severity == DiffSeverity::Warning) ++count;
  }
  return count;
}

int DiffResult::criticalCount() const {
  int count = 0;
  for (const auto& item : items) {
    if (item.severity == DiffSeverity::Critical) ++count;
  }
  return count;
}

QList<DiffItem> DiffResult::itemsByType(DiffType type) const {
  QList<DiffItem> result;
  for (const auto& item : items) {
    if (item.type == type) {
      result.append(item);
    }
  }
  return result;
}

QList<DiffItem> DiffResult::itemsBySeverity(DiffSeverity severity) const {
  QList<DiffItem> result;
  for (const auto& item : items) {
    if (item.severity == severity) {
      result.append(item);
    }
  }
  return result;
}

QList<DiffItem> DiffResult::itemsByCategory(const QString& category) const {
  QList<DiffItem> result;
  for (const auto& item : items) {
    if (item.category == category) {
      result.append(item);
    }
  }
  return result;
}

// =============================================================================
// ArchitectureDiff
// =============================================================================

DiffResult ArchitectureDiff::compare(const Project* before, const Project* after) {
  DiffResult result;
  result.beforeLabel = before ? before->metadata().name : "Before";
  result.afterLabel = after ? after->metadata().name : "After";

  if (!before || !after) {
    return result;
  }

  compareBlocks(before, after, result);
  compareConnections(before, after, result);
  compareGroups(before, after, result);

  return result;
}

DiffResult ArchitectureDiff::compareWithLiveSystem(const Project* project) {
  DiffResult result;
  result.beforeLabel = "Canvas Design";
  result.afterLabel = "Running System";

  // This would require integration with SystemDiscovery
  // For now, return empty result
  // TODO: Implement live system comparison

  return result;
}

DiffResult ArchitectureDiff::compareFiles(const QString& beforePath, const QString& afterPath) {
  QString errorMsg;
  Project beforeProject = Project::loadFromFile(beforePath, &errorMsg);
  if (!errorMsg.isEmpty()) {
    return DiffResult();
  }

  Project afterProject = Project::loadFromFile(afterPath, &errorMsg);
  if (!errorMsg.isEmpty()) {
    return DiffResult();
  }

  return compare(&beforeProject, &afterProject);
}

void ArchitectureDiff::compareBlocks(const Project* before, const Project* after, DiffResult& result) {
  QMap<QString, BlockData> beforeBlocks;
  QMap<QString, BlockData> afterBlocks;

  for (const auto& block : before->blocks()) {
    beforeBlocks[block.id.toString()] = block;
  }
  for (const auto& block : after->blocks()) {
    afterBlocks[block.id.toString()] = block;
  }

  // Find added and modified blocks
  for (auto it = afterBlocks.begin(); it != afterBlocks.end(); ++it) {
    const QString& id = it.key();
    const BlockData& afterBlock = it.value();

    if (!beforeBlocks.contains(id)) {
      // Block added
      DiffItem item;
      item.type = DiffType::NodeAdded;
      item.severity = DiffSeverity::Info;
      item.elementId = id;
      item.elementName = afterBlock.name;
      item.description = QString("Node '%1' was added").arg(afterBlock.name);
      item.category = "Nodes";
      result.items.append(item);
    } else {
      // Block exists in both - check for modifications
      const BlockData& beforeBlock = beforeBlocks[id];

      // Check name change
      if (beforeBlock.name != afterBlock.name) {
        DiffItem item;
        item.type = DiffType::NodeModified;
        item.severity = DiffSeverity::Warning;
        item.elementId = id;
        item.elementName = afterBlock.name;
        item.description = QString("Node renamed from '%1' to '%2'")
          .arg(beforeBlock.name).arg(afterBlock.name);
        item.beforeValue = beforeBlock.name;
        item.afterValue = afterBlock.name;
        item.category = "Nodes";
        result.items.append(item);
      }

      // Check parameters
      compareParameters(afterBlock.name, beforeBlock.parameters, afterBlock.parameters, result);
    }
  }

  // Find removed blocks
  for (auto it = beforeBlocks.begin(); it != beforeBlocks.end(); ++it) {
    const QString& id = it.key();
    const BlockData& beforeBlock = it.value();

    if (!afterBlocks.contains(id)) {
      DiffItem item;
      item.type = DiffType::NodeRemoved;
      item.severity = DiffSeverity::Critical;
      item.elementId = id;
      item.elementName = beforeBlock.name;
      item.description = QString("Node '%1' was removed").arg(beforeBlock.name);
      item.category = "Nodes";
      result.items.append(item);
    }
  }
}

void ArchitectureDiff::compareConnections(const Project* before, const Project* after, DiffResult& result) {
  // Create connection signatures for comparison
  auto makeSignature = [](const ConnectionData& conn) {
    return QString("%1:%2->%3:%4")
      .arg(conn.sourceBlockId.toString())
      .arg(conn.sourcePinIndex)
      .arg(conn.targetBlockId.toString())
      .arg(conn.targetPinIndex);
  };

  QMap<QString, ConnectionData> beforeConns;
  QMap<QString, ConnectionData> afterConns;

  for (const auto& conn : before->connections()) {
    beforeConns[makeSignature(conn)] = conn;
  }
  for (const auto& conn : after->connections()) {
    afterConns[makeSignature(conn)] = conn;
  }

  // Find added connections
  for (auto it = afterConns.begin(); it != afterConns.end(); ++it) {
    if (!beforeConns.contains(it.key())) {
      const ConnectionData& conn = it.value();
      DiffItem item;
      item.type = DiffType::ConnectionAdded;
      item.severity = DiffSeverity::Info;
      item.elementId = conn.id.toString();
      item.elementName = QString("%1 -> %2").arg(conn.sourcePinIndex).arg(conn.targetPinIndex);
      item.description = QString("Connection added: %1:%2 -> %3:%4")
        .arg(conn.sourceBlockId.toString().left(8))
        .arg(conn.sourcePinIndex)
        .arg(conn.targetBlockId.toString().left(8))
        .arg(conn.targetPinIndex);
      item.category = "Connections";
      result.items.append(item);
    }
  }

  // Find removed connections
  for (auto it = beforeConns.begin(); it != beforeConns.end(); ++it) {
    if (!afterConns.contains(it.key())) {
      const ConnectionData& conn = it.value();
      DiffItem item;
      item.type = DiffType::ConnectionRemoved;
      item.severity = DiffSeverity::Critical;
      item.elementId = conn.id.toString();
      item.elementName = QString("%1 -> %2").arg(conn.sourcePinIndex).arg(conn.targetPinIndex);
      item.description = QString("Connection removed: %1:%2 -> %3:%4")
        .arg(conn.sourceBlockId.toString().left(8))
        .arg(conn.sourcePinIndex)
        .arg(conn.targetBlockId.toString().left(8))
        .arg(conn.targetPinIndex);
      item.category = "Connections";
      result.items.append(item);
    }
  }
}

void ArchitectureDiff::compareGroups(const Project* before, const Project* after, DiffResult& result) {
  QMap<QString, NodeGroupData> beforeGroups;
  QMap<QString, NodeGroupData> afterGroups;

  for (const auto& grp : before->nodeGroups()) {
    beforeGroups[grp.id.toString()] = grp;
  }
  for (const auto& grp : after->nodeGroups()) {
    afterGroups[grp.id.toString()] = grp;
  }

  // Find added groups
  for (auto it = afterGroups.begin(); it != afterGroups.end(); ++it) {
    if (!beforeGroups.contains(it.key())) {
      const NodeGroupData& grp = it.value();
      DiffItem item;
      item.type = DiffType::GroupAdded;
      item.severity = DiffSeverity::Info;
      item.elementId = it.key();
      item.elementName = grp.title;
      item.description = QString("Group '%1' was added with %2 blocks")
        .arg(grp.title).arg(grp.containedNodeIds.size());
      item.category = "Groups";
      result.items.append(item);
    }
  }

  // Find removed groups
  for (auto it = beforeGroups.begin(); it != beforeGroups.end(); ++it) {
    if (!afterGroups.contains(it.key())) {
      const NodeGroupData& grp = it.value();
      DiffItem item;
      item.type = DiffType::GroupRemoved;
      item.severity = DiffSeverity::Warning;
      item.elementId = it.key();
      item.elementName = grp.title;
      item.description = QString("Group '%1' was removed").arg(grp.title);
      item.category = "Groups";
      result.items.append(item);
    }
  }
}

void ArchitectureDiff::compareParameters(const QString& nodeName,
                                         const QList<BlockParamData>& before,
                                         const QList<BlockParamData>& after,
                                         DiffResult& result) {
  QMap<QString, BlockParamData> beforeParams;
  QMap<QString, BlockParamData> afterParams;

  for (const auto& param : before) {
    beforeParams[param.name] = param;
  }
  for (const auto& param : after) {
    afterParams[param.name] = param;
  }

  // Find added and changed parameters
  for (auto it = afterParams.begin(); it != afterParams.end(); ++it) {
    const QString& name = it.key();
    const BlockParamData& afterParam = it.value();

    if (!beforeParams.contains(name)) {
      DiffItem item;
      item.type = DiffType::ParameterAdded;
      item.severity = DiffSeverity::Info;
      item.elementId = nodeName + "/" + name;
      item.elementName = QString("%1/%2").arg(nodeName).arg(name);
      item.description = QString("Parameter '%1' added to node '%2'").arg(name).arg(nodeName);
      item.afterValue = afterParam.currentValue.toString();
      item.category = "Parameters";
      result.items.append(item);
    } else {
      const BlockParamData& beforeParam = beforeParams[name];
      if (beforeParam.currentValue != afterParam.currentValue) {
        if (ignoreMinorParams_ && name.startsWith("_")) {
          continue;  // Skip internal parameters
        }

        DiffItem item;
        item.type = DiffType::ParameterChanged;
        item.severity = DiffSeverity::Warning;
        item.elementId = nodeName + "/" + name;
        item.elementName = QString("%1/%2").arg(nodeName).arg(name);
        item.description = QString("Parameter '%1' changed in node '%2'").arg(name).arg(nodeName);
        item.beforeValue = beforeParam.currentValue.toString();
        item.afterValue = afterParam.currentValue.toString();
        item.category = "Parameters";
        result.items.append(item);
      }
    }
  }

  // Find removed parameters
  for (auto it = beforeParams.begin(); it != beforeParams.end(); ++it) {
    if (!afterParams.contains(it.key())) {
      const QString& name = it.key();
      const BlockParamData& beforeParam = it.value();

      DiffItem item;
      item.type = DiffType::ParameterRemoved;
      item.severity = DiffSeverity::Warning;
      item.elementId = nodeName + "/" + name;
      item.elementName = QString("%1/%2").arg(nodeName).arg(name);
      item.description = QString("Parameter '%1' removed from node '%2'").arg(name).arg(nodeName);
      item.beforeValue = beforeParam.currentValue.toString();
      item.category = "Parameters";
      result.items.append(item);
    }
  }
}

DiffSeverity ArchitectureDiff::determineSeverity(DiffType type, const QString& elementName) {
  Q_UNUSED(elementName)

  switch (type) {
    case DiffType::NodeAdded:
    case DiffType::ConnectionAdded:
    case DiffType::ParameterAdded:
    case DiffType::GroupAdded:
      return DiffSeverity::Info;

    case DiffType::NodeRemoved:
    case DiffType::ConnectionRemoved:
      return DiffSeverity::Critical;

    case DiffType::ParameterRemoved:
    case DiffType::GroupRemoved:
    case DiffType::NodeModified:
    case DiffType::ConnectionModified:
    case DiffType::ParameterChanged:
    case DiffType::TopicChanged:
    case DiffType::GroupModified:
      return DiffSeverity::Warning;
  }

  return DiffSeverity::Info;
}

}  // namespace ros_weaver
