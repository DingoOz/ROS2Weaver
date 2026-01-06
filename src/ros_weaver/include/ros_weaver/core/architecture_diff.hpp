#ifndef ROS_WEAVER_CORE_ARCHITECTURE_DIFF_HPP
#define ROS_WEAVER_CORE_ARCHITECTURE_DIFF_HPP

#include <QString>
#include <QList>
#include <QUuid>
#include <QVariant>
#include <QColor>

namespace ros_weaver {

class Project;

/**
 * @brief Type of difference between architectures
 */
enum class DiffType {
  NodeAdded,       // Node exists only in "after"
  NodeRemoved,     // Node exists only in "before"
  NodeModified,    // Node exists in both but differs
  ConnectionAdded,
  ConnectionRemoved,
  ConnectionModified,
  ParameterAdded,
  ParameterRemoved,
  ParameterChanged,
  TopicChanged,    // Topic type or QoS changed
  GroupAdded,
  GroupRemoved,
  GroupModified
};

/**
 * @brief Severity/importance of a difference
 */
enum class DiffSeverity {
  Info,      // Minor change (e.g., position moved)
  Warning,   // Moderate change (e.g., parameter changed)
  Critical   // Breaking change (e.g., connection removed)
};

/**
 * @brief A single difference item
 */
struct DiffItem {
  DiffType type;
  DiffSeverity severity;
  QString elementId;       // ID of the affected element
  QString elementName;     // Human-readable name
  QString description;     // Detailed description of the change
  QString beforeValue;     // Previous value (for modifications)
  QString afterValue;      // New value (for modifications)
  QString category;        // Grouping category (nodes, connections, parameters)

  // For UI
  QColor getTypeColor() const;
  QString getTypeIcon() const;
  QString getTypeLabel() const;
  QString getSeverityIcon() const;
};

/**
 * @brief Result of comparing two architectures
 */
struct DiffResult {
  QString beforeLabel;     // Label for "before" state
  QString afterLabel;      // Label for "after" state
  QList<DiffItem> items;   // All differences

  // Counts by type
  int addedCount() const;
  int removedCount() const;
  int modifiedCount() const;

  // Counts by severity
  int infoCount() const;
  int warningCount() const;
  int criticalCount() const;

  // Filtering
  QList<DiffItem> itemsByType(DiffType type) const;
  QList<DiffItem> itemsBySeverity(DiffSeverity severity) const;
  QList<DiffItem> itemsByCategory(const QString& category) const;

  bool isEmpty() const { return items.isEmpty(); }
  bool hasCriticalChanges() const { return criticalCount() > 0; }
};

/**
 * @brief Engine for computing architecture differences
 */
class ArchitectureDiff {
public:
  ArchitectureDiff() = default;

  /**
   * @brief Compare two projects
   */
  DiffResult compare(const Project* before, const Project* after);

  /**
   * @brief Compare a project with the live running system
   */
  DiffResult compareWithLiveSystem(const Project* project);

  /**
   * @brief Compare two project files
   */
  DiffResult compareFiles(const QString& beforePath, const QString& afterPath);

  /**
   * @brief Settings
   */
  void setIgnorePositionChanges(bool ignore) { ignorePositions_ = ignore; }
  void setIgnoreMinorParameterChanges(bool ignore) { ignoreMinorParams_ = ignore; }

private:
  void compareBlocks(const Project* before, const Project* after, DiffResult& result);
  void compareConnections(const Project* before, const Project* after, DiffResult& result);
  void compareGroups(const Project* before, const Project* after, DiffResult& result);
  void compareParameters(const QString& nodeName,
                        const QList<struct BlockParamData>& before,
                        const QList<struct BlockParamData>& after,
                        DiffResult& result);

  DiffSeverity determineSeverity(DiffType type, const QString& elementName);

  bool ignorePositions_ = true;
  bool ignoreMinorParams_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ARCHITECTURE_DIFF_HPP
