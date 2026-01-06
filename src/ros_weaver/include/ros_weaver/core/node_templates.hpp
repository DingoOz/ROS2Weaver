#ifndef ROS_WEAVER_CORE_NODE_TEMPLATES_HPP
#define ROS_WEAVER_CORE_NODE_TEMPLATES_HPP

#include <QString>
#include <QList>
#include <QMap>
#include <QVariant>
#include <QUuid>

namespace ros_weaver {

/**
 * @brief Pin definition for a node template
 */
struct TemplatePinDef {
  QString name;
  QString messageType;
  bool isOutput;
  QString description;
};

/**
 * @brief Parameter definition for a node template
 */
struct TemplateParamDef {
  QString name;
  QVariant defaultValue;
  QString type;  // "int", "double", "string", "bool", "array"
  QString description;
  bool required = false;
};

/**
 * @brief Category for organizing templates
 */
enum class TemplateCategory {
  Basic,           // Simple pub/sub nodes
  SensorFusion,    // Multi-input fusion nodes
  StateMachine,    // State machine patterns
  BehaviorTree,    // BT executor nodes
  Bridge,          // ROS1-ROS2 bridge
  Lifecycle,       // Managed lifecycle nodes
  Component,       // Composable nodes
  Navigation,      // Nav2 related
  Perception,      // Perception pipeline
  Control,         // Control nodes
  Custom           // User-defined
};

// Hash function for TemplateCategory to use with QSet/QHash
inline uint qHash(TemplateCategory key, uint seed = 0) {
  return ::qHash(static_cast<int>(key), seed);
}

/**
 * @brief A node template definition
 */
struct NodeTemplate {
  QUuid id;
  QString name;
  QString displayName;
  QString description;
  TemplateCategory category;
  QString iconName;

  // Node structure
  QList<TemplatePinDef> inputPins;
  QList<TemplatePinDef> outputPins;
  QList<TemplateParamDef> parameters;

  // Code generation hints
  QString baseClass;           // e.g., "rclcpp::Node", "rclcpp_lifecycle::LifecycleNode"
  QStringList requiredPackages;
  QString codeTemplate;        // Optional C++ skeleton

  // Metadata
  QString author;
  QString version;
  QStringList tags;

  // Factory methods
  static NodeTemplate createBasicPublisher();
  static NodeTemplate createBasicSubscriber();
  static NodeTemplate createSensorFusion();
  static NodeTemplate createStateMachine();
  static NodeTemplate createBehaviorTreeExecutor();
  static NodeTemplate createRosBridge();
  static NodeTemplate createLifecycleNode();
  static NodeTemplate createComponentNode();
  static NodeTemplate createNav2Controller();
  static NodeTemplate createImageProcessor();

  // Utility
  static QString categoryToString(TemplateCategory category);
  static TemplateCategory stringToCategory(const QString& str);
  static QString categoryToIcon(TemplateCategory category);
};

/**
 * @brief Manager for node templates
 */
class NodeTemplatesManager : public QObject {
  Q_OBJECT

public:
  static NodeTemplatesManager& instance();

  // Template access
  QList<NodeTemplate> allTemplates() const { return templates_; }
  QList<NodeTemplate> templatesByCategory(TemplateCategory category) const;
  NodeTemplate* findTemplate(const QUuid& id);
  NodeTemplate* findTemplateByName(const QString& name);

  // Template management
  void addTemplate(const NodeTemplate& tmpl);
  void removeTemplate(const QUuid& id);
  void updateTemplate(const NodeTemplate& tmpl);

  // Categories
  QList<TemplateCategory> categories() const;
  int templateCountForCategory(TemplateCategory category) const;

  // Search
  QList<NodeTemplate> searchTemplates(const QString& query) const;

  // Serialization
  bool saveCustomTemplates(const QString& filePath);
  bool loadCustomTemplates(const QString& filePath);

  // Initialize built-in templates
  void createBuiltInTemplates();

signals:
  void templateAdded(const NodeTemplate& tmpl);
  void templateRemoved(const QUuid& id);
  void templateUpdated(const NodeTemplate& tmpl);
  void templatesChanged();

private:
  NodeTemplatesManager(QObject* parent = nullptr);

  QList<NodeTemplate> templates_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_NODE_TEMPLATES_HPP
