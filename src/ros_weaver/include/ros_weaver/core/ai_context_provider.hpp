#ifndef ROS_WEAVER_AI_CONTEXT_PROVIDER_HPP
#define ROS_WEAVER_AI_CONTEXT_PROVIDER_HPP

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QPointF>

namespace ros_weaver {

class WeaverCanvas;
class PackageBlock;
class ConnectionLine;
class NodeGroup;
class SystemDiscovery;

// Provides context about the current project state for the AI
class AIContextProvider : public QObject {
  Q_OBJECT

public:
  static AIContextProvider& instance();

  // Set the canvas and system discovery references
  void setCanvas(WeaverCanvas* canvas);
  void setSystemDiscovery(SystemDiscovery* discovery);

  // Get the full project context as a string for the AI system prompt
  QString getProjectContext() const;

  // Get project context as JSON
  QJsonObject getProjectContextJson() const;

  // Get context for a specific element (for "Ask AI about this...")
  QString getBlockContext(PackageBlock* block) const;
  QString getConnectionContext(ConnectionLine* connection) const;
  QString getGroupContext(NodeGroup* group) const;
  QString getPinContext(PackageBlock* block, int pinIndex, bool isOutput) const;
  QString getTopicContext(const QString& topicName) const;
  QString getFrameContext(const QString& frameName) const;

  // Get list of available packages that can be added
  QJsonArray getAvailablePackages() const;

  // Get current blocks on canvas
  QJsonArray getBlocksInfo() const;

  // Get current connections
  QJsonArray getConnectionsInfo() const;

  // Get current groups
  QJsonArray getGroupsInfo() const;

  // Get ROS2 system state (topics, nodes, services)
  QString getROS2SystemState() const;

  // Build a complete system prompt for the AI including project context
  QString buildSystemPrompt() const;

  // Build a context-aware prompt for asking about a specific element
  QString buildElementPrompt(const QString& elementContext,
                             const QString& userQuestion = QString()) const;

private:
  AIContextProvider();
  ~AIContextProvider();
  AIContextProvider(const AIContextProvider&) = delete;
  AIContextProvider& operator=(const AIContextProvider&) = delete;

  QString formatBlockInfo(PackageBlock* block) const;
  QString formatConnectionInfo(ConnectionLine* conn) const;
  QString formatPinInfo(const QString& pinName, const QString& type,
                        const QString& messageType, bool isInput) const;

  WeaverCanvas* canvas_ = nullptr;
  SystemDiscovery* systemDiscovery_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_AI_CONTEXT_PROVIDER_HPP
