#ifndef ROS_WEAVER_AI_TOOL_MANAGER_HPP
#define ROS_WEAVER_AI_TOOL_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QJsonObject>
#include <QJsonArray>
#include <QVariant>
#include <QUuid>
#include <QStack>
#include <functional>
#include <memory>

namespace ros_weaver {

class WeaverCanvas;
class PackageBlock;

// Represents an action that can be undone
struct AIAction {
  QString description;
  QString toolName;
  QJsonObject parameters;
  QJsonObject undoData;  // Data needed to undo this action
  QDateTime timestamp;
  bool approved = false;
};

// Result of a tool execution
struct AIToolResult {
  bool success = false;
  QString message;
  QJsonObject data;
  QString undoDescription;  // Human-readable description of what was changed
};

// Represents a tool that the AI can call
struct AITool {
  QString name;
  QString description;
  QJsonObject parametersSchema;  // JSON Schema for parameters
  std::function<AIToolResult(const QJsonObject& params)> execute;
  bool requiresPermission = true;  // Whether this tool needs user approval
};

// Manages AI tools and their execution with permission handling
class AIToolManager : public QObject {
  Q_OBJECT

public:
  static AIToolManager& instance();

  // Register tools - called during initialization
  void registerTools(WeaverCanvas* canvas);

  // Get available tools for the AI (returns JSON schema description)
  QString getToolsDescription() const;
  QJsonArray getToolsSchema() const;

  // Parse AI response for tool calls
  // Returns list of tool calls found in the response
  QList<QPair<QString, QJsonObject>> parseToolCalls(const QString& response) const;

  // Execute a tool (will trigger permission dialog if needed)
  // Returns the result of the tool execution
  AIToolResult executeTool(const QString& toolName, const QJsonObject& params);

  // Permission handling
  bool isAutoApproveEnabled() const { return autoApproveSession_; }
  void setAutoApproveSession(bool enabled) { autoApproveSession_ = enabled; }
  void resetSessionApproval() { autoApproveSession_ = false; }

  // Undo support
  bool canUndo() const { return !undoStack_.isEmpty(); }
  QString lastActionDescription() const;
  void undoLastAction();
  void clearUndoStack();
  int undoStackSize() const { return undoStack_.size(); }

  // Get available tools list
  QStringList availableTools() const;

  // Get tool by name
  const AITool* getTool(const QString& name) const;

signals:
  // Emitted when a tool requires permission
  void permissionRequired(const QString& toolName, const QString& description,
                          const QJsonObject& params);
  // Emitted when a tool is executed
  void toolExecuted(const QString& toolName, bool success, const QString& message);
  // Emitted when an action is undone
  void actionUndone(const QString& description);
  // Emitted when the undo stack changes
  void undoStackChanged(int size);
  // Emitted when auto-approve state changes
  void autoApproveChanged(bool enabled);

public slots:
  // Called when permission is granted/denied
  void onPermissionGranted(const QString& toolName, const QJsonObject& params,
                           bool approveAll);
  void onPermissionDenied(const QString& toolName);

private:
  AIToolManager();
  ~AIToolManager();
  AIToolManager(const AIToolManager&) = delete;
  AIToolManager& operator=(const AIToolManager&) = delete;

  // Register individual tools
  void registerLoadExampleTool();
  void registerAddBlockTool();
  void registerRemoveBlockTool();
  void registerSetParameterTool();
  void registerCreateConnectionTool();
  void registerRemoveConnectionTool();
  void registerCreateGroupTool();
  void registerGetProjectStateTool();
  void registerGetBlockInfoTool();
  void registerListAvailablePackagesTool();

  // Tool registry
  QMap<QString, AITool> tools_;

  // Canvas reference (set during registration)
  WeaverCanvas* canvas_ = nullptr;

  // Undo stack
  QStack<AIAction> undoStack_;

  // Session auto-approve flag
  bool autoApproveSession_ = false;

  // Pending tool execution (waiting for permission)
  QString pendingToolName_;
  QJsonObject pendingParams_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_AI_TOOL_MANAGER_HPP
