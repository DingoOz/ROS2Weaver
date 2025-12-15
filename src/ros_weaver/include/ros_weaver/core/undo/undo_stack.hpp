#ifndef ROS_WEAVER_UNDO_STACK_HPP
#define ROS_WEAVER_UNDO_STACK_HPP

#include <QObject>
#include <QList>
#include <QString>
#include <memory>

namespace ros_weaver {

class UndoCommand;

// Manages undo/redo command history
// Implements a two-stack approach: undo stack and redo stack
// Supports:
// - Command limit (default 50)
// - Command merging for consecutive similar operations
// - Macro commands for grouping multiple operations
// - Clean state tracking for "has unsaved changes"
class UndoStack : public QObject {
  Q_OBJECT

public:
  explicit UndoStack(QObject* parent = nullptr);
  ~UndoStack() override;

  // Push a new command onto the stack
  // Takes ownership of the command
  // Executes redo() on the command immediately
  // Clears the redo stack
  void push(UndoCommand* command);

  // Undo the last command
  void undo();

  // Redo the last undone command
  void redo();

  // Check if undo is available
  bool canUndo() const;

  // Check if redo is available
  bool canRedo() const;

  // Get the text of the next command to undo
  QString undoText() const;

  // Get the text of the next command to redo
  QString redoText() const;

  // Get number of commands in undo stack
  int count() const;

  // Get current index (position in history)
  int index() const;

  // Set the maximum number of commands to keep
  void setUndoLimit(int limit);
  int undoLimit() const;

  // Clear all commands
  void clear();

  // Macro support for grouping multiple commands
  // All commands pushed between beginMacro/endMacro are treated as one
  void beginMacro(const QString& text);
  void endMacro();
  bool isInMacro() const;

  // Clean state tracking (for "has unsaved changes")
  void setClean();
  bool isClean() const;
  int cleanIndex() const;

signals:
  // Emitted when canUndo() changes
  void canUndoChanged(bool canUndo);

  // Emitted when canRedo() changes
  void canRedoChanged(bool canRedo);

  // Emitted when the undo text changes
  void undoTextChanged(const QString& text);

  // Emitted when the redo text changes
  void redoTextChanged(const QString& text);

  // Emitted when the clean state changes
  void cleanChanged(bool clean);

  // Emitted when the current index changes
  void indexChanged(int index);

private:
  void enforceLimit();
  void emitSignals();

  QList<UndoCommand*> commands_;  // All commands in order
  int currentIndex_ = 0;          // Points to next command to redo
  int cleanIndex_ = 0;            // Index where stack was marked clean
  int undoLimit_ = 50;            // Maximum commands to keep

  // Macro support
  QList<UndoCommand*> macroCommands_;
  QString macroText_;
  bool inMacro_ = false;

  // Cached state for signal emission
  bool lastCanUndo_ = false;
  bool lastCanRedo_ = false;
  QString lastUndoText_;
  QString lastRedoText_;
  bool lastClean_ = true;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_UNDO_STACK_HPP
