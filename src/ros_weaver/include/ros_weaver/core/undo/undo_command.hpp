#ifndef ROS_WEAVER_UNDO_COMMAND_HPP
#define ROS_WEAVER_UNDO_COMMAND_HPP

#include <QString>
#include <QDateTime>
#include "ros_weaver/core/undo/command_ids.hpp"

namespace ros_weaver {

// Forward declarations
class WeaverCanvas;

// Base class for all undoable commands
// Implements the Command pattern for undo/redo functionality
class UndoCommand {
public:
  explicit UndoCommand(WeaverCanvas* canvas);
  virtual ~UndoCommand() = default;

  // Execute the undo operation (restore previous state)
  virtual void undo() = 0;

  // Execute the redo operation (reapply the change)
  virtual void redo() = 0;

  // Get human-readable description of the command
  virtual QString text() const = 0;

  // Get command ID for merging consecutive commands
  // Commands with same non-negative ID can potentially be merged
  // Default returns None (-1) which means no merging
  virtual CommandId id() const { return CommandId::None; }

  // Try to merge this command with another command
  // Returns true if merge was successful
  // Called when a new command has the same id() as the top of the stack
  virtual bool mergeWith(const UndoCommand* other) {
    Q_UNUSED(other);
    return false;
  }

  // Get timestamp when command was created
  QDateTime timestamp() const { return timestamp_; }

  // Check if command is obsolete (referenced objects no longer exist)
  virtual bool isObsolete() const { return false; }

  // Accessor for canvas (needed by UndoStack for macro creation)
  WeaverCanvas* canvas() const { return canvas_; }

protected:
  WeaverCanvas* canvas_;
  QDateTime timestamp_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_UNDO_COMMAND_HPP
