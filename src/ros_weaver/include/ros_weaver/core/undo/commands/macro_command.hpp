#ifndef ROS_WEAVER_MACRO_COMMAND_HPP
#define ROS_WEAVER_MACRO_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include <QList>

namespace ros_weaver {

// A compound command that groups multiple commands as one undo/redo unit
// Used for operations like "Delete Selected" which may delete multiple items
class MacroCommand : public UndoCommand {
public:
  MacroCommand(WeaverCanvas* canvas, const QString& text);
  ~MacroCommand() override;

  void undo() override;
  void redo() override;
  QString text() const override { return text_; }

  // Add a child command (takes ownership)
  void addCommand(UndoCommand* command);

  // Get number of child commands
  int childCount() const { return commands_.size(); }

  // Check if macro has any commands
  bool isEmpty() const { return commands_.isEmpty(); }

private:
  QString text_;
  QList<UndoCommand*> commands_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MACRO_COMMAND_HPP
