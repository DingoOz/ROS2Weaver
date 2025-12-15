#include "ros_weaver/core/undo/commands/macro_command.hpp"

namespace ros_weaver {

MacroCommand::MacroCommand(WeaverCanvas* canvas, const QString& text)
    : UndoCommand(canvas)
    , text_(text) {
}

MacroCommand::~MacroCommand() {
  qDeleteAll(commands_);
  commands_.clear();
}

void MacroCommand::undo() {
  // Undo in reverse order
  for (int i = commands_.size() - 1; i >= 0; --i) {
    commands_[i]->undo();
  }
}

void MacroCommand::redo() {
  // Redo in forward order
  for (UndoCommand* cmd : commands_) {
    cmd->redo();
  }
}

void MacroCommand::addCommand(UndoCommand* command) {
  if (command) {
    commands_.append(command);
  }
}

}  // namespace ros_weaver
