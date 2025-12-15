#include "ros_weaver/core/undo/undo_command.hpp"

namespace ros_weaver {

UndoCommand::UndoCommand(WeaverCanvas* canvas)
    : canvas_(canvas)
    , timestamp_(QDateTime::currentDateTime()) {
}

}  // namespace ros_weaver
