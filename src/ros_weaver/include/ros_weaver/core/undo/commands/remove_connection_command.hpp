#ifndef ROS_WEAVER_REMOVE_CONNECTION_COMMAND_HPP
#define ROS_WEAVER_REMOVE_CONNECTION_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"

namespace ros_weaver {

// Command to remove a connection from the canvas
// Stores the connection data for restoration
// Undo: re-adds the connection
// Redo: removes the connection
class RemoveConnectionCommand : public UndoCommand {
public:
  RemoveConnectionCommand(WeaverCanvas* canvas, const ConnectionData& connectionData);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  ConnectionData connectionData_;
  bool firstRedo_ = true;  // Skip first redo since connection already removed
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_REMOVE_CONNECTION_COMMAND_HPP
