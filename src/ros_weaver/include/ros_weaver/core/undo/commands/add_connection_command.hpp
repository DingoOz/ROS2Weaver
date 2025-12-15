#ifndef ROS_WEAVER_ADD_CONNECTION_COMMAND_HPP
#define ROS_WEAVER_ADD_CONNECTION_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"

namespace ros_weaver {

// Command to add a connection between two pins
// Undo: removes the connection
// Redo: re-adds the connection with the same data
class AddConnectionCommand : public UndoCommand {
public:
  AddConnectionCommand(WeaverCanvas* canvas, const ConnectionData& connectionData);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  ConnectionData connectionData_;
  bool firstRedo_ = true;  // Skip first redo since connection already exists
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_ADD_CONNECTION_COMMAND_HPP
