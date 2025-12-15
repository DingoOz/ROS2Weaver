#ifndef ROS_WEAVER_ADD_GROUP_COMMAND_HPP
#define ROS_WEAVER_ADD_GROUP_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"

namespace ros_weaver {

// Command to add a node group to the canvas
// Undo: removes the group
// Redo: re-adds the group with the same data
class AddGroupCommand : public UndoCommand {
public:
  AddGroupCommand(WeaverCanvas* canvas, const NodeGroupData& groupData);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  NodeGroupData groupData_;
  bool firstRedo_ = true;  // Skip first redo since group already exists
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_ADD_GROUP_COMMAND_HPP
