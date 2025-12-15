#ifndef ROS_WEAVER_REMOVE_GROUP_COMMAND_HPP
#define ROS_WEAVER_REMOVE_GROUP_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"

namespace ros_weaver {

// Command to remove a node group from the canvas
// Stores the group data for restoration
// Undo: re-adds the group
// Redo: removes the group
class RemoveGroupCommand : public UndoCommand {
public:
  RemoveGroupCommand(WeaverCanvas* canvas, const NodeGroupData& groupData);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  NodeGroupData groupData_;
  bool firstRedo_ = true;  // Skip first redo since group already removed
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_REMOVE_GROUP_COMMAND_HPP
