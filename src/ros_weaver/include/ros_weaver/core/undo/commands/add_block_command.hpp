#ifndef ROS_WEAVER_ADD_BLOCK_COMMAND_HPP
#define ROS_WEAVER_ADD_BLOCK_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"

namespace ros_weaver {

// Command to add a block to the canvas
// Undo: removes the block
// Redo: re-adds the block with the same data
class AddBlockCommand : public UndoCommand {
public:
  AddBlockCommand(WeaverCanvas* canvas, const BlockData& blockData);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  BlockData blockData_;
  bool firstRedo_ = true;  // Skip first redo since block already exists
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_ADD_BLOCK_COMMAND_HPP
