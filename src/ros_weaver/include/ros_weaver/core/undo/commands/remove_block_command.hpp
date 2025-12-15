#ifndef ROS_WEAVER_REMOVE_BLOCK_COMMAND_HPP
#define ROS_WEAVER_REMOVE_BLOCK_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"
#include <QList>

namespace ros_weaver {

// Command to remove a block from the canvas
// Stores the block data and all associated connections for restoration
// Undo: re-adds the block and its connections
// Redo: removes the block and its connections
class RemoveBlockCommand : public UndoCommand {
public:
  RemoveBlockCommand(WeaverCanvas* canvas, const BlockData& blockData,
                     const QList<ConnectionData>& connections);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  BlockData blockData_;
  QList<ConnectionData> connections_;  // Connections that were attached to this block
  bool firstRedo_ = true;  // Skip first redo since block already removed
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_REMOVE_BLOCK_COMMAND_HPP
