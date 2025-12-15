#ifndef ROS_WEAVER_MOVE_BLOCK_COMMAND_HPP
#define ROS_WEAVER_MOVE_BLOCK_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include <QUuid>
#include <QPointF>

namespace ros_weaver {

// Command to move a block on the canvas
// Supports merging consecutive moves of the same block
// Undo: moves block back to original position
// Redo: moves block to new position
class MoveBlockCommand : public UndoCommand {
public:
  MoveBlockCommand(WeaverCanvas* canvas, const QUuid& blockId,
                   const QPointF& oldPos, const QPointF& newPos);

  void undo() override;
  void redo() override;
  QString text() const override;

  CommandId id() const override { return CommandId::MoveBlock; }
  bool mergeWith(const UndoCommand* other) override;

private:
  QUuid blockId_;
  QPointF oldPos_;
  QPointF newPos_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MOVE_BLOCK_COMMAND_HPP
