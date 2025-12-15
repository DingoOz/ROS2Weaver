#include "ros_weaver/core/undo/commands/move_block_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

namespace ros_weaver {

MoveBlockCommand::MoveBlockCommand(WeaverCanvas* canvas, const QUuid& blockId,
                                   const QPointF& oldPos, const QPointF& newPos)
    : UndoCommand(canvas)
    , blockId_(blockId)
    , oldPos_(oldPos)
    , newPos_(newPos) {
}

void MoveBlockCommand::undo() {
  // Find the block by ID and move it back to original position
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      if (block->id() == blockId_) {
        block->setPos(oldPos_);
        // Connections are automatically updated via itemChange when position changes
        return;
      }
    }
  }
}

void MoveBlockCommand::redo() {
  // Find the block by ID and move it to new position
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      if (block->id() == blockId_) {
        block->setPos(newPos_);
        // Connections are automatically updated via itemChange when position changes
        return;
      }
    }
  }
}

QString MoveBlockCommand::text() const {
  return QObject::tr("Move Block");
}

bool MoveBlockCommand::mergeWith(const UndoCommand* other) {
  // Only merge with another MoveBlockCommand for the same block
  const MoveBlockCommand* moveCmd = dynamic_cast<const MoveBlockCommand*>(other);
  if (!moveCmd) {
    return false;
  }

  // Only merge if it's the same block
  if (moveCmd->blockId_ != blockId_) {
    return false;
  }

  // Only merge if commands are recent (within 500ms)
  qint64 timeDiff = timestamp_.msecsTo(moveCmd->timestamp_);
  if (timeDiff > 500) {
    return false;
  }

  // Merge: keep our old position, take the new command's new position
  newPos_ = moveCmd->newPos_;
  return true;
}

}  // namespace ros_weaver
