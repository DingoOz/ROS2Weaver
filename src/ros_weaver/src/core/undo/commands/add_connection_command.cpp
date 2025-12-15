#include "ros_weaver/core/undo/commands/add_connection_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

namespace ros_weaver {

AddConnectionCommand::AddConnectionCommand(WeaverCanvas* canvas,
                                           const ConnectionData& connectionData)
    : UndoCommand(canvas)
    , connectionData_(connectionData) {
}

void AddConnectionCommand::undo() {
  // Find and remove the connection by ID
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (ConnectionLine* conn = dynamic_cast<ConnectionLine*>(item)) {
      if (conn->id() == connectionData_.id) {
        canvas_->removeConnection(conn);
        return;
      }
    }
  }
}

void AddConnectionCommand::redo() {
  // Skip first redo since connection was already added when command was created
  if (firstRedo_) {
    firstRedo_ = false;
    return;
  }

  // Find source and target blocks by ID
  PackageBlock* sourceBlock = nullptr;
  PackageBlock* targetBlock = nullptr;

  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      if (block->id() == connectionData_.sourceBlockId) {
        sourceBlock = block;
      }
      if (block->id() == connectionData_.targetBlockId) {
        targetBlock = block;
      }
      if (sourceBlock && targetBlock) {
        break;
      }
    }
  }

  if (sourceBlock && targetBlock) {
    // Create connection using pin indices
    ConnectionLine* conn = canvas_->createConnection(
        sourceBlock, connectionData_.sourcePinIndex,
        targetBlock, connectionData_.targetPinIndex);
    if (conn) {
      // Restore the original ID
      conn->setId(connectionData_.id);
    }
  }
}

QString AddConnectionCommand::text() const {
  return QObject::tr("Add Connection");
}

}  // namespace ros_weaver
