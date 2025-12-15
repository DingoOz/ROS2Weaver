#include "ros_weaver/core/undo/commands/remove_block_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

namespace ros_weaver {

RemoveBlockCommand::RemoveBlockCommand(WeaverCanvas* canvas,
                                       const BlockData& blockData,
                                       const QList<ConnectionData>& connections)
    : UndoCommand(canvas)
    , blockData_(blockData)
    , connections_(connections) {
}

void RemoveBlockCommand::undo() {
  // Re-add the block with original data
  QList<QPair<QString, QString>> inputPins;
  for (const PinData& pin : blockData_.inputPins) {
    inputPins.append({pin.name, pin.messageType});
  }

  QList<QPair<QString, QString>> outputPins;
  for (const PinData& pin : blockData_.outputPins) {
    outputPins.append({pin.name, pin.messageType});
  }

  PackageBlock* block = canvas_->addCustomBlock(
      blockData_.name, blockData_.position, inputPins, outputPins);

  if (block) {
    // Restore the original ID
    block->setId(blockData_.id);

    // Restore parameters
    block->setParameters(blockData_.parameters);

    // Restore YAML source preference
    if (!blockData_.preferredYamlSource.isEmpty()) {
      block->setPreferredYamlSource(blockData_.preferredYamlSource);
    }
  }

  // Restore all connections that were attached to this block
  for (const ConnectionData& connData : connections_) {
    // Find source and target blocks by ID
    PackageBlock* sourceBlock = nullptr;
    PackageBlock* targetBlock = nullptr;

    for (QGraphicsItem* item : canvas_->scene()->items()) {
      if (PackageBlock* b = dynamic_cast<PackageBlock*>(item)) {
        if (b->id() == connData.sourceBlockId) {
          sourceBlock = b;
        }
        if (b->id() == connData.targetBlockId) {
          targetBlock = b;
        }
        if (sourceBlock && targetBlock) {
          break;
        }
      }
    }

    if (sourceBlock && targetBlock) {
      // Create connection using pin indices
      ConnectionLine* conn = canvas_->createConnection(
          sourceBlock, connData.sourcePinIndex,
          targetBlock, connData.targetPinIndex);
      if (conn) {
        conn->setId(connData.id);
      }
    }
  }
}

void RemoveBlockCommand::redo() {
  // Skip first redo since block was already removed when command was created
  if (firstRedo_) {
    firstRedo_ = false;
    return;
  }

  // Find and remove the block by ID
  for (QGraphicsItem* item : canvas_->scene()->items()) {
    if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
      if (block->id() == blockData_.id) {
        canvas_->removePackageBlock(block);
        return;
      }
    }
  }
}

QString RemoveBlockCommand::text() const {
  return QObject::tr("Remove Block '%1'").arg(blockData_.name);
}

}  // namespace ros_weaver
