#include "ros_weaver/core/undo/commands/add_block_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

namespace ros_weaver {

AddBlockCommand::AddBlockCommand(WeaverCanvas* canvas, const BlockData& blockData)
    : UndoCommand(canvas)
    , blockData_(blockData) {
}

void AddBlockCommand::undo() {
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

void AddBlockCommand::redo() {
  // Skip first redo since block was already added when command was created
  if (firstRedo_) {
    firstRedo_ = false;
    return;
  }

  // Convert pins to the format expected by addCustomBlock
  QList<QPair<QString, QString>> inputPins;
  for (const PinData& pin : blockData_.inputPins) {
    inputPins.append({pin.name, pin.messageType});
  }

  QList<QPair<QString, QString>> outputPins;
  for (const PinData& pin : blockData_.outputPins) {
    outputPins.append({pin.name, pin.messageType});
  }

  // Re-add the block with original data
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
}

QString AddBlockCommand::text() const {
  return QObject::tr("Add Block '%1'").arg(blockData_.name);
}

}  // namespace ros_weaver
