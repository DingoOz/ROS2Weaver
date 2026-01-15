#include "ros_weaver/core/undo/commands/modify_block_param_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

namespace ros_weaver {

namespace {
PackageBlock* findBlockById(WeaverCanvas* canvas, const QUuid& id) {
  for (PackageBlock* block : canvas->allBlocks()) {
    if (block->id() == id) {
      return block;
    }
  }
  return nullptr;
}
}  // namespace

// ModifyBlockParamCommand implementation

ModifyBlockParamCommand::ModifyBlockParamCommand(WeaverCanvas* canvas, const QUuid& blockId,
                                                 const QList<BlockParamData>& oldParams,
                                                 const QList<BlockParamData>& newParams)
    : UndoCommand(canvas)
    , blockId_(blockId)
    , oldParams_(oldParams)
    , newParams_(newParams) {
}

void ModifyBlockParamCommand::undo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setParameters(oldParams_);
  }
}

void ModifyBlockParamCommand::redo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setParameters(newParams_);
  }
}

QString ModifyBlockParamCommand::text() const {
  return QObject::tr("Modify Parameters");
}

bool ModifyBlockParamCommand::mergeWith(const UndoCommand* other) {
  const ModifyBlockParamCommand* paramCmd = dynamic_cast<const ModifyBlockParamCommand*>(other);
  if (!paramCmd) {
    return false;
  }

  // Only merge if it's the same block
  if (paramCmd->blockId_ != blockId_) {
    return false;
  }

  // Only merge if commands are recent (within 500ms)
  qint64 timeDiff = timestamp_.msecsTo(paramCmd->timestamp_);
  if (timeDiff > 500) {
    return false;
  }

  // Merge: keep our old values, take the new command's new values
  newParams_ = paramCmd->newParams_;
  return true;
}

bool ModifyBlockParamCommand::isObsolete() const {
  return findBlockById(canvas_, blockId_) == nullptr;
}

// ModifyBlockNamespaceCommand implementation

ModifyBlockNamespaceCommand::ModifyBlockNamespaceCommand(WeaverCanvas* canvas, const QUuid& blockId,
                                                         const QString& oldNamespace,
                                                         const QString& newNamespace)
    : UndoCommand(canvas)
    , blockId_(blockId)
    , oldNamespace_(oldNamespace)
    , newNamespace_(newNamespace) {
}

void ModifyBlockNamespaceCommand::undo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setNodeNamespace(oldNamespace_);
  }
}

void ModifyBlockNamespaceCommand::redo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setNodeNamespace(newNamespace_);
  }
}

QString ModifyBlockNamespaceCommand::text() const {
  return QObject::tr("Change Namespace");
}

bool ModifyBlockNamespaceCommand::mergeWith(const UndoCommand* other) {
  const ModifyBlockNamespaceCommand* nsCmd = dynamic_cast<const ModifyBlockNamespaceCommand*>(other);
  if (!nsCmd) {
    return false;
  }

  // Only merge if it's the same block
  if (nsCmd->blockId_ != blockId_) {
    return false;
  }

  // Only merge if commands are recent (within 500ms)
  qint64 timeDiff = timestamp_.msecsTo(nsCmd->timestamp_);
  if (timeDiff > 500) {
    return false;
  }

  // Merge: keep our old values, take the new command's new values
  newNamespace_ = nsCmd->newNamespace_;
  return true;
}

bool ModifyBlockNamespaceCommand::isObsolete() const {
  return findBlockById(canvas_, blockId_) == nullptr;
}

// ModifyBlockRemappingsCommand implementation

ModifyBlockRemappingsCommand::ModifyBlockRemappingsCommand(WeaverCanvas* canvas, const QUuid& blockId,
                                                           const QList<Remapping>& oldRemappings,
                                                           const QList<Remapping>& newRemappings)
    : UndoCommand(canvas)
    , blockId_(blockId)
    , oldRemappings_(oldRemappings)
    , newRemappings_(newRemappings) {
}

void ModifyBlockRemappingsCommand::undo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setRemappings(oldRemappings_);
  }
}

void ModifyBlockRemappingsCommand::redo() {
  PackageBlock* block = findBlockById(canvas_, blockId_);
  if (block) {
    block->setRemappings(newRemappings_);
  }
}

QString ModifyBlockRemappingsCommand::text() const {
  return QObject::tr("Modify Remappings");
}

bool ModifyBlockRemappingsCommand::isObsolete() const {
  return findBlockById(canvas_, blockId_) == nullptr;
}

}  // namespace ros_weaver
