#ifndef ROS_WEAVER_MODIFY_BLOCK_PARAM_COMMAND_HPP
#define ROS_WEAVER_MODIFY_BLOCK_PARAM_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/widgets/remapping_editor.hpp"  // For Remapping struct
#include <QUuid>
#include <QList>

namespace ros_weaver {

// Command to modify a block's parameters
// Supports merging consecutive parameter changes to the same block
// Undo: restores old parameters
// Redo: applies new parameters
class ModifyBlockParamCommand : public UndoCommand {
public:
  // Constructor for full parameter list replacement
  ModifyBlockParamCommand(WeaverCanvas* canvas, const QUuid& blockId,
                          const QList<BlockParamData>& oldParams,
                          const QList<BlockParamData>& newParams);

  void undo() override;
  void redo() override;
  QString text() const override;

  CommandId id() const override { return CommandId::ModifyBlockParam; }
  bool mergeWith(const UndoCommand* other) override;

  // Check if the referenced block still exists
  bool isObsolete() const override;

private:
  QUuid blockId_;
  QList<BlockParamData> oldParams_;
  QList<BlockParamData> newParams_;
};

// Command to modify a block's namespace
class ModifyBlockNamespaceCommand : public UndoCommand {
public:
  ModifyBlockNamespaceCommand(WeaverCanvas* canvas, const QUuid& blockId,
                              const QString& oldNamespace, const QString& newNamespace);

  void undo() override;
  void redo() override;
  QString text() const override;

  CommandId id() const override { return CommandId::ModifyBlockNamespace; }
  bool mergeWith(const UndoCommand* other) override;
  bool isObsolete() const override;

private:
  QUuid blockId_;
  QString oldNamespace_;
  QString newNamespace_;
};

// Command to modify a block's remappings
class ModifyBlockRemappingsCommand : public UndoCommand {
public:
  ModifyBlockRemappingsCommand(WeaverCanvas* canvas, const QUuid& blockId,
                               const QList<Remapping>& oldRemappings,
                               const QList<Remapping>& newRemappings);

  void undo() override;
  void redo() override;
  QString text() const override;

  CommandId id() const override { return CommandId::None; }  // No merging for remappings
  bool isObsolete() const override;

private:
  QUuid blockId_;
  QList<Remapping> oldRemappings_;
  QList<Remapping> newRemappings_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MODIFY_BLOCK_PARAM_COMMAND_HPP
