#include "ros_weaver/core/undo/commands/add_group_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/canvas/package_block.hpp"

namespace ros_weaver {

AddGroupCommand::AddGroupCommand(WeaverCanvas* canvas, const NodeGroupData& groupData)
    : UndoCommand(canvas)
    , groupData_(groupData) {
}

void AddGroupCommand::undo() {
  // Find and remove the group by ID
  for (NodeGroup* group : canvas_->nodeGroups()) {
    if (group->id() == groupData_.id) {
      canvas_->removeNodeGroup(group);
      return;
    }
  }
}

void AddGroupCommand::redo() {
  // Skip first redo since group was already added when command was created
  if (firstRedo_) {
    firstRedo_ = false;
    return;
  }

  // Re-create the group
  NodeGroup* group = canvas_->createNodeGroup(groupData_.title);

  if (group) {
    // Restore the original ID
    group->setId(groupData_.id);

    // Restore properties
    group->setPos(groupData_.position);
    group->setSize(groupData_.size);
    group->setColor(groupData_.color);

    // Restore contained nodes
    for (const QUuid& nodeId : groupData_.containedNodeIds) {
      // Find the block by ID
      for (QGraphicsItem* item : canvas_->scene()->items()) {
        if (PackageBlock* block = dynamic_cast<PackageBlock*>(item)) {
          if (block->id() == nodeId) {
            group->addNode(block);
            break;
          }
        }
      }
    }
  }
}

QString AddGroupCommand::text() const {
  return QObject::tr("Add Group '%1'").arg(groupData_.title);
}

}  // namespace ros_weaver
