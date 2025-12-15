#include "ros_weaver/core/undo/commands/remove_group_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/canvas/package_block.hpp"

namespace ros_weaver {

RemoveGroupCommand::RemoveGroupCommand(WeaverCanvas* canvas, const NodeGroupData& groupData)
    : UndoCommand(canvas)
    , groupData_(groupData) {
}

void RemoveGroupCommand::undo() {
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

void RemoveGroupCommand::redo() {
  // Skip first redo since group was already removed when command was created
  if (firstRedo_) {
    firstRedo_ = false;
    return;
  }

  // Find and remove the group by ID
  for (NodeGroup* group : canvas_->nodeGroups()) {
    if (group->id() == groupData_.id) {
      canvas_->removeNodeGroup(group);
      return;
    }
  }
}

QString RemoveGroupCommand::text() const {
  return QObject::tr("Remove Group '%1'").arg(groupData_.title);
}

}  // namespace ros_weaver
