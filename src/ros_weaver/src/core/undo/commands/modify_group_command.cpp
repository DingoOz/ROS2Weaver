#include "ros_weaver/core/undo/commands/modify_group_command.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/node_group.hpp"

namespace ros_weaver {

// Constructor for title modification
ModifyGroupCommand::ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                                       const QString& oldTitle, const QString& newTitle)
    : UndoCommand(canvas)
    , groupId_(groupId)
    , modifyType_(ModifyType::Title)
    , oldTitle_(oldTitle)
    , newTitle_(newTitle) {
}

// Constructor for color modification
ModifyGroupCommand::ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                                       const QColor& oldColor, const QColor& newColor)
    : UndoCommand(canvas)
    , groupId_(groupId)
    , modifyType_(ModifyType::Color)
    , oldColor_(oldColor)
    , newColor_(newColor) {
}

// Constructor for geometry modification
ModifyGroupCommand::ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                                       const QPointF& oldPos, const QSizeF& oldSize,
                                       const QPointF& newPos, const QSizeF& newSize)
    : UndoCommand(canvas)
    , groupId_(groupId)
    , modifyType_(ModifyType::Geometry)
    , oldPos_(oldPos)
    , oldSize_(oldSize)
    , newPos_(newPos)
    , newSize_(newSize) {
}

NodeGroup* findGroupById(WeaverCanvas* canvas, const QUuid& id) {
  for (NodeGroup* group : canvas->nodeGroups()) {
    if (group->id() == id) {
      return group;
    }
  }
  return nullptr;
}

void ModifyGroupCommand::undo() {
  NodeGroup* group = findGroupById(canvas_, groupId_);
  if (!group) {
    return;
  }

  switch (modifyType_) {
    case ModifyType::Title:
      group->setTitle(oldTitle_);
      break;
    case ModifyType::Color:
      group->setColor(oldColor_);
      break;
    case ModifyType::Geometry:
      group->setPos(oldPos_);
      group->setSize(oldSize_);
      break;
  }
}

void ModifyGroupCommand::redo() {
  NodeGroup* group = findGroupById(canvas_, groupId_);
  if (!group) {
    return;
  }

  switch (modifyType_) {
    case ModifyType::Title:
      group->setTitle(newTitle_);
      break;
    case ModifyType::Color:
      group->setColor(newColor_);
      break;
    case ModifyType::Geometry:
      group->setPos(newPos_);
      group->setSize(newSize_);
      break;
  }
}

QString ModifyGroupCommand::text() const {
  switch (modifyType_) {
    case ModifyType::Title:
      return QObject::tr("Rename Group");
    case ModifyType::Color:
      return QObject::tr("Change Group Color");
    case ModifyType::Geometry:
      return QObject::tr("Resize Group");
  }
  return QObject::tr("Modify Group");
}

CommandId ModifyGroupCommand::id() const {
  switch (modifyType_) {
    case ModifyType::Title:
      return CommandId::ModifyGroupTitle;
    case ModifyType::Color:
      return CommandId::ModifyGroupColor;
    case ModifyType::Geometry:
      return CommandId::ModifyGroupSize;
  }
  return CommandId::None;
}

bool ModifyGroupCommand::mergeWith(const UndoCommand* other) {
  const ModifyGroupCommand* modifyCmd = dynamic_cast<const ModifyGroupCommand*>(other);
  if (!modifyCmd) {
    return false;
  }

  // Only merge if it's the same group and same modification type
  if (modifyCmd->groupId_ != groupId_ || modifyCmd->modifyType_ != modifyType_) {
    return false;
  }

  // Only merge if commands are recent (within 500ms)
  qint64 timeDiff = timestamp_.msecsTo(modifyCmd->timestamp_);
  if (timeDiff > 500) {
    return false;
  }

  // Merge: keep our old values, take the new command's new values
  switch (modifyType_) {
    case ModifyType::Title:
      newTitle_ = modifyCmd->newTitle_;
      break;
    case ModifyType::Color:
      newColor_ = modifyCmd->newColor_;
      break;
    case ModifyType::Geometry:
      newPos_ = modifyCmd->newPos_;
      newSize_ = modifyCmd->newSize_;
      break;
  }

  return true;
}

}  // namespace ros_weaver
