#ifndef ROS_WEAVER_MODIFY_GROUP_COMMAND_HPP
#define ROS_WEAVER_MODIFY_GROUP_COMMAND_HPP

#include "ros_weaver/core/undo/undo_command.hpp"
#include <QUuid>
#include <QString>
#include <QColor>
#include <QSizeF>
#include <QPointF>

namespace ros_weaver {

// Command to modify a node group's properties (title, color, size, position)
// Supports merging consecutive modifications of the same property
// Undo: restores old values
// Redo: applies new values
class ModifyGroupCommand : public UndoCommand {
public:
  enum class ModifyType {
    Title,
    Color,
    Geometry  // Position and size combined
  };

  // Constructor for title modification
  ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                     const QString& oldTitle, const QString& newTitle);

  // Constructor for color modification
  ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                     const QColor& oldColor, const QColor& newColor);

  // Constructor for geometry modification (position + size)
  ModifyGroupCommand(WeaverCanvas* canvas, const QUuid& groupId,
                     const QPointF& oldPos, const QSizeF& oldSize,
                     const QPointF& newPos, const QSizeF& newSize);

  void undo() override;
  void redo() override;
  QString text() const override;

  CommandId id() const override;
  bool mergeWith(const UndoCommand* other) override;

private:
  QUuid groupId_;
  ModifyType modifyType_;

  // For title changes
  QString oldTitle_;
  QString newTitle_;

  // For color changes
  QColor oldColor_;
  QColor newColor_;

  // For geometry changes
  QPointF oldPos_;
  QSizeF oldSize_;
  QPointF newPos_;
  QSizeF newSize_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MODIFY_GROUP_COMMAND_HPP
