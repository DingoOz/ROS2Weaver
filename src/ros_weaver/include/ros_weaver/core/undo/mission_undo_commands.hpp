#ifndef ROS_WEAVER_MISSION_UNDO_COMMANDS_HPP
#define ROS_WEAVER_MISSION_UNDO_COMMANDS_HPP

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QPointF>
#include <QList>
#include "ros_weaver/core/undo/command_ids.hpp"
#include "ros_weaver/core/mission_data.hpp"

namespace ros_weaver {

// Forward declarations
class MissionMapView;
class MissionPlannerPanel;

/**
 * @brief Base class for mission planner undo commands
 *
 * Similar to UndoCommand but operates on MissionMapView/MissionPlannerPanel
 * instead of WeaverCanvas.
 */
class MissionUndoCommand {
public:
  explicit MissionUndoCommand(MissionPlannerPanel* panel);
  virtual ~MissionUndoCommand() = default;

  virtual void undo() = 0;
  virtual void redo() = 0;
  virtual QString text() const = 0;
  virtual CommandId id() const { return CommandId::None; }
  virtual bool mergeWith(const MissionUndoCommand* other) {
    Q_UNUSED(other);
    return false;
  }

  QDateTime timestamp() const { return timestamp_; }
  virtual bool isObsolete() const { return false; }
  MissionPlannerPanel* panel() const { return panel_; }

protected:
  MissionPlannerPanel* panel_;
  QDateTime timestamp_;
};

/**
 * @brief Command for adding a waypoint
 */
class AddWaypointCommand : public MissionUndoCommand {
public:
  AddWaypointCommand(MissionPlannerPanel* panel, const Waypoint& waypoint);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  Waypoint waypoint_;
};

/**
 * @brief Command for removing a waypoint
 */
class RemoveWaypointCommand : public MissionUndoCommand {
public:
  RemoveWaypointCommand(MissionPlannerPanel* panel, const Waypoint& waypoint, int index);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  Waypoint waypoint_;
  int index_;
};

/**
 * @brief Command for moving a waypoint
 */
class MoveWaypointCommand : public MissionUndoCommand {
public:
  MoveWaypointCommand(MissionPlannerPanel* panel, int waypointId,
                      const QPointF& oldPosition, const QPointF& newPosition);

  void undo() override;
  void redo() override;
  QString text() const override;
  CommandId id() const override { return CommandId::MoveWaypoint; }
  bool mergeWith(const MissionUndoCommand* other) override;

private:
  int waypointId_;
  QPointF oldPosition_;
  QPointF newPosition_;
};

/**
 * @brief Command for changing waypoint orientation
 */
class ChangeWaypointOrientationCommand : public MissionUndoCommand {
public:
  ChangeWaypointOrientationCommand(MissionPlannerPanel* panel, int waypointId,
                                    double oldTheta, double newTheta);

  void undo() override;
  void redo() override;
  QString text() const override;
  CommandId id() const override { return CommandId::ChangeWaypointOrientation; }
  bool mergeWith(const MissionUndoCommand* other) override;

private:
  int waypointId_;
  double oldTheta_;
  double newTheta_;
};

/**
 * @brief Command for setting/moving start pose
 */
class SetStartPoseCommand : public MissionUndoCommand {
public:
  SetStartPoseCommand(MissionPlannerPanel* panel,
                      const RobotStartPose& oldPose, const RobotStartPose& newPose);

  void undo() override;
  void redo() override;
  QString text() const override;
  CommandId id() const override { return CommandId::MoveStartPose; }
  bool mergeWith(const MissionUndoCommand* other) override;

private:
  RobotStartPose oldPose_;
  RobotStartPose newPose_;
};

/**
 * @brief Command for clearing all waypoints
 */
class ClearWaypointsCommand : public MissionUndoCommand {
public:
  ClearWaypointsCommand(MissionPlannerPanel* panel, const QList<Waypoint>& waypoints);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  QList<Waypoint> waypoints_;
};

/**
 * @brief Command for reordering waypoints
 */
class ReorderWaypointCommand : public MissionUndoCommand {
public:
  ReorderWaypointCommand(MissionPlannerPanel* panel, int fromIndex, int toIndex);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  int fromIndex_;
  int toIndex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MISSION_UNDO_COMMANDS_HPP
