#include "ros_weaver/core/undo/mission_undo_commands.hpp"
#include "ros_weaver/widgets/mission_planner_panel.hpp"
#include "ros_weaver/widgets/mission_map_view.hpp"

namespace ros_weaver {

// --- MissionUndoCommand ---

MissionUndoCommand::MissionUndoCommand(MissionPlannerPanel* panel)
    : panel_(panel)
    , timestamp_(QDateTime::currentDateTime()) {
}

// --- AddWaypointCommand ---

AddWaypointCommand::AddWaypointCommand(MissionPlannerPanel* panel, const Waypoint& waypoint)
    : MissionUndoCommand(panel)
    , waypoint_(waypoint) {
}

void AddWaypointCommand::undo() {
  panel_->removeWaypointById(waypoint_.id);
}

void AddWaypointCommand::redo() {
  panel_->addWaypointFromUndo(waypoint_);
}

QString AddWaypointCommand::text() const {
  return QObject::tr("Add Waypoint %1").arg(waypoint_.name);
}

// --- RemoveWaypointCommand ---

RemoveWaypointCommand::RemoveWaypointCommand(MissionPlannerPanel* panel,
                                              const Waypoint& waypoint, int index)
    : MissionUndoCommand(panel)
    , waypoint_(waypoint)
    , index_(index) {
}

void RemoveWaypointCommand::undo() {
  panel_->insertWaypointFromUndo(waypoint_, index_);
}

void RemoveWaypointCommand::redo() {
  panel_->removeWaypointById(waypoint_.id);
}

QString RemoveWaypointCommand::text() const {
  return QObject::tr("Remove Waypoint %1").arg(waypoint_.name);
}

// --- MoveWaypointCommand ---

MoveWaypointCommand::MoveWaypointCommand(MissionPlannerPanel* panel, int waypointId,
                                          const QPointF& oldPosition, const QPointF& newPosition)
    : MissionUndoCommand(panel)
    , waypointId_(waypointId)
    , oldPosition_(oldPosition)
    , newPosition_(newPosition) {
}

void MoveWaypointCommand::undo() {
  panel_->moveWaypointFromUndo(waypointId_, oldPosition_);
}

void MoveWaypointCommand::redo() {
  panel_->moveWaypointFromUndo(waypointId_, newPosition_);
}

QString MoveWaypointCommand::text() const {
  return QObject::tr("Move Waypoint");
}

bool MoveWaypointCommand::mergeWith(const MissionUndoCommand* other) {
  const auto* moveCmd = dynamic_cast<const MoveWaypointCommand*>(other);
  if (!moveCmd) return false;
  if (moveCmd->waypointId_ != waypointId_) return false;

  // Merge by updating the new position
  newPosition_ = moveCmd->newPosition_;
  return true;
}

// --- ChangeWaypointOrientationCommand ---

ChangeWaypointOrientationCommand::ChangeWaypointOrientationCommand(
    MissionPlannerPanel* panel, int waypointId, double oldTheta, double newTheta)
    : MissionUndoCommand(panel)
    , waypointId_(waypointId)
    , oldTheta_(oldTheta)
    , newTheta_(newTheta) {
}

void ChangeWaypointOrientationCommand::undo() {
  panel_->setWaypointOrientationFromUndo(waypointId_, oldTheta_);
}

void ChangeWaypointOrientationCommand::redo() {
  panel_->setWaypointOrientationFromUndo(waypointId_, newTheta_);
}

QString ChangeWaypointOrientationCommand::text() const {
  return QObject::tr("Change Waypoint Orientation");
}

bool ChangeWaypointOrientationCommand::mergeWith(const MissionUndoCommand* other) {
  const auto* orientCmd = dynamic_cast<const ChangeWaypointOrientationCommand*>(other);
  if (!orientCmd) return false;
  if (orientCmd->waypointId_ != waypointId_) return false;

  newTheta_ = orientCmd->newTheta_;
  return true;
}

// --- SetStartPoseCommand ---

SetStartPoseCommand::SetStartPoseCommand(MissionPlannerPanel* panel,
                                          const RobotStartPose& oldPose,
                                          const RobotStartPose& newPose)
    : MissionUndoCommand(panel)
    , oldPose_(oldPose)
    , newPose_(newPose) {
}

void SetStartPoseCommand::undo() {
  panel_->setStartPoseFromUndo(oldPose_);
}

void SetStartPoseCommand::redo() {
  panel_->setStartPoseFromUndo(newPose_);
}

QString SetStartPoseCommand::text() const {
  return QObject::tr("Set Start Pose");
}

bool SetStartPoseCommand::mergeWith(const MissionUndoCommand* other) {
  const auto* poseCmd = dynamic_cast<const SetStartPoseCommand*>(other);
  if (!poseCmd) return false;

  newPose_ = poseCmd->newPose_;
  return true;
}

// --- ClearWaypointsCommand ---

ClearWaypointsCommand::ClearWaypointsCommand(MissionPlannerPanel* panel,
                                              const QList<Waypoint>& waypoints)
    : MissionUndoCommand(panel)
    , waypoints_(waypoints) {
}

void ClearWaypointsCommand::undo() {
  panel_->setWaypointsFromUndo(waypoints_);
}

void ClearWaypointsCommand::redo() {
  panel_->clearWaypointsFromUndo();
}

QString ClearWaypointsCommand::text() const {
  return QObject::tr("Clear Waypoints");
}

// --- ReorderWaypointCommand ---

ReorderWaypointCommand::ReorderWaypointCommand(MissionPlannerPanel* panel,
                                                int fromIndex, int toIndex)
    : MissionUndoCommand(panel)
    , fromIndex_(fromIndex)
    , toIndex_(toIndex) {
}

void ReorderWaypointCommand::undo() {
  panel_->reorderWaypointFromUndo(toIndex_, fromIndex_);
}

void ReorderWaypointCommand::redo() {
  panel_->reorderWaypointFromUndo(fromIndex_, toIndex_);
}

QString ReorderWaypointCommand::text() const {
  return QObject::tr("Reorder Waypoints");
}

}  // namespace ros_weaver
