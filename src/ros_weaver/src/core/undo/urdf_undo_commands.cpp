#include "ros_weaver/core/undo/urdf_undo_commands.hpp"
#include "ros_weaver/core/urdf_model.hpp"

namespace ros_weaver {

// ============================================================================
// URDFUndoCommand
// ============================================================================

URDFUndoCommand::URDFUndoCommand(URDFModel* model, const QString& text)
  : QUndoCommand(text)
  , model_(model)
  , timestamp_(QDateTime::currentDateTime()) {
}

// ============================================================================
// MoveLinkOriginCommand
// ============================================================================

MoveLinkOriginCommand::MoveLinkOriginCommand(URDFModel* model,
                                             const QString& linkName,
                                             const URDFPose& oldPose,
                                             const URDFPose& newPose)
  : URDFUndoCommand(model, QObject::tr("Move Link '%1'").arg(linkName))
  , linkName_(linkName)
  , oldPose_(oldPose)
  , newPose_(newPose) {
}

void MoveLinkOriginCommand::undo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && !link->visuals.isEmpty()) {
    link->visuals[0].origin = oldPose_;
    model_->setModified(true);
  }
}

void MoveLinkOriginCommand::redo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && !link->visuals.isEmpty()) {
    link->visuals[0].origin = newPose_;
    model_->setModified(true);
  }
}

bool MoveLinkOriginCommand::mergeWith(const QUndoCommand* other) {
  if (other->id() != id()) {
    return false;
  }

  const MoveLinkOriginCommand* cmd = static_cast<const MoveLinkOriginCommand*>(other);
  if (cmd->linkName_ != linkName_) {
    return false;
  }

  // Only merge if commands are recent (within 500ms sliding window)
  qint64 timeDiff = std::abs(timestamp_.msecsTo(cmd->timestamp_));
  if (timeDiff > 500) {
    return false;
  }

  newPose_ = cmd->newPose_;
  timestamp_ = cmd->timestamp_;  // Slide window forward
  return true;
}

// ============================================================================
// RotateLinkOriginCommand
// ============================================================================

RotateLinkOriginCommand::RotateLinkOriginCommand(URDFModel* model,
                                                 const QString& linkName,
                                                 const URDFPose& oldPose,
                                                 const URDFPose& newPose)
  : URDFUndoCommand(model, QObject::tr("Rotate Link '%1'").arg(linkName))
  , linkName_(linkName)
  , oldPose_(oldPose)
  , newPose_(newPose) {
}

void RotateLinkOriginCommand::undo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && !link->visuals.isEmpty()) {
    link->visuals[0].origin = oldPose_;
    model_->setModified(true);
  }
}

void RotateLinkOriginCommand::redo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && !link->visuals.isEmpty()) {
    link->visuals[0].origin = newPose_;
    model_->setModified(true);
  }
}

bool RotateLinkOriginCommand::mergeWith(const QUndoCommand* other) {
  if (other->id() != id()) {
    return false;
  }

  const RotateLinkOriginCommand* cmd = static_cast<const RotateLinkOriginCommand*>(other);
  if (cmd->linkName_ != linkName_) {
    return false;
  }

  qint64 timeDiff = std::abs(timestamp_.msecsTo(cmd->timestamp_));
  if (timeDiff > 500) {
    return false;
  }

  newPose_ = cmd->newPose_;
  timestamp_ = cmd->timestamp_;  // Slide window forward
  return true;
}

// ============================================================================
// MoveJointOriginCommand
// ============================================================================

MoveJointOriginCommand::MoveJointOriginCommand(URDFModel* model,
                                               const QString& jointName,
                                               const URDFPose& oldPose,
                                               const URDFPose& newPose)
  : URDFUndoCommand(model, QObject::tr("Move Joint '%1'").arg(jointName))
  , jointName_(jointName)
  , oldPose_(oldPose)
  , newPose_(newPose) {
}

void MoveJointOriginCommand::undo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->origin = oldPose_;
    model_->setModified(true);
  }
}

void MoveJointOriginCommand::redo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->origin = newPose_;
    model_->setModified(true);
  }
}

bool MoveJointOriginCommand::mergeWith(const QUndoCommand* other) {
  if (other->id() != id()) {
    return false;
  }

  const MoveJointOriginCommand* cmd = static_cast<const MoveJointOriginCommand*>(other);
  if (cmd->jointName_ != jointName_) {
    return false;
  }

  qint64 timeDiff = std::abs(timestamp_.msecsTo(cmd->timestamp_));
  if (timeDiff > 500) {
    return false;
  }

  newPose_ = cmd->newPose_;
  timestamp_ = cmd->timestamp_;  // Slide window forward
  return true;
}

// ============================================================================
// RotateJointOriginCommand
// ============================================================================

RotateJointOriginCommand::RotateJointOriginCommand(URDFModel* model,
                                                   const QString& jointName,
                                                   const URDFPose& oldPose,
                                                   const URDFPose& newPose)
  : URDFUndoCommand(model, QObject::tr("Rotate Joint '%1'").arg(jointName))
  , jointName_(jointName)
  , oldPose_(oldPose)
  , newPose_(newPose) {
}

void RotateJointOriginCommand::undo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->origin = oldPose_;
    model_->setModified(true);
  }
}

void RotateJointOriginCommand::redo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->origin = newPose_;
    model_->setModified(true);
  }
}

bool RotateJointOriginCommand::mergeWith(const QUndoCommand* other) {
  if (other->id() != id()) {
    return false;
  }

  const RotateJointOriginCommand* cmd = static_cast<const RotateJointOriginCommand*>(other);
  if (cmd->jointName_ != jointName_) {
    return false;
  }

  qint64 timeDiff = std::abs(timestamp_.msecsTo(cmd->timestamp_));
  if (timeDiff > 500) {
    return false;
  }

  newPose_ = cmd->newPose_;
  timestamp_ = cmd->timestamp_;  // Slide window forward
  return true;
}

// ============================================================================
// ModifyJointLimitsCommand
// ============================================================================

ModifyJointLimitsCommand::ModifyJointLimitsCommand(URDFModel* model,
                                                   const QString& jointName,
                                                   const URDFJointLimits& oldLimits,
                                                   const URDFJointLimits& newLimits)
  : URDFUndoCommand(model, QObject::tr("Modify Joint '%1' Limits").arg(jointName))
  , jointName_(jointName)
  , oldLimits_(oldLimits)
  , newLimits_(newLimits) {
}

void ModifyJointLimitsCommand::undo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->limits = oldLimits_;
    model_->setModified(true);
  }
}

void ModifyJointLimitsCommand::redo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->limits = newLimits_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyJointAxisCommand
// ============================================================================

ModifyJointAxisCommand::ModifyJointAxisCommand(URDFModel* model,
                                               const QString& jointName,
                                               const QVector3D& oldAxis,
                                               const QVector3D& newAxis)
  : URDFUndoCommand(model, QObject::tr("Modify Joint '%1' Axis").arg(jointName))
  , jointName_(jointName)
  , oldAxis_(oldAxis)
  , newAxis_(newAxis) {
}

void ModifyJointAxisCommand::undo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->axis = oldAxis_;
    model_->setModified(true);
  }
}

void ModifyJointAxisCommand::redo() {
  if (!model_) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->axis = newAxis_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyLinkInertialCommand
// ============================================================================

ModifyLinkInertialCommand::ModifyLinkInertialCommand(URDFModel* model,
                                                     const QString& linkName,
                                                     const URDFInertial& oldInertial,
                                                     const URDFInertial& newInertial)
  : URDFUndoCommand(model, QObject::tr("Modify Link '%1' Inertial").arg(linkName))
  , linkName_(linkName)
  , oldInertial_(oldInertial)
  , newInertial_(newInertial) {
}

void ModifyLinkInertialCommand::undo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link) {
    link->inertial = oldInertial_;
    model_->setModified(true);
  }
}

void ModifyLinkInertialCommand::redo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link) {
    link->inertial = newInertial_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyVisualMaterialCommand
// ============================================================================

ModifyVisualMaterialCommand::ModifyVisualMaterialCommand(URDFModel* model,
                                                         const QString& linkName,
                                                         int visualIndex,
                                                         const URDFMaterial& oldMaterial,
                                                         const URDFMaterial& newMaterial)
  : URDFUndoCommand(model, QObject::tr("Modify Link '%1' Material").arg(linkName))
  , linkName_(linkName)
  , visualIndex_(visualIndex)
  , oldMaterial_(oldMaterial)
  , newMaterial_(newMaterial) {
}

void ModifyVisualMaterialCommand::undo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && visualIndex_ < link->visuals.size()) {
    link->visuals[visualIndex_].material = oldMaterial_;
    model_->setModified(true);
  }
}

void ModifyVisualMaterialCommand::redo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (link && visualIndex_ < link->visuals.size()) {
    link->visuals[visualIndex_].material = newMaterial_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyGeometryCommand
// ============================================================================

ModifyGeometryCommand::ModifyGeometryCommand(URDFModel* model,
                                             const QString& linkName,
                                             GeometryTarget target,
                                             int index,
                                             const URDFGeometry& oldGeometry,
                                             const URDFGeometry& newGeometry)
  : URDFUndoCommand(model, QObject::tr("Modify Link '%1' Geometry").arg(linkName))
  , linkName_(linkName)
  , target_(target)
  , index_(index)
  , oldGeometry_(oldGeometry)
  , newGeometry_(newGeometry) {
}

void ModifyGeometryCommand::undo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (!link) return;

  if (target_ == GeometryTarget::Visual && index_ < link->visuals.size()) {
    link->visuals[index_].geometry = oldGeometry_;
    model_->setModified(true);
  } else if (target_ == GeometryTarget::Collision && index_ < link->collisions.size()) {
    link->collisions[index_].geometry = oldGeometry_;
    model_->setModified(true);
  }
}

void ModifyGeometryCommand::redo() {
  if (!model_) return;

  URDFLink* link = model_->link(linkName_);
  if (!link) return;

  if (target_ == GeometryTarget::Visual && index_ < link->visuals.size()) {
    link->visuals[index_].geometry = newGeometry_;
    model_->setModified(true);
  } else if (target_ == GeometryTarget::Collision && index_ < link->collisions.size()) {
    link->collisions[index_].geometry = newGeometry_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyJointTypeCommand
// ============================================================================

ModifyJointTypeCommand::ModifyJointTypeCommand(URDFModel* model,
                                               const QString& jointName,
                                               URDFJointType oldType,
                                               URDFJointType newType)
  : URDFUndoCommand(model, QObject::tr("Change Joint '%1' Type").arg(jointName))
  , jointName_(jointName)
  , oldType_(oldType)
  , newType_(newType) {
}

void ModifyJointTypeCommand::undo() {
  if (!model_) return;
  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->type = oldType_;
    model_->setModified(true);
  }
}

void ModifyJointTypeCommand::redo() {
  if (!model_) return;
  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->type = newType_;
    model_->setModified(true);
  }
}

// ============================================================================
// ModifyJointDynamicsCommand
// ============================================================================

ModifyJointDynamicsCommand::ModifyJointDynamicsCommand(URDFModel* model,
                                                       const QString& jointName,
                                                       const URDFJointDynamics& oldDynamics,
                                                       const URDFJointDynamics& newDynamics)
  : URDFUndoCommand(model, QObject::tr("Modify Joint '%1' Dynamics").arg(jointName))
  , jointName_(jointName)
  , oldDynamics_(oldDynamics)
  , newDynamics_(newDynamics) {
}

void ModifyJointDynamicsCommand::undo() {
  if (!model_) return;
  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->dynamics = oldDynamics_;
    model_->setModified(true);
  }
}

void ModifyJointDynamicsCommand::redo() {
  if (!model_) return;
  URDFJoint* joint = model_->joint(jointName_);
  if (joint) {
    joint->dynamics = newDynamics_;
    model_->setModified(true);
  }
}

}  // namespace ros_weaver
