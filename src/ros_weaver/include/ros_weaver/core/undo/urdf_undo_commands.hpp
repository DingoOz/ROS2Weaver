#ifndef ROS_WEAVER_CORE_UNDO_URDF_UNDO_COMMANDS_HPP
#define ROS_WEAVER_CORE_UNDO_URDF_UNDO_COMMANDS_HPP

#include "ros_weaver/core/undo/command_ids.hpp"
#include "ros_weaver/core/urdf_model.hpp"
#include <QUndoCommand>
#include <QString>
#include <QDateTime>

namespace ros_weaver {

// Forward declaration
class URDFModel;

/**
 * @brief Base class for URDF undo commands
 */
class URDFUndoCommand : public QUndoCommand {
public:
  explicit URDFUndoCommand(URDFModel* model, const QString& text = QString());

  URDFModel* model() const { return model_; }
  QDateTime timestamp() const { return timestamp_; }

protected:
  URDFModel* model_;
  QDateTime timestamp_;
};

/**
 * @brief Command to move a link's visual/collision origin
 */
class MoveLinkOriginCommand : public URDFUndoCommand {
public:
  MoveLinkOriginCommand(URDFModel* model,
                        const QString& linkName,
                        const URDFPose& oldPose,
                        const URDFPose& newPose);

  void undo() override;
  void redo() override;
  int id() const override { return static_cast<int>(CommandId::MoveLinkOrigin); }
  bool mergeWith(const QUndoCommand* other) override;

private:
  QString linkName_;
  URDFPose oldPose_;
  URDFPose newPose_;
};

/**
 * @brief Command to rotate a link's visual/collision origin
 */
class RotateLinkOriginCommand : public URDFUndoCommand {
public:
  RotateLinkOriginCommand(URDFModel* model,
                          const QString& linkName,
                          const URDFPose& oldPose,
                          const URDFPose& newPose);

  void undo() override;
  void redo() override;
  int id() const override { return static_cast<int>(CommandId::RotateLinkOrigin); }
  bool mergeWith(const QUndoCommand* other) override;

private:
  QString linkName_;
  URDFPose oldPose_;
  URDFPose newPose_;
};

/**
 * @brief Command to move a joint's origin
 */
class MoveJointOriginCommand : public URDFUndoCommand {
public:
  MoveJointOriginCommand(URDFModel* model,
                         const QString& jointName,
                         const URDFPose& oldPose,
                         const URDFPose& newPose);

  void undo() override;
  void redo() override;
  int id() const override { return static_cast<int>(CommandId::MoveJointOrigin); }
  bool mergeWith(const QUndoCommand* other) override;

private:
  QString jointName_;
  URDFPose oldPose_;
  URDFPose newPose_;
};

/**
 * @brief Command to rotate a joint's origin
 */
class RotateJointOriginCommand : public URDFUndoCommand {
public:
  RotateJointOriginCommand(URDFModel* model,
                           const QString& jointName,
                           const URDFPose& oldPose,
                           const URDFPose& newPose);

  void undo() override;
  void redo() override;
  int id() const override { return static_cast<int>(CommandId::RotateJointOrigin); }
  bool mergeWith(const QUndoCommand* other) override;

private:
  QString jointName_;
  URDFPose oldPose_;
  URDFPose newPose_;
};

/**
 * @brief Command to modify joint limits
 */
class ModifyJointLimitsCommand : public URDFUndoCommand {
public:
  ModifyJointLimitsCommand(URDFModel* model,
                           const QString& jointName,
                           const URDFJointLimits& oldLimits,
                           const URDFJointLimits& newLimits);

  void undo() override;
  void redo() override;

private:
  QString jointName_;
  URDFJointLimits oldLimits_;
  URDFJointLimits newLimits_;
};

/**
 * @brief Command to modify joint axis
 */
class ModifyJointAxisCommand : public URDFUndoCommand {
public:
  ModifyJointAxisCommand(URDFModel* model,
                         const QString& jointName,
                         const QVector3D& oldAxis,
                         const QVector3D& newAxis);

  void undo() override;
  void redo() override;

private:
  QString jointName_;
  QVector3D oldAxis_;
  QVector3D newAxis_;
};

/**
 * @brief Command to modify link inertial properties
 */
class ModifyLinkInertialCommand : public URDFUndoCommand {
public:
  ModifyLinkInertialCommand(URDFModel* model,
                            const QString& linkName,
                            const URDFInertial& oldInertial,
                            const URDFInertial& newInertial);

  void undo() override;
  void redo() override;

private:
  QString linkName_;
  URDFInertial oldInertial_;
  URDFInertial newInertial_;
};

/**
 * @brief Command to modify visual material
 */
class ModifyVisualMaterialCommand : public URDFUndoCommand {
public:
  ModifyVisualMaterialCommand(URDFModel* model,
                              const QString& linkName,
                              int visualIndex,
                              const URDFMaterial& oldMaterial,
                              const URDFMaterial& newMaterial);

  void undo() override;
  void redo() override;

private:
  QString linkName_;
  int visualIndex_;
  URDFMaterial oldMaterial_;
  URDFMaterial newMaterial_;
};

/**
 * @brief Command to modify geometry
 */
class ModifyGeometryCommand : public URDFUndoCommand {
public:
  enum class GeometryTarget { Visual, Collision };

  ModifyGeometryCommand(URDFModel* model,
                        const QString& linkName,
                        GeometryTarget target,
                        int index,
                        const URDFGeometry& oldGeometry,
                        const URDFGeometry& newGeometry);

  void undo() override;
  void redo() override;

private:
  QString linkName_;
  GeometryTarget target_;
  int index_;
  URDFGeometry oldGeometry_;
  URDFGeometry newGeometry_;
};

/**
 * @brief Command to modify joint type
 */
class ModifyJointTypeCommand : public URDFUndoCommand {
public:
  ModifyJointTypeCommand(URDFModel* model,
                         const QString& jointName,
                         URDFJointType oldType,
                         URDFJointType newType);

  void undo() override;
  void redo() override;

private:
  QString jointName_;
  URDFJointType oldType_;
  URDFJointType newType_;
};

/**
 * @brief Command to modify joint dynamics (damping and friction)
 */
class ModifyJointDynamicsCommand : public URDFUndoCommand {
public:
  ModifyJointDynamicsCommand(URDFModel* model,
                             const QString& jointName,
                             const URDFJointDynamics& oldDynamics,
                             const URDFJointDynamics& newDynamics);

  void undo() override;
  void redo() override;

private:
  QString jointName_;
  URDFJointDynamics oldDynamics_;
  URDFJointDynamics newDynamics_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_UNDO_URDF_UNDO_COMMANDS_HPP
