#include "ros_weaver/core/urdf_joint_controller.hpp"

#include <cmath>
#include <QDebug>

namespace ros_weaver {

URDFJointController::URDFJointController(URDFParser* parser, QObject* parent)
  : QObject(parent)
  , parser_(parser) {
}

void URDFJointController::setRotationMode(RotationMode mode) {
  rotationMode_ = mode;
}

void URDFJointController::setSelectedJoints(const QStringList& jointNames) {
  selectedJoints_ = jointNames;
  emit selectionChanged(selectedJoints_);
}

void URDFJointController::clearSelection() {
  selectedJoints_.clear();
  emit selectionChanged(selectedJoints_);
}

void URDFJointController::rotateSelectedJoints(RotationAxis axis, double angleDegrees) {
  for (const QString& jointName : selectedJoints_) {
    rotateJoint(jointName, axis, angleDegrees);
  }
}

void URDFJointController::rotateJoint(const QString& jointName, RotationAxis axis, double angleDegrees) {
  if (!parser_) return;

  QVector3D axisVector = getAxisVector(axis);
  parser_->rotateJointBy(jointName, axisVector, angleDegrees);

  URDFJoint* joint = parser_->getJoint(jointName);
  if (joint) {
    emit jointRotated(jointName, joint->orientation);
  }
}

void URDFJointController::snapRotateSelectedJoints(RotationAxis axis, bool positive) {
  double angle = positive ? SNAP_ANGLE : -SNAP_ANGLE;
  rotateSelectedJoints(axis, angle);
  emit rotationCompleted();
}

void URDFJointController::snapRotateJoint(const QString& jointName, RotationAxis axis, bool positive) {
  double angle = positive ? SNAP_ANGLE : -SNAP_ANGLE;
  rotateJoint(jointName, axis, angle);
  emit rotationCompleted();
}

void URDFJointController::freeRotateSelectedJoints(RotationAxis axis, double angleDegrees) {
  rotateSelectedJoints(axis, angleDegrees);
  emit rotationCompleted();
}

void URDFJointController::freeRotateJoint(const QString& jointName, RotationAxis axis, double angleDegrees) {
  rotateJoint(jointName, axis, angleDegrees);
  emit rotationCompleted();
}

void URDFJointController::resetSelectedJoints() {
  for (const QString& jointName : selectedJoints_) {
    resetJoint(jointName);
  }
  emit rotationCompleted();
}

void URDFJointController::resetJoint(const QString& jointName) {
  if (!parser_) return;

  parser_->setJointOrientation(jointName, QQuaternion());

  emit jointRotated(jointName, QQuaternion());
}

void URDFJointController::setJointOrientation(const QString& jointName, const QQuaternion& orientation) {
  if (!parser_) return;

  parser_->setJointOrientation(jointName, orientation);
  emit jointRotated(jointName, orientation);
}

void URDFJointController::startMouseRotation(const QString& jointName, RotationAxis axis) {
  if (!parser_) return;

  URDFJoint* joint = parser_->getJoint(jointName);
  if (!joint) return;

  activeRotationJoint_ = jointName;
  activeRotationAxis_ = axis;
  originalOrientation_ = joint->orientation;
  accumulatedRotation_ = 0.0;
}

void URDFJointController::updateMouseRotation(double deltaAngle) {
  if (activeRotationJoint_.isEmpty() || !parser_) return;

  accumulatedRotation_ += deltaAngle;

  // Apply rotation preview
  QVector3D axisVector = getAxisVector(activeRotationAxis_);
  QQuaternion rotation = QQuaternion::fromAxisAndAngle(axisVector, static_cast<float>(accumulatedRotation_));
  QQuaternion newOrientation = rotation * originalOrientation_;

  // If in snap mode, snap to nearest 90-degree increment
  if (rotationMode_ == RotationMode::Snap90Degrees) {
    // Round accumulated rotation to nearest 90 degrees
    double snappedAngle = std::round(accumulatedRotation_ / SNAP_ANGLE) * SNAP_ANGLE;
    rotation = QQuaternion::fromAxisAndAngle(axisVector, static_cast<float>(snappedAngle));
    newOrientation = rotation * originalOrientation_;
  }

  emit rotationPreview(activeRotationJoint_, newOrientation);
}

void URDFJointController::finishMouseRotation() {
  if (activeRotationJoint_.isEmpty() || !parser_) return;

  // Calculate final orientation
  double finalAngle = accumulatedRotation_;
  if (rotationMode_ == RotationMode::Snap90Degrees) {
    finalAngle = std::round(accumulatedRotation_ / SNAP_ANGLE) * SNAP_ANGLE;
  }

  QVector3D axisVector = getAxisVector(activeRotationAxis_);
  QQuaternion rotation = QQuaternion::fromAxisAndAngle(axisVector, static_cast<float>(finalAngle));
  QQuaternion newOrientation = rotation * originalOrientation_;

  // Apply final orientation
  parser_->setJointOrientation(activeRotationJoint_, newOrientation);
  emit jointRotated(activeRotationJoint_, newOrientation);

  // Clear state
  activeRotationJoint_.clear();
  emit rotationCompleted();
}

void URDFJointController::cancelMouseRotation() {
  if (activeRotationJoint_.isEmpty() || !parser_) return;

  // Restore original orientation
  parser_->setJointOrientation(activeRotationJoint_, originalOrientation_);
  emit jointRotated(activeRotationJoint_, originalOrientation_);

  // Clear state
  activeRotationJoint_.clear();
  emit rotationCancelled();
}

QVector3D URDFJointController::getAxisVector(RotationAxis axis) {
  switch (axis) {
    case RotationAxis::X:
      return QVector3D(1, 0, 0);
    case RotationAxis::Y:
      return QVector3D(0, 1, 0);
    case RotationAxis::Z:
      return QVector3D(0, 0, 1);
    default:
      return QVector3D(0, 0, 1);
  }
}

}  // namespace ros_weaver
