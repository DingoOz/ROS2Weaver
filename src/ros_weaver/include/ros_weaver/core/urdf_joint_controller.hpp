#ifndef ROS_WEAVER_URDF_JOINT_CONTROLLER_HPP
#define ROS_WEAVER_URDF_JOINT_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector3D>
#include <QQuaternion>

#include "ros_weaver/core/urdf_parser.hpp"

namespace ros_weaver {

// Rotation mode options
enum class RotationMode {
  Snap90Degrees,  // Default: rotate in 90-degree increments
  FreeRotation    // Optional: arbitrary rotation angles
};

// Rotation axis options
enum class RotationAxis {
  X,
  Y,
  Z
};

class URDFJointController : public QObject {
  Q_OBJECT

public:
  explicit URDFJointController(URDFParser* parser, QObject* parent = nullptr);
  ~URDFJointController() override = default;

  // Parser access
  void setParser(URDFParser* parser) { parser_ = parser; }
  URDFParser* parser() const { return parser_; }

  // Rotation mode
  void setRotationMode(RotationMode mode);
  RotationMode rotationMode() const { return rotationMode_; }

  // Selection management
  void setSelectedJoints(const QStringList& jointNames);
  QStringList selectedJoints() const { return selectedJoints_; }
  void clearSelection();

  // Rotate selected joints
  void rotateSelectedJoints(RotationAxis axis, double angleDegrees);
  void rotateJoint(const QString& jointName, RotationAxis axis, double angleDegrees);

  // For snap mode: always use 90-degree increments
  void snapRotateSelectedJoints(RotationAxis axis, bool positive);
  void snapRotateJoint(const QString& jointName, RotationAxis axis, bool positive);

  // For free mode: rotate by arbitrary angle
  void freeRotateSelectedJoints(RotationAxis axis, double angleDegrees);
  void freeRotateJoint(const QString& jointName, RotationAxis axis, double angleDegrees);

  // Reset joint orientation
  void resetSelectedJoints();
  void resetJoint(const QString& jointName);

  // Set absolute orientation
  void setJointOrientation(const QString& jointName, const QQuaternion& orientation);

  // Mouse-based rotation (for interactive editing)
  void startMouseRotation(const QString& jointName, RotationAxis axis);
  void updateMouseRotation(double deltaAngle);
  void finishMouseRotation();
  void cancelMouseRotation();
  bool isMouseRotationActive() const { return !activeRotationJoint_.isEmpty(); }

  // Get rotation axis vector
  static QVector3D getAxisVector(RotationAxis axis);

signals:
  void jointRotated(const QString& jointName, const QQuaternion& newOrientation);
  void rotationPreview(const QString& jointName, const QQuaternion& previewOrientation);
  void rotationCompleted();
  void rotationCancelled();
  void selectionChanged(const QStringList& selectedJoints);

private:
  URDFParser* parser_;
  QStringList selectedJoints_;
  RotationMode rotationMode_ = RotationMode::Snap90Degrees;

  // Mouse rotation state
  QString activeRotationJoint_;
  RotationAxis activeRotationAxis_;
  QQuaternion originalOrientation_;
  double accumulatedRotation_ = 0.0;

  // Snap angle (degrees)
  static constexpr double SNAP_ANGLE = 90.0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_URDF_JOINT_CONTROLLER_HPP
