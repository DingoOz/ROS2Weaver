#ifndef ROS_WEAVER_WIDGETS_JOINT_PROPERTIES_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_JOINT_PROPERTIES_WIDGET_HPP

#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QUndoStack>

#include "ros_weaver/core/urdf_model.hpp"

namespace ros_weaver {

/**
 * @brief Properties widget for URDF joints
 *
 * Provides editors for:
 * - Joint origin (position X,Y,Z and orientation R,P,Y)
 * - Joint axis (X, Y, Z)
 * - Joint type
 * - Joint limits (lower, upper, effort, velocity)
 * - Dynamics (damping, friction)
 */
class JointPropertiesWidget : public QWidget {
  Q_OBJECT

public:
  explicit JointPropertiesWidget(QWidget* parent = nullptr);
  ~JointPropertiesWidget() override;

  void setModel(URDFModel* model);
  void setUndoStack(QUndoStack* stack);

  void setJointName(const QString& name);
  QString jointName() const { return jointName_; }

  void refresh();

signals:
  void propertyChanged();

private slots:
  void onOriginPositionChanged();
  void onOriginOrientationChanged();
  void onAxisChanged();
  void onTypeChanged(int index);
  void onLimitsChanged();
  void onDynamicsChanged();

private:
  void setupUi();
  void setupConnections();

  void blockAllSignals(bool block);
  void updateFromModel();
  void updateLimitsEnabled();

  URDFModel* model_ = nullptr;
  QUndoStack* undoStack_ = nullptr;
  QString jointName_;

  // Joint type
  QComboBox* typeCombo_ = nullptr;

  // Origin
  QGroupBox* originGroup_ = nullptr;
  QDoubleSpinBox* originPosX_ = nullptr;
  QDoubleSpinBox* originPosY_ = nullptr;
  QDoubleSpinBox* originPosZ_ = nullptr;
  QDoubleSpinBox* originRoll_ = nullptr;
  QDoubleSpinBox* originPitch_ = nullptr;
  QDoubleSpinBox* originYaw_ = nullptr;

  // Axis
  QGroupBox* axisGroup_ = nullptr;
  QDoubleSpinBox* axisX_ = nullptr;
  QDoubleSpinBox* axisY_ = nullptr;
  QDoubleSpinBox* axisZ_ = nullptr;

  // Limits
  QGroupBox* limitsGroup_ = nullptr;
  QDoubleSpinBox* lowerLimit_ = nullptr;
  QDoubleSpinBox* upperLimit_ = nullptr;
  QDoubleSpinBox* effort_ = nullptr;
  QDoubleSpinBox* velocity_ = nullptr;

  // Dynamics
  QGroupBox* dynamicsGroup_ = nullptr;
  QDoubleSpinBox* damping_ = nullptr;
  QDoubleSpinBox* friction_ = nullptr;

  // Parent/child info (read-only)
  QLabel* parentLinkLabel_ = nullptr;
  QLabel* childLinkLabel_ = nullptr;

  // Saved state for undo
  URDFPose savedOrigin_;
  QVector3D savedAxis_;
  URDFJointLimits savedLimits_;
  URDFJointDynamics savedDynamics_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_JOINT_PROPERTIES_WIDGET_HPP
