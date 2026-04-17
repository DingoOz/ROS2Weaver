#include "ros_weaver/widgets/joint_properties_widget.hpp"
#include "ros_weaver/core/undo/urdf_undo_commands.hpp"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <cmath>

namespace ros_weaver {

// Helper to create a configured spinbox
static QDoubleSpinBox* createSpinBox(double min, double max, double step, int decimals) {
  auto* spin = new QDoubleSpinBox();
  spin->setRange(min, max);
  spin->setSingleStep(step);
  spin->setDecimals(decimals);
  spin->setKeyboardTracking(false);
  return spin;
}

JointPropertiesWidget::JointPropertiesWidget(QWidget* parent)
  : QWidget(parent) {
  setupUi();
  setupConnections();
}

JointPropertiesWidget::~JointPropertiesWidget() = default;

void JointPropertiesWidget::setModel(URDFModel* model) {
  model_ = model;
  refresh();
}

void JointPropertiesWidget::setUndoStack(QUndoStack* stack) {
  undoStack_ = stack;
}

void JointPropertiesWidget::setJointName(const QString& name) {
  jointName_ = name;
  refresh();
}

void JointPropertiesWidget::refresh() {
  updateFromModel();
}

void JointPropertiesWidget::onOriginPositionChanged() {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  URDFPose newOrigin = joint->origin;
  newOrigin.position.setX(static_cast<float>(originPosX_->value()));
  newOrigin.position.setY(static_cast<float>(originPosY_->value()));
  newOrigin.position.setZ(static_cast<float>(originPosZ_->value()));

  if (undoStack_) {
    undoStack_->push(new MoveJointOriginCommand(model_, jointName_, savedOrigin_, newOrigin));
  } else {
    joint->origin = newOrigin;
  }

  savedOrigin_ = newOrigin;
  emit propertyChanged();
}

void JointPropertiesWidget::onOriginOrientationChanged() {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  URDFPose newOrigin = joint->origin;
  // Convert degrees to radians
  newOrigin.rpy.setX(static_cast<float>(originRoll_->value() * M_PI / 180.0));
  newOrigin.rpy.setY(static_cast<float>(originPitch_->value() * M_PI / 180.0));
  newOrigin.rpy.setZ(static_cast<float>(originYaw_->value() * M_PI / 180.0));

  if (undoStack_) {
    undoStack_->push(new RotateJointOriginCommand(model_, jointName_, savedOrigin_, newOrigin));
  } else {
    joint->origin = newOrigin;
  }

  savedOrigin_ = newOrigin;
  emit propertyChanged();
}

void JointPropertiesWidget::onAxisChanged() {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  QVector3D newAxis(
    static_cast<float>(axisX_->value()),
    static_cast<float>(axisY_->value()),
    static_cast<float>(axisZ_->value())
  );

  // Normalize the axis
  if (newAxis.length() > 0.001f) {
    newAxis.normalize();
  }

  if (undoStack_) {
    undoStack_->push(new ModifyJointAxisCommand(model_, jointName_, savedAxis_, newAxis));
  } else {
    joint->axis = newAxis;
  }

  savedAxis_ = newAxis;
  emit propertyChanged();
}

void JointPropertiesWidget::onTypeChanged(int index) {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  URDFJointType newType = static_cast<URDFJointType>(typeCombo_->itemData(index).toInt());
  URDFJointType oldType = joint->type;
  if (newType == oldType) return;

  if (undoStack_) {
    undoStack_->push(new ModifyJointTypeCommand(model_, jointName_, oldType, newType));
  } else {
    joint->type = newType;
    model_->setModified(true);
  }

  updateLimitsEnabled();
  emit propertyChanged();
}

void JointPropertiesWidget::onLimitsChanged() {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  URDFJointLimits newLimits;
  newLimits.lower = lowerLimit_->value();
  newLimits.upper = upperLimit_->value();
  newLimits.effort = effort_->value();
  newLimits.velocity = velocity_->value();

  if (undoStack_) {
    undoStack_->push(new ModifyJointLimitsCommand(model_, jointName_, savedLimits_, newLimits));
  } else {
    joint->limits = newLimits;
  }

  savedLimits_ = newLimits;
  emit propertyChanged();
}

void JointPropertiesWidget::onDynamicsChanged() {
  if (!model_ || jointName_.isEmpty()) return;

  URDFJoint* joint = model_->joint(jointName_);
  if (!joint) return;

  URDFJointDynamics newDynamics;
  newDynamics.damping = damping_->value();
  newDynamics.friction = friction_->value();

  if (undoStack_) {
    undoStack_->push(new ModifyJointDynamicsCommand(model_, jointName_, savedDynamics_, newDynamics));
  } else {
    joint->dynamics = newDynamics;
    model_->setModified(true);
  }

  savedDynamics_ = newDynamics;
  emit propertyChanged();
}

void JointPropertiesWidget::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Joint type
  auto* typeLayout = new QFormLayout();
  typeCombo_ = new QComboBox(this);
  typeCombo_->addItem(tr("Revolute"), static_cast<int>(URDFJointType::Revolute));
  typeCombo_->addItem(tr("Continuous"), static_cast<int>(URDFJointType::Continuous));
  typeCombo_->addItem(tr("Prismatic"), static_cast<int>(URDFJointType::Prismatic));
  typeCombo_->addItem(tr("Fixed"), static_cast<int>(URDFJointType::Fixed));
  typeCombo_->addItem(tr("Floating"), static_cast<int>(URDFJointType::Floating));
  typeCombo_->addItem(tr("Planar"), static_cast<int>(URDFJointType::Planar));
  typeLayout->addRow(tr("Type:"), typeCombo_);
  mainLayout->addLayout(typeLayout);

  // Parent/child info
  auto* linkInfoLayout = new QFormLayout();
  parentLinkLabel_ = new QLabel("-");
  childLinkLabel_ = new QLabel("-");
  linkInfoLayout->addRow(tr("Parent:"), parentLinkLabel_);
  linkInfoLayout->addRow(tr("Child:"), childLinkLabel_);
  mainLayout->addLayout(linkInfoLayout);

  // Origin group
  originGroup_ = new QGroupBox(tr("Origin"), this);
  auto* originLayout = new QFormLayout(originGroup_);

  // Position row
  auto* posLayout = new QHBoxLayout();
  originPosX_ = createSpinBox(-100, 100, 0.01, 4);
  originPosY_ = createSpinBox(-100, 100, 0.01, 4);
  originPosZ_ = createSpinBox(-100, 100, 0.01, 4);
  posLayout->addWidget(new QLabel("X:"));
  posLayout->addWidget(originPosX_);
  posLayout->addWidget(new QLabel("Y:"));
  posLayout->addWidget(originPosY_);
  posLayout->addWidget(new QLabel("Z:"));
  posLayout->addWidget(originPosZ_);
  originLayout->addRow(tr("Position (m):"), posLayout);

  // Orientation row
  auto* rpyLayout = new QHBoxLayout();
  originRoll_ = createSpinBox(-180, 180, 1, 2);
  originPitch_ = createSpinBox(-180, 180, 1, 2);
  originYaw_ = createSpinBox(-180, 180, 1, 2);
  rpyLayout->addWidget(new QLabel("R:"));
  rpyLayout->addWidget(originRoll_);
  rpyLayout->addWidget(new QLabel("P:"));
  rpyLayout->addWidget(originPitch_);
  rpyLayout->addWidget(new QLabel("Y:"));
  rpyLayout->addWidget(originYaw_);
  originLayout->addRow(tr("Orientation (\u00B0):"), rpyLayout);

  mainLayout->addWidget(originGroup_);

  // Axis group
  axisGroup_ = new QGroupBox(tr("Axis"), this);
  auto* axisLayout = new QHBoxLayout(axisGroup_);
  axisX_ = createSpinBox(-1, 1, 0.1, 2);
  axisY_ = createSpinBox(-1, 1, 0.1, 2);
  axisZ_ = createSpinBox(-1, 1, 0.1, 2);
  axisLayout->addWidget(new QLabel("X:"));
  axisLayout->addWidget(axisX_);
  axisLayout->addWidget(new QLabel("Y:"));
  axisLayout->addWidget(axisY_);
  axisLayout->addWidget(new QLabel("Z:"));
  axisLayout->addWidget(axisZ_);
  mainLayout->addWidget(axisGroup_);

  // Limits group
  limitsGroup_ = new QGroupBox(tr("Limits"), this);
  auto* limitsLayout = new QFormLayout(limitsGroup_);
  lowerLimit_ = createSpinBox(-10, 10, 0.01, 4);
  upperLimit_ = createSpinBox(-10, 10, 0.01, 4);
  effort_ = createSpinBox(0, 10000, 1, 2);
  velocity_ = createSpinBox(0, 100, 0.1, 2);
  limitsLayout->addRow(tr("Lower (rad/m):"), lowerLimit_);
  limitsLayout->addRow(tr("Upper (rad/m):"), upperLimit_);
  limitsLayout->addRow(tr("Effort (N/Nm):"), effort_);
  limitsLayout->addRow(tr("Velocity (rad/m/s):"), velocity_);
  mainLayout->addWidget(limitsGroup_);

  // Dynamics group
  dynamicsGroup_ = new QGroupBox(tr("Dynamics"), this);
  auto* dynamicsLayout = new QFormLayout(dynamicsGroup_);
  damping_ = createSpinBox(0, 1000, 0.1, 4);
  friction_ = createSpinBox(0, 1000, 0.1, 4);
  dynamicsLayout->addRow(tr("Damping:"), damping_);
  dynamicsLayout->addRow(tr("Friction:"), friction_);
  mainLayout->addWidget(dynamicsGroup_);

  mainLayout->addStretch();
}

void JointPropertiesWidget::setupConnections() {
  // Type
  connect(typeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &JointPropertiesWidget::onTypeChanged);

  // Origin position
  connect(originPosX_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginPositionChanged);
  connect(originPosY_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginPositionChanged);
  connect(originPosZ_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginPositionChanged);

  // Origin orientation
  connect(originRoll_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginOrientationChanged);
  connect(originPitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginOrientationChanged);
  connect(originYaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onOriginOrientationChanged);

  // Axis
  connect(axisX_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onAxisChanged);
  connect(axisY_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onAxisChanged);
  connect(axisZ_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onAxisChanged);

  // Limits
  connect(lowerLimit_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onLimitsChanged);
  connect(upperLimit_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onLimitsChanged);
  connect(effort_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onLimitsChanged);
  connect(velocity_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onLimitsChanged);

  // Dynamics
  connect(damping_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onDynamicsChanged);
  connect(friction_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JointPropertiesWidget::onDynamicsChanged);
}

void JointPropertiesWidget::blockAllSignals(bool block) {
  typeCombo_->blockSignals(block);
  originPosX_->blockSignals(block);
  originPosY_->blockSignals(block);
  originPosZ_->blockSignals(block);
  originRoll_->blockSignals(block);
  originPitch_->blockSignals(block);
  originYaw_->blockSignals(block);
  axisX_->blockSignals(block);
  axisY_->blockSignals(block);
  axisZ_->blockSignals(block);
  lowerLimit_->blockSignals(block);
  upperLimit_->blockSignals(block);
  effort_->blockSignals(block);
  velocity_->blockSignals(block);
  damping_->blockSignals(block);
  friction_->blockSignals(block);
}

void JointPropertiesWidget::updateFromModel() {
  blockAllSignals(true);

  if (!model_ || jointName_.isEmpty()) {
    // Clear all
    typeCombo_->setCurrentIndex(typeCombo_->findData(static_cast<int>(URDFJointType::Fixed)));
    parentLinkLabel_->setText("-");
    childLinkLabel_->setText("-");
    originPosX_->setValue(0);
    originPosY_->setValue(0);
    originPosZ_->setValue(0);
    originRoll_->setValue(0);
    originPitch_->setValue(0);
    originYaw_->setValue(0);
    axisX_->setValue(1);
    axisY_->setValue(0);
    axisZ_->setValue(0);
    lowerLimit_->setValue(0);
    upperLimit_->setValue(0);
    effort_->setValue(0);
    velocity_->setValue(0);
    damping_->setValue(0);
    friction_->setValue(0);
    blockAllSignals(false);
    return;
  }

  const URDFJoint* joint = model_->joint(jointName_);
  if (!joint) {
    blockAllSignals(false);
    return;
  }

  // Type
  int typeIndex = typeCombo_->findData(static_cast<int>(joint->type));
  if (typeIndex >= 0) {
    typeCombo_->setCurrentIndex(typeIndex);
  }

  // Parent/child
  parentLinkLabel_->setText(joint->parentLink);
  childLinkLabel_->setText(joint->childLink);

  // Origin
  savedOrigin_ = joint->origin;
  originPosX_->setValue(static_cast<double>(joint->origin.position.x()));
  originPosY_->setValue(static_cast<double>(joint->origin.position.y()));
  originPosZ_->setValue(static_cast<double>(joint->origin.position.z()));
  originRoll_->setValue(static_cast<double>(joint->origin.rpy.x()) * 180.0 / M_PI);
  originPitch_->setValue(static_cast<double>(joint->origin.rpy.y()) * 180.0 / M_PI);
  originYaw_->setValue(static_cast<double>(joint->origin.rpy.z()) * 180.0 / M_PI);

  // Axis
  savedAxis_ = joint->axis;
  axisX_->setValue(static_cast<double>(joint->axis.x()));
  axisY_->setValue(static_cast<double>(joint->axis.y()));
  axisZ_->setValue(static_cast<double>(joint->axis.z()));

  // Limits
  savedLimits_ = joint->limits;
  lowerLimit_->setValue(joint->limits.lower);
  upperLimit_->setValue(joint->limits.upper);
  effort_->setValue(joint->limits.effort);
  velocity_->setValue(joint->limits.velocity);

  // Dynamics
  savedDynamics_ = joint->dynamics;
  damping_->setValue(joint->dynamics.damping);
  friction_->setValue(joint->dynamics.friction);

  updateLimitsEnabled();
  blockAllSignals(false);
}

void JointPropertiesWidget::updateLimitsEnabled() {
  if (!model_ || jointName_.isEmpty()) {
    limitsGroup_->setEnabled(false);
    axisGroup_->setEnabled(false);
    return;
  }

  const URDFJoint* joint = model_->joint(jointName_);
  if (!joint) {
    limitsGroup_->setEnabled(false);
    axisGroup_->setEnabled(false);
    return;
  }

  // Limits only apply to revolute and prismatic joints
  bool hasLimits = (joint->type == URDFJointType::Revolute ||
                    joint->type == URDFJointType::Prismatic);
  limitsGroup_->setEnabled(hasLimits);

  // Axis applies to all movable joint types
  bool hasAxis = (joint->type != URDFJointType::Fixed &&
                  joint->type != URDFJointType::Floating);
  axisGroup_->setEnabled(hasAxis);
}

}  // namespace ros_weaver
