#include "ros_weaver/widgets/link_properties_widget.hpp"
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

LinkPropertiesWidget::LinkPropertiesWidget(QWidget* parent)
  : QWidget(parent) {
  setupUi();
  setupConnections();
}

LinkPropertiesWidget::~LinkPropertiesWidget() = default;

void LinkPropertiesWidget::setModel(URDFModel* model) {
  model_ = model;
  refresh();
}

void LinkPropertiesWidget::setUndoStack(QUndoStack* stack) {
  undoStack_ = stack;
}

void LinkPropertiesWidget::setLinkName(const QString& name) {
  linkName_ = name;
  refresh();
}

void LinkPropertiesWidget::refresh() {
  updateFromModel();
}

void LinkPropertiesWidget::onVisualPositionChanged() {
  if (!model_ || linkName_.isEmpty()) return;

  URDFLink* link = model_->link(linkName_);
  if (!link || link->visuals.isEmpty()) return;

  URDFPose newPose = link->visuals[0].origin;
  newPose.position.setX(static_cast<float>(visualPosX_->value()));
  newPose.position.setY(static_cast<float>(visualPosY_->value()));
  newPose.position.setZ(static_cast<float>(visualPosZ_->value()));

  if (undoStack_) {
    undoStack_->push(new MoveLinkOriginCommand(model_, linkName_, savedVisualPose_, newPose));
  } else {
    link->visuals[0].origin = newPose;
  }

  savedVisualPose_ = newPose;
  emit propertyChanged();
}

void LinkPropertiesWidget::onVisualOrientationChanged() {
  if (!model_ || linkName_.isEmpty()) return;

  URDFLink* link = model_->link(linkName_);
  if (!link || link->visuals.isEmpty()) return;

  URDFPose newPose = link->visuals[0].origin;
  // Convert degrees to radians for storage
  newPose.rpy.setX(static_cast<float>(visualRoll_->value() * M_PI / 180.0));
  newPose.rpy.setY(static_cast<float>(visualPitch_->value() * M_PI / 180.0));
  newPose.rpy.setZ(static_cast<float>(visualYaw_->value() * M_PI / 180.0));

  if (undoStack_) {
    undoStack_->push(new RotateLinkOriginCommand(model_, linkName_, savedVisualPose_, newPose));
  } else {
    link->visuals[0].origin = newPose;
  }

  savedVisualPose_ = newPose;
  emit propertyChanged();
}

void LinkPropertiesWidget::onInertialChanged() {
  if (!model_ || linkName_.isEmpty()) return;

  URDFLink* link = model_->link(linkName_);
  if (!link) return;

  URDFInertial newInertial = link->inertial;
  newInertial.ixx = ixx_->value();
  newInertial.ixy = ixy_->value();
  newInertial.ixz = ixz_->value();
  newInertial.iyy = iyy_->value();
  newInertial.iyz = iyz_->value();
  newInertial.izz = izz_->value();

  if (undoStack_) {
    undoStack_->push(new ModifyLinkInertialCommand(model_, linkName_, savedInertial_, newInertial));
  } else {
    link->inertial = newInertial;
  }

  savedInertial_ = newInertial;
  emit propertyChanged();
}

void LinkPropertiesWidget::onMassChanged() {
  if (!model_ || linkName_.isEmpty()) return;

  URDFLink* link = model_->link(linkName_);
  if (!link) return;

  URDFInertial newInertial = link->inertial;
  newInertial.mass = mass_->value();

  if (undoStack_) {
    undoStack_->push(new ModifyLinkInertialCommand(model_, linkName_, savedInertial_, newInertial));
  } else {
    link->inertial = newInertial;
  }

  savedInertial_ = newInertial;
  emit propertyChanged();
}

void LinkPropertiesWidget::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Visual origin group
  visualGroup_ = new QGroupBox(tr("Visual Origin"), this);
  auto* visualLayout = new QFormLayout(visualGroup_);

  // Position row
  auto* posLayout = new QHBoxLayout();
  visualPosX_ = createSpinBox(-100, 100, 0.01, 4);
  visualPosY_ = createSpinBox(-100, 100, 0.01, 4);
  visualPosZ_ = createSpinBox(-100, 100, 0.01, 4);
  posLayout->addWidget(new QLabel("X:"));
  posLayout->addWidget(visualPosX_);
  posLayout->addWidget(new QLabel("Y:"));
  posLayout->addWidget(visualPosY_);
  posLayout->addWidget(new QLabel("Z:"));
  posLayout->addWidget(visualPosZ_);
  visualLayout->addRow(tr("Position (m):"), posLayout);

  // Orientation row (in degrees for user convenience)
  auto* rpyLayout = new QHBoxLayout();
  visualRoll_ = createSpinBox(-180, 180, 1, 2);
  visualPitch_ = createSpinBox(-180, 180, 1, 2);
  visualYaw_ = createSpinBox(-180, 180, 1, 2);
  rpyLayout->addWidget(new QLabel("R:"));
  rpyLayout->addWidget(visualRoll_);
  rpyLayout->addWidget(new QLabel("P:"));
  rpyLayout->addWidget(visualPitch_);
  rpyLayout->addWidget(new QLabel("Y:"));
  rpyLayout->addWidget(visualYaw_);
  visualLayout->addRow(tr("Orientation (\u00B0):"), rpyLayout);

  mainLayout->addWidget(visualGroup_);

  // Inertial group
  inertialGroup_ = new QGroupBox(tr("Inertial"), this);
  auto* inertialLayout = new QFormLayout(inertialGroup_);

  mass_ = createSpinBox(0, 10000, 0.1, 4);
  inertialLayout->addRow(tr("Mass (kg):"), mass_);

  // Inertia tensor
  auto* inertiaLayout = new QGridLayout();
  ixx_ = createSpinBox(-1000, 1000, 0.001, 6);
  ixy_ = createSpinBox(-1000, 1000, 0.001, 6);
  ixz_ = createSpinBox(-1000, 1000, 0.001, 6);
  iyy_ = createSpinBox(-1000, 1000, 0.001, 6);
  iyz_ = createSpinBox(-1000, 1000, 0.001, 6);
  izz_ = createSpinBox(-1000, 1000, 0.001, 6);

  inertiaLayout->addWidget(new QLabel("Ixx:"), 0, 0);
  inertiaLayout->addWidget(ixx_, 0, 1);
  inertiaLayout->addWidget(new QLabel("Ixy:"), 0, 2);
  inertiaLayout->addWidget(ixy_, 0, 3);
  inertiaLayout->addWidget(new QLabel("Ixz:"), 0, 4);
  inertiaLayout->addWidget(ixz_, 0, 5);

  inertiaLayout->addWidget(new QLabel("Iyy:"), 1, 0);
  inertiaLayout->addWidget(iyy_, 1, 1);
  inertiaLayout->addWidget(new QLabel("Iyz:"), 1, 2);
  inertiaLayout->addWidget(iyz_, 1, 3);
  inertiaLayout->addWidget(new QLabel("Izz:"), 1, 4);
  inertiaLayout->addWidget(izz_, 1, 5);

  inertialLayout->addRow(tr("Inertia:"), inertiaLayout);

  mainLayout->addWidget(inertialGroup_);

  mainLayout->addStretch();
}

void LinkPropertiesWidget::setupConnections() {
  // Visual position
  connect(visualPosX_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualPositionChanged);
  connect(visualPosY_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualPositionChanged);
  connect(visualPosZ_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualPositionChanged);

  // Visual orientation
  connect(visualRoll_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualOrientationChanged);
  connect(visualPitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualOrientationChanged);
  connect(visualYaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onVisualOrientationChanged);

  // Mass
  connect(mass_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onMassChanged);

  // Inertia tensor
  connect(ixx_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
  connect(ixy_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
  connect(ixz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
  connect(iyy_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
  connect(iyz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
  connect(izz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LinkPropertiesWidget::onInertialChanged);
}

void LinkPropertiesWidget::blockAllSignals(bool block) {
  visualPosX_->blockSignals(block);
  visualPosY_->blockSignals(block);
  visualPosZ_->blockSignals(block);
  visualRoll_->blockSignals(block);
  visualPitch_->blockSignals(block);
  visualYaw_->blockSignals(block);
  mass_->blockSignals(block);
  ixx_->blockSignals(block);
  ixy_->blockSignals(block);
  ixz_->blockSignals(block);
  iyy_->blockSignals(block);
  iyz_->blockSignals(block);
  izz_->blockSignals(block);
}

void LinkPropertiesWidget::updateFromModel() {
  blockAllSignals(true);

  if (!model_ || linkName_.isEmpty()) {
    // Clear all
    visualPosX_->setValue(0);
    visualPosY_->setValue(0);
    visualPosZ_->setValue(0);
    visualRoll_->setValue(0);
    visualPitch_->setValue(0);
    visualYaw_->setValue(0);
    mass_->setValue(0);
    ixx_->setValue(0);
    ixy_->setValue(0);
    ixz_->setValue(0);
    iyy_->setValue(0);
    iyz_->setValue(0);
    izz_->setValue(0);
    blockAllSignals(false);
    return;
  }

  const URDFLink* link = model_->link(linkName_);
  if (!link) {
    blockAllSignals(false);
    return;
  }

  // Visual origin
  if (!link->visuals.isEmpty()) {
    const URDFPose& pose = link->visuals[0].origin;
    savedVisualPose_ = pose;

    visualPosX_->setValue(static_cast<double>(pose.position.x()));
    visualPosY_->setValue(static_cast<double>(pose.position.y()));
    visualPosZ_->setValue(static_cast<double>(pose.position.z()));

    // Convert radians to degrees for display
    visualRoll_->setValue(static_cast<double>(pose.rpy.x()) * 180.0 / M_PI);
    visualPitch_->setValue(static_cast<double>(pose.rpy.y()) * 180.0 / M_PI);
    visualYaw_->setValue(static_cast<double>(pose.rpy.z()) * 180.0 / M_PI);
  }

  // Inertial
  savedInertial_ = link->inertial;
  mass_->setValue(link->inertial.mass);
  ixx_->setValue(link->inertial.ixx);
  ixy_->setValue(link->inertial.ixy);
  ixz_->setValue(link->inertial.ixz);
  iyy_->setValue(link->inertial.iyy);
  iyz_->setValue(link->inertial.iyz);
  izz_->setValue(link->inertial.izz);

  blockAllSignals(false);
}

}  // namespace ros_weaver
