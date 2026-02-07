#include "ros_weaver/widgets/urdf_gizmo.hpp"

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QGeometryRenderer>

#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QPhongMaterial>

#include <cmath>

namespace ros_weaver {

URDFGizmo::URDFGizmo(QObject* parent)
  : QObject(parent) {
}

URDFGizmo::~URDFGizmo() {
  destroyEntity();
}

void URDFGizmo::createEntity(Qt3DCore::QEntity* parent) {
  if (gizmoEntity_) {
    destroyEntity();
  }

  gizmoEntity_ = new Qt3DCore::QEntity(parent);

  gizmoTransform_ = new Qt3DCore::QTransform(gizmoEntity_);
  gizmoEntity_->addComponent(gizmoTransform_);

  // Create arrow for each axis
  // X axis - Red
  xArrow_ = createArrow(QColor(255, 50, 50), QVector3D(1, 0, 0));
  xArrow_->setParent(gizmoEntity_);

  // Y axis - Green
  yArrow_ = createArrow(QColor(50, 255, 50), QVector3D(0, 1, 0));
  yArrow_->setParent(gizmoEntity_);

  // Z axis - Blue
  zArrow_ = createArrow(QColor(50, 50, 255), QVector3D(0, 0, 1));
  zArrow_->setParent(gizmoEntity_);

  // Initially hidden
  gizmoEntity_->setEnabled(false);
}

Qt3DCore::QEntity* URDFGizmo::createArrow(const QColor& color, const QVector3D& direction) {
  Qt3DCore::QEntity* arrow = new Qt3DCore::QEntity();

  // Shaft
  Qt3DCore::QEntity* shaft = new Qt3DCore::QEntity(arrow);
  Qt3DExtras::QCylinderMesh* shaftMesh = new Qt3DExtras::QCylinderMesh(shaft);
  shaftMesh->setRadius(0.02f);
  shaftMesh->setLength(0.5f);
  shaftMesh->setSlices(12);

  Qt3DExtras::QPhongMaterial* shaftMaterial = new Qt3DExtras::QPhongMaterial(shaft);
  shaftMaterial->setAmbient(color.darker(120));
  shaftMaterial->setDiffuse(color);

  Qt3DCore::QTransform* shaftTransform = new Qt3DCore::QTransform(shaft);
  shaftTransform->setTranslation(direction * 0.25f);

  // Rotate to align with direction
  if (direction.x() > 0.5f) {
    shaftTransform->setRotationZ(-90.0f);
  } else if (direction.z() > 0.5f) {
    shaftTransform->setRotationX(90.0f);
  }
  // Y is default cylinder orientation

  shaft->addComponent(shaftMesh);
  shaft->addComponent(shaftMaterial);
  shaft->addComponent(shaftTransform);

  // Cone (arrow head)
  Qt3DCore::QEntity* head = new Qt3DCore::QEntity(arrow);
  Qt3DExtras::QConeMesh* coneMesh = new Qt3DExtras::QConeMesh(head);
  coneMesh->setBottomRadius(0.05f);
  coneMesh->setTopRadius(0.0f);
  coneMesh->setLength(0.1f);
  coneMesh->setSlices(12);

  Qt3DExtras::QPhongMaterial* headMaterial = new Qt3DExtras::QPhongMaterial(head);
  headMaterial->setAmbient(color.darker(120));
  headMaterial->setDiffuse(color);

  Qt3DCore::QTransform* headTransform = new Qt3DCore::QTransform(head);
  headTransform->setTranslation(direction * 0.55f);

  if (direction.x() > 0.5f) {
    headTransform->setRotationZ(-90.0f);
  } else if (direction.z() > 0.5f) {
    headTransform->setRotationX(90.0f);
  }

  head->addComponent(coneMesh);
  head->addComponent(headMaterial);
  head->addComponent(headTransform);

  return arrow;
}

void URDFGizmo::destroyEntity() {
  if (gizmoEntity_) {
    gizmoEntity_->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
    delete gizmoEntity_;
    gizmoEntity_ = nullptr;
    gizmoTransform_ = nullptr;
    xArrow_ = nullptr;
    yArrow_ = nullptr;
    zArrow_ = nullptr;
    xRing_ = nullptr;
    yRing_ = nullptr;
    zRing_ = nullptr;
  }
}

void URDFGizmo::setVisible(bool visible) {
  visible_ = visible;
  if (gizmoEntity_) {
    gizmoEntity_->setEnabled(visible);
  }
}

void URDFGizmo::setMode(Mode mode) {
  mode_ = mode;
  // Would switch between arrows (translate) and rings (rotate)
}

void URDFGizmo::setPosition(const QVector3D& position) {
  position_ = position;
  updateEntityTransform();
}

void URDFGizmo::setRotation(const QQuaternion& rotation) {
  rotation_ = rotation;
  updateEntityTransform();
}

void URDFGizmo::setCamera(Qt3DRender::QCamera* camera) {
  camera_ = camera;
}

void URDFGizmo::constrainToAxis(Axis axis) {
  constrainedAxis_ = axis;
}

void URDFGizmo::startDrag(const QPoint& screenPos, Axis axis) {
  isDragging_ = true;
  dragStartPos_ = screenPos;
  dragStartPosition_ = position_;
  dragStartRotation_ = rotation_;
  constrainedAxis_ = axis;

  emit dragStarted();
}

void URDFGizmo::updateDrag(const QPoint& screenPos) {
  if (!isDragging_ || !camera_) return;

  QPoint delta = screenPos - dragStartPos_;

  // Convert screen delta to world delta
  // This is a simplified version - real implementation would use proper ray casting
  float sensitivity = 0.01f;
  if (precisionEnabled_) {
    sensitivity *= precisionFactor_;
  }

  QVector3D worldDelta;

  switch (constrainedAxis_) {
    case Axis::X:
      worldDelta = QVector3D(delta.x() * sensitivity, 0, 0);
      break;
    case Axis::Y:
      worldDelta = QVector3D(0, delta.x() * sensitivity, 0);
      break;
    case Axis::Z:
      worldDelta = QVector3D(0, 0, -delta.y() * sensitivity);
      break;
    case Axis::Free:
    case Axis::None:
    default:
      worldDelta = QVector3D(delta.x() * sensitivity,
                             -delta.y() * sensitivity * 0.5f,
                             -delta.y() * sensitivity * 0.5f);
      break;
  }

  QVector3D newPosition = dragStartPosition_ + worldDelta;

  // Apply snap if enabled
  if (snapEnabled_) {
    newPosition.setX(snapValue(newPosition.x(), snapIncrement_));
    newPosition.setY(snapValue(newPosition.y(), snapIncrement_));
    newPosition.setZ(snapValue(newPosition.z(), snapIncrement_));
  }

  position_ = newPosition;
  updateEntityTransform();

  emit transformChanged(position_, rotation_);
}

void URDFGizmo::endDrag() {
  isDragging_ = false;
  constrainedAxis_ = Axis::Free;

  emit dragEnded();
}

URDFGizmo::Axis URDFGizmo::hitTest(const QPoint& /* screenPos */) {
  // Simplified - would do ray-axis intersection tests
  // For now, return None (clicking directly on model to drag)
  return Axis::None;
}

void URDFGizmo::updateEntityTransform() {
  if (gizmoTransform_) {
    gizmoTransform_->setTranslation(position_);
    // Don't apply rotation to gizmo itself - keep axes aligned with world
  }
}

QVector3D URDFGizmo::projectToPlane(const QPoint& /* screenPos */, const QVector3D& /* planeNormal */) {
  // Would project screen point to a plane for constrained movement
  return QVector3D();
}

QVector3D URDFGizmo::screenToWorldRay(const QPoint& /* screenPos */) {
  // Would convert screen position to world ray direction
  return QVector3D();
}

float URDFGizmo::snapValue(float value, float increment) {
  return std::round(value / increment) * increment;
}

}  // namespace ros_weaver
