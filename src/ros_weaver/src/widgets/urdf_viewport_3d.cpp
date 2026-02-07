#include "ros_weaver/widgets/urdf_viewport_3d.hpp"
#include "ros_weaver/widgets/urdf_gizmo.hpp"
#include "ros_weaver/core/urdf_model.hpp"
#include "ros_weaver/core/urdf_parser.hpp"

#include <QVBoxLayout>
#include <QResizeEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFileInfo>
#include <QtMath>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QPickingSettings>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DInput/QInputSettings>

#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QText2DEntity>

namespace ros_weaver {

URDFViewport3D::URDFViewport3D(QWidget* parent)
  : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  // Create Qt3D window and embed it
  view3d_ = new Qt3DExtras::Qt3DWindow();
  view3d_->defaultFrameGraph()->setClearColor(QColor(45, 45, 48));

  view3dContainer_ = QWidget::createWindowContainer(view3d_, this);
  view3dContainer_->installEventFilter(this);  // Capture mouse events
  layout->addWidget(view3dContainer_);

  setupScene();
}

URDFViewport3D::~URDFViewport3D() {
  // Destroy gizmo before view3d_ to avoid use-after-free:
  // gizmo_ is parented to 'this' and would normally be destroyed by ~QObject
  // AFTER ~QWidget destroys view3d_ and all Qt3D entities, causing gizmo_->destroyEntity()
  // to access already-freed memory.
  delete gizmo_;
  gizmo_ = nullptr;
  clearScene();
}

void URDFViewport3D::setModel(URDFModel* model) {
  model_ = model;
  rebuildScene();
}

void URDFViewport3D::resetView() {
  if (!camera_) return;

  camera_->setPosition(QVector3D(3.0f, 3.0f, 3.0f));
  camera_->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));
  camera_->setUpVector(QVector3D(0.0f, 0.0f, 1.0f));
}

void URDFViewport3D::setCameraView(const QVector3D& position, const QVector3D& upVector) {
  if (!camera_) return;

  camera_->setPosition(position);
  camera_->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));
  camera_->setUpVector(upVector);
}

void URDFViewport3D::setViewFront() {
  // Looking from +Y towards origin (Numpad 1)
  setCameraView(QVector3D(0.0f, viewDistance_, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
}

void URDFViewport3D::setViewBack() {
  // Looking from -Y towards origin (Ctrl+Numpad 1)
  setCameraView(QVector3D(0.0f, -viewDistance_, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
}

void URDFViewport3D::setViewRight() {
  // Looking from +X towards origin (Numpad 3)
  setCameraView(QVector3D(viewDistance_, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
}

void URDFViewport3D::setViewLeft() {
  // Looking from -X towards origin (Ctrl+Numpad 3)
  setCameraView(QVector3D(-viewDistance_, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
}

void URDFViewport3D::setViewTop() {
  // Looking from +Z down (Numpad 7)
  setCameraView(QVector3D(0.0f, 0.0f, viewDistance_), QVector3D(0.0f, 1.0f, 0.0f));
}

void URDFViewport3D::setViewBottom() {
  // Looking from -Z up (Ctrl+Numpad 7)
  setCameraView(QVector3D(0.0f, 0.0f, -viewDistance_), QVector3D(0.0f, 1.0f, 0.0f));
}

void URDFViewport3D::focusOnSelected() {
  if (!camera_ || !selectedEntity_) return;

  // Use world position for correct behavior in nested hierarchies
  QVector3D center = getWorldPosition(selectedEntity_);
  camera_->setViewCenter(center);
}

void URDFViewport3D::setGridVisible(bool visible) {
  gridVisible_ = visible;
  if (gridEntity_) {
    gridEntity_->setEnabled(visible);
  }
}

void URDFViewport3D::setWireframeMode(bool wireframe) {
  wireframeMode_ = wireframe;
  // Would need to switch materials to wireframe - simplified for now
}

void URDFViewport3D::selectElement(const QString& name, bool isJoint) {
  // Deselect previous
  if (selectedEntity_) {
    highlightEntity(selectedEntity_, false);
  }

  selectedName_ = name;
  selectedIsJoint_ = isJoint;

  // Find and highlight new selection
  if (isJoint) {
    selectedEntity_ = jointEntities_.value(name, nullptr);
  } else {
    selectedEntity_ = linkEntities_.value(name, nullptr);
  }

  if (selectedEntity_) {
    highlightEntity(selectedEntity_, true);

    // Position gizmo at selected entity using FindDirectChildrenOnly to avoid
    // accidentally finding a child entity's transform
    Qt3DCore::QTransform* transform = selectedEntity_->findChild<Qt3DCore::QTransform*>(QString(), Qt::FindDirectChildrenOnly);
    if (transform && gizmo_) {
      gizmo_->setPosition(transform->translation());
      gizmo_->setRotation(transform->rotation());
      gizmo_->setVisible(true);
    }
  } else if (gizmo_) {
    gizmo_->setVisible(false);
  }
}

void URDFViewport3D::refresh() {
  // Update transforms from model data
  if (!model_) return;

  for (auto it = linkEntities_.constBegin(); it != linkEntities_.constEnd(); ++it) {
    const URDFLink* link = model_->link(it.key());
    if (!link || link->visuals.isEmpty()) continue;

    Qt3DCore::QTransform* transform = it.value()->findChild<Qt3DCore::QTransform*>(QString(), Qt::FindDirectChildrenOnly);
    if (transform) {
      const URDFPose& pose = link->visuals[0].origin;
      transform->setTranslation(pose.position);
      transform->setRotation(pose.toQuaternion());
    }
  }
}

void URDFViewport3D::rebuildScene() {
  clearScene();

  if (!rootEntity_) {
    setupScene();
  }

  buildModelEntities();

  // Set up overlays after model is built
  if (labelsVisible_) {
    setupLabels();
  }
  if (hierarchyLinesVisible_) {
    setupHierarchyLines();
  }
  if (localAxesVisible_) {
    setupLocalAxes();
  }
}

void URDFViewport3D::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);

  // Update camera aspect ratio to match widget dimensions
  if (camera_ && event->size().height() > 0) {
    float aspect = static_cast<float>(event->size().width()) / static_cast<float>(event->size().height());
    camera_->lens()->setAspectRatio(aspect);
  }
}

void URDFViewport3D::keyPressEvent(QKeyEvent* event) {
  ctrlPressed_ = event->modifiers() & Qt::ControlModifier;
  shiftPressed_ = event->modifiers() & Qt::ShiftModifier;

  if (gizmo_) {
    gizmo_->setSnapEnabled(ctrlPressed_);
    gizmo_->setPrecisionEnabled(shiftPressed_);

    // Axis constraint keys
    if (gizmo_->isDragging()) {
      switch (event->key()) {
        case Qt::Key_X:
          gizmo_->constrainToAxis(URDFGizmo::Axis::X);
          break;
        case Qt::Key_Y:
          gizmo_->constrainToAxis(URDFGizmo::Axis::Y);
          break;
        case Qt::Key_Z:
          gizmo_->constrainToAxis(URDFGizmo::Axis::Z);
          break;
        case Qt::Key_Escape:
          gizmo_->endDrag();
          break;
      }
    }
  }

  // Blender-style numpad view shortcuts
  switch (event->key()) {
    case Qt::Key_1:
      if (event->modifiers() & Qt::KeypadModifier) {
        if (ctrlPressed_) {
          setViewBack();
        } else {
          setViewFront();
        }
        return;
      }
      break;
    case Qt::Key_3:
      if (event->modifiers() & Qt::KeypadModifier) {
        if (ctrlPressed_) {
          setViewLeft();
        } else {
          setViewRight();
        }
        return;
      }
      break;
    case Qt::Key_7:
      if (event->modifiers() & Qt::KeypadModifier) {
        if (ctrlPressed_) {
          setViewBottom();
        } else {
          setViewTop();
        }
        return;
      }
      break;
    case Qt::Key_5:
      // Numpad 5 - toggle perspective/orthographic (future enhancement)
      if (event->modifiers() & Qt::KeypadModifier) {
        return;
      }
      break;
  }

  QWidget::keyPressEvent(event);
}

void URDFViewport3D::keyReleaseEvent(QKeyEvent* event) {
  ctrlPressed_ = event->modifiers() & Qt::ControlModifier;
  shiftPressed_ = event->modifiers() & Qt::ShiftModifier;

  if (gizmo_) {
    gizmo_->setSnapEnabled(ctrlPressed_);
    gizmo_->setPrecisionEnabled(shiftPressed_);
  }

  QWidget::keyReleaseEvent(event);
}

void URDFViewport3D::mousePressEvent(QMouseEvent* event) {
  // Right mouse button for orbiting
  if (event->button() == Qt::RightButton) {
    middleMousePressed_ = true;  // Reusing the flag for orbit state
    lastMousePos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QWidget::mousePressEvent(event);
}

void URDFViewport3D::orbitCamera(const QPoint& delta) {
  if (!camera_) return;

  QVector3D viewCenter = camera_->viewCenter();
  QVector3D position = camera_->position();
  QVector3D offset = position - viewCenter;
  float distance = offset.length();

  // Guard against zero distance (camera at view center)
  if (distance < 1e-5f) return;

  // Convert to spherical coordinates with clamped acos input
  float theta = qAtan2(offset.x(), offset.y());
  float phi = qAcos(qBound(-1.0f, offset.z() / distance, 1.0f));

  // Update angles based on mouse movement
  theta -= delta.x() * orbitSpeed_ * 0.01f;
  phi += delta.y() * orbitSpeed_ * 0.01f;

  // Clamp phi to avoid flipping
  phi = qBound(0.01f, phi, static_cast<float>(M_PI) - 0.01f);

  // Convert back to Cartesian coordinates
  QVector3D newOffset;
  newOffset.setX(distance * qSin(phi) * qSin(theta));
  newOffset.setY(distance * qSin(phi) * qCos(theta));
  newOffset.setZ(distance * qCos(phi));

  camera_->setPosition(viewCenter + newOffset);
  camera_->setUpVector(QVector3D(0, 0, 1));
}

void URDFViewport3D::zoomCamera(int angleDeltaY) {
  if (!camera_) return;

  QVector3D viewCenter = camera_->viewCenter();
  QVector3D position = camera_->position();
  QVector3D direction = position - viewCenter;
  float distance = direction.length();

  // Guard against zero-length direction (would produce NaN on normalize)
  if (distance < 1e-5f) return;

  float zoomFactor = angleDeltaY > 0 ? 0.9f : 1.1f;
  distance *= zoomFactor;
  distance = qBound(0.5f, distance, 100.0f);

  direction.normalize();
  camera_->setPosition(viewCenter + direction * distance);
  viewDistance_ = distance;
}

void URDFViewport3D::mouseMoveEvent(QMouseEvent* event) {
  if (middleMousePressed_ && camera_) {
    QPoint delta = event->pos() - lastMousePos_;
    lastMousePos_ = event->pos();
    orbitCamera(delta);
    event->accept();
    return;
  }
  QWidget::mouseMoveEvent(event);
}

void URDFViewport3D::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton) {
    middleMousePressed_ = false;
    setCursor(Qt::ArrowCursor);
    event->accept();
    return;
  }
  QWidget::mouseReleaseEvent(event);
}

void URDFViewport3D::wheelEvent(QWheelEvent* event) {
  if (camera_) {
    zoomCamera(event->angleDelta().y());
    event->accept();
    return;
  }
  QWidget::wheelEvent(event);
}

bool URDFViewport3D::eventFilter(QObject* watched, QEvent* event) {
  if (watched == view3dContainer_) {
    if (event->type() == QEvent::MouseButtonPress) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
      if (mouseEvent->button() == Qt::RightButton) {
        middleMousePressed_ = true;
        lastMousePos_ = mouseEvent->pos();
        view3dContainer_->setCursor(Qt::ClosedHandCursor);
        return true;  // Consume the event
      }
      // Let left clicks through for Qt3D picking
    }
    else if (event->type() == QEvent::MouseMove) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
      if (middleMousePressed_ && camera_) {
        QPoint delta = mouseEvent->pos() - lastMousePos_;
        lastMousePos_ = mouseEvent->pos();
        orbitCamera(delta);
        return true;  // Consume the event
      }
    }
    else if (event->type() == QEvent::MouseButtonRelease) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
      if (mouseEvent->button() == Qt::RightButton) {
        middleMousePressed_ = false;
        view3dContainer_->setCursor(Qt::ArrowCursor);
        return true;  // Consume the event
      }
    }
    else if (event->type() == QEvent::Wheel) {
      QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
      if (camera_) {
        zoomCamera(wheelEvent->angleDelta().y());
        return true;  // Consume the event
      }
    }
  }
  return QWidget::eventFilter(watched, event);
}

// Helper function (not a class member, used in onEntityPicked)
static bool isChildOf(Qt3DCore::QEntity* entity, Qt3DCore::QEntity* parent) {
  Qt3DCore::QNode* node = entity->parentNode();
  while (node) {
    if (node == parent) return true;
    node = node->parentNode();
  }
  return false;
}

void URDFViewport3D::onEntityPicked(Qt3DRender::QPickEvent* event) {
  Qt3DCore::QEntity* entity = qobject_cast<Qt3DCore::QEntity*>(event->entity());
  if (!entity) {
    return;
  }

  // Find which link/joint this entity belongs to
  for (auto it = linkEntities_.constBegin(); it != linkEntities_.constEnd(); ++it) {
    if (it.value() == entity || isChildOf(entity, it.value())) {
      emit selectionChanged(it.key(), false);
      return;
    }
  }

  for (auto it = jointEntities_.constBegin(); it != jointEntities_.constEnd(); ++it) {
    if (it.value() == entity || isChildOf(entity, it.value())) {
      emit selectionChanged(it.key(), true);
      return;
    }
  }
}

void URDFViewport3D::onGizmoTransformChanged(const QVector3D& position, const QQuaternion& rotation) {
  if (!model_ || selectedName_.isEmpty()) return;

  if (selectedIsJoint_) {
    URDFJoint* joint = model_->joint(selectedName_);
    if (joint) {
      joint->origin = URDFPose::fromQuaternion(position, rotation);
    }
  } else {
    URDFLink* link = model_->link(selectedName_);
    if (link && !link->visuals.isEmpty()) {
      link->visuals[0].origin = URDFPose::fromQuaternion(position, rotation);
    }
  }

  model_->setModified(true);
  emit transformChanged();
}

void URDFViewport3D::setupScene() {
  rootEntity_ = new Qt3DCore::QEntity();
  view3d_->setRootEntity(rootEntity_);

  // Configure picking settings for mouse selection
  Qt3DRender::QPickingSettings* pickingSettings = view3d_->renderSettings()->pickingSettings();
  pickingSettings->setPickMethod(Qt3DRender::QPickingSettings::TrianglePicking);
  pickingSettings->setPickResultMode(Qt3DRender::QPickingSettings::NearestPick);
  pickingSettings->setFaceOrientationPickingMode(Qt3DRender::QPickingSettings::FrontAndBackFace);
  pickingSettings->setWorldSpaceTolerance(0.1f);

  // Enable input handling for picking
  Qt3DInput::QInputSettings* inputSettings = new Qt3DInput::QInputSettings();
  inputSettings->setEventSource(view3d_);
  rootEntity_->addComponent(inputSettings);

  setupCamera();
  setupLighting();
  setupGrid();
  setupGizmo();

  modelRoot_ = new Qt3DCore::QEntity(rootEntity_);
}

void URDFViewport3D::setupCamera() {
  camera_ = view3d_->camera();
  // Aspect ratio will be updated in resizeEvent; use 1.0 as initial placeholder
  camera_->lens()->setPerspectiveProjection(45.0f, 1.0f, 0.01f, 1000.0f);
  camera_->setPosition(QVector3D(3.0f, 3.0f, 3.0f));
  camera_->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));
  camera_->setUpVector(QVector3D(0.0f, 0.0f, 1.0f));

  // Note: We don't use QOrbitCameraController because:
  // 1. We want custom right-mouse-button orbiting
  // 2. We need left-click for object picking/selection
  // Camera control is handled in mousePressEvent/mouseMoveEvent/wheelEvent
  cameraController_ = nullptr;
}

void URDFViewport3D::setupLighting() {
  // Main light
  Qt3DCore::QEntity* lightEntity = new Qt3DCore::QEntity(rootEntity_);
  Qt3DRender::QPointLight* light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor(Qt::white);
  light->setIntensity(1.0f);

  Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(QVector3D(5.0f, 5.0f, 10.0f));

  lightEntity->addComponent(light);
  lightEntity->addComponent(lightTransform);

  // Fill light
  Qt3DCore::QEntity* fillLightEntity = new Qt3DCore::QEntity(rootEntity_);
  Qt3DRender::QPointLight* fillLight = new Qt3DRender::QPointLight(fillLightEntity);
  fillLight->setColor(QColor(200, 200, 255));
  fillLight->setIntensity(0.5f);

  Qt3DCore::QTransform* fillLightTransform = new Qt3DCore::QTransform(fillLightEntity);
  fillLightTransform->setTranslation(QVector3D(-5.0f, -5.0f, 5.0f));

  fillLightEntity->addComponent(fillLight);
  fillLightEntity->addComponent(fillLightTransform);
}

void URDFViewport3D::setupGrid() {
  gridEntity_ = new Qt3DCore::QEntity(rootEntity_);

  // Create a grid using a plane mesh
  Qt3DExtras::QPlaneMesh* planeMesh = new Qt3DExtras::QPlaneMesh(gridEntity_);
  planeMesh->setWidth(10.0f);
  planeMesh->setHeight(10.0f);

  Qt3DExtras::QPhongMaterial* gridMaterial = new Qt3DExtras::QPhongMaterial(gridEntity_);
  gridMaterial->setAmbient(QColor(60, 60, 60));
  gridMaterial->setDiffuse(QColor(80, 80, 80));

  Qt3DCore::QTransform* gridTransform = new Qt3DCore::QTransform(gridEntity_);
  gridTransform->setRotationX(90.0f);  // Rotate to XY plane

  gridEntity_->addComponent(planeMesh);
  gridEntity_->addComponent(gridMaterial);
  gridEntity_->addComponent(gridTransform);
}

void URDFViewport3D::setupGizmo() {
  gizmo_ = new URDFGizmo(this);
  gizmo_->setCamera(camera_);
  gizmo_->createEntity(rootEntity_);
  gizmo_->setVisible(false);

  connect(gizmo_, &URDFGizmo::transformChanged,
          this, &URDFViewport3D::onGizmoTransformChanged);
}

void URDFViewport3D::clearScene() {
  linkEntities_.clear();
  jointEntities_.clear();

  if (modelRoot_) {
    // Delete all children
    auto children = modelRoot_->childNodes();
    for (auto* child : children) {
      child->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
      delete child;
    }
  }

  selectedEntity_ = nullptr;
  selectedName_.clear();
}

void URDFViewport3D::buildModelEntities() {
  if (!model_ || !modelRoot_) return;

  // Start from root link
  QString rootName = model_->rootLinkName();
  if (!rootName.isEmpty()) {
    createLinkEntity(rootName, modelRoot_);
  }
}

void URDFViewport3D::createLinkEntity(const QString& linkName, Qt3DCore::QEntity* parent) {
  const URDFLink* link = model_->link(linkName);
  if (!link) return;

  Qt3DCore::QEntity* linkEntity = new Qt3DCore::QEntity(parent);
  linkEntities_[linkName] = linkEntity;

  // Create visual entities
  for (const URDFVisual& visual : link->visuals) {
    createVisualEntity(visual, linkEntity);
  }

  // Add object picker for selection
  Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(linkEntity);
  picker->setHoverEnabled(false);
  connect(picker, &Qt3DRender::QObjectPicker::pressed,
          this, &URDFViewport3D::onEntityPicked);
  linkEntity->addComponent(picker);

  // Process child joints and links
  const auto& joints = model_->joints();
  for (auto it = joints.constBegin(); it != joints.constEnd(); ++it) {
    if (it->parentLink == linkName) {
      // Create joint entity
      Qt3DCore::QEntity* jointEntity = new Qt3DCore::QEntity(linkEntity);
      jointEntities_[it->name] = jointEntity;

      Qt3DCore::QTransform* jointTransform = new Qt3DCore::QTransform(jointEntity);
      jointTransform->setTranslation(it->origin.position);
      jointTransform->setRotation(it->origin.toQuaternion());
      jointEntity->addComponent(jointTransform);

      // Recursively create child link
      createLinkEntity(it->childLink, jointEntity);
    }
  }
}

void URDFViewport3D::createVisualEntity(const URDFVisual& visual, Qt3DCore::QEntity* parent) {
  Qt3DCore::QEntity* visualEntity = new Qt3DCore::QEntity(parent);

  // Transform
  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform(visualEntity);
  transform->setTranslation(visual.origin.position);
  transform->setRotation(visual.origin.toQuaternion());
  visualEntity->addComponent(transform);

  // Material - only add to geometry entity, not visual entity, since
  // a Qt3D component can only belong to one entity at a time
  Qt3DExtras::QPhongMaterial* material = createMaterial(visual.material);

  // Geometry
  Qt3DCore::QEntity* geometryEntity = nullptr;
  switch (visual.geometry.type) {
    case URDFGeometryType::Box:
      geometryEntity = createBoxEntity(visual.geometry.boxSize, material);
      break;
    case URDFGeometryType::Cylinder:
      geometryEntity = createCylinderEntity(visual.geometry.cylinderRadius,
                                            visual.geometry.cylinderLength, material);
      break;
    case URDFGeometryType::Sphere:
      geometryEntity = createSphereEntity(visual.geometry.sphereRadius, material);
      break;
    case URDFGeometryType::Mesh:
      geometryEntity = createMeshEntity(visual.geometry.meshFilename,
                                        visual.geometry.meshScale, material);
      break;
    default:
      qWarning() << "Unknown geometry type for visual" << visual.name;
      break;
  }

  if (geometryEntity) {
    geometryEntity->setParent(visualEntity);
  }
}

Qt3DCore::QEntity* URDFViewport3D::createBoxEntity(const QVector3D& size, Qt3DRender::QMaterial* material) {
  Qt3DCore::QEntity* entity = new Qt3DCore::QEntity();

  Qt3DExtras::QCuboidMesh* mesh = new Qt3DExtras::QCuboidMesh(entity);
  mesh->setXExtent(size.x());
  mesh->setYExtent(size.y());
  mesh->setZExtent(size.z());

  // Add object picker for selection
  Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(entity);
  picker->setHoverEnabled(false);
  connect(picker, &Qt3DRender::QObjectPicker::pressed,
          this, &URDFViewport3D::onEntityPicked);

  entity->addComponent(mesh);
  entity->addComponent(material);
  entity->addComponent(picker);

  return entity;
}

Qt3DCore::QEntity* URDFViewport3D::createCylinderEntity(double radius, double length, Qt3DRender::QMaterial* material) {
  Qt3DCore::QEntity* entity = new Qt3DCore::QEntity();

  Qt3DExtras::QCylinderMesh* mesh = new Qt3DExtras::QCylinderMesh(entity);
  mesh->setRadius(static_cast<float>(radius));
  mesh->setLength(static_cast<float>(length));
  mesh->setSlices(32);

  // URDF cylinders are along Z-axis, Qt3D cylinders are along Y
  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform(entity);
  transform->setRotationX(90.0f);

  // Add object picker for selection
  Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(entity);
  picker->setHoverEnabled(false);
  connect(picker, &Qt3DRender::QObjectPicker::pressed,
          this, &URDFViewport3D::onEntityPicked);

  entity->addComponent(mesh);
  entity->addComponent(material);
  entity->addComponent(transform);
  entity->addComponent(picker);

  return entity;
}

Qt3DCore::QEntity* URDFViewport3D::createSphereEntity(double radius, Qt3DRender::QMaterial* material) {
  Qt3DCore::QEntity* entity = new Qt3DCore::QEntity();

  Qt3DExtras::QSphereMesh* mesh = new Qt3DExtras::QSphereMesh(entity);
  mesh->setRadius(static_cast<float>(radius));
  mesh->setSlices(32);
  mesh->setRings(16);

  // Add object picker for selection
  Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(entity);
  picker->setHoverEnabled(false);
  connect(picker, &Qt3DRender::QObjectPicker::pressed,
          this, &URDFViewport3D::onEntityPicked);

  entity->addComponent(mesh);
  entity->addComponent(material);
  entity->addComponent(picker);

  return entity;
}

Qt3DCore::QEntity* URDFViewport3D::createMeshEntity(const QString& filename, const QVector3D& scale, Qt3DRender::QMaterial* material) {
  Qt3DCore::QEntity* entity = new Qt3DCore::QEntity();

  // Resolve the mesh path (handle package:// URIs)
  QString resolvedPath = resolveMeshPath(filename);

  if (resolvedPath.isEmpty() || !QFileInfo::exists(resolvedPath)) {
    qWarning() << "Mesh file not found:" << filename << "resolved to:" << resolvedPath;
    // Create a small placeholder cube instead
    Qt3DExtras::QCuboidMesh* placeholderMesh = new Qt3DExtras::QCuboidMesh(entity);
    placeholderMesh->setXExtent(0.05f);
    placeholderMesh->setYExtent(0.05f);
    placeholderMesh->setZExtent(0.05f);

    Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(entity);
    picker->setHoverEnabled(false);
    connect(picker, &Qt3DRender::QObjectPicker::pressed,
            this, &URDFViewport3D::onEntityPicked);

    entity->addComponent(placeholderMesh);
    entity->addComponent(material);
    entity->addComponent(picker);
    return entity;
  }

  Qt3DRender::QMesh* mesh = new Qt3DRender::QMesh(entity);
  mesh->setSource(QUrl::fromLocalFile(resolvedPath));

  // Connect to status to debug loading issues
  connect(mesh, &Qt3DRender::QMesh::statusChanged, this, [filename, resolvedPath](Qt3DRender::QMesh::Status status) {
    if (status == Qt3DRender::QMesh::Error) {
      qWarning() << "Failed to load mesh:" << filename << "from:" << resolvedPath;
    }
  });

  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform(entity);
  transform->setScale3D(scale);

  // Add object picker for selection
  Qt3DRender::QObjectPicker* picker = new Qt3DRender::QObjectPicker(entity);
  picker->setHoverEnabled(false);
  connect(picker, &Qt3DRender::QObjectPicker::pressed,
          this, &URDFViewport3D::onEntityPicked);

  entity->addComponent(mesh);
  entity->addComponent(material);
  entity->addComponent(transform);
  entity->addComponent(picker);

  return entity;
}

Qt3DExtras::QPhongMaterial* URDFViewport3D::createMaterial(const URDFMaterial& urdfMaterial) {
  Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();

  if (urdfMaterial.hasColor()) {
    material->setAmbient(urdfMaterial.color.darker(150));
    material->setDiffuse(urdfMaterial.color);
    material->setSpecular(QColor(255, 255, 255));
    material->setShininess(50.0f);
  } else {
    // Default gray material
    material->setAmbient(QColor(80, 80, 80));
    material->setDiffuse(QColor(150, 150, 150));
    material->setSpecular(QColor(200, 200, 200));
    material->setShininess(30.0f);
  }

  return material;
}

void URDFViewport3D::updateSelection() {
  // Called when selection changes to update visual state
}

void URDFViewport3D::highlightEntity(Qt3DCore::QEntity* entity, bool highlight) {
  if (!entity) return;

  Qt3DExtras::QPhongMaterial* material = entity->findChild<Qt3DExtras::QPhongMaterial*>();
  if (material) {
    if (highlight) {
      // Save original color before highlighting
      savedAmbientColor_ = material->ambient();
      material->setAmbient(QColor(255, 200, 100));  // Orange highlight
    } else {
      // Restore original color
      material->setAmbient(savedAmbientColor_);
    }
  }
}

QVector3D URDFViewport3D::getWorldPosition(Qt3DCore::QEntity* entity) {
  // Collect transforms from leaf to root, then apply root-to-leaf
  QList<Qt3DCore::QTransform*> transforms;
  Qt3DCore::QNode* node = entity;

  while (node) {
    Qt3DCore::QEntity* ent = qobject_cast<Qt3DCore::QEntity*>(node);
    if (ent) {
      Qt3DCore::QTransform* transform = ent->findChild<Qt3DCore::QTransform*>(QString(), Qt::FindDirectChildrenOnly);
      if (transform) {
        transforms.prepend(transform);
      }
    }
    node = node->parentNode();
  }

  // Apply transforms from root to leaf
  QVector3D worldPos;
  QQuaternion worldRot;
  for (Qt3DCore::QTransform* t : transforms) {
    worldPos = worldPos + worldRot.rotatedVector(t->translation());
    worldRot = worldRot * t->rotation();
  }

  return worldPos;
}

QQuaternion URDFViewport3D::getWorldRotation(Qt3DCore::QEntity* entity) {
  QQuaternion worldRot;
  Qt3DCore::QNode* node = entity;

  while (node) {
    Qt3DCore::QEntity* ent = qobject_cast<Qt3DCore::QEntity*>(node);
    if (ent) {
      Qt3DCore::QTransform* transform = ent->findChild<Qt3DCore::QTransform*>(QString(), Qt::FindDirectChildrenOnly);
      if (transform) {
        worldRot = transform->rotation() * worldRot;
      }
    }
    node = node->parentNode();
  }

  return worldRot;
}

QString URDFViewport3D::resolveMeshPath(const QString& meshPath) {
  if (meshPath.isEmpty()) {
    return QString();
  }

  // If it's a package:// URI, resolve it
  if (meshPath.startsWith("package://") && parser_) {
    QString resolved = parser_->resolvePackageUri(meshPath);
    if (!resolved.isEmpty()) {
      return resolved;
    }
    qWarning() << "Failed to resolve mesh path:" << meshPath;
  }

  // If it's already an absolute path, use it
  if (QFileInfo(meshPath).isAbsolute()) {
    return meshPath;
  }

  return meshPath;
}

void URDFViewport3D::setLabelsVisible(bool visible) {
  labelsVisible_ = visible;
  updateLabelsVisibility();
}

void URDFViewport3D::setHierarchyLinesVisible(bool visible) {
  hierarchyLinesVisible_ = visible;
  updateHierarchyLinesVisibility();
}

void URDFViewport3D::setLocalAxesVisible(bool visible) {
  localAxesVisible_ = visible;
  updateLocalAxesVisibility();
}

void URDFViewport3D::setupLabels() {
  if (!rootEntity_) return;

  // Clean up existing labels
  if (labelsRoot_) {
    labelsRoot_->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
    delete labelsRoot_;
  }
  labelEntities_.clear();

  labelsRoot_ = new Qt3DCore::QEntity(rootEntity_);
  labelsRoot_->setEnabled(labelsVisible_);

  if (!model_) return;

  // Create labels for each link using small 3D text planes
  for (auto it = linkEntities_.constBegin(); it != linkEntities_.constEnd(); ++it) {
    Qt3DCore::QEntity* linkEntity = it.value();
    QVector3D pos = getWorldPosition(linkEntity);

    // Create a text entity
    Qt3DExtras::QText2DEntity* textEntity = new Qt3DExtras::QText2DEntity(labelsRoot_);
    textEntity->setText(it.key());
    textEntity->setColor(Qt::white);
    textEntity->setFont(QFont("Sans", 16, QFont::Bold));
    textEntity->setWidth(300);
    textEntity->setHeight(40);

    // Position the text above the link
    Qt3DCore::QTransform* textTransform = new Qt3DCore::QTransform(textEntity);
    textTransform->setTranslation(pos + QVector3D(0.0f, 0.0f, 0.15f));
    textTransform->setScale(0.005f);  // Scale to reasonable world size
    textEntity->addComponent(textTransform);

    labelEntities_[it.key()] = textEntity;
  }

  // Create labels for joints
  for (auto it = jointEntities_.constBegin(); it != jointEntities_.constEnd(); ++it) {
    Qt3DCore::QEntity* jointEntity = it.value();
    QVector3D pos = getWorldPosition(jointEntity);

    Qt3DExtras::QText2DEntity* textEntity = new Qt3DExtras::QText2DEntity(labelsRoot_);
    textEntity->setText(it.key());
    textEntity->setColor(QColor(255, 200, 100));  // Orange for joints
    textEntity->setFont(QFont("Sans", 14, QFont::Bold));
    textEntity->setWidth(300);
    textEntity->setHeight(40);

    Qt3DCore::QTransform* textTransform = new Qt3DCore::QTransform(textEntity);
    textTransform->setTranslation(pos + QVector3D(0.0f, 0.0f, 0.1f));
    textTransform->setScale(0.004f);
    textEntity->addComponent(textTransform);

    labelEntities_[it.key()] = textEntity;
  }
}

void URDFViewport3D::setupHierarchyLines() {
  if (!rootEntity_) return;

  // Clean up existing lines
  if (hierarchyLinesRoot_) {
    hierarchyLinesRoot_->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
    delete hierarchyLinesRoot_;
  }

  hierarchyLinesRoot_ = new Qt3DCore::QEntity(rootEntity_);
  hierarchyLinesRoot_->setEnabled(hierarchyLinesVisible_);

  if (!model_) return;

  // Create lines connecting parent links to child links via joints
  const auto& joints = model_->joints();
  for (auto it = joints.constBegin(); it != joints.constEnd(); ++it) {
    Qt3DCore::QEntity* parentEntity = linkEntities_.value(it->parentLink);
    Qt3DCore::QEntity* childEntity = linkEntities_.value(it->childLink);

    if (!parentEntity || !childEntity) continue;

    // Get world positions
    QVector3D from = getWorldPosition(parentEntity);
    QVector3D to = getWorldPosition(childEntity);

    createHierarchyLine(from, to, hierarchyLinesRoot_);
  }
}

Qt3DCore::QEntity* URDFViewport3D::createHierarchyLine(const QVector3D& from, const QVector3D& to, Qt3DCore::QEntity* parent) {
  Qt3DCore::QEntity* lineEntity = new Qt3DCore::QEntity(parent);

  // Calculate line properties
  QVector3D direction = to - from;
  float length = direction.length();
  if (length < 0.001f) return lineEntity;

  // Use a thin cylinder as a line
  Qt3DExtras::QCylinderMesh* lineMesh = new Qt3DExtras::QCylinderMesh(lineEntity);
  lineMesh->setRadius(0.005f);  // Thin line
  lineMesh->setLength(length);
  lineMesh->setSlices(8);

  // Material - cyan like RViz TF lines
  Qt3DExtras::QPhongMaterial* lineMaterial = new Qt3DExtras::QPhongMaterial(lineEntity);
  lineMaterial->setAmbient(QColor(0, 255, 255));
  lineMaterial->setDiffuse(QColor(0, 255, 255));

  // Position and rotate the cylinder to connect the two points
  Qt3DCore::QTransform* lineTransform = new Qt3DCore::QTransform(lineEntity);

  // Move to midpoint
  QVector3D midpoint = (from + to) / 2.0f;
  lineTransform->setTranslation(midpoint);

  // Rotate to align with direction
  // Qt3D cylinder is along Y axis, we need to rotate to align with direction
  QVector3D yAxis(0, 1, 0);
  direction.normalize();

  QQuaternion rotation = QQuaternion::rotationTo(yAxis, direction);
  lineTransform->setRotation(rotation);

  lineEntity->addComponent(lineMesh);
  lineEntity->addComponent(lineMaterial);
  lineEntity->addComponent(lineTransform);

  return lineEntity;
}

void URDFViewport3D::setupLocalAxes() {
  if (!rootEntity_) return;

  // Clean up existing axes
  if (localAxesRoot_) {
    localAxesRoot_->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
    delete localAxesRoot_;
  }
  axisEntities_.clear();

  localAxesRoot_ = new Qt3DCore::QEntity(rootEntity_);
  localAxesRoot_->setEnabled(localAxesVisible_);

  if (!model_) return;

  // Create axis indicators for each link
  for (auto it = linkEntities_.constBegin(); it != linkEntities_.constEnd(); ++it) {
    Qt3DCore::QEntity* axisEntity = createAxisIndicator(localAxesRoot_);
    axisEntities_["link:" + it.key()] = axisEntity;

    // Position at the link's world location
    Qt3DCore::QEntity* linkEntity = it.value();
    QVector3D worldPos = getWorldPosition(linkEntity);
    QQuaternion worldRot = getWorldRotation(linkEntity);

    Qt3DCore::QTransform* axisTransform = new Qt3DCore::QTransform(axisEntity);
    axisTransform->setTranslation(worldPos);
    axisTransform->setRotation(worldRot);
    axisEntity->addComponent(axisTransform);
  }

  // Create axis indicators for each joint
  for (auto it = jointEntities_.constBegin(); it != jointEntities_.constEnd(); ++it) {
    Qt3DCore::QEntity* axisEntity = createAxisIndicator(localAxesRoot_);
    axisEntities_["joint:" + it.key()] = axisEntity;

    // Position at the joint's world location
    Qt3DCore::QEntity* jointEntity = it.value();
    QVector3D worldPos = getWorldPosition(jointEntity);
    QQuaternion worldRot = getWorldRotation(jointEntity);

    Qt3DCore::QTransform* axisTransform = new Qt3DCore::QTransform(axisEntity);
    axisTransform->setTranslation(worldPos);
    axisTransform->setRotation(worldRot);
    axisEntity->addComponent(axisTransform);
  }
}

Qt3DCore::QEntity* URDFViewport3D::createAxisIndicator(Qt3DCore::QEntity* parent) {
  Qt3DCore::QEntity* axisRoot = new Qt3DCore::QEntity(parent);

  const float axisLength = 0.15f;
  const float axisRadius = 0.008f;
  const float coneLength = 0.03f;
  const float coneRadius = 0.015f;

  // Colors matching RViz: Red=X, Green=Y, Blue=Z
  QColor colors[3] = {QColor(255, 0, 0), QColor(0, 255, 0), QColor(0, 0, 255)};
  QVector3D directions[3] = {QVector3D(1, 0, 0), QVector3D(0, 1, 0), QVector3D(0, 0, 1)};

  for (int i = 0; i < 3; ++i) {
    // Cylinder (axis shaft)
    Qt3DCore::QEntity* shaftEntity = new Qt3DCore::QEntity(axisRoot);

    Qt3DExtras::QCylinderMesh* shaftMesh = new Qt3DExtras::QCylinderMesh(shaftEntity);
    shaftMesh->setRadius(axisRadius);
    shaftMesh->setLength(axisLength);
    shaftMesh->setSlices(12);

    Qt3DExtras::QPhongMaterial* shaftMaterial = new Qt3DExtras::QPhongMaterial(shaftEntity);
    shaftMaterial->setAmbient(colors[i]);
    shaftMaterial->setDiffuse(colors[i]);

    Qt3DCore::QTransform* shaftTransform = new Qt3DCore::QTransform(shaftEntity);

    // Position at half the length along the axis
    shaftTransform->setTranslation(directions[i] * (axisLength / 2.0f));

    // Rotate cylinder (Y-aligned) to point along axis direction
    QQuaternion rot = QQuaternion::rotationTo(QVector3D(0, 1, 0), directions[i]);
    shaftTransform->setRotation(rot);

    shaftEntity->addComponent(shaftMesh);
    shaftEntity->addComponent(shaftMaterial);
    shaftEntity->addComponent(shaftTransform);

    // Cone (arrowhead)
    Qt3DCore::QEntity* coneEntity = new Qt3DCore::QEntity(axisRoot);

    Qt3DExtras::QConeMesh* coneMesh = new Qt3DExtras::QConeMesh(coneEntity);
    coneMesh->setBottomRadius(coneRadius);
    coneMesh->setTopRadius(0.0f);
    coneMesh->setLength(coneLength);
    coneMesh->setSlices(12);

    Qt3DExtras::QPhongMaterial* coneMaterial = new Qt3DExtras::QPhongMaterial(coneEntity);
    coneMaterial->setAmbient(colors[i]);
    coneMaterial->setDiffuse(colors[i]);

    Qt3DCore::QTransform* coneTransform = new Qt3DCore::QTransform(coneEntity);

    // Position at the end of the axis
    coneTransform->setTranslation(directions[i] * (axisLength + coneLength / 2.0f));

    // Rotate cone to point along axis
    coneTransform->setRotation(rot);

    coneEntity->addComponent(coneMesh);
    coneEntity->addComponent(coneMaterial);
    coneEntity->addComponent(coneTransform);
  }

  return axisRoot;
}

void URDFViewport3D::updateLabelsVisibility() {
  if (labelsRoot_) {
    labelsRoot_->setEnabled(labelsVisible_);
  } else if (labelsVisible_) {
    setupLabels();
  }
}

void URDFViewport3D::updateHierarchyLinesVisibility() {
  if (hierarchyLinesRoot_) {
    hierarchyLinesRoot_->setEnabled(hierarchyLinesVisible_);
  } else if (hierarchyLinesVisible_) {
    setupHierarchyLines();
  }
}

void URDFViewport3D::updateLocalAxesVisibility() {
  if (localAxesRoot_) {
    localAxesRoot_->setEnabled(localAxesVisible_);
  } else if (localAxesVisible_) {
    setupLocalAxes();
  }
}

}  // namespace ros_weaver
