#include "ros_weaver/widgets/urdf_3d_view.hpp"

#include <QDebug>
#include <QtMath>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DInput/QInputSettings>

namespace ros_weaver {

// =============================================================================
// URDF3DView Implementation
// =============================================================================

URDF3DView::URDF3DView(QWidget* parent)
  : QWidget(parent) {
  // Create the 3D window
  view3D_ = new Qt3DExtras::Qt3DWindow();
  view3D_->defaultFrameGraph()->setClearColor(backgroundColor_);

  // Create a container widget for the 3D window
  container_ = QWidget::createWindowContainer(view3D_, this);
  container_->setFocusPolicy(Qt::StrongFocus);
  container_->installEventFilter(this);

  // Layout
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(container_);

  setupScene();
  setupCamera();
  setupLighting();
  setupGrid();

  // Set initial camera position
  resetCamera();
}

URDF3DView::~URDF3DView() {
  clearModel();
}

void URDF3DView::setupScene() {
  // Root entity
  rootEntity_ = new Qt3DCore::QEntity();

  // Model container entity (will hold all links and joints)
  modelEntity_ = new Qt3DCore::QEntity(rootEntity_);

  // Set as root
  view3D_->setRootEntity(rootEntity_);
}

void URDF3DView::setupCamera() {
  camera_ = view3D_->camera();
  camera_->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.01f, 1000.0f);
  camera_->setUpVector(QVector3D(0, 0, 1));  // Z-up for ROS convention
}

void URDF3DView::setupLighting() {
  // Ambient light (global illumination)
  ambientLightEntity_ = new Qt3DCore::QEntity(rootEntity_);
  Qt3DRender::QPointLight* ambientLight = new Qt3DRender::QPointLight(ambientLightEntity_);
  ambientLight->setColor(Qt::white);
  ambientLight->setIntensity(ambientIntensity_);
  ambientLightEntity_->addComponent(ambientLight);

  Qt3DCore::QTransform* ambientTransform = new Qt3DCore::QTransform();
  ambientTransform->setTranslation(QVector3D(0, 0, 10));
  ambientLightEntity_->addComponent(ambientTransform);

  // Directional light (main light for shading)
  directionalLightEntity_ = new Qt3DCore::QEntity(rootEntity_);
  Qt3DRender::QDirectionalLight* dirLight = new Qt3DRender::QDirectionalLight(directionalLightEntity_);
  dirLight->setColor(Qt::white);
  dirLight->setIntensity(0.7f);
  dirLight->setWorldDirection(QVector3D(-1, -1, -1).normalized());
  directionalLightEntity_->addComponent(dirLight);

  // Fill light (softer, from opposite side)
  pointLightEntity_ = new Qt3DCore::QEntity(rootEntity_);
  Qt3DRender::QPointLight* fillLight = new Qt3DRender::QPointLight(pointLightEntity_);
  fillLight->setColor(QColor(200, 200, 255));
  fillLight->setIntensity(0.3f);
  pointLightEntity_->addComponent(fillLight);

  Qt3DCore::QTransform* fillTransform = new Qt3DCore::QTransform();
  fillTransform->setTranslation(QVector3D(5, 5, 5));
  pointLightEntity_->addComponent(fillTransform);
}

void URDF3DView::setupGrid() {
  gridEntity_ = new Qt3DCore::QEntity(rootEntity_);

  // Create a simple grid using lines (represented as thin boxes for simplicity)
  const int gridSize = 10;
  const float spacing = 0.5f;
  const QColor gridColor(100, 100, 100, 128);

  for (int i = -gridSize; i <= gridSize; ++i) {
    // Line along X axis
    Qt3DCore::QEntity* lineX = new Qt3DCore::QEntity(gridEntity_);
    Qt3DExtras::QCuboidMesh* meshX = new Qt3DExtras::QCuboidMesh();
    meshX->setXExtent(gridSize * 2 * spacing);
    meshX->setYExtent(0.005f);
    meshX->setZExtent(0.005f);

    Qt3DCore::QTransform* transformX = new Qt3DCore::QTransform();
    transformX->setTranslation(QVector3D(0, i * spacing, 0));
    lineX->addComponent(meshX);
    lineX->addComponent(transformX);

    Qt3DExtras::QPhongMaterial* matX = new Qt3DExtras::QPhongMaterial();
    matX->setAmbient(gridColor);
    matX->setDiffuse(gridColor);
    lineX->addComponent(matX);

    // Line along Y axis
    Qt3DCore::QEntity* lineY = new Qt3DCore::QEntity(gridEntity_);
    Qt3DExtras::QCuboidMesh* meshY = new Qt3DExtras::QCuboidMesh();
    meshY->setXExtent(0.005f);
    meshY->setYExtent(gridSize * 2 * spacing);
    meshY->setZExtent(0.005f);

    Qt3DCore::QTransform* transformY = new Qt3DCore::QTransform();
    transformY->setTranslation(QVector3D(i * spacing, 0, 0));
    lineY->addComponent(meshY);
    lineY->addComponent(transformY);

    Qt3DExtras::QPhongMaterial* matY = new Qt3DExtras::QPhongMaterial();
    matY->setAmbient(gridColor);
    matY->setDiffuse(gridColor);
    lineY->addComponent(matY);
  }

  gridEntity_->setEnabled(gridVisible_);
}

void URDF3DView::setModel(const URDFModel& model) {
  clearModel();
  buildModel(model);
  fitToModel();
}

void URDF3DView::clearModel() {
  // Clear axis indicators
  for (AxisIndicator* indicator : axisIndicators_) {
    delete indicator;
  }
  axisIndicators_.clear();

  // Clear entity maps
  linkEntities_.clear();
  jointEntities_.clear();
  linkTransforms_.clear();
  jointTransforms_.clear();
  entityToJoint_.clear();
  selectedJoints_.clear();

  // Delete model entity children
  if (modelEntity_) {
    for (Qt3DCore::QNode* child : modelEntity_->childNodes()) {
      child->deleteLater();
    }
  }
}

void URDF3DView::buildModel(const URDFModel& model) {
  // Create link entities
  for (const URDFLink& link : model.links) {
    createLinkEntity(link, modelEntity_);
  }

  // Create joint axis indicators
  for (const URDFJoint& joint : model.joints) {
    createJointAxisIndicator(joint);
  }

  // Update all transforms
  updateAllTransforms();
}

void URDF3DView::createLinkEntity(const URDFLink& link, Qt3DCore::QEntity* parent) {
  Qt3DCore::QEntity* entity = new Qt3DCore::QEntity(parent);

  // Transform
  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform();
  entity->addComponent(transform);
  linkTransforms_[link.name] = transform;
  linkEntities_[link.name] = entity;

  // Create geometry based on mesh path
  if (!link.meshPath.isEmpty()) {
    if (link.meshPath.startsWith("primitive://")) {
      createPrimitiveMesh(link.meshPath, entity);
    } else {
      // For mesh files, create a simple placeholder box
      // Full mesh loading would require assimp integration
      Qt3DExtras::QCuboidMesh* mesh = new Qt3DExtras::QCuboidMesh();
      mesh->setXExtent(0.1f);
      mesh->setYExtent(0.1f);
      mesh->setZExtent(0.1f);
      entity->addComponent(mesh);
    }

    // Material
    Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(link.color.darker(150));
    material->setDiffuse(link.color);
    material->setSpecular(QColor(255, 255, 255));
    material->setShininess(50.0f);
    entity->addComponent(material);
  }
}

void URDF3DView::createPrimitiveMesh(const QString& meshPath, Qt3DCore::QEntity* entity) {
  QUrl url(meshPath);
  QUrlQuery query(url);

  if (meshPath.contains("box")) {
    QString size = query.queryItemValue("size");
    QStringList parts = size.split(' ', Qt::SkipEmptyParts);
    Qt3DExtras::QCuboidMesh* mesh = new Qt3DExtras::QCuboidMesh();
    if (parts.size() == 3) {
      mesh->setXExtent(parts[0].toFloat());
      mesh->setYExtent(parts[1].toFloat());
      mesh->setZExtent(parts[2].toFloat());
    } else {
      mesh->setXExtent(0.1f);
      mesh->setYExtent(0.1f);
      mesh->setZExtent(0.1f);
    }
    entity->addComponent(mesh);
  } else if (meshPath.contains("cylinder")) {
    float radius = query.queryItemValue("radius").toFloat();
    float length = query.queryItemValue("length").toFloat();
    Qt3DExtras::QCylinderMesh* mesh = new Qt3DExtras::QCylinderMesh();
    mesh->setRadius(radius > 0 ? radius : 0.05f);
    mesh->setLength(length > 0 ? length : 0.1f);
    mesh->setSlices(32);
    entity->addComponent(mesh);
  } else if (meshPath.contains("sphere")) {
    float radius = query.queryItemValue("radius").toFloat();
    Qt3DExtras::QSphereMesh* mesh = new Qt3DExtras::QSphereMesh();
    mesh->setRadius(radius > 0 ? radius : 0.05f);
    mesh->setSlices(32);
    mesh->setRings(16);
    entity->addComponent(mesh);
  }
}

void URDF3DView::createJointAxisIndicator(const URDFJoint& joint) {
  // Create axis indicator for the joint
  AxisIndicator* indicator = new AxisIndicator(modelEntity_, axisScale_);
  axisIndicators_[joint.name] = indicator;

  // Create a transform for the joint
  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform();
  jointTransforms_[joint.name] = transform;

  // Map entity back to joint name for picking
  entityToJoint_[indicator] = joint.name;
}

void URDF3DView::updateJointTransform(const QString& jointName, const QQuaternion& orientation) {
  Q_UNUSED(orientation);
  if (!parser_) return;

  // Get the global transform for this joint
  QMatrix4x4 globalTransform = parser_->getJointTransform(jointName);

  // Update axis indicator
  if (axisIndicators_.contains(jointName)) {
    axisIndicators_[jointName]->setTransform(globalTransform);
  }

  // Update child link transform
  const URDFJoint* joint = parser_->getJoint(jointName);
  if (joint && linkTransforms_.contains(joint->childLink)) {
    QMatrix4x4 linkGlobal = parser_->getGlobalTransform(joint->childLink);
    Qt3DCore::QTransform* linkTransform = linkTransforms_[joint->childLink];

    // Apply visual origin offset
    const URDFLink* link = parser_->getLink(joint->childLink);
    if (link) {
      QMatrix4x4 visualOffset;
      visualOffset.translate(link->visualOrigin);
      visualOffset.rotate(link->visualOrientation);
      linkGlobal = linkGlobal * visualOffset;
    }

    linkTransform->setMatrix(linkGlobal);
  }
}

void URDF3DView::updateAllTransforms() {
  if (!parser_) return;

  const URDFModel& model = parser_->getModel();

  // Update root link (no joint transform)
  if (linkTransforms_.contains(model.rootLink)) {
    const URDFLink* rootLink = parser_->getLink(model.rootLink);
    if (rootLink) {
      QMatrix4x4 transform;
      transform.translate(rootLink->visualOrigin);
      transform.rotate(rootLink->visualOrientation);
      linkTransforms_[model.rootLink]->setMatrix(transform);
    }
  }

  // Update all joints and their child links
  for (const URDFJoint& joint : model.joints) {
    // Update axis indicator
    QMatrix4x4 jointGlobal = parser_->getJointTransform(joint.name);
    if (axisIndicators_.contains(joint.name)) {
      axisIndicators_[joint.name]->setTransform(jointGlobal);
    }

    // Update child link
    if (linkTransforms_.contains(joint.childLink)) {
      QMatrix4x4 linkGlobal = parser_->getGlobalTransform(joint.childLink);

      // Apply visual origin offset
      const URDFLink* link = parser_->getLink(joint.childLink);
      if (link) {
        QMatrix4x4 visualOffset;
        visualOffset.translate(link->visualOrigin);
        visualOffset.rotate(link->visualOrientation);
        linkGlobal = linkGlobal * visualOffset;
      }

      linkTransforms_[joint.childLink]->setMatrix(linkGlobal);
    }
  }
}

void URDF3DView::selectJoint(const QString& jointName, bool addToSelection) {
  if (!addToSelection) {
    selectedJoints_.clear();
  }

  if (!jointName.isEmpty() && !selectedJoints_.contains(jointName)) {
    selectedJoints_.append(jointName);
  }

  highlightSelectedJoints();
  emit selectionChanged(selectedJoints_);
}

void URDF3DView::selectJoints(const QStringList& jointNames) {
  selectedJoints_ = jointNames;
  highlightSelectedJoints();
  emit selectionChanged(selectedJoints_);
}

void URDF3DView::clearSelection() {
  selectedJoints_.clear();
  highlightSelectedJoints();
  emit selectionChanged(selectedJoints_);
}

void URDF3DView::highlightSelectedJoints() {
  // Reset all indicators to default
  for (auto it = axisIndicators_.begin(); it != axisIndicators_.end(); ++it) {
    it.value()->setHighlighted(selectedJoints_.contains(it.key()));
  }
}

QString URDF3DView::findJointForEntity(Qt3DCore::QEntity* entity) const {
  return entityToJoint_.value(entity, QString());
}

void URDF3DView::resetCamera() {
  cameraTarget_ = QVector3D(0, 0, 0);
  cameraDistance_ = 3.0f;
  cameraPitch_ = 30.0f;
  cameraYaw_ = 45.0f;

  // Update camera position based on spherical coordinates
  float pitchRad = qDegreesToRadians(cameraPitch_);
  float yawRad = qDegreesToRadians(cameraYaw_);

  QVector3D offset(
      cameraDistance_ * qCos(pitchRad) * qCos(yawRad),
      cameraDistance_ * qCos(pitchRad) * qSin(yawRad),
      cameraDistance_ * qSin(pitchRad));

  camera_->setPosition(cameraTarget_ + offset);
  camera_->setViewCenter(cameraTarget_);
  camera_->setUpVector(QVector3D(0, 0, 1));
}

void URDF3DView::setCameraPosition(const QVector3D& position) {
  camera_->setPosition(position);
}

void URDF3DView::setCameraTarget(const QVector3D& target) {
  cameraTarget_ = target;
  camera_->setViewCenter(target);
}

void URDF3DView::fitToModel() {
  // Simple fit: reset to default view
  // A more sophisticated version would compute bounding box
  resetCamera();
}

void URDF3DView::setRenderMode(RenderMode mode) {
  renderMode_ = mode;

  // Adjust materials based on render mode
  for (Qt3DCore::QEntity* entity : linkEntities_) {
    Qt3DExtras::QPhongMaterial* material = entity->findChild<Qt3DExtras::QPhongMaterial*>();
    if (!material) continue;

    switch (mode) {
      case RenderMode::Wireframe:
        material->setAmbient(QColor(200, 200, 200));
        material->setDiffuse(QColor(50, 50, 50));
        material->setSpecular(QColor(0, 0, 0));
        material->setShininess(0.0f);
        break;

      case RenderMode::BasicShading:
        material->setAmbient(QColor(100, 100, 100));
        material->setDiffuse(QColor(150, 150, 150));
        material->setSpecular(QColor(50, 50, 50));
        material->setShininess(10.0f);
        break;

      case RenderMode::FullLighting:
        material->setAmbient(material->diffuse().darker(150));
        material->setSpecular(QColor(255, 255, 255));
        material->setShininess(50.0f);
        break;
    }
  }

  // Enable/disable lights based on mode
  if (directionalLightEntity_) {
    directionalLightEntity_->setEnabled(mode == RenderMode::FullLighting);
  }
  if (pointLightEntity_) {
    pointLightEntity_->setEnabled(mode == RenderMode::FullLighting);
  }
}

void URDF3DView::setShadowsEnabled(bool enabled) {
  shadowsEnabled_ = enabled;
  // Note: Qt3D shadow implementation requires custom frame graph setup
  // This is a placeholder for future implementation
}

void URDF3DView::setBackgroundColor(const QColor& color) {
  backgroundColor_ = color;
  view3D_->defaultFrameGraph()->setClearColor(color);
}

void URDF3DView::setAmbientLightIntensity(float intensity) {
  ambientIntensity_ = intensity;
  if (ambientLightEntity_) {
    Qt3DRender::QPointLight* light = ambientLightEntity_->findChild<Qt3DRender::QPointLight*>();
    if (light) {
      light->setIntensity(intensity);
    }
  }
}

void URDF3DView::setAxisIndicatorScale(float scale) {
  axisScale_ = scale;
  for (AxisIndicator* indicator : axisIndicators_) {
    indicator->setScale(scale);
  }
}

void URDF3DView::setGridVisible(bool visible) {
  gridVisible_ = visible;
  if (gridEntity_) {
    gridEntity_->setEnabled(visible);
  }
}

bool URDF3DView::eventFilter(QObject* obj, QEvent* event) {
  if (obj == container_) {
    switch (event->type()) {
      case QEvent::MouseButtonPress:
        handleMousePress(static_cast<QMouseEvent*>(event));
        return true;
      case QEvent::MouseMove:
        handleMouseMove(static_cast<QMouseEvent*>(event));
        return true;
      case QEvent::MouseButtonRelease:
        handleMouseRelease(static_cast<QMouseEvent*>(event));
        return true;
      case QEvent::Wheel:
        handleWheel(static_cast<QWheelEvent*>(event));
        return true;
      case QEvent::KeyPress:
        handleKeyPress(static_cast<QKeyEvent*>(event));
        return true;
      default:
        break;
    }
  }
  return QWidget::eventFilter(obj, event);
}

void URDF3DView::handleMousePress(QMouseEvent* event) {
  lastMousePos_ = event->pos();

  if (event->button() == Qt::MiddleButton) {
    if (event->modifiers() & Qt::ShiftModifier) {
      isPanning_ = true;
      container_->setCursor(Qt::ClosedHandCursor);
    } else {
      isOrbiting_ = true;
      container_->setCursor(Qt::SizeAllCursor);
    }
  } else if (event->button() == Qt::LeftButton) {
    // TODO: Implement ray picking for joint selection
    // For now, this is a placeholder
  }
}

void URDF3DView::handleMouseMove(QMouseEvent* event) {
  int dx = event->pos().x() - lastMousePos_.x();
  int dy = event->pos().y() - lastMousePos_.y();
  lastMousePos_ = event->pos();

  if (isOrbiting_) {
    orbitCamera(dx, dy);
  } else if (isPanning_) {
    panCamera(dx, dy);
  }
}

void URDF3DView::handleMouseRelease(QMouseEvent* event) {
  Q_UNUSED(event);
  isPanning_ = false;
  isOrbiting_ = false;
  container_->setCursor(Qt::ArrowCursor);
}

void URDF3DView::handleWheel(QWheelEvent* event) {
  float delta = event->angleDelta().y() / 120.0f;
  zoomCamera(delta);
}

void URDF3DView::handleKeyPress(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Home:
      resetCamera();
      break;
    case Qt::Key_Escape:
      clearSelection();
      break;
    default:
      break;
  }
}

void URDF3DView::orbitCamera(int dx, int dy) {
  const float sensitivity = 0.3f;

  cameraYaw_ -= dx * sensitivity;
  cameraPitch_ += dy * sensitivity;

  // Clamp pitch to avoid gimbal lock
  cameraPitch_ = qBound(-89.0f, cameraPitch_, 89.0f);

  // Update camera position
  float pitchRad = qDegreesToRadians(cameraPitch_);
  float yawRad = qDegreesToRadians(cameraYaw_);

  QVector3D offset(
      cameraDistance_ * qCos(pitchRad) * qCos(yawRad),
      cameraDistance_ * qCos(pitchRad) * qSin(yawRad),
      cameraDistance_ * qSin(pitchRad));

  camera_->setPosition(cameraTarget_ + offset);
  camera_->setViewCenter(cameraTarget_);
}

void URDF3DView::panCamera(int dx, int dy) {
  const float sensitivity = 0.005f * cameraDistance_;

  // Get camera right and up vectors
  QVector3D viewDir = (camera_->viewCenter() - camera_->position()).normalized();
  QVector3D right = QVector3D::crossProduct(viewDir, camera_->upVector()).normalized();
  QVector3D up = camera_->upVector();

  // Pan the target
  cameraTarget_ += right * (-dx * sensitivity) + up * (dy * sensitivity);

  // Update camera position to maintain same view direction
  float pitchRad = qDegreesToRadians(cameraPitch_);
  float yawRad = qDegreesToRadians(cameraYaw_);

  QVector3D offset(
      cameraDistance_ * qCos(pitchRad) * qCos(yawRad),
      cameraDistance_ * qCos(pitchRad) * qSin(yawRad),
      cameraDistance_ * qSin(pitchRad));

  camera_->setPosition(cameraTarget_ + offset);
  camera_->setViewCenter(cameraTarget_);
}

void URDF3DView::zoomCamera(float delta) {
  const float zoomFactor = 0.1f;
  cameraDistance_ *= (1.0f - delta * zoomFactor);
  cameraDistance_ = qBound(0.1f, cameraDistance_, 100.0f);

  // Update camera position
  float pitchRad = qDegreesToRadians(cameraPitch_);
  float yawRad = qDegreesToRadians(cameraYaw_);

  QVector3D offset(
      cameraDistance_ * qCos(pitchRad) * qCos(yawRad),
      cameraDistance_ * qCos(pitchRad) * qSin(yawRad),
      cameraDistance_ * qSin(pitchRad));

  camera_->setPosition(cameraTarget_ + offset);
}

// =============================================================================
// AxisIndicator Implementation
// =============================================================================

AxisIndicator::AxisIndicator(Qt3DCore::QEntity* parent, float length)
  : Qt3DCore::QEntity(parent)
  , length_(length) {
  transform_ = new Qt3DCore::QTransform(this);
  addComponent(transform_);

  createAxis(QVector3D(1, 0, 0), X_AXIS_COLOR, length);
  createAxis(QVector3D(0, 1, 0), Y_AXIS_COLOR, length);
  createAxis(QVector3D(0, 0, 1), Z_AXIS_COLOR, length);
}

void AxisIndicator::createAxis(const QVector3D& direction, const QColor& color, float length) {
  Qt3DCore::QEntity* axisEntity = new Qt3DCore::QEntity(this);

  // Cylinder for the axis shaft
  Qt3DExtras::QCylinderMesh* cylinder = new Qt3DExtras::QCylinderMesh();
  cylinder->setRadius(length * 0.03f);
  cylinder->setLength(length * 0.8f);
  cylinder->setSlices(12);
  axisEntity->addComponent(cylinder);

  // Position and orient the cylinder
  Qt3DCore::QTransform* transform = new Qt3DCore::QTransform();
  QQuaternion rotation;
  if (direction == QVector3D(1, 0, 0)) {
    rotation = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), -90);
    transform->setTranslation(QVector3D(length * 0.4f, 0, 0));
    xAxisEntity_ = axisEntity;
  } else if (direction == QVector3D(0, 1, 0)) {
    rotation = QQuaternion();  // Default is along Y
    transform->setTranslation(QVector3D(0, length * 0.4f, 0));
    yAxisEntity_ = axisEntity;
  } else {
    rotation = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), 90);
    transform->setTranslation(QVector3D(0, 0, length * 0.4f));
    zAxisEntity_ = axisEntity;
  }
  transform->setRotation(rotation);
  axisEntity->addComponent(transform);

  // Material
  Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
  material->setAmbient(color.darker(130));
  material->setDiffuse(color);
  material->setSpecular(Qt::white);
  material->setShininess(50.0f);
  axisEntity->addComponent(material);

  // Cone tip (arrow head)
  Qt3DCore::QEntity* coneEntity = new Qt3DCore::QEntity(this);
  Qt3DExtras::QConeMesh* cone = new Qt3DExtras::QConeMesh();
  cone->setBottomRadius(length * 0.06f);
  cone->setLength(length * 0.2f);
  cone->setSlices(12);
  coneEntity->addComponent(cone);

  Qt3DCore::QTransform* coneTransform = new Qt3DCore::QTransform();
  if (direction == QVector3D(1, 0, 0)) {
    coneTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), -90));
    coneTransform->setTranslation(QVector3D(length * 0.9f, 0, 0));
    xConeEntity_ = coneEntity;
  } else if (direction == QVector3D(0, 1, 0)) {
    coneTransform->setTranslation(QVector3D(0, length * 0.9f, 0));
    yConeEntity_ = coneEntity;
  } else {
    coneTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), 90));
    coneTransform->setTranslation(QVector3D(0, 0, length * 0.9f));
    zConeEntity_ = coneEntity;
  }
  coneEntity->addComponent(coneTransform);

  Qt3DExtras::QPhongMaterial* coneMaterial = new Qt3DExtras::QPhongMaterial();
  coneMaterial->setAmbient(color.darker(130));
  coneMaterial->setDiffuse(color);
  coneMaterial->setSpecular(Qt::white);
  coneMaterial->setShininess(50.0f);
  coneEntity->addComponent(coneMaterial);
}

void AxisIndicator::setHighlighted(bool highlighted) {
  highlighted_ = highlighted;

  // Change axis colors when highlighted
  QColor highlightMult = highlighted ? HIGHLIGHT_COLOR : Qt::white;

  auto updateMaterial = [&](Qt3DCore::QEntity* entity, const QColor& baseColor) {
    if (!entity) return;
    Qt3DExtras::QPhongMaterial* mat = entity->findChild<Qt3DExtras::QPhongMaterial*>();
    if (mat) {
      QColor color = highlighted ? HIGHLIGHT_COLOR : baseColor;
      mat->setDiffuse(color);
      mat->setAmbient(color.darker(130));
    }
  };

  updateMaterial(xAxisEntity_, X_AXIS_COLOR);
  updateMaterial(yAxisEntity_, Y_AXIS_COLOR);
  updateMaterial(zAxisEntity_, Z_AXIS_COLOR);
  updateMaterial(xConeEntity_, X_AXIS_COLOR);
  updateMaterial(yConeEntity_, Y_AXIS_COLOR);
  updateMaterial(zConeEntity_, Z_AXIS_COLOR);
}

void AxisIndicator::setScale(float scale) {
  length_ = scale;
  // Recreating geometry would be expensive, so we just scale the transform
  transform_->setScale(scale / 0.1f);  // Normalize to default length
}

void AxisIndicator::setTransform(const QMatrix4x4& transformMatrix) {
  transform_->setMatrix(transformMatrix);
}

}  // namespace ros_weaver
