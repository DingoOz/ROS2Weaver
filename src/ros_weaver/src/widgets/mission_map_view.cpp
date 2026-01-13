#include "ros_weaver/widgets/mission_map_view.hpp"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QDebug>
#include <QScrollBar>
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace ros_weaver {

MissionMapView::MissionMapView(QWidget* parent)
    : QGraphicsView(parent) {
  setupScene();
}

MissionMapView::~MissionMapView() = default;

void MissionMapView::setupScene() {
  scene_ = new QGraphicsScene(this);
  scene_->setBackgroundBrush(QColor(220, 220, 220));
  setScene(scene_);

  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setMinimumSize(400, 300);

  mapItem_ = nullptr;
  startPoseItem_ = nullptr;
  scaleOverlay_ = nullptr;
  pathItem_ = nullptr;
}

void MissionMapView::loadMapImage(const QString& imagePath) {
  QPixmap pixmap(imagePath);
  if (pixmap.isNull()) {
    qWarning() << "Failed to load map image:" << imagePath;
    return;
  }

  clearMap();

  mapItem_ = scene_->addPixmap(pixmap);
  mapItem_->setZValue(-100);  // Behind everything

  // Set scene rect to image bounds
  scene_->setSceneRect(mapItem_->boundingRect());

  // Fit the map in view
  zoomToFit();

  emit mapLoaded(imagePath);
}

void MissionMapView::loadNav2Map(const QString& yamlPath) {
  try {
    YAML::Node config = YAML::LoadFile(yamlPath.toStdString());

    // Get image path (relative to yaml file)
    QString imageName = QString::fromStdString(config["image"].as<std::string>());
    QFileInfo yamlInfo(yamlPath);
    QString imagePath = yamlInfo.absoluteDir().filePath(imageName);

    // Get resolution and origin
    mapResolution_ = config["resolution"].as<double>();
    std::vector<double> origin = config["origin"].as<std::vector<double>>();
    mapOrigin_ = QPointF(origin[0], origin[1]);

    // Set scale from resolution
    scale_.metersPerPixel = mapResolution_;

    // Load the image
    loadMapImage(imagePath);

  } catch (const YAML::Exception& e) {
    qWarning() << "Failed to parse Nav2 map yaml:" << e.what();
  }
}

void MissionMapView::clearMap() {
  if (mapItem_) {
    scene_->removeItem(mapItem_);
    delete mapItem_;
    mapItem_ = nullptr;
  }
  clearWaypoints();
  if (startPoseItem_) {
    scene_->removeItem(startPoseItem_);
    delete startPoseItem_;
    startPoseItem_ = nullptr;
  }
}

void MissionMapView::enterScaleCalibrationMode() {
  setMode(ScaleCalibration);

  // Create or reset scale overlay
  if (!scaleOverlay_) {
    scaleOverlay_ = new ScaleCalibrationOverlay();
    scene_->addItem(scaleOverlay_);
    connect(scaleOverlay_, &ScaleCalibrationOverlay::calibrationComplete,
            this, &MissionMapView::onScaleCalibrationComplete);
  }
  scaleOverlay_->clear();
  scaleOverlay_->setMeasuring(true);
  scaleCalibrationFirstPointSet_ = false;

  setDragMode(QGraphicsView::NoDrag);
  setCursor(Qt::CrossCursor);
}

void MissionMapView::exitScaleCalibrationMode() {
  if (scaleOverlay_) {
    scaleOverlay_->setMeasuring(false);
  }
  setMode(Normal);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setCursor(Qt::ArrowCursor);
}

void MissionMapView::setScale(const MapScale& scale) {
  scale_ = scale;
  updateWaypointPath();
}

void MissionMapView::addWaypoint(const QPointF& positionMeters) {
  QPointF pixelPos = metersToPixel(positionMeters);
  addWaypointAtPixel(pixelPos);
}

void MissionMapView::addWaypointAtPixel(const QPointF& pixelPos) {
  Waypoint wp;
  wp.id = nextWaypointId_++;
  wp.name = QString("WP%1").arg(wp.id);

  QPointF meterPos = pixelToMeters(pixelPos);
  wp.x = meterPos.x();
  wp.y = meterPos.y();
  wp.theta = 0.0;
  wp.tolerance = scale_.metersPerPixel > 0 ? 0.3 : 30.0;  // 30cm or 30 pixels

  addWaypointItem(wp);

  emit waypointAdded(wp);
}

void MissionMapView::addWaypointItem(const Waypoint& waypoint) {
  WaypointGraphicsItem* item = new WaypointGraphicsItem(waypoint);

  QPointF pixelPos = metersToPixel(QPointF(waypoint.x, waypoint.y));
  item->setPos(pixelPos);
  item->setSequenceNumber(waypointItems_.size() + 1);

  connect(item, &WaypointGraphicsItem::waypointMoved,
          this, &MissionMapView::onWaypointMoved);
  connect(item, &WaypointGraphicsItem::waypointOrientationChanged,
          this, &MissionMapView::onWaypointOrientationChanged);
  connect(item, &WaypointGraphicsItem::waypointClicked,
          this, &MissionMapView::onWaypointClicked);
  connect(item, &WaypointGraphicsItem::waypointDoubleClicked,
          this, &MissionMapView::onWaypointDoubleClicked);

  scene_->addItem(item);
  waypointItems_.append(item);

  updateWaypointPath();
}

void MissionMapView::removeWaypoint(int waypointId) {
  WaypointGraphicsItem* item = findWaypointItem(waypointId);
  if (!item) return;

  scene_->removeItem(item);
  waypointItems_.removeOne(item);
  delete item;

  updateWaypointNumbers();
  updateWaypointPath();

  emit waypointRemoved(waypointId);
}

void MissionMapView::updateWaypoint(const Waypoint& waypoint) {
  WaypointGraphicsItem* item = findWaypointItem(waypoint.id);
  if (!item) return;

  item->setWaypoint(waypoint);
  QPointF pixelPos = metersToPixel(QPointF(waypoint.x, waypoint.y));
  item->setPos(pixelPos);

  updateWaypointPath();
}

void MissionMapView::selectWaypoint(int waypointId) {
  clearSelection();
  selectedWaypointId_ = waypointId;

  WaypointGraphicsItem* item = findWaypointItem(waypointId);
  if (item) {
    item->setSelected(true);
    centerOn(item);
  }

  emit waypointSelected(waypointId);
}

void MissionMapView::clearWaypoints() {
  for (auto* item : waypointItems_) {
    scene_->removeItem(item);
    delete item;
  }
  waypointItems_.clear();
  nextWaypointId_ = 1;
  selectedWaypointId_ = -1;

  if (pathItem_) {
    scene_->removeItem(pathItem_);
    delete pathItem_;
    pathItem_ = nullptr;
  }
}

void MissionMapView::setWaypoints(const QList<Waypoint>& waypoints) {
  clearWaypoints();

  int maxId = 0;
  for (const auto& wp : waypoints) {
    addWaypointItem(wp);
    if (wp.id > maxId) {
      maxId = wp.id;
    }
  }
  nextWaypointId_ = maxId + 1;

  updateWaypointNumbers();
  updateWaypointPath();
}

QList<Waypoint> MissionMapView::getWaypoints() const {
  QList<Waypoint> waypoints;
  for (const auto* item : waypointItems_) {
    Waypoint wp = item->waypoint();
    // Update position from item position
    QPointF meters = pixelToMeters(item->pos());
    wp.x = meters.x();
    wp.y = meters.y();
    waypoints.append(wp);
  }
  return waypoints;
}

void MissionMapView::setStartPose(const RobotStartPose& pose) {
  startPose_ = pose;

  if (!startPoseItem_) {
    startPoseItem_ = new RobotStartPoseItem();
    connect(startPoseItem_, &RobotStartPoseItem::poseMoved,
            this, &MissionMapView::onStartPoseMoved);
    connect(startPoseItem_, &RobotStartPoseItem::poseOrientationChanged,
            this, &MissionMapView::onStartPoseOrientationChanged);
    scene_->addItem(startPoseItem_);
  }

  QPointF pixelPos = metersToPixel(QPointF(pose.x, pose.y));
  startPoseItem_->setPos(pixelPos);
  startPoseItem_->setPose(pose);

  updateWaypointPath();
}

void MissionMapView::enterStartPoseMode() {
  setMode(SetStartPose);
  setDragMode(QGraphicsView::NoDrag);
  setCursor(Qt::CrossCursor);
}

void MissionMapView::setMode(Mode mode) {
  currentMode_ = mode;
  emit modeChanged(mode);
}

void MissionMapView::zoomIn() {
  scale(1.2, 1.2);
}

void MissionMapView::zoomOut() {
  scale(1.0 / 1.2, 1.0 / 1.2);
}

void MissionMapView::zoomToFit() {
  if (!mapItem_) return;

  QRectF bounds = mapItem_->boundingRect();
  bounds.adjust(-20, -20, 20, 20);
  fitInView(bounds, Qt::KeepAspectRatio);
}

void MissionMapView::setShowGrid(bool show) {
  showGrid_ = show;
  viewport()->update();
}

void MissionMapView::setShowCoordinates(bool show) {
  showCoordinates_ = show;
}

void MissionMapView::setShowPath(bool show) {
  showPath_ = show;
  if (pathItem_) {
    pathItem_->setVisible(show);
  }
}

QPointF MissionMapView::pixelToMeters(const QPointF& pixel) const {
  return scale_.pixelToMeters(pixel) + mapOrigin_;
}

QPointF MissionMapView::metersToPixel(const QPointF& meters) const {
  return scale_.metersToPixel(meters - mapOrigin_);
}

void MissionMapView::wheelEvent(QWheelEvent* event) {
  double scaleFactor = 1.15;
  if (event->angleDelta().y() > 0) {
    scale(scaleFactor, scaleFactor);
  } else {
    scale(1.0 / scaleFactor, 1.0 / scaleFactor);
  }
}

void MissionMapView::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    QPointF scenePos = mapToScene(event->pos());

    switch (currentMode_) {
      case ScaleCalibration:
        handleScaleCalibrationClick(scenePos);
        return;
      case AddWaypoint:
        handleAddWaypointClick(scenePos);
        return;
      case SetStartPose:
        handleSetStartPoseClick(scenePos);
        return;
      default:
        break;
    }
  }

  QGraphicsView::mousePressEvent(event);
}

void MissionMapView::mouseMoveEvent(QMouseEvent* event) {
  QPointF scenePos = mapToScene(event->pos());

  if (showCoordinates_) {
    QPointF meters = pixelToMeters(scenePos);
    emit coordinateHovered(meters);
  }

  QGraphicsView::mouseMoveEvent(event);
}

void MissionMapView::mouseReleaseEvent(QMouseEvent* event) {
  QGraphicsView::mouseReleaseEvent(event);
}

void MissionMapView::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    if (currentMode_ != Normal) {
      setMode(Normal);
      setDragMode(QGraphicsView::ScrollHandDrag);
      setCursor(Qt::ArrowCursor);
      if (scaleOverlay_) {
        scaleOverlay_->setMeasuring(false);
      }
      return;
    }
  } else if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
    if (selectedWaypointId_ >= 0) {
      removeWaypoint(selectedWaypointId_);
      selectedWaypointId_ = -1;
      return;
    }
  }

  QGraphicsView::keyPressEvent(event);
}

void MissionMapView::drawBackground(QPainter* painter, const QRectF& rect) {
  QGraphicsView::drawBackground(painter, rect);

  if (showGrid_ && scale_.metersPerPixel > 0) {
    // Draw grid lines at 1m intervals
    painter->setPen(QPen(QColor(200, 200, 200, 100), 1));

    double gridSpacing = 1.0 / scale_.metersPerPixel;  // pixels per meter
    if (gridSpacing < 20) {
      gridSpacing *= 5;  // Scale up for visibility
    }

    double startX = std::floor(rect.left() / gridSpacing) * gridSpacing;
    double startY = std::floor(rect.top() / gridSpacing) * gridSpacing;

    for (double x = startX; x <= rect.right(); x += gridSpacing) {
      painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
    }
    for (double y = startY; y <= rect.bottom(); y += gridSpacing) {
      painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
    }
  }
}

void MissionMapView::onWaypointMoved(int waypointId, const QPointF& newPos) {
  QPointF meters = pixelToMeters(newPos);

  WaypointGraphicsItem* item = findWaypointItem(waypointId);
  if (item) {
    Waypoint wp = item->waypoint();
    wp.x = meters.x();
    wp.y = meters.y();
    item->setWaypoint(wp);
  }

  updateWaypointPath();
  emit waypointMoved(waypointId, meters);
}

void MissionMapView::onWaypointOrientationChanged(int waypointId, double theta) {
  WaypointGraphicsItem* item = findWaypointItem(waypointId);
  if (item) {
    Waypoint wp = item->waypoint();
    wp.theta = theta;
    item->setWaypoint(wp);
  }

  emit waypointOrientationChanged(waypointId, theta);
}

void MissionMapView::onWaypointClicked(int waypointId) {
  clearSelection();
  selectedWaypointId_ = waypointId;

  WaypointGraphicsItem* item = findWaypointItem(waypointId);
  if (item) {
    item->setSelected(true);
  }

  emit waypointSelected(waypointId);
}

void MissionMapView::onWaypointDoubleClicked(int waypointId) {
  emit waypointDoubleClicked(waypointId);
}

void MissionMapView::onStartPoseMoved(const QPointF& newPos) {
  QPointF meters = pixelToMeters(newPos);
  startPose_.x = meters.x();
  startPose_.y = meters.y();

  updateWaypointPath();
  emit startPoseChanged(startPose_);
}

void MissionMapView::onStartPoseOrientationChanged(double theta) {
  startPose_.theta = theta;
  emit startPoseChanged(startPose_);
}

void MissionMapView::onScaleCalibrationComplete(const QPointF& p1, const QPointF& p2) {
  scale_.pixelPoint1 = p1;
  scale_.pixelPoint2 = p2;
  // Emit signal to prompt user for real-world distance
  emit scaleCalibrated(scale_);
}

WaypointGraphicsItem* MissionMapView::findWaypointItem(int waypointId) {
  for (auto* item : waypointItems_) {
    if (item->waypointId() == waypointId) {
      return item;
    }
  }
  return nullptr;
}

void MissionMapView::clearSelection() {
  for (auto* item : waypointItems_) {
    item->setSelected(false);
  }
  selectedWaypointId_ = -1;
}

void MissionMapView::handleScaleCalibrationClick(const QPointF& scenePos) {
  if (!scaleCalibrationFirstPointSet_) {
    scaleOverlay_->setFirstPoint(scenePos);
    scaleCalibrationFirstPointSet_ = true;
  } else {
    scaleOverlay_->setSecondPoint(scenePos);
    // Don't exit mode yet - wait for distance input
  }
}

void MissionMapView::handleAddWaypointClick(const QPointF& scenePos) {
  addWaypointAtPixel(scenePos);
  // Stay in add waypoint mode for rapid placement
}

void MissionMapView::handleSetStartPoseClick(const QPointF& scenePos) {
  QPointF meters = pixelToMeters(scenePos);
  RobotStartPose pose;
  pose.x = meters.x();
  pose.y = meters.y();
  pose.theta = 0.0;
  pose.frameId = "map";

  setStartPose(pose);
  setMode(Normal);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setCursor(Qt::ArrowCursor);
}

void MissionMapView::updateWaypointPath() {
  if (!showPath_) return;

  QList<QPointF> pathPoints;

  // Add start pose if exists
  if (startPoseItem_) {
    pathPoints.append(startPoseItem_->pos());
  }

  // Add waypoint positions in order
  for (const auto* item : waypointItems_) {
    pathPoints.append(item->pos());
  }

  if (pathPoints.size() < 2) {
    if (pathItem_) {
      pathItem_->setVisible(false);
    }
    return;
  }

  if (!pathItem_) {
    pathItem_ = new WaypointPathItem();
    scene_->addItem(pathItem_);
  }

  pathItem_->setPath(pathPoints);
  pathItem_->setVisible(true);
}

void MissionMapView::updateWaypointNumbers() {
  for (int i = 0; i < waypointItems_.size(); ++i) {
    waypointItems_[i]->setSequenceNumber(i + 1);
  }
}

}  // namespace ros_weaver
