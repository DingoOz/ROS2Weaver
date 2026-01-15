#ifndef ROS_WEAVER_MISSION_MAP_VIEW_HPP
#define ROS_WEAVER_MISSION_MAP_VIEW_HPP

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QRubberBand>
#include <QSet>
#include <QMap>

#include "ros_weaver/core/mission_data.hpp"
#include "ros_weaver/widgets/waypoint_graphics_item.hpp"

namespace ros_weaver {

/**
 * @brief QGraphicsView-based widget for mission planning on a map
 *
 * Provides:
 * - Map image loading and display
 * - Scale calibration mode
 * - Waypoint placement and editing
 * - Robot start pose placement
 * - Pan and zoom navigation
 */
class MissionMapView : public QGraphicsView {
  Q_OBJECT

public:
  explicit MissionMapView(QWidget* parent = nullptr);
  ~MissionMapView() override;

  // Map management
  void loadMapImage(const QString& imagePath);
  void loadNav2Map(const QString& yamlPath);  // Load map.yaml with resolution/origin
  void clearMap();

  // Scale calibration
  void enterScaleCalibrationMode();
  void exitScaleCalibrationMode();
  void setScale(const MapScale& scale);
  MapScale getScale() const { return scale_; }

  // Waypoint management
  void addWaypoint(const QPointF& positionMeters);
  void addWaypointAtPixel(const QPointF& pixelPos);
  void removeWaypoint(int waypointId);
  void updateWaypoint(const Waypoint& waypoint);
  void updateWaypointOrientation(int waypointId, double theta);
  void selectWaypoint(int waypointId);
  void selectWaypoints(const QList<int>& waypointIds);
  void addToSelection(int waypointId);
  void toggleSelection(int waypointId);
  QList<int> selectedWaypointIds() const;
  bool isWaypointSelected(int waypointId) const;
  void clearWaypoints();
  void setWaypoints(const QList<Waypoint>& waypoints);
  QList<Waypoint> getWaypoints() const;

  // Robot start pose
  void setStartPose(const RobotStartPose& pose);
  void updateStartPoseOrientation(double theta);
  RobotStartPose getStartPose() const { return startPose_; }
  void enterStartPoseMode();

  // Interaction modes
  enum Mode {
    Normal,
    AddWaypoint,
    ScaleCalibration,
    SetStartPose,
    SetOrientation
  };
  void setMode(Mode mode);
  Mode mode() const { return currentMode_; }

  // View controls
  void zoomIn();
  void zoomOut();
  void zoomToFit();
  void setShowGrid(bool show);
  void setShowCoordinates(bool show);
  void setShowPath(bool show);
  void setLoopMission(bool loop);

  // Coordinate conversion using current scale
  QPointF pixelToMeters(const QPointF& pixel) const;
  QPointF metersToPixel(const QPointF& meters) const;

signals:
  void waypointAdded(const Waypoint& waypoint);
  void waypointRemoved(int waypointId);
  void waypointSelected(int waypointId);
  void selectionChanged(const QList<int>& selectedIds);
  void waypointMoved(int waypointId, const QPointF& newPositionMeters);
  void waypointsMovedTogether(const QList<int>& waypointIds, const QPointF& delta);
  void waypointOrientationChanged(int waypointId, double theta);
  void waypointDoubleClicked(int waypointId);
  void startPoseChanged(const RobotStartPose& pose);
  void scaleCalibrated(const MapScale& scale);
  void coordinateHovered(const QPointF& meters);
  void modeChanged(Mode mode);
  void mapLoaded(const QString& path);

protected:
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void drawBackground(QPainter* painter, const QRectF& rect) override;

private slots:
  void onWaypointMoved(int waypointId, const QPointF& newPos);
  void onWaypointDragStarted(int waypointId, const QPointF& startPos);
  void onWaypointDragging(int waypointId, const QPointF& currentPos);
  void onWaypointOrientationChanged(int waypointId, double theta);
  void onWaypointClicked(int waypointId, Qt::KeyboardModifiers modifiers);
  void onWaypointDoubleClicked(int waypointId);
  void onStartPoseMoved(const QPointF& newPos);
  void onStartPoseOrientationChanged(double theta);
  void onScaleCalibrationComplete(const QPointF& p1, const QPointF& p2);

private:
  void setupScene();
  void updateWaypointPath();
  void updateWaypointNumbers();
  void addWaypointItem(const Waypoint& waypoint);
  WaypointGraphicsItem* findWaypointItem(int waypointId);
  void clearSelection();
  void handleScaleCalibrationClick(const QPointF& scenePos);
  void handleAddWaypointClick(const QPointF& scenePos);
  void handleSetStartPoseClick(const QPointF& scenePos);

  QGraphicsScene* scene_;
  QGraphicsPixmapItem* mapItem_;
  QList<WaypointGraphicsItem*> waypointItems_;
  RobotStartPoseItem* startPoseItem_;
  ScaleCalibrationOverlay* scaleOverlay_;
  WaypointPathItem* pathItem_;

  Mode currentMode_ = Normal;
  MapScale scale_;
  RobotStartPose startPose_;

  int nextWaypointId_ = 1;
  int selectedWaypointId_ = -1;
  QSet<int> selectedWaypointIds_;

  // Rubber band selection
  QRubberBand* rubberBand_ = nullptr;
  QPoint rubberBandOrigin_;
  bool isRubberBandSelecting_ = false;

  // Multi-waypoint drag tracking
  bool isMultiDragging_ = false;
  QPointF multiDragStartPos_;
  QMap<int, QPointF> dragStartPositions_;

  // Display options
  bool showGrid_ = false;
  bool showCoordinates_ = true;
  bool showPath_ = true;
  bool loopMission_ = false;

  // Scale calibration state
  bool scaleCalibrationFirstPointSet_ = false;

  // Middle mouse panning
  bool isMiddleMousePanning_ = false;
  QPoint lastPanPoint_;

  // Map metadata (from Nav2 map.yaml)
  double mapResolution_ = 0.05;  // meters per pixel
  QPointF mapOrigin_;            // origin in meters
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MISSION_MAP_VIEW_HPP
