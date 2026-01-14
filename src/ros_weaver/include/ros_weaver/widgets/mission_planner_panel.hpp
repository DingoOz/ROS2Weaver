#ifndef ROS_WEAVER_MISSION_PLANNER_PANEL_HPP
#define ROS_WEAVER_MISSION_PLANNER_PANEL_HPP

#include <QWidget>
#include <QSplitter>
#include <QTabWidget>
#include <QToolBar>
#include <QAction>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QPushButton>
#include <QDial>
#include <QComboBox>
#include <QInputDialog>

#include "ros_weaver/core/mission_data.hpp"
#include "ros_weaver/widgets/mission_map_view.hpp"
#include "ros_weaver/widgets/waypoint_editor_widget.hpp"
#include "ros_weaver/core/nav2_exporter.hpp"
#include "ros_weaver/core/undo/mission_undo_stack.hpp"

namespace ros_weaver {

class BehaviorTreePanel;
class MissionUndoCommand;

/**
 * @brief Main panel for visual mission planning
 *
 * Provides:
 * - Map loading and display
 * - Scale calibration
 * - Waypoint placement and editing
 * - Robot start pose configuration
 * - Mission settings
 * - Export to Nav2 formats
 */
class MissionPlannerPanel : public QWidget {
  Q_OBJECT

public:
  explicit MissionPlannerPanel(QWidget* parent = nullptr);
  ~MissionPlannerPanel() override;

  void setMission(const Mission& mission);
  Mission getMission() const;

  // Integration with behavior tree editor
  void setBehaviorTreeEditor(BehaviorTreePanel* editor);

  // Undo/Redo support
  MissionUndoStack* undoStack() const { return undoStack_; }
  void undo();
  void redo();

  // Methods called by undo commands (not for direct use)
  void addWaypointFromUndo(const Waypoint& waypoint);
  void insertWaypointFromUndo(const Waypoint& waypoint, int index);
  void removeWaypointById(int waypointId);
  void moveWaypointFromUndo(int waypointId, const QPointF& position);
  void setWaypointOrientationFromUndo(int waypointId, double theta);
  void setStartPoseFromUndo(const RobotStartPose& pose);
  void setWaypointsFromUndo(const QList<Waypoint>& waypoints);
  void clearWaypointsFromUndo();
  void reorderWaypointFromUndo(int fromIndex, int toIndex);

public slots:
  void newMission();
  void loadMission();
  void saveMission();
  void saveMissionAs();

signals:
  void missionChanged(const Mission& mission);
  void missionLoaded(const QString& path);
  void missionSaved(const QString& path);

private slots:
  // Toolbar actions
  void onLoadMap();
  void onLoadNav2Map();
  void onCalibrateScale();
  void onScaleDistanceEntered();
  void onAddWaypoint();
  void onSetStartPose();
  void onClearWaypoints();
  void onZoomFit();
  void onExportNav2();

  // Map view events
  void onWaypointAdded(const Waypoint& waypoint);
  void onWaypointSelected(int waypointId);
  void onWaypointMoved(int waypointId, const QPointF& newPositionMeters);
  void onWaypointOrientationChanged(int waypointId, double theta);
  void onWaypointDoubleClicked(int waypointId);
  void onStartPoseChanged(const RobotStartPose& pose);
  void onScaleCalibrated(const MapScale& scale);
  void onCoordinateHovered(const QPointF& meters);
  void onModeChanged(MissionMapView::Mode mode);

  // Waypoint list events
  void onWaypointListItemSelected();
  void onSelectionChanged(const QList<int>& waypointIds);
  void onMoveWaypointUp();
  void onMoveWaypointDown();

  // Waypoint editor events
  void onWaypointEditorChanged(const Waypoint& waypoint);
  void onWaypointDeleteRequested(int waypointId);

  // Mission settings changes
  void onMissionSettingsChanged();

  // Gradient colors
  void onGradientStartColorClicked();
  void onGradientEndColorClicked();
  void onApplyGradientColors();

private:
  void setupUi();
  void setupToolbar();
  void setupMapView();
  void setupPropertiesPanel();
  void setupMissionTab();
  void setupWaypointsTab();
  void setupStartPoseTab();
  void setupConnections();

  void updateWaypointsList();
  void updateMissionFromUi();
  void updateUiFromMission();
  void markMissionModified();

  // UI Components
  QToolBar* toolbar_;
  QSplitter* mainSplitter_;

  // Toolbar actions
  QAction* loadMapAction_;
  QAction* loadNav2MapAction_;
  QAction* calibrateScaleAction_;
  QAction* addWaypointAction_;
  QAction* setStartPoseAction_;
  QAction* clearWaypointsAction_;
  QAction* zoomFitAction_;
  QAction* exportNav2Action_;
  QAction* saveMissionAction_;
  QAction* loadMissionAction_;

  // Map view (left side)
  MissionMapView* mapView_;
  QLabel* coordinateLabel_;
  QLabel* scaleLabel_;
  QLabel* modeLabel_;

  // Properties panel (right side)
  QTabWidget* propertiesTabs_;

  // Mission tab
  QLineEdit* missionNameEdit_;
  QTextEdit* missionDescriptionEdit_;
  QCheckBox* loopMissionCheckbox_;
  QSpinBox* loopCountSpinBox_;
  QDoubleSpinBox* defaultSpeedSpinBox_;
  QDoubleSpinBox* defaultToleranceSpinBox_;

  // Waypoints tab
  QListWidget* waypointsList_;
  QPushButton* moveUpButton_;
  QPushButton* moveDownButton_;
  WaypointEditorWidget* waypointEditor_;

  // Gradient color controls
  QPushButton* gradientStartColorButton_;
  QPushButton* gradientEndColorButton_;
  QPushButton* applyGradientButton_;
  QColor gradientStartColor_ = QColor(0, 100, 255);   // Blue
  QColor gradientEndColor_ = QColor(255, 100, 0);     // Orange

  // Start pose tab
  QDoubleSpinBox* startXSpinBox_;
  QDoubleSpinBox* startYSpinBox_;
  QDoubleSpinBox* startThetaSpinBox_;
  QDial* startOrientationDial_;

  // Data
  Mission currentMission_;
  QString currentMissionPath_;
  bool missionModified_ = false;

  // Undo/Redo
  MissionUndoStack* undoStack_;

  // State tracking for undo commands
  QPointF waypointDragStartPos_;
  double waypointOrientationStart_ = 0.0;
  int draggingWaypointId_ = -1;
  RobotStartPose startPoseDragStart_;
  bool isDraggingStartPose_ = false;

  // Integration
  BehaviorTreePanel* behaviorTreeEditor_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MISSION_PLANNER_PANEL_HPP
