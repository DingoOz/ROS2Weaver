#ifndef ROS_WEAVER_WAYPOINT_EDITOR_WIDGET_HPP
#define ROS_WEAVER_WAYPOINT_EDITOR_WIDGET_HPP

#include <QWidget>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QDial>
#include <QListWidget>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QColorDialog>
#include <QTextEdit>
#include <QDialog>

#include "ros_weaver/core/mission_data.hpp"

namespace ros_weaver {

/**
 * @brief Widget for editing waypoint properties
 *
 * Displays and allows editing of:
 * - Name and description
 * - Position (X, Y) in meters
 * - Orientation (theta) with dial control
 * - Tolerance
 * - Color
 * - Behavior triggers
 */
class WaypointEditorWidget : public QWidget {
  Q_OBJECT

public:
  explicit WaypointEditorWidget(QWidget* parent = nullptr);

  void setWaypoint(const Waypoint& waypoint);
  Waypoint getWaypoint() const;
  void clear();

  void setReadOnly(bool readOnly);

signals:
  void waypointChanged(const Waypoint& waypoint);
  void deleteRequested(int waypointId);
  void behaviorEditRequested(int waypointId, int triggerIndex);

private slots:
  void onNameChanged();
  void onPositionChanged();
  void onOrientationChanged();
  void onToleranceChanged();
  void onColorClicked();
  void onDescriptionChanged();

  void onAddTrigger();
  void onEditTrigger();
  void onRemoveTrigger();
  void onDeleteWaypoint();

private:
  void setupUi();
  void updateFromWaypoint();
  void emitWaypointChanged();
  void updateTriggersList();

  // Basic properties
  QLineEdit* nameEdit_;
  QDoubleSpinBox* xSpinBox_;
  QDoubleSpinBox* ySpinBox_;
  QDoubleSpinBox* thetaSpinBox_;
  QDial* orientationDial_;
  QDoubleSpinBox* toleranceSpinBox_;
  QPushButton* colorButton_;
  QTextEdit* descriptionEdit_;

  // Behavior triggers
  QListWidget* triggersList_;
  QPushButton* addTriggerButton_;
  QPushButton* editTriggerButton_;
  QPushButton* removeTriggerButton_;

  // Actions
  QPushButton* deleteWaypointButton_;

  Waypoint currentWaypoint_;
  bool updatingUi_ = false;
};

/**
 * @brief Dialog for editing behavior triggers
 */
class BehaviorTriggerDialog : public QDialog {
  Q_OBJECT

public:
  explicit BehaviorTriggerDialog(const BehaviorTrigger& trigger,
                                  QWidget* parent = nullptr);

  BehaviorTrigger getTrigger() const;

private slots:
  void onTriggerTypeChanged(int index);
  void onBrowseBehaviorTree();

private:
  void setupUi();
  void updateUiForTriggerType();
  void loadBehaviorTrees();

  QComboBox* triggerTypeCombo_;
  QDoubleSpinBox* approachDistanceSpinBox_;
  QComboBox* behaviorTreeCombo_;
  QPushButton* browseBTButton_;
  QLineEdit* behaviorNodeEdit_;
  QLabel* approachDistanceLabel_;

  BehaviorTrigger currentTrigger_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WAYPOINT_EDITOR_WIDGET_HPP
