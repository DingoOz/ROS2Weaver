#ifndef ROS_WEAVER_WIDGETS_LINK_PROPERTIES_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_LINK_PROPERTIES_WIDGET_HPP

#include <QWidget>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QUndoStack>

#include "ros_weaver/core/urdf_model.hpp"

namespace ros_weaver {

/**
 * @brief Properties widget for URDF links
 *
 * Provides editors for:
 * - Visual origin (position X,Y,Z and orientation R,P,Y)
 * - Collision origin
 * - Inertial properties (mass, inertia tensor)
 */
class LinkPropertiesWidget : public QWidget {
  Q_OBJECT

public:
  explicit LinkPropertiesWidget(QWidget* parent = nullptr);
  ~LinkPropertiesWidget() override;

  void setModel(URDFModel* model);
  void setUndoStack(QUndoStack* stack);

  void setLinkName(const QString& name);
  QString linkName() const { return linkName_; }

  void refresh();

signals:
  void propertyChanged();

private slots:
  void onVisualPositionChanged();
  void onVisualOrientationChanged();
  void onInertialChanged();
  void onMassChanged();

private:
  void setupUi();
  void setupConnections();

  void blockAllSignals(bool block);
  void updateFromModel();

  URDFModel* model_ = nullptr;
  QUndoStack* undoStack_ = nullptr;
  QString linkName_;

  // Visual origin
  QGroupBox* visualGroup_ = nullptr;
  QDoubleSpinBox* visualPosX_ = nullptr;
  QDoubleSpinBox* visualPosY_ = nullptr;
  QDoubleSpinBox* visualPosZ_ = nullptr;
  QDoubleSpinBox* visualRoll_ = nullptr;
  QDoubleSpinBox* visualPitch_ = nullptr;
  QDoubleSpinBox* visualYaw_ = nullptr;

  // Inertial
  QGroupBox* inertialGroup_ = nullptr;
  QDoubleSpinBox* mass_ = nullptr;
  QDoubleSpinBox* ixx_ = nullptr;
  QDoubleSpinBox* ixy_ = nullptr;
  QDoubleSpinBox* ixz_ = nullptr;
  QDoubleSpinBox* iyy_ = nullptr;
  QDoubleSpinBox* iyz_ = nullptr;
  QDoubleSpinBox* izz_ = nullptr;

  // Saved state for undo
  URDFPose savedVisualPose_;
  URDFInertial savedInertial_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_LINK_PROPERTIES_WIDGET_HPP
