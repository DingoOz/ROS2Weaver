#ifndef ROS_WEAVER_WIDGETS_URDF_PROPERTIES_PANEL_HPP
#define ROS_WEAVER_WIDGETS_URDF_PROPERTIES_PANEL_HPP

#include <QWidget>
#include <QStackedWidget>
#include <QUndoStack>

namespace ros_weaver {

class URDFModel;
class LinkPropertiesWidget;
class JointPropertiesWidget;

/**
 * @brief Stacked widget container for link and joint properties
 *
 * Shows the appropriate properties widget based on selection type.
 */
class URDFPropertiesPanel : public QWidget {
  Q_OBJECT

public:
  explicit URDFPropertiesPanel(QWidget* parent = nullptr);
  ~URDFPropertiesPanel() override;

  void setModel(URDFModel* model);
  URDFModel* model() const { return model_; }

  void setUndoStack(QUndoStack* stack);
  QUndoStack* undoStack() const { return undoStack_; }

  void showElement(const QString& name, bool isJoint);
  void showEmpty();

signals:
  void propertyChanged();

private:
  URDFModel* model_ = nullptr;
  QUndoStack* undoStack_ = nullptr;

  QStackedWidget* stackedWidget_ = nullptr;
  QWidget* emptyWidget_ = nullptr;
  LinkPropertiesWidget* linkWidget_ = nullptr;
  JointPropertiesWidget* jointWidget_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_URDF_PROPERTIES_PANEL_HPP
