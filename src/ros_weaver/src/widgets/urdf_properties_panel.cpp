#include "ros_weaver/widgets/urdf_properties_panel.hpp"
#include "ros_weaver/widgets/link_properties_widget.hpp"
#include "ros_weaver/widgets/joint_properties_widget.hpp"

#include <QVBoxLayout>
#include <QLabel>

namespace ros_weaver {

URDFPropertiesPanel::URDFPropertiesPanel(QWidget* parent)
  : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  stackedWidget_ = new QStackedWidget(this);

  // Empty widget (shown when nothing is selected)
  emptyWidget_ = new QWidget(this);
  auto* emptyLayout = new QVBoxLayout(emptyWidget_);
  QLabel* emptyLabel = new QLabel(tr("Select a link or joint\nto view properties"));
  emptyLabel->setAlignment(Qt::AlignCenter);
  emptyLabel->setStyleSheet("color: gray;");
  emptyLayout->addWidget(emptyLabel);
  stackedWidget_->addWidget(emptyWidget_);

  // Link properties widget
  linkWidget_ = new LinkPropertiesWidget(this);
  stackedWidget_->addWidget(linkWidget_);
  connect(linkWidget_, &LinkPropertiesWidget::propertyChanged,
          this, &URDFPropertiesPanel::propertyChanged);

  // Joint properties widget
  jointWidget_ = new JointPropertiesWidget(this);
  stackedWidget_->addWidget(jointWidget_);
  connect(jointWidget_, &JointPropertiesWidget::propertyChanged,
          this, &URDFPropertiesPanel::propertyChanged);

  layout->addWidget(stackedWidget_);

  // Start with empty widget
  stackedWidget_->setCurrentWidget(emptyWidget_);
}

URDFPropertiesPanel::~URDFPropertiesPanel() = default;

void URDFPropertiesPanel::setModel(URDFModel* model) {
  model_ = model;
  linkWidget_->setModel(model);
  jointWidget_->setModel(model);

  showEmpty();
}

void URDFPropertiesPanel::setUndoStack(QUndoStack* stack) {
  undoStack_ = stack;
  linkWidget_->setUndoStack(stack);
  jointWidget_->setUndoStack(stack);
}

void URDFPropertiesPanel::showElement(const QString& name, bool isJoint) {
  if (!model_ || name.isEmpty()) {
    showEmpty();
    return;
  }

  if (isJoint) {
    jointWidget_->setJointName(name);
    stackedWidget_->setCurrentWidget(jointWidget_);
  } else {
    linkWidget_->setLinkName(name);
    stackedWidget_->setCurrentWidget(linkWidget_);
  }
}

void URDFPropertiesPanel::showEmpty() {
  stackedWidget_->setCurrentWidget(emptyWidget_);
}

}  // namespace ros_weaver
