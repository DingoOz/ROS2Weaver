#ifndef ROS_WEAVER_URDF_TREE_VIEW_HPP
#define ROS_WEAVER_URDF_TREE_VIEW_HPP

#include <QTreeWidget>
#include <QMap>
#include <QStringList>
#include <QMenu>

#include "ros_weaver/core/urdf_parser.hpp"

namespace ros_weaver {

class URDFTreeView : public QTreeWidget {
  Q_OBJECT

public:
  explicit URDFTreeView(QWidget* parent = nullptr);
  ~URDFTreeView() override = default;

  // Set the model to display
  void setModel(const URDFModel& model);

  // Update a specific joint's display
  void updateJoint(const QString& jointName, const URDFJoint& joint);

  // Selection management (synchronized with 3D view)
  void selectJoint(const QString& jointName, bool addToSelection = false);
  void selectJoints(const QStringList& jointNames);
  void clearJointSelection();
  QStringList selectedJointNames() const;

  // Expand/collapse
  void expandAll();
  void collapseAll();
  void expandToJoint(const QString& jointName);

signals:
  void jointSelected(const QString& jointName, bool ctrlPressed);
  void jointDoubleClicked(const QString& jointName);
  void selectionChanged(const QStringList& selectedJoints);
  void jointContextMenuRequested(const QString& jointName, const QPoint& globalPos);

protected:
  void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;

private slots:
  void onItemSelectionChanged();

private:
  void setupUi();
  void buildTree(const URDFModel& model);
  void buildTreeRecursive(const QString& linkName, QTreeWidgetItem* parent, const URDFModel& model);
  QTreeWidgetItem* findJointItem(const QString& jointName) const;
  QTreeWidgetItem* findLinkItem(const QString& linkName) const;
  void populateJointItem(QTreeWidgetItem* item, const URDFJoint& joint);
  void populateLinkItem(QTreeWidgetItem* item, const URDFLink& link);

  // Tree items maps
  QMap<QString, QTreeWidgetItem*> jointItems_;
  QMap<QString, QTreeWidgetItem*> linkItems_;

  // Block selection signals during programmatic changes
  bool blockSelectionSignals_ = false;

  // Icons for different item types
  QIcon jointIcon_;
  QIcon linkIcon_;
  QIcon fixedJointIcon_;
  QIcon revoluteJointIcon_;
  QIcon prismaticJointIcon_;
  QIcon continuousJointIcon_;

  // Column indices
  static constexpr int COL_NAME = 0;
  static constexpr int COL_TYPE = 1;
  static constexpr int COL_PARENT = 2;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_URDF_TREE_VIEW_HPP
