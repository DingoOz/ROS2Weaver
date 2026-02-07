#ifndef ROS_WEAVER_WIDGETS_URDF_HIERARCHY_TREE_HPP
#define ROS_WEAVER_WIDGETS_URDF_HIERARCHY_TREE_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QMap>

namespace ros_weaver {

class URDFModel;

/**
 * @brief Blender-style hierarchy tree for URDF links and joints
 *
 * Features:
 * - Tree structure showing link/joint hierarchy
 * - Search/filter functionality
 * - Icons differentiating links and joints
 * - Expand/collapse all
 * - Context menu for operations
 */
class URDFHierarchyTree : public QWidget {
  Q_OBJECT

public:
  explicit URDFHierarchyTree(QWidget* parent = nullptr);
  ~URDFHierarchyTree() override;

  // Model management
  void setModel(URDFModel* model);
  URDFModel* model() const { return model_; }
  void refreshTree();

  // Selection
  QString selectedName() const;
  bool isSelectedJoint() const;
  void selectByName(const QString& name, bool isJoint);
  void clearSelection();

  // Tree operations
  void expandAll();
  void collapseAll();
  void expandToSelected();

signals:
  void selectionChanged(const QString& name, bool isJoint);
  void itemDoubleClicked(const QString& name, bool isJoint);

public slots:
  void onFilterTextChanged(const QString& text);
  void onExpandAll();
  void onCollapseAll();

private slots:
  void onTreeSelectionChanged();
  void onTreeItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onContextMenuRequested(const QPoint& pos);

private:
  void setupUi();
  void setupConnections();

  void populateTree();
  void addLinkToTree(const QString& linkName, QTreeWidgetItem* parent);
  QTreeWidgetItem* createLinkItem(const QString& linkName);
  QTreeWidgetItem* createJointItem(const QString& jointName);

  void applyFilter(const QString& filter);
  bool itemMatchesFilter(QTreeWidgetItem* item, const QString& filter);

  // Icons
  QIcon linkIcon() const;
  QIcon jointRevoluteIcon() const;
  QIcon jointPrismaticIcon() const;
  QIcon jointFixedIcon() const;
  QIcon jointContinuousIcon() const;

  URDFModel* model_ = nullptr;

  // UI elements
  QLineEdit* searchEdit_ = nullptr;
  QTreeWidget* treeWidget_ = nullptr;

  // Item tracking for fast lookup
  QMap<QString, QTreeWidgetItem*> linkItems_;
  QMap<QString, QTreeWidgetItem*> jointItems_;

  // Constants for tree item data
  static constexpr int NameRole = Qt::UserRole;
  static constexpr int IsJointRole = Qt::UserRole + 1;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_URDF_HIERARCHY_TREE_HPP
