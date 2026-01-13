#ifndef ROS_WEAVER_BEHAVIOR_TREE_PANEL_HPP
#define ROS_WEAVER_BEHAVIOR_TREE_PANEL_HPP

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QFileDialog>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros_weaver/core/behavior_tree_parser.hpp"
#include "ros_weaver/widgets/bt_node_item.hpp"

namespace ros_weaver {

/**
 * @brief Dockable panel for visualizing BehaviorTree.CPP behavior trees
 *
 * MVP Features:
 * - Load BT XML files
 * - Display tree structure with colored nodes
 * - Show node details on hover
 *
 * Future Features:
 * - Live execution state from ROS2 topic
 * - Interactive editing
 */
class BehaviorTreePanel : public QWidget {
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget* parent = nullptr);
  ~BehaviorTreePanel() override;

  /**
   * @brief Load a behavior tree from an XML file
   */
  bool loadFromFile(const QString& filePath);

  /**
   * @brief Load a behavior tree from XML string
   */
  bool loadFromString(const QString& xmlContent);

  /**
   * @brief Clear the current tree
   */
  void clear();

  /**
   * @brief Set the ROS node for live state subscription (future)
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

public slots:
  void onLoadFileClicked();
  void onZoomInClicked();
  void onZoomOutClicked();
  void onZoomFitClicked();
  void onRefreshClicked();

signals:
  void treeLoaded(const QString& treeName);
  void loadError(const QString& error);

private:
  void setupUi();
  void buildTreeVisualization();
  void addNodeToScene(std::shared_ptr<BTNode> node, BTNodeItem* parentItem = nullptr);
  void updateStatusLabel();

  // UI components
  QVBoxLayout* mainLayout_;
  QHBoxLayout* toolbarLayout_;
  QPushButton* loadButton_;
  QPushButton* refreshButton_;
  QPushButton* zoomInButton_;
  QPushButton* zoomOutButton_;
  QPushButton* zoomFitButton_;
  QLabel* statusLabel_;
  QLabel* treeNameLabel_;
  QGraphicsScene* scene_;
  QGraphicsView* view_;

  // Data
  BehaviorTree currentTree_;
  QString currentFilePath_;
  BehaviorTreeParser parser_;

  // Node items for updates
  QMap<QString, BTNodeItem*> nodeItems_;

  // ROS2 integration (for future live state)
  rclcpp::Node::SharedPtr rosNode_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_BEHAVIOR_TREE_PANEL_HPP
