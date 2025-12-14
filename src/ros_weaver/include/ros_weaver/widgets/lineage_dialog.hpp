#ifndef ROS_WEAVER_LINEAGE_DIALOG_HPP
#define ROS_WEAVER_LINEAGE_DIALOG_HPP

#include <QDialog>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include "ros_weaver/core/data_lineage.hpp"

namespace ros_weaver {

// Dialog that displays data lineage information with options to open files
class LineageDialog : public QDialog {
  Q_OBJECT

public:
  explicit LineageDialog(const DataLineage& lineage, QWidget* parent = nullptr);
  ~LineageDialog() override;

  // Set the lineage to display
  void setLineage(const DataLineage& lineage);

private slots:
  void onOpenInVsCode();
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onCopyPath();

private:
  void setupUi();
  void populateTree();
  void addLineageNodeToTree(QTreeWidgetItem* parent, const LineageNode& node,
                            const QString& label);
  QString getSelectedFilePath() const;
  int getSelectedLineNumber() const;

  DataLineage lineage_;
  QLabel* headerLabel_;
  QTreeWidget* lineageTree_;
  QPushButton* openVsCodeButton_;
  QPushButton* copyPathButton_;
  QPushButton* closeButton_;
};

// Context menu helper for adding lineage actions to any widget
class LineageContextMenu {
public:
  // Add "Show Data Origin" action to a context menu
  // Returns the action so it can be connected
  static QAction* addLineageAction(QMenu* menu, const DataLineage& lineage,
                                   QWidget* parent);

  // Show a context menu with lineage information at the given position
  static void showContextMenu(const QPoint& globalPos,
                              const DataLineage& lineage,
                              QWidget* parent);

  // Show the lineage dialog
  static void showLineageDialog(const DataLineage& lineage, QWidget* parent);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_LINEAGE_DIALOG_HPP
