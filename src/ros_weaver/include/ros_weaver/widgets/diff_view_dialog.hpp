#ifndef ROS_WEAVER_WIDGETS_DIFF_VIEW_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_DIFF_VIEW_DIALOG_HPP

#include <QDialog>
#include <QTreeWidget>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QCheckBox>
#include "ros_weaver/core/architecture_diff.hpp"

namespace ros_weaver {

class Project;

/**
 * @brief Dialog for viewing architecture differences
 *
 * Features:
 * - Tree view of differences by category
 * - Color-coded by change type (added/removed/modified)
 * - Severity filtering
 * - Before/after value comparison
 * - Export diff report
 */
class DiffViewDialog : public QDialog {
  Q_OBJECT

public:
  explicit DiffViewDialog(QWidget* parent = nullptr);
  ~DiffViewDialog() override = default;

  /**
   * @brief Set the diff result to display
   */
  void setDiffResult(const DiffResult& result);

  /**
   * @brief Compare two projects and display result
   */
  void compareProjects(const Project* before, const Project* after);

  /**
   * @brief Compare two project files and display result
   */
  void compareFiles(const QString& beforePath, const QString& afterPath);

signals:
  /**
   * @brief Emitted when user clicks on a diff item to navigate to it
   */
  void navigateToElement(const QString& elementId);

private slots:
  void onItemClicked(QTreeWidgetItem* item, int column);
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onFilterChanged();
  void onExportClicked();

private:
  void setupUi();
  void populateTree();
  void updateSummary();
  QTreeWidgetItem* createDiffItem(const DiffItem& item);

  // UI components
  QLabel* summaryLabel_ = nullptr;
  QLabel* beforeLabel_ = nullptr;
  QLabel* afterLabel_ = nullptr;
  QTreeWidget* diffTree_ = nullptr;
  QComboBox* filterCombo_ = nullptr;
  QCheckBox* showInfoCheck_ = nullptr;
  QCheckBox* showWarningCheck_ = nullptr;
  QCheckBox* showCriticalCheck_ = nullptr;
  QPushButton* exportButton_ = nullptr;
  QPushButton* closeButton_ = nullptr;

  // Data
  DiffResult currentResult_;
  ArchitectureDiff diffEngine_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_DIFF_VIEW_DIALOG_HPP
