#ifndef ROS_WEAVER_WIDGETS_EKF_RESULTS_PANEL_HPP
#define ROS_WEAVER_WIDGETS_EKF_RESULTS_PANEL_HPP

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

/**
 * @brief Results panel showing EKF evaluation metrics
 *
 * Displays a sortable table of metrics from simulation runs or parameter sweeps.
 * Supports:
 * - Multiple result entries
 * - Sortable columns
 * - Selection for highlighting in visualizer
 * - Export of best configuration
 */
class EKFResultsPanel : public QWidget {
  Q_OBJECT

public:
  explicit EKFResultsPanel(QWidget* parent = nullptr);
  ~EKFResultsPanel() override;

  // Add/clear results
  void addResult(const EKFSweepResult& result);
  void setResults(const QVector<EKFSweepResult>& results);
  void clearResults();

  // Access results
  QVector<EKFSweepResult> results() const;
  EKFSweepResult selectedResult() const;
  int selectedIndex() const;

  // Get best result (lowest ATE RMSE)
  EKFSweepResult bestResult() const;
  int bestResultIndex() const;

  // Single result display (for non-sweep runs)
  void showSingleResult(const EKFMetrics& metrics);

signals:
  void resultSelected(int index, const EKFSweepResult& result);
  void exportBestRequested();
  void exportAllRequested();

private slots:
  void onSelectionChanged();
  void onExportBestClicked();
  void onExportAllClicked();
  void onHeaderClicked(int logicalIndex);

private:
  void setupUi();
  void updateTable();
  void highlightBestResult();

  // Results storage
  QVector<EKFSweepResult> results_;

  // UI components
  QTableWidget* resultsTable_ = nullptr;
  QLabel* summaryLabel_ = nullptr;
  QPushButton* exportBestButton_ = nullptr;
  QPushButton* exportAllButton_ = nullptr;

  // Sorting state
  int sortColumn_ = 1;  // Default sort by ATE RMSE
  Qt::SortOrder sortOrder_ = Qt::AscendingOrder;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_EKF_RESULTS_PANEL_HPP
