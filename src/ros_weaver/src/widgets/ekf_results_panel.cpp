#include "ros_weaver/widgets/ekf_results_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>

#include <algorithm>

namespace ros_weaver {

EKFResultsPanel::EKFResultsPanel(QWidget* parent)
  : QWidget(parent)
{
  setupUi();
}

EKFResultsPanel::~EKFResultsPanel() = default;

void EKFResultsPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);

  // Summary label
  summaryLabel_ = new QLabel("No results yet", this);
  mainLayout->addWidget(summaryLabel_);

  // Results table
  resultsTable_ = new QTableWidget(this);
  resultsTable_->setColumnCount(7);
  resultsTable_->setHorizontalHeaderLabels({
    "Config", "ATE RMSE", "ATE Mean", "ATE Max", "RPE RMSE", "Yaw Drift", "Length"
  });

  resultsTable_->horizontalHeader()->setStretchLastSection(true);
  resultsTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  resultsTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  resultsTable_->setSelectionMode(QAbstractItemView::SingleSelection);
  resultsTable_->setSortingEnabled(false);  // We handle sorting manually

  connect(resultsTable_->selectionModel(), &QItemSelectionModel::selectionChanged,
          this, &EKFResultsPanel::onSelectionChanged);
  connect(resultsTable_->horizontalHeader(), &QHeaderView::sectionClicked,
          this, &EKFResultsPanel::onHeaderClicked);

  mainLayout->addWidget(resultsTable_, 1);

  // Buttons
  auto* buttonLayout = new QHBoxLayout();

  exportBestButton_ = new QPushButton("Export Best Config", this);
  exportBestButton_->setEnabled(false);
  connect(exportBestButton_, &QPushButton::clicked,
          this, &EKFResultsPanel::onExportBestClicked);
  buttonLayout->addWidget(exportBestButton_);

  exportAllButton_ = new QPushButton("Export All (CSV)", this);
  exportAllButton_->setEnabled(false);
  connect(exportAllButton_, &QPushButton::clicked,
          this, &EKFResultsPanel::onExportAllClicked);
  buttonLayout->addWidget(exportAllButton_);

  buttonLayout->addStretch();

  mainLayout->addLayout(buttonLayout);
}

void EKFResultsPanel::addResult(const EKFSweepResult& result) {
  results_.append(result);
  updateTable();

  exportBestButton_->setEnabled(!results_.isEmpty());
  exportAllButton_->setEnabled(!results_.isEmpty());
}

void EKFResultsPanel::setResults(const QVector<EKFSweepResult>& results) {
  results_ = results;
  updateTable();

  exportBestButton_->setEnabled(!results_.isEmpty());
  exportAllButton_->setEnabled(!results_.isEmpty());
}

void EKFResultsPanel::clearResults() {
  results_.clear();
  resultsTable_->setRowCount(0);
  summaryLabel_->setText("No results yet");

  exportBestButton_->setEnabled(false);
  exportAllButton_->setEnabled(false);
}

QVector<EKFSweepResult> EKFResultsPanel::results() const {
  return results_;
}

EKFSweepResult EKFResultsPanel::selectedResult() const {
  int idx = selectedIndex();
  if (idx >= 0 && idx < results_.size()) {
    return results_[idx];
  }
  return EKFSweepResult();
}

int EKFResultsPanel::selectedIndex() const {
  auto selected = resultsTable_->selectedItems();
  if (selected.isEmpty()) {
    return -1;
  }
  return selected.first()->row();
}

EKFSweepResult EKFResultsPanel::bestResult() const {
  int idx = bestResultIndex();
  if (idx >= 0) {
    return results_[idx];
  }
  return EKFSweepResult();
}

int EKFResultsPanel::bestResultIndex() const {
  if (results_.isEmpty()) {
    return -1;
  }

  int bestIdx = 0;
  double bestRmse = results_[0].metrics.ateRmse;

  for (int i = 1; i < results_.size(); ++i) {
    if (results_[i].metrics.ateRmse < bestRmse) {
      bestRmse = results_[i].metrics.ateRmse;
      bestIdx = i;
    }
  }

  return bestIdx;
}

void EKFResultsPanel::showSingleResult(const EKFMetrics& metrics) {
  EKFSweepResult result;
  result.metrics = metrics;
  result.metrics.configName = "Current Config";
  result.sweepIndex = 0;

  results_.clear();
  results_.append(result);
  updateTable();

  exportBestButton_->setEnabled(true);
  exportAllButton_->setEnabled(false);
}

void EKFResultsPanel::updateTable() {
  // Create sorted indices
  QVector<int> sortedIndices;
  for (int i = 0; i < results_.size(); ++i) {
    sortedIndices.append(i);
  }

  // Sort by selected column
  std::sort(sortedIndices.begin(), sortedIndices.end(), [this](int a, int b) {
    double valA = 0, valB = 0;

    switch (sortColumn_) {
      case 0:  // Config name (string sort)
        if (sortOrder_ == Qt::AscendingOrder) {
          return results_[a].metrics.configName < results_[b].metrics.configName;
        } else {
          return results_[a].metrics.configName > results_[b].metrics.configName;
        }
      case 1: valA = results_[a].metrics.ateRmse; valB = results_[b].metrics.ateRmse; break;
      case 2: valA = results_[a].metrics.ateMean; valB = results_[b].metrics.ateMean; break;
      case 3: valA = results_[a].metrics.ateMax; valB = results_[b].metrics.ateMax; break;
      case 4: valA = results_[a].metrics.rpeRmse; valB = results_[b].metrics.rpeRmse; break;
      case 5: valA = results_[a].metrics.yawDrift; valB = results_[b].metrics.yawDrift; break;
      case 6: valA = results_[a].metrics.trajectoryLength; valB = results_[b].metrics.trajectoryLength; break;
      default: return false;
    }

    if (sortOrder_ == Qt::AscendingOrder) {
      return valA < valB;
    } else {
      return valA > valB;
    }
  });

  // Update table
  resultsTable_->setRowCount(results_.size());

  for (int row = 0; row < sortedIndices.size(); ++row) {
    int idx = sortedIndices[row];
    const auto& result = results_[idx];
    const auto& m = result.metrics;

    auto setItem = [this, row](int col, const QString& text, int originalIdx) {
      auto* item = new QTableWidgetItem(text);
      item->setData(Qt::UserRole, originalIdx);  // Store original index
      item->setFlags(item->flags() & ~Qt::ItemIsEditable);
      resultsTable_->setItem(row, col, item);
    };

    setItem(0, m.configName, idx);
    setItem(1, QString::number(m.ateRmse, 'f', 4), idx);
    setItem(2, QString::number(m.ateMean, 'f', 4), idx);
    setItem(3, QString::number(m.ateMax, 'f', 4), idx);
    setItem(4, QString::number(m.rpeRmse, 'f', 4), idx);
    setItem(5, QString::number(m.yawDrift, 'f', 4), idx);
    setItem(6, QString::number(m.trajectoryLength, 'f', 2), idx);
  }

  highlightBestResult();

  // Update summary
  if (!results_.isEmpty()) {
    int bestIdx = bestResultIndex();
    double bestRmse = results_[bestIdx].metrics.ateRmse;
    summaryLabel_->setText(QString("%1 results | Best ATE RMSE: %2 m (%3)")
        .arg(results_.size())
        .arg(bestRmse, 0, 'f', 4)
        .arg(results_[bestIdx].metrics.configName));
  } else {
    summaryLabel_->setText("No results yet");
  }
}

void EKFResultsPanel::highlightBestResult() {
  int bestIdx = bestResultIndex();
  if (bestIdx < 0) return;

  // Find the row containing the best result
  for (int row = 0; row < resultsTable_->rowCount(); ++row) {
    auto* item = resultsTable_->item(row, 0);
    if (item && item->data(Qt::UserRole).toInt() == bestIdx) {
      // Highlight this row
      QColor highlightColor(200, 255, 200);  // Light green
      for (int col = 0; col < resultsTable_->columnCount(); ++col) {
        auto* cell = resultsTable_->item(row, col);
        if (cell) {
          cell->setBackground(highlightColor);
        }
      }
    }
  }
}

void EKFResultsPanel::onSelectionChanged() {
  int row = selectedIndex();
  if (row < 0) return;

  auto* item = resultsTable_->item(row, 0);
  if (!item) return;

  int originalIdx = item->data(Qt::UserRole).toInt();
  if (originalIdx >= 0 && originalIdx < results_.size()) {
    emit resultSelected(originalIdx, results_[originalIdx]);
  }
}

void EKFResultsPanel::onExportBestClicked() {
  emit exportBestRequested();
}

void EKFResultsPanel::onExportAllClicked() {
  emit exportAllRequested();
}

void EKFResultsPanel::onHeaderClicked(int logicalIndex) {
  if (sortColumn_ == logicalIndex) {
    // Toggle sort order
    sortOrder_ = (sortOrder_ == Qt::AscendingOrder) ?
                 Qt::DescendingOrder : Qt::AscendingOrder;
  } else {
    sortColumn_ = logicalIndex;
    sortOrder_ = Qt::AscendingOrder;
  }

  updateTable();

  // Update header indicator
  resultsTable_->horizontalHeader()->setSortIndicator(sortColumn_, sortOrder_);
  resultsTable_->horizontalHeader()->setSortIndicatorShown(true);
}

}  // namespace ros_weaver
