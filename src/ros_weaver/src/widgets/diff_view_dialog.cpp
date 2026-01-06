#include "ros_weaver/widgets/diff_view_dialog.hpp"
#include "ros_weaver/core/project.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QGroupBox>
#include <QSplitter>

namespace ros_weaver {

DiffViewDialog::DiffViewDialog(QWidget* parent)
  : QDialog(parent)
{
  setupUi();
  setWindowTitle(tr("Architecture Diff View"));
  setMinimumSize(800, 600);
}

void DiffViewDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(12, 12, 12, 12);
  mainLayout->setSpacing(8);

  // Header with labels
  QHBoxLayout* headerLayout = new QHBoxLayout();

  beforeLabel_ = new QLabel(this);
  beforeLabel_->setStyleSheet("font-weight: bold; color: #d32f2f;");
  headerLayout->addWidget(new QLabel(tr("Before:"), this));
  headerLayout->addWidget(beforeLabel_);

  headerLayout->addStretch();

  QLabel* vsLabel = new QLabel(tr("vs"), this);
  vsLabel->setStyleSheet("font-size: 16px; color: #666;");
  headerLayout->addWidget(vsLabel);

  headerLayout->addStretch();

  afterLabel_ = new QLabel(this);
  afterLabel_->setStyleSheet("font-weight: bold; color: #388e3c;");
  headerLayout->addWidget(new QLabel(tr("After:"), this));
  headerLayout->addWidget(afterLabel_);

  mainLayout->addLayout(headerLayout);

  // Summary
  summaryLabel_ = new QLabel(this);
  summaryLabel_->setStyleSheet(R"(
    QLabel {
      background-color: #f5f5f5;
      padding: 8px;
      border-radius: 4px;
      font-size: 12px;
    }
  )");
  mainLayout->addWidget(summaryLabel_);

  // Filter controls
  QHBoxLayout* filterLayout = new QHBoxLayout();

  filterLayout->addWidget(new QLabel(tr("Category:"), this));
  filterCombo_ = new QComboBox(this);
  filterCombo_->addItem(tr("All"), "");
  filterCombo_->addItem(tr("Nodes"), "Nodes");
  filterCombo_->addItem(tr("Connections"), "Connections");
  filterCombo_->addItem(tr("Parameters"), "Parameters");
  filterCombo_->addItem(tr("Groups"), "Groups");
  connect(filterCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &DiffViewDialog::onFilterChanged);
  filterLayout->addWidget(filterCombo_);

  filterLayout->addSpacing(20);

  showInfoCheck_ = new QCheckBox(tr("Info"), this);
  showInfoCheck_->setChecked(true);
  connect(showInfoCheck_, &QCheckBox::toggled, this, &DiffViewDialog::onFilterChanged);
  filterLayout->addWidget(showInfoCheck_);

  showWarningCheck_ = new QCheckBox(tr("Warning"), this);
  showWarningCheck_->setChecked(true);
  connect(showWarningCheck_, &QCheckBox::toggled, this, &DiffViewDialog::onFilterChanged);
  filterLayout->addWidget(showWarningCheck_);

  showCriticalCheck_ = new QCheckBox(tr("Critical"), this);
  showCriticalCheck_->setChecked(true);
  connect(showCriticalCheck_, &QCheckBox::toggled, this, &DiffViewDialog::onFilterChanged);
  filterLayout->addWidget(showCriticalCheck_);

  filterLayout->addStretch();

  mainLayout->addLayout(filterLayout);

  // Diff tree
  diffTree_ = new QTreeWidget(this);
  diffTree_->setHeaderLabels(QStringList{tr("Change"), tr("Element"), tr("Before"), tr("After")});
  diffTree_->setColumnWidth(0, 150);
  diffTree_->setColumnWidth(1, 250);
  diffTree_->setColumnWidth(2, 180);
  diffTree_->setColumnWidth(3, 180);
  diffTree_->setRootIsDecorated(true);
  diffTree_->setAlternatingRowColors(true);
  diffTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  diffTree_->setStyleSheet(R"(
    QTreeWidget {
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    QTreeWidget::item {
      padding: 4px 2px;
    }
    QTreeWidget::item:selected {
      background-color: #1a73e8;
      color: white;
    }
  )");

  connect(diffTree_, &QTreeWidget::itemClicked,
          this, &DiffViewDialog::onItemClicked);
  connect(diffTree_, &QTreeWidget::itemDoubleClicked,
          this, &DiffViewDialog::onItemDoubleClicked);

  mainLayout->addWidget(diffTree_, 1);

  // Button row
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  exportButton_ = new QPushButton(tr("Export Report..."), this);
  connect(exportButton_, &QPushButton::clicked,
          this, &DiffViewDialog::onExportClicked);
  buttonLayout->addWidget(exportButton_);

  buttonLayout->addStretch();

  closeButton_ = new QPushButton(tr("Close"), this);
  connect(closeButton_, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton_);

  mainLayout->addLayout(buttonLayout);
}

void DiffViewDialog::setDiffResult(const DiffResult& result) {
  currentResult_ = result;
  beforeLabel_->setText(result.beforeLabel);
  afterLabel_->setText(result.afterLabel);
  updateSummary();
  populateTree();
}

void DiffViewDialog::compareProjects(const Project* before, const Project* after) {
  DiffResult result = diffEngine_.compare(before, after);
  setDiffResult(result);
}

void DiffViewDialog::compareFiles(const QString& beforePath, const QString& afterPath) {
  DiffResult result = diffEngine_.compareFiles(beforePath, afterPath);
  result.beforeLabel = QFileInfo(beforePath).fileName();
  result.afterLabel = QFileInfo(afterPath).fileName();
  setDiffResult(result);
}

void DiffViewDialog::updateSummary() {
  int added = currentResult_.addedCount();
  int removed = currentResult_.removedCount();
  int modified = currentResult_.modifiedCount();
  int total = currentResult_.items.size();

  QString summary;
  if (total == 0) {
    summary = tr("No differences found - architectures are identical");
  } else {
    summary = tr("%1 difference(s) found: ")
      .arg(total);

    QStringList parts;
    if (added > 0) {
      parts.append(QString("<span style='color: #388e3c;'>+%1 added</span>").arg(added));
    }
    if (removed > 0) {
      parts.append(QString("<span style='color: #d32f2f;'>-%1 removed</span>").arg(removed));
    }
    if (modified > 0) {
      parts.append(QString("<span style='color: #f57c00;'>~%1 modified</span>").arg(modified));
    }
    summary += parts.join(", ");

    // Add severity info
    int critical = currentResult_.criticalCount();
    if (critical > 0) {
      summary += QString(" | <span style='color: #d32f2f; font-weight: bold;'>%1 critical</span>").arg(critical);
    }
  }

  summaryLabel_->setText(summary);
}

void DiffViewDialog::populateTree() {
  diffTree_->clear();

  QString categoryFilter = filterCombo_->currentData().toString();

  // Create category nodes
  QMap<QString, QTreeWidgetItem*> categoryItems;

  for (const auto& item : currentResult_.items) {
    // Apply filters
    if (!categoryFilter.isEmpty() && item.category != categoryFilter) {
      continue;
    }
    if (item.severity == DiffSeverity::Info && !showInfoCheck_->isChecked()) {
      continue;
    }
    if (item.severity == DiffSeverity::Warning && !showWarningCheck_->isChecked()) {
      continue;
    }
    if (item.severity == DiffSeverity::Critical && !showCriticalCheck_->isChecked()) {
      continue;
    }

    // Get or create category item
    if (!categoryItems.contains(item.category)) {
      QTreeWidgetItem* catItem = new QTreeWidgetItem(diffTree_);
      catItem->setText(0, item.category);
      catItem->setExpanded(true);
      QFont font = catItem->font(0);
      font.setBold(true);
      catItem->setFont(0, font);
      categoryItems[item.category] = catItem;
    }

    QTreeWidgetItem* diffItem = createDiffItem(item);
    categoryItems[item.category]->addChild(diffItem);
  }

  // Update category counts
  for (auto it = categoryItems.begin(); it != categoryItems.end(); ++it) {
    it.value()->setText(0, QString("%1 (%2)")
      .arg(it.key()).arg(it.value()->childCount()));
  }
}

QTreeWidgetItem* DiffViewDialog::createDiffItem(const DiffItem& item) {
  QTreeWidgetItem* treeItem = new QTreeWidgetItem();

  // Type column with icon
  QString typeText = QString("%1 %2 %3")
    .arg(item.getSeverityIcon())
    .arg(item.getTypeIcon())
    .arg(item.getTypeLabel());
  treeItem->setText(0, typeText);
  treeItem->setForeground(0, item.getTypeColor());

  // Element name
  treeItem->setText(1, item.elementName);
  treeItem->setToolTip(1, item.description);

  // Before/after values
  treeItem->setText(2, item.beforeValue);
  treeItem->setText(3, item.afterValue);

  // Store element ID for navigation
  treeItem->setData(0, Qt::UserRole, item.elementId);

  return treeItem;
}

void DiffViewDialog::onItemClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString elementId = item->data(0, Qt::UserRole).toString();
  if (!elementId.isEmpty()) {
    // Show tooltip with full description
    // The actual navigation would be handled by the parent
  }
}

void DiffViewDialog::onItemDoubleClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString elementId = item->data(0, Qt::UserRole).toString();
  if (!elementId.isEmpty()) {
    emit navigateToElement(elementId);
  }
}

void DiffViewDialog::onFilterChanged() {
  populateTree();
}

void DiffViewDialog::onExportClicked() {
  QString filePath = QFileDialog::getSaveFileName(
    this,
    tr("Export Diff Report"),
    "diff_report.md",
    tr("Markdown Files (*.md);;Text Files (*.txt);;All Files (*)")
  );

  if (filePath.isEmpty()) {
    return;
  }

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Export Failed"),
      tr("Could not open file for writing: %1").arg(file.errorString()));
    return;
  }

  QTextStream out(&file);

  // Write header
  out << "# Architecture Diff Report\n\n";
  out << QString("**Before:** %1\n\n").arg(currentResult_.beforeLabel);
  out << QString("**After:** %1\n\n").arg(currentResult_.afterLabel);

  // Summary
  out << "## Summary\n\n";
  out << QString("- Total changes: %1\n").arg(currentResult_.items.size());
  out << QString("- Added: %1\n").arg(currentResult_.addedCount());
  out << QString("- Removed: %1\n").arg(currentResult_.removedCount());
  out << QString("- Modified: %1\n").arg(currentResult_.modifiedCount());
  out << QString("- Critical: %1\n").arg(currentResult_.criticalCount());
  out << "\n";

  // Group by category
  QMap<QString, QList<DiffItem>> byCategory;
  for (const auto& item : currentResult_.items) {
    byCategory[item.category].append(item);
  }

  for (auto it = byCategory.begin(); it != byCategory.end(); ++it) {
    out << QString("## %1\n\n").arg(it.key());

    out << "| Change | Element | Before | After |\n";
    out << "|--------|---------|--------|-------|\n";

    for (const auto& item : it.value()) {
      QString typeIcon = item.type == DiffType::NodeAdded ||
                         item.type == DiffType::ConnectionAdded ||
                         item.type == DiffType::ParameterAdded ||
                         item.type == DiffType::GroupAdded ? "+" :
                         item.type == DiffType::NodeRemoved ||
                         item.type == DiffType::ConnectionRemoved ||
                         item.type == DiffType::ParameterRemoved ||
                         item.type == DiffType::GroupRemoved ? "-" : "~";

      out << QString("| %1 %2 | %3 | %4 | %5 |\n")
        .arg(typeIcon)
        .arg(item.getTypeLabel())
        .arg(item.elementName)
        .arg(item.beforeValue.isEmpty() ? "-" : item.beforeValue)
        .arg(item.afterValue.isEmpty() ? "-" : item.afterValue);
    }

    out << "\n";
  }

  file.close();

  QMessageBox::information(this, tr("Export Complete"),
    tr("Diff report exported to:\n%1").arg(filePath));
}

}  // namespace ros_weaver
