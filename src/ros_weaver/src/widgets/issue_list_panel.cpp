#include "ros_weaver/widgets/issue_list_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QStyle>

namespace ros_weaver {

IssueListPanel::IssueListPanel(QWidget* parent)
    : QWidget(parent)
    , titleLabel_(nullptr)
    , analyzeButton_(nullptr)
    , progressBar_(nullptr)
    , severityFilter_(nullptr)
    , categoryFilter_(nullptr)
    , showHintsCheck_(nullptr)
    , issueTree_(nullptr)
    , summaryLabel_(nullptr)
    , autoFixButton_(nullptr)
    , askAIButton_(nullptr)
    , refreshButton_(nullptr)
{
  setupUi();

  // Connect to analyzer signals
  connect(&StaticAnalyzer::instance(), &StaticAnalyzer::analysisProgress,
          this, &IssueListPanel::onAnalysisProgress);
  connect(&StaticAnalyzer::instance(), &StaticAnalyzer::analysisCompleted,
          this, &IssueListPanel::setAnalysisResult);
}

void IssueListPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Header with title and analyze button
  QHBoxLayout* headerLayout = new QHBoxLayout();

  titleLabel_ = new QLabel(tr("Static Analysis"));
  titleLabel_->setStyleSheet("font-weight: bold; font-size: 14px;");

  analyzeButton_ = new QPushButton(tr("Analyze"));
  analyzeButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  connect(analyzeButton_, &QPushButton::clicked, this, &IssueListPanel::runAnalysis);

  headerLayout->addWidget(titleLabel_);
  headerLayout->addStretch();
  headerLayout->addWidget(analyzeButton_);
  mainLayout->addLayout(headerLayout);

  // Progress bar
  progressBar_ = new QProgressBar();
  progressBar_->setVisible(false);
  progressBar_->setTextVisible(true);
  mainLayout->addWidget(progressBar_);

  // Filters
  QHBoxLayout* filterLayout = new QHBoxLayout();

  QLabel* severityLabel = new QLabel(tr("Severity:"));
  severityFilter_ = new QComboBox();
  severityFilter_->addItem(tr("All"), -1);
  severityFilter_->addItem(tr("Errors"), static_cast<int>(IssueSeverity::Error));
  severityFilter_->addItem(tr("Warnings"), static_cast<int>(IssueSeverity::Warning));
  severityFilter_->addItem(tr("Info"), static_cast<int>(IssueSeverity::Info));
  connect(severityFilter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &IssueListPanel::onFilterChanged);

  QLabel* categoryLabel = new QLabel(tr("Category:"));
  categoryFilter_ = new QComboBox();
  categoryFilter_->addItem(tr("All"), -1);
  categoryFilter_->addItem(tr("Type Mismatch"), static_cast<int>(IssueCategory::TypeMismatch));
  categoryFilter_->addItem(tr("Unused Pub/Sub"), static_cast<int>(IssueCategory::UnusedPublisher));
  categoryFilter_->addItem(tr("QoS"), static_cast<int>(IssueCategory::QoSIncompatible));
  categoryFilter_->addItem(tr("Cyclic"), static_cast<int>(IssueCategory::CyclicDependency));
  categoryFilter_->addItem(tr("Security"), static_cast<int>(IssueCategory::Security));
  connect(categoryFilter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &IssueListPanel::onFilterChanged);

  showHintsCheck_ = new QCheckBox(tr("Show hints"));
  showHintsCheck_->setChecked(true);
  connect(showHintsCheck_, &QCheckBox::toggled, this, &IssueListPanel::onFilterChanged);

  filterLayout->addWidget(severityLabel);
  filterLayout->addWidget(severityFilter_);
  filterLayout->addWidget(categoryLabel);
  filterLayout->addWidget(categoryFilter_);
  filterLayout->addWidget(showHintsCheck_);
  filterLayout->addStretch();
  mainLayout->addLayout(filterLayout);

  // Issue tree
  issueTree_ = new QTreeWidget();
  issueTree_->setHeaderLabels({tr(""), tr("Issue"), tr("Location"), tr("Suggestion")});
  issueTree_->setRootIsDecorated(false);
  issueTree_->setAlternatingRowColors(true);
  issueTree_->header()->setStretchLastSection(true);
  issueTree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  issueTree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  issueTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  issueTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(issueTree_, &QTreeWidget::itemClicked,
          this, &IssueListPanel::onIssueSelected);
  connect(issueTree_, &QTreeWidget::itemDoubleClicked,
          this, &IssueListPanel::onIssueDoubleClicked);
  mainLayout->addWidget(issueTree_, 1);

  // Action buttons
  QHBoxLayout* actionLayout = new QHBoxLayout();

  autoFixButton_ = new QPushButton(tr("Auto Fix"));
  autoFixButton_->setEnabled(false);
  autoFixButton_->setToolTip(tr("Automatically fix the selected issue"));
  connect(autoFixButton_, &QPushButton::clicked, this, &IssueListPanel::onAutoFixClicked);

  askAIButton_ = new QPushButton(tr("Ask AI"));
  askAIButton_->setEnabled(false);
  askAIButton_->setToolTip(tr("Get AI suggestions for fixing this issue"));
  connect(askAIButton_, &QPushButton::clicked, this, &IssueListPanel::onAskAIClicked);

  summaryLabel_ = new QLabel();
  summaryLabel_->setStyleSheet("color: #666; font-size: 11px;");

  actionLayout->addWidget(autoFixButton_);
  actionLayout->addWidget(askAIButton_);
  actionLayout->addStretch();
  actionLayout->addWidget(summaryLabel_);
  mainLayout->addLayout(actionLayout);

  // Initial state
  summaryLabel_->setText(tr("Click 'Analyze' to run static analysis"));
}

void IssueListPanel::setAnalysisResult(const AnalysisResult& result) {
  currentResult_ = result;
  progressBar_->setVisible(false);
  populateIssueTree(result);
  updateSummary();
}

void IssueListPanel::clear() {
  issueTree_->clear();
  currentResult_ = AnalysisResult();
  hasSelection_ = false;
  autoFixButton_->setEnabled(false);
  askAIButton_->setEnabled(false);
  summaryLabel_->setText(tr("Click 'Analyze' to run static analysis"));
}

void IssueListPanel::runAnalysis() {
  progressBar_->setVisible(true);
  progressBar_->setValue(0);
  analyzeButton_->setEnabled(false);
  emit analysisRequested();
}

void IssueListPanel::onFilterChanged() {
  populateIssueTree(currentResult_);
}

void IssueListPanel::onIssueSelected(QTreeWidgetItem* item, int /*column*/) {
  if (!item) {
    hasSelection_ = false;
    autoFixButton_->setEnabled(false);
    askAIButton_->setEnabled(false);
    return;
  }

  QUuid issueId = item->data(0, Qt::UserRole).toUuid();

  for (const AnalysisIssue& issue : currentResult_.issues) {
    if (issue.id == issueId) {
      selectedIssue_ = issue;
      hasSelection_ = true;
      autoFixButton_->setEnabled(issue.canAutoFix);
      askAIButton_->setEnabled(true);

      // Emit navigation signal
      if (!issue.affectedBlockId.isNull()) {
        emit navigateToBlock(issue.affectedBlockId);
      } else if (!issue.affectedConnectionId.isNull()) {
        emit navigateToConnection(issue.affectedConnectionId);
      }
      break;
    }
  }
}

void IssueListPanel::onIssueDoubleClicked(QTreeWidgetItem* item, int column) {
  onIssueSelected(item, column);
  // Double click could open a detail dialog or jump to location
}

void IssueListPanel::onAutoFixClicked() {
  if (!hasSelection_) return;

  // TODO: Implement auto-fix through StaticAnalyzer
  // For now, just show that it was clicked
}

void IssueListPanel::onAskAIClicked() {
  if (!hasSelection_) return;

  emit askAIAboutIssue(selectedIssue_);
}

void IssueListPanel::onAnalysisProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  progressBar_->setFormat(message);

  if (percent >= 100) {
    analyzeButton_->setEnabled(true);
  }
}

void IssueListPanel::populateIssueTree(const AnalysisResult& result) {
  issueTree_->clear();

  for (const AnalysisIssue& issue : result.issues) {
    if (!matchesFilter(issue)) {
      continue;
    }

    QTreeWidgetItem* item = createIssueItem(issue);
    issueTree_->addTopLevelItem(item);
  }

  updateSummary();
}

QTreeWidgetItem* IssueListPanel::createIssueItem(const AnalysisIssue& issue) {
  QTreeWidgetItem* item = new QTreeWidgetItem();

  // Severity icon
  QString icon = StaticAnalyzer::severityIcon(issue.severity);
  item->setText(0, icon);

  // Issue title and description
  item->setText(1, issue.title);
  item->setToolTip(1, issue.description);

  // Location (affected element)
  item->setText(2, issue.affectedElement);

  // Suggestion
  item->setText(3, issue.suggestion);
  item->setToolTip(3, issue.suggestion);

  // Color by severity
  QColor textColor;
  switch (issue.severity) {
    case IssueSeverity::Error:
      textColor = QColor("#c0392b");
      break;
    case IssueSeverity::Warning:
      textColor = QColor("#d35400");
      break;
    case IssueSeverity::Info:
      textColor = QColor("#2980b9");
      break;
    case IssueSeverity::Hint:
      textColor = QColor("#7f8c8d");
      break;
  }

  for (int col = 0; col < 4; ++col) {
    item->setForeground(col, textColor);
  }

  // Store issue ID for later retrieval
  item->setData(0, Qt::UserRole, issue.id);

  return item;
}

void IssueListPanel::updateSummary() {
  if (!currentResult_.analysisComplete) {
    summaryLabel_->setText(tr("Click 'Analyze' to run static analysis"));
    return;
  }

  int visibleCount = issueTree_->topLevelItemCount();
  int totalCount = currentResult_.issues.size();

  QString summary;
  if (visibleCount == totalCount) {
    summary = tr("%1 issues (%2 errors, %3 warnings)")
                  .arg(totalCount)
                  .arg(currentResult_.errorCount)
                  .arg(currentResult_.warningCount);
  } else {
    summary = tr("%1 of %2 issues shown")
                  .arg(visibleCount)
                  .arg(totalCount);
  }

  if (!currentResult_.analysisDuration.isEmpty()) {
    summary += tr(" - %1").arg(currentResult_.analysisDuration);
  }

  summaryLabel_->setText(summary);
}

bool IssueListPanel::matchesFilter(const AnalysisIssue& issue) const {
  // Check severity filter
  int severityFilter = severityFilter_->currentData().toInt();
  if (severityFilter >= 0 && static_cast<int>(issue.severity) != severityFilter) {
    return false;
  }

  // Check category filter
  int categoryFilter = categoryFilter_->currentData().toInt();
  if (categoryFilter >= 0) {
    // Group unused pub/sub together
    if (categoryFilter == static_cast<int>(IssueCategory::UnusedPublisher)) {
      if (issue.category != IssueCategory::UnusedPublisher &&
          issue.category != IssueCategory::UnusedSubscriber) {
        return false;
      }
    } else if (static_cast<int>(issue.category) != categoryFilter) {
      return false;
    }
  }

  // Check hints checkbox
  if (!showHintsCheck_->isChecked() && issue.severity == IssueSeverity::Hint) {
    return false;
  }

  return true;
}

}  // namespace ros_weaver
