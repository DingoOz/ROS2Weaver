// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/widgets/diagnostics_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QApplication>
#include <QStyle>
#include <QRegularExpression>

namespace ros_weaver {

DiagnosticsPanel::DiagnosticsPanel(QWidget* parent)
    : QWidget(parent)
    , doctorProcess_(new QProcess(this)) {
  setupUi();
  setupConnections();
}

DiagnosticsPanel::~DiagnosticsPanel() {
  if (doctorProcess_ && doctorProcess_->state() != QProcess::NotRunning) {
    doctorProcess_->kill();
    doctorProcess_->waitForFinished(1000);
  }
}

void DiagnosticsPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Toolbar
  auto* toolbarLayout = new QHBoxLayout();
  toolbarLayout->setSpacing(8);

  runButton_ = new QPushButton(tr("Run Diagnostics"), this);
  runButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  toolbarLayout->addWidget(runButton_);

  stopButton_ = new QPushButton(tr("Stop"), this);
  stopButton_->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
  stopButton_->setEnabled(false);
  toolbarLayout->addWidget(stopButton_);

  clearButton_ = new QPushButton(tr("Clear"), this);
  clearButton_->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
  toolbarLayout->addWidget(clearButton_);

  toolbarLayout->addSpacing(16);

  excludePackagesCheck_ = new QCheckBox(tr("Exclude Package Checks"), this);
  excludePackagesCheck_->setToolTip(tr("Skip package version checks (faster)"));
  toolbarLayout->addWidget(excludePackagesCheck_);

  toolbarLayout->addStretch();
  mainLayout->addLayout(toolbarLayout);

  // Progress bar and status
  auto* progressLayout = new QHBoxLayout();
  progressBar_ = new QProgressBar(this);
  progressBar_->setRange(0, 0);  // Indeterminate
  progressBar_->setVisible(false);
  progressBar_->setMaximumHeight(16);
  progressLayout->addWidget(progressBar_, 1);

  statusLabel_ = new QLabel(tr("Ready"), this);
  statusLabel_->setStyleSheet("color: #888;");
  progressLayout->addWidget(statusLabel_);
  mainLayout->addLayout(progressLayout);

  // Summary
  summaryLabel_ = new QLabel(this);
  summaryLabel_->setStyleSheet("font-weight: bold; padding: 4px;");
  updateSummary();
  mainLayout->addWidget(summaryLabel_);

  // Splitter with results tree and details view
  splitter_ = new QSplitter(Qt::Horizontal, this);

  resultsTree_ = new QTreeWidget(this);
  resultsTree_->setHeaderLabels({tr("Status"), tr("Check"), tr("Value")});
  resultsTree_->setColumnWidth(0, 60);
  resultsTree_->setColumnWidth(1, 200);
  resultsTree_->setAlternatingRowColors(true);
  resultsTree_->setRootIsDecorated(true);
  resultsTree_->header()->setStretchLastSection(true);
  splitter_->addWidget(resultsTree_);

  detailsView_ = new QTextEdit(this);
  detailsView_->setReadOnly(true);
  detailsView_->setPlaceholderText(tr("Select an item to view details..."));
  splitter_->addWidget(detailsView_);

  splitter_->setSizes({400, 200});
  mainLayout->addWidget(splitter_, 1);
}

void DiagnosticsPanel::setupConnections() {
  connect(runButton_, &QPushButton::clicked, this, &DiagnosticsPanel::runDiagnostics);
  connect(stopButton_, &QPushButton::clicked, this, &DiagnosticsPanel::stopDiagnostics);
  connect(clearButton_, &QPushButton::clicked, this, &DiagnosticsPanel::clearResults);

  connect(doctorProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &DiagnosticsPanel::onProcessFinished);
  connect(doctorProcess_, &QProcess::errorOccurred,
          this, &DiagnosticsPanel::onProcessError);

  connect(resultsTree_, &QTreeWidget::itemClicked,
          this, &DiagnosticsPanel::onItemClicked);
}

void DiagnosticsPanel::runDiagnostics() {
  if (isRunning_) {
    return;
  }

  clearResults();
  isRunning_ = true;

  runButton_->setEnabled(false);
  stopButton_->setEnabled(true);
  progressBar_->setVisible(true);
  statusLabel_->setText(tr("Running ros2 doctor..."));

  QStringList args{"doctor", "--report"};
  if (excludePackagesCheck_->isChecked()) {
    args << "--exclude-packages";
  }

  doctorProcess_->start("ros2", args);

  if (!doctorProcess_->waitForStarted(5000)) {
    onProcessError(QProcess::FailedToStart);
  }
}

void DiagnosticsPanel::stopDiagnostics() {
  if (doctorProcess_ && doctorProcess_->state() != QProcess::NotRunning) {
    doctorProcess_->kill();
    statusLabel_->setText(tr("Stopped"));
  }
  isRunning_ = false;
  runButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  progressBar_->setVisible(false);
}

void DiagnosticsPanel::clearResults() {
  resultsTree_->clear();
  detailsView_->clear();
  passedCount_ = 0;
  warningCount_ = 0;
  failedCount_ = 0;
  updateSummary();
  statusLabel_->setText(tr("Ready"));
}

void DiagnosticsPanel::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  Q_UNUSED(exitCode);

  isRunning_ = false;
  runButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  progressBar_->setVisible(false);

  if (exitStatus == QProcess::CrashExit) {
    statusLabel_->setText(tr("Process crashed"));
    return;
  }

  QString output = doctorProcess_->readAllStandardOutput();
  parseOutput(output);
  updateSummary();

  bool allPassed = (failedCount_ == 0 && warningCount_ == 0);
  if (allPassed) {
    statusLabel_->setText(tr("All checks passed"));
  } else if (failedCount_ > 0) {
    statusLabel_->setText(tr("Issues found - see results"));
  } else {
    statusLabel_->setText(tr("Warnings found - see results"));
  }

  emit diagnosticsCompleted(allPassed);
}

void DiagnosticsPanel::onProcessError(QProcess::ProcessError error) {
  isRunning_ = false;
  runButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  progressBar_->setVisible(false);

  QString errorMsg;
  switch (error) {
    case QProcess::FailedToStart:
      errorMsg = tr("Failed to start ros2 doctor. Is ROS2 sourced?");
      break;
    case QProcess::Crashed:
      errorMsg = tr("ros2 doctor crashed");
      break;
    case QProcess::Timedout:
      errorMsg = tr("ros2 doctor timed out");
      break;
    default:
      errorMsg = tr("Unknown error running ros2 doctor");
  }

  statusLabel_->setText(errorMsg);
  detailsView_->setPlainText(errorMsg);
}

void DiagnosticsPanel::onItemClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column);

  if (!item) {
    return;
  }

  QString category = item->data(0, Qt::UserRole).toString();
  QString name = item->text(1);
  QString value = item->text(2);

  QString details = QString("<h3>%1</h3>").arg(name);

  if (item->childCount() > 0) {
    details += QString("<p>%1 items</p>").arg(item->childCount());
  } else {
    details += QString("<p><b>Value:</b> %1</p>").arg(value);
    if (!category.isEmpty()) {
      details += QString("<p><b>Category:</b> %1</p>").arg(category);
    }

    if (isPackageUpdateAvailable(value)) {
      QRegularExpression re("latest=([^,]+), local=(.+)");
      auto match = re.match(value);
      details += QString("<p style='color: orange;'><b>Update available:</b> %1 -> %2</p>")
                     .arg(match.captured(2), match.captured(1));
    }
  }

  detailsView_->setHtml(details);
}

void DiagnosticsPanel::parseOutput(const QString& output) {
  QStringList lines = output.split('\n');
  int index = 0;

  while (index < lines.size()) {
    QString line = lines[index];
    QString trimmed = line.trimmed();

    // Detect section headers (3 leading spaces + ALL CAPS)
    if (line.startsWith("   ") && !trimmed.isEmpty() &&
        trimmed == trimmed.toUpper() && trimmed.length() > 3) {
      QString currentSection = trimmed;
      index++;
      parseSection(currentSection, lines, index);
    } else {
      index++;
    }
  }
}

void DiagnosticsPanel::parseSection(const QString& sectionName, const QStringList& lines, int& index) {
  DiagnosticStatus sectionStatus = DiagnosticStatus::Passed;
  QList<QPair<QString, QString>> items;

  // Parse until next section or end
  while (index < lines.size()) {
    QString line = lines[index];
    QString trimmed = line.trimmed();

    // Check for next section header
    if (line.startsWith("   ") && !trimmed.isEmpty() &&
        trimmed == trimmed.toUpper() && trimmed.length() > 3) {
      break;  // Don't increment, let outer loop handle it
    }

    // Skip empty lines
    if (trimmed.isEmpty()) {
      index++;
      continue;
    }

    // Parse key-value pairs
    int colonPos = trimmed.indexOf(':');
    if (colonPos > 0) {
      QString key = trimmed.left(colonPos).trimmed();
      QString value = trimmed.mid(colonPos + 1).trimmed();

      if (isPackageUpdateAvailable(value)) {
        sectionStatus = DiagnosticStatus::Warning;
        warningCount_++;
      } else {
        passedCount_++;
      }

      items.append({key, value});
    }

    index++;
  }

  // Add section to tree if it has items
  if (!items.isEmpty()) {
    QTreeWidgetItem* categoryItem = addCategoryItem(sectionName, sectionStatus);
    for (const auto& item : items) {
      DiagnosticStatus itemStatus = isPackageUpdateAvailable(item.second)
                                        ? DiagnosticStatus::Warning
                                        : DiagnosticStatus::Passed;
      addCheckItem(categoryItem, item.first, item.second, itemStatus);
    }
    categoryItem->setExpanded(sectionStatus != DiagnosticStatus::Passed);
  }
}

bool DiagnosticsPanel::isPackageUpdateAvailable(const QString& value) const {
  if (!value.contains("latest=") || !value.contains("local=")) {
    return false;
  }

  QRegularExpression re("latest=([^,]+), local=(.+)");
  auto match = re.match(value);
  return match.hasMatch() && match.captured(1) != match.captured(2);
}

QTreeWidgetItem* DiagnosticsPanel::addCategoryItem(const QString& category, DiagnosticStatus status) {
  auto* item = new QTreeWidgetItem(resultsTree_);
  item->setIcon(0, getStatusIcon(status));
  item->setText(1, category);
  item->setData(0, Qt::UserRole, category);

  QFont font = item->font(1);
  font.setBold(true);
  item->setFont(1, font);

  return item;
}

void DiagnosticsPanel::addCheckItem(QTreeWidgetItem* parent, const QString& name,
                                     const QString& value, DiagnosticStatus status) {
  auto* item = new QTreeWidgetItem(parent);
  item->setIcon(0, getStatusIcon(status));
  item->setText(1, name);
  item->setText(2, value);
  item->setData(0, Qt::UserRole, parent->text(1));

  if (status == DiagnosticStatus::Warning) {
    item->setForeground(2, QColor(255, 165, 0));  // Orange
  } else if (status == DiagnosticStatus::Failed) {
    item->setForeground(2, QColor(255, 80, 80));  // Red
  }
}

void DiagnosticsPanel::updateSummary() {
  QString summary = QString("Passed: <span style='color: green;'>%1</span> | "
                            "Warnings: <span style='color: orange;'>%2</span> | "
                            "Failed: <span style='color: red;'>%3</span>")
                        .arg(passedCount_)
                        .arg(warningCount_)
                        .arg(failedCount_);
  summaryLabel_->setText(summary);
  summaryLabel_->setTextFormat(Qt::RichText);
}

QColor DiagnosticsPanel::getStatusColor(DiagnosticStatus status) const {
  switch (status) {
    case DiagnosticStatus::Passed:
      return QColor(100, 200, 100);
    case DiagnosticStatus::Warning:
      return QColor(255, 165, 0);
    case DiagnosticStatus::Failed:
      return QColor(255, 80, 80);
    case DiagnosticStatus::Running:
      return QColor(100, 150, 255);
    default:
      return QColor(150, 150, 150);
  }
}

QIcon DiagnosticsPanel::getStatusIcon(DiagnosticStatus status) const {
  QStyle* style = QApplication::style();
  switch (status) {
    case DiagnosticStatus::Passed:
      return style->standardIcon(QStyle::SP_DialogApplyButton);
    case DiagnosticStatus::Warning:
      return style->standardIcon(QStyle::SP_MessageBoxWarning);
    case DiagnosticStatus::Failed:
      return style->standardIcon(QStyle::SP_DialogCancelButton);
    case DiagnosticStatus::Running:
      return style->standardIcon(QStyle::SP_BrowserReload);
    default:
      return style->standardIcon(QStyle::SP_MessageBoxQuestion);
  }
}

}  // namespace ros_weaver
