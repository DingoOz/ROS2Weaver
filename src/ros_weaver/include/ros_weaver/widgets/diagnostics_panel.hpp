// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#ifndef ROS_WEAVER_WIDGETS_DIAGNOSTICS_PANEL_HPP
#define ROS_WEAVER_WIDGETS_DIAGNOSTICS_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QProgressBar>
#include <QProcess>
#include <QSplitter>

namespace ros_weaver {

/**
 * @brief Status of a diagnostic check
 */
enum class DiagnosticStatus {
  Passed,
  Warning,
  Failed,
  Running,
  Unknown
};

/**
 * @brief Panel for running ros2 doctor diagnostics
 *
 * Executes `ros2 doctor --report` and displays categorized results
 * with pass/warning/fail status indicators.
 */
class DiagnosticsPanel : public QWidget {
  Q_OBJECT

public:
  explicit DiagnosticsPanel(QWidget* parent = nullptr);
  ~DiagnosticsPanel() override;

public slots:
  void runDiagnostics();
  void stopDiagnostics();
  void clearResults();

signals:
  void diagnosticsCompleted(bool allPassed);

private slots:
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onProcessError(QProcess::ProcessError error);
  void onItemClicked(QTreeWidgetItem* item, int column);

private:
  void setupUi();
  void setupConnections();

  // Output parsing
  void parseOutput(const QString& output);
  void parseSection(const QString& sectionName, const QStringList& lines, int& index);
  bool isPackageUpdateAvailable(const QString& value) const;

  // UI helpers
  QTreeWidgetItem* addCategoryItem(const QString& category, DiagnosticStatus status);
  void addCheckItem(QTreeWidgetItem* parent, const QString& name,
                    const QString& value, DiagnosticStatus status = DiagnosticStatus::Passed);
  void updateSummary();
  QColor getStatusColor(DiagnosticStatus status) const;
  QIcon getStatusIcon(DiagnosticStatus status) const;

  // UI Components - Toolbar
  QPushButton* runButton_ = nullptr;
  QPushButton* stopButton_ = nullptr;
  QPushButton* clearButton_ = nullptr;
  QCheckBox* excludePackagesCheck_ = nullptr;
  QProgressBar* progressBar_ = nullptr;
  QLabel* statusLabel_ = nullptr;

  // UI Components - Results
  QSplitter* splitter_ = nullptr;
  QTreeWidget* resultsTree_ = nullptr;
  QTextEdit* detailsView_ = nullptr;

  // UI Components - Summary
  QLabel* summaryLabel_ = nullptr;

  // Process management
  QProcess* doctorProcess_ = nullptr;
  bool isRunning_ = false;

  // Results storage
  int passedCount_ = 0;
  int warningCount_ = 0;
  int failedCount_ = 0;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_DIAGNOSTICS_PANEL_HPP
