#ifndef ROS_WEAVER_WIDGETS_ISSUE_LIST_PANEL_HPP
#define ROS_WEAVER_WIDGETS_ISSUE_LIST_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QProgressBar>

#include "ros_weaver/core/static_analyzer.hpp"

namespace ros_weaver {

class Project;

// Panel for displaying static analysis issues with fix suggestions
class IssueListPanel : public QWidget {
  Q_OBJECT

public:
  explicit IssueListPanel(QWidget* parent = nullptr);
  ~IssueListPanel() override = default;

  // Set the current analysis result
  void setAnalysisResult(const AnalysisResult& result);

  // Clear all issues
  void clear();

  // Get current issue count
  int issueCount() const { return issueTree_->topLevelItemCount(); }

public slots:
  // Run analysis on current project
  void runAnalysis();

  // Handle filter changes
  void onFilterChanged();

  // Handle issue selection
  void onIssueSelected(QTreeWidgetItem* item, int column);
  void onIssueDoubleClicked(QTreeWidgetItem* item, int column);

  // Fix actions
  void onAutoFixClicked();
  void onAskAIClicked();

  // Update progress
  void onAnalysisProgress(int percent, const QString& message);

signals:
  // Emitted when user wants to navigate to an issue
  void navigateToBlock(const QUuid& blockId);
  void navigateToConnection(const QUuid& connectionId);

  // Emitted when asking AI about an issue
  void askAIAboutIssue(const AnalysisIssue& issue);

  // Emitted when analysis is requested
  void analysisRequested();

private:
  void setupUi();
  void populateIssueTree(const AnalysisResult& result);
  QTreeWidgetItem* createIssueItem(const AnalysisIssue& issue);
  void updateSummary();
  bool matchesFilter(const AnalysisIssue& issue) const;

  // UI components
  QLabel* titleLabel_;
  QPushButton* analyzeButton_;
  QProgressBar* progressBar_;
  QComboBox* severityFilter_;
  QComboBox* categoryFilter_;
  QCheckBox* showHintsCheck_;
  QTreeWidget* issueTree_;
  QLabel* summaryLabel_;

  // Action buttons
  QPushButton* autoFixButton_;
  QPushButton* askAIButton_;
  QPushButton* refreshButton_;

  // State
  AnalysisResult currentResult_;
  AnalysisIssue selectedIssue_;
  bool hasSelection_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ISSUE_LIST_PANEL_HPP
