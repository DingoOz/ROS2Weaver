#ifndef ROS_WEAVER_WIDGETS_ADVANCED_SEARCH_PANEL_HPP
#define ROS_WEAVER_WIDGETS_ADVANCED_SEARCH_PANEL_HPP

#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QListWidget>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>

namespace ros_weaver {

// Search result item
struct SearchResult {
  QString name;
  QString packageName;
  QString category;
  QString description;
  QStringList messageTypes;
  QStringList features;
  double relevanceScore = 1.0;
};

// Advanced search panel for node library
class AdvancedSearchPanel : public QWidget {
  Q_OBJECT

public:
  explicit AdvancedSearchPanel(QWidget* parent = nullptr);
  ~AdvancedSearchPanel() override = default;

  // Get current search results
  QList<SearchResult> results() const { return results_; }

  // Set available packages for searching
  void setPackages(const QStringList& packages);

  // Clear search
  void clear();

signals:
  void searchChanged(const QString& query);
  void resultSelected(const SearchResult& result);
  void resultDoubleClicked(const SearchResult& result);
  void addToCanvasRequested(const SearchResult& result);

public slots:
  void performSearch();

private slots:
  void onSearchTextChanged(const QString& text);
  void onCategoryChanged(int index);
  void onMessageTypeChanged(int index);
  void onResultClicked(QListWidgetItem* item);
  void onResultDoubleClicked(QListWidgetItem* item);
  void onAddClicked();
  void updateFilters();

private:
  void setupUI();
  void populateCategories();
  void populateMessageTypes();
  void updateResults();
  bool matchesFilters(const SearchResult& result) const;
  void addResultToList(const SearchResult& result);

  // UI elements
  QLineEdit* searchEdit_;
  QComboBox* categoryCombo_;
  QComboBox* messageTypeCombo_;
  QCheckBox* hasInputsCheck_;
  QCheckBox* hasOutputsCheck_;
  QCheckBox* hasServicesCheck_;
  QCheckBox* hasActionsCheck_;
  QListWidget* resultsList_;
  QPushButton* addButton_;
  QLabel* statusLabel_;

  // Data
  QStringList allPackages_;
  QList<SearchResult> allResults_;
  QList<SearchResult> results_;

  // Search state
  QString currentQuery_;
  QString currentCategory_;
  QString currentMessageType_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ADVANCED_SEARCH_PANEL_HPP
