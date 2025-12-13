#ifndef ROS_WEAVER_HELP_BROWSER_HPP
#define ROS_WEAVER_HELP_BROWSER_HPP

#include <QDialog>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QLineEdit>
#include <QListWidget>
#include <QSplitter>
#include <QStackedWidget>
#include <QLabel>
#include <QMap>
#include <QPushButton>

namespace ros_weaver {

// Structure to hold help topic information
struct HelpTopic {
  QString id;
  QString title;
  QString content;
  QString parentId;
  QStringList keywords;
};

class HelpBrowser : public QDialog {
  Q_OBJECT

public:
  explicit HelpBrowser(QWidget* parent = nullptr);
  ~HelpBrowser() override;

  // Navigate to specific topic by ID
  void showTopic(const QString& topicId);

  // Search documentation
  void search(const QString& query);

  // Get singleton instance
  static HelpBrowser* instance(QWidget* parent = nullptr);

signals:
  void topicChanged(const QString& topicId);

private slots:
  void onTocItemClicked(QTreeWidgetItem* item, int column);
  void onSearchTextChanged(const QString& text);
  void onSearchResultClicked(QListWidgetItem* item);
  void onAnchorClicked(const QUrl& url);
  void onBackClicked();
  void onForwardClicked();
  void onHomeClicked();

private:
  void setupUi();
  void loadHelpContent();
  void buildTableOfContents();
  void addTopicToToc(const QString& topicId, QTreeWidgetItem* parentItem = nullptr);
  void updateNavigationButtons();
  QString renderMarkdownToHtml(const QString& markdown);
  QString getTopicHtml(const QString& topicId);

  // UI components
  QSplitter* mainSplitter_;
  QWidget* leftPanel_;
  QStackedWidget* leftStack_;
  QTreeWidget* tocTree_;
  QListWidget* searchResults_;
  QLineEdit* searchEdit_;
  QTextBrowser* contentBrowser_;
  QPushButton* backButton_;
  QPushButton* forwardButton_;
  QPushButton* homeButton_;
  QLabel* breadcrumbLabel_;

  // Help content storage
  QMap<QString, HelpTopic> topics_;
  QStringList topicOrder_;  // Maintains order for navigation

  // Navigation history
  QStringList history_;
  int historyIndex_;
  QString currentTopicId_;

  // Singleton instance
  static HelpBrowser* instance_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_HELP_BROWSER_HPP
