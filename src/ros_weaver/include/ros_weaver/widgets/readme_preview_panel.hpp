#ifndef ROS_WEAVER_WIDGETS_README_PREVIEW_PANEL_HPP
#define ROS_WEAVER_WIDGETS_README_PREVIEW_PANEL_HPP

#include <QWidget>
#include <QTextBrowser>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>

namespace ros_weaver {

// Panel for displaying README.md and package.xml descriptions
class ReadmePreviewPanel : public QWidget {
  Q_OBJECT

public:
  explicit ReadmePreviewPanel(QWidget* parent = nullptr);
  ~ReadmePreviewPanel() override = default;

  // Set the package to display
  void setPackage(const QString& packageName);

  // Clear the display
  void clear();

  // Get current package
  QString currentPackage() const { return currentPackage_; }

public slots:
  // Handle view mode change
  void onViewModeChanged(int index);

  // Open in external browser/editor
  void onOpenExternalClicked();

  // Refresh content
  void onRefreshClicked();

signals:
  // Emitted when a link is clicked
  void linkClicked(const QString& url);

private:
  void setupUi();
  void loadPackageInfo();
  void loadReadme();
  QString markdownToHtml(const QString& markdown) const;

  // UI components
  QLabel* titleLabel_;
  QComboBox* viewModeCombo_;
  QPushButton* openExternalButton_;
  QPushButton* refreshButton_;
  QTextBrowser* contentBrowser_;
  QLabel* statusLabel_;

  // State
  QString currentPackage_;
  QString readmePath_;
  QString packageXmlPath_;

  enum class ViewMode { Readme, PackageXml, Both };
  ViewMode currentViewMode_ = ViewMode::Readme;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_README_PREVIEW_PANEL_HPP
