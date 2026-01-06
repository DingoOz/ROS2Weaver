#ifndef ROS_WEAVER_WIDGETS_NODE_TEMPLATES_PANEL_HPP
#define ROS_WEAVER_WIDGETS_NODE_TEMPLATES_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QLabel>
#include <QTextBrowser>
#include <QPushButton>
#include <QSplitter>
#include "ros_weaver/core/node_templates.hpp"

namespace ros_weaver {

/**
 * @brief A dock panel for browsing and selecting node templates
 *
 * Features:
 * - Category-organized tree view of templates
 * - Search/filter functionality
 * - Template preview with details
 * - Drag-and-drop to canvas
 * - Import/export custom templates
 */
class NodeTemplatesPanel : public QWidget {
  Q_OBJECT

public:
  explicit NodeTemplatesPanel(QWidget* parent = nullptr);
  ~NodeTemplatesPanel() override = default;

signals:
  /**
   * @brief Emitted when user wants to add a template to the canvas
   */
  void templateSelected(const NodeTemplate& tmpl);

  /**
   * @brief Emitted for drag-and-drop template creation
   */
  void templateDragStarted(const NodeTemplate& tmpl);

public slots:
  /**
   * @brief Refresh the template list from the manager
   */
  void refreshTemplates();

  /**
   * @brief Set search/filter text
   */
  void setFilterText(const QString& text);

private slots:
  void onSearchTextChanged(const QString& text);
  void onTemplateClicked(QTreeWidgetItem* item, int column);
  void onTemplateDoubleClicked(QTreeWidgetItem* item, int column);
  void onAddButtonClicked();
  void onImportClicked();
  void onExportClicked();
  void onCreateCustomClicked();

private:
  void setupUi();
  void populateTemplates();
  void populateWithFilter(const QString& filter);
  void showTemplateDetails(const NodeTemplate& tmpl);
  void clearDetails();
  QTreeWidgetItem* findOrCreateCategoryItem(TemplateCategory category);

  // Widgets
  QLineEdit* searchEdit_ = nullptr;
  QTreeWidget* templateTree_ = nullptr;
  QTextBrowser* detailsBrowser_ = nullptr;
  QPushButton* addButton_ = nullptr;
  QPushButton* importButton_ = nullptr;
  QPushButton* exportButton_ = nullptr;
  QPushButton* createButton_ = nullptr;
  QLabel* previewLabel_ = nullptr;

  // State
  QUuid selectedTemplateId_;
  QMap<TemplateCategory, QTreeWidgetItem*> categoryItems_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_NODE_TEMPLATES_PANEL_HPP
