#ifndef ROS_WEAVER_WIDGETS_SCHEMA_VIEWER_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_SCHEMA_VIEWER_WIDGET_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QTextBrowser>
#include <QLabel>
#include <QStackedWidget>
#include <QComboBox>
#include <QPushButton>
#include <QSplitter>

#include "ros_weaver/core/ros_docs_provider.hpp"

namespace ros_weaver {

// Widget that displays ROS2 interface schemas in an expandable tree view
class SchemaViewerWidget : public QWidget {
  Q_OBJECT

public:
  explicit SchemaViewerWidget(QWidget* parent = nullptr);
  ~SchemaViewerWidget() override = default;

  // Set the interface to display (e.g., "std_msgs/msg/String")
  void setInterface(const QString& fullName);

  // Clear the current display
  void clear();

  // Get current interface name
  QString currentInterface() const { return currentInterface_; }

public slots:
  // Search/filter the interface list
  void onSearchTextChanged(const QString& text);

  // Handle interface type filter change
  void onTypeFilterChanged(int index);

  // Handle interface selection
  void onInterfaceSelected(QTreeWidgetItem* item, int column);

  // Refresh interface list
  void onRefreshClicked();

  // Copy schema to clipboard
  void onCopyClicked();

signals:
  // Emitted when an interface is selected
  void interfaceSelected(const QString& fullName);

private:
  void setupUi();
  void populateInterfaceList();
  void displaySchema(const InterfaceDoc& doc);
  void buildSchemaTree(const QList<FieldInfo>& fields);
  QTreeWidgetItem* createFieldItem(const FieldInfo& field);

  // Filter interfaces based on search and type filter
  void filterInterfaces();

  // UI components
  QLineEdit* searchEdit_;
  QComboBox* typeFilter_;
  QPushButton* refreshButton_;
  QTreeWidget* interfaceList_;
  QSplitter* splitter_;

  // Schema display
  QTreeWidget* schemaTree_;
  QTextBrowser* rawDefinition_;
  QStackedWidget* displayStack_;
  QPushButton* copyButton_;
  QLabel* statusLabel_;

  // State
  QString currentInterface_;
  QStringList allMessages_;
  QStringList allServices_;
  QStringList allActions_;

  // Interface type filter values
  enum class TypeFilter { All, Messages, Services, Actions };
  TypeFilter currentTypeFilter_ = TypeFilter::All;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_SCHEMA_VIEWER_WIDGET_HPP
