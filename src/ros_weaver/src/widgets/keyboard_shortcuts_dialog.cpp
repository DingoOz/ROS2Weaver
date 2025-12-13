#include "ros_weaver/widgets/keyboard_shortcuts_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QHeaderView>
#include <QApplication>
#include <QStyle>

namespace ros_weaver {

KeyboardShortcutsDialog::KeyboardShortcutsDialog(QWidget* parent)
  : QDialog(parent)
  , searchEdit_(nullptr)
  , shortcutsTree_(nullptr)
  , countLabel_(nullptr)
{
  setWindowTitle(tr("Keyboard Shortcuts"));
  setMinimumSize(550, 500);
  resize(600, 550);

  setupUi();
  loadShortcuts();
  filterShortcuts("");
}

KeyboardShortcutsDialog::~KeyboardShortcutsDialog() = default;

void KeyboardShortcutsDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(16, 16, 16, 16);
  mainLayout->setSpacing(12);

  // Title
  QLabel* titleLabel = new QLabel(tr("Keyboard Shortcuts Reference"));
  titleLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: #2c3e50;");
  mainLayout->addWidget(titleLabel);

  // Search box
  QHBoxLayout* searchLayout = new QHBoxLayout();
  QLabel* searchIcon = new QLabel();
  searchIcon->setPixmap(QApplication::style()->standardIcon(QStyle::SP_FileDialogContentsView).pixmap(16, 16));
  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search shortcuts..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged, this, &KeyboardShortcutsDialog::onSearchTextChanged);

  searchLayout->addWidget(searchIcon);
  searchLayout->addWidget(searchEdit_);
  mainLayout->addLayout(searchLayout);

  // Shortcuts tree
  shortcutsTree_ = new QTreeWidget();
  shortcutsTree_->setHeaderLabels({tr("Action"), tr("Shortcut")});
  shortcutsTree_->setRootIsDecorated(true);
  shortcutsTree_->setAlternatingRowColors(true);
  shortcutsTree_->setIndentation(20);
  shortcutsTree_->header()->setStretchLastSection(false);
  shortcutsTree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  shortcutsTree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  shortcutsTree_->setAnimated(true);

  // Style the tree
  shortcutsTree_->setStyleSheet(R"(
    QTreeWidget {
      border: 1px solid #ddd;
      border-radius: 4px;
    }
    QTreeWidget::item {
      padding: 4px;
    }
    QTreeWidget::item:selected {
      background-color: #3498db;
      color: white;
    }
  )");

  mainLayout->addWidget(shortcutsTree_, 1);

  // Count label
  countLabel_ = new QLabel();
  countLabel_->setStyleSheet("color: #666; font-size: 11px;");
  mainLayout->addWidget(countLabel_);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();

  QPushButton* closeButton = new QPushButton(tr("Close"));
  closeButton->setDefault(true);
  connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton);

  mainLayout->addLayout(buttonLayout);
}

void KeyboardShortcutsDialog::loadShortcuts() {
  shortcuts_.clear();

  // File Operations
  shortcuts_ << ShortcutInfo{"File", tr("New Project"), "Ctrl+N", tr("Create a new project")};
  shortcuts_ << ShortcutInfo{"File", tr("Open Project"), "Ctrl+O", tr("Open an existing project")};
  shortcuts_ << ShortcutInfo{"File", tr("Save Project"), "Ctrl+S", tr("Save the current project")};
  shortcuts_ << ShortcutInfo{"File", tr("Save Project As"), "Ctrl+Shift+S", tr("Save project to a new location")};
  shortcuts_ << ShortcutInfo{"File", tr("Generate ROS2 Package"), "Ctrl+G", tr("Generate code from project")};
  shortcuts_ << ShortcutInfo{"File", tr("Generate (Wizard)"), "Ctrl+Shift+G", tr("Step-by-step code generation")};
  shortcuts_ << ShortcutInfo{"File", tr("Exit"), "Ctrl+Q", tr("Exit the application")};

  // Edit Operations
  shortcuts_ << ShortcutInfo{"Edit", tr("Undo"), "Ctrl+Z", tr("Undo last action")};
  shortcuts_ << ShortcutInfo{"Edit", tr("Redo"), "Ctrl+Y", tr("Redo last undone action")};
  shortcuts_ << ShortcutInfo{"Edit", tr("Delete"), "Delete", tr("Delete selected items")};
  shortcuts_ << ShortcutInfo{"Edit", tr("Select All"), "Ctrl+A", tr("Select all items on canvas")};
  shortcuts_ << ShortcutInfo{"Edit", tr("Settings"), "Ctrl+,", tr("Open application settings")};

  // View Operations
  shortcuts_ << ShortcutInfo{"View", tr("Zoom In"), "Ctrl++", tr("Zoom in on canvas")};
  shortcuts_ << ShortcutInfo{"View", tr("Zoom Out"), "Ctrl+-", tr("Zoom out on canvas")};
  shortcuts_ << ShortcutInfo{"View", tr("Reset Zoom"), "Ctrl+0", tr("Reset zoom to default")};
  shortcuts_ << ShortcutInfo{"View", tr("Fit All Nodes"), "F", tr("Fit all nodes in view")};

  // ROS2 Operations
  shortcuts_ << ShortcutInfo{"ROS2", tr("Build Workspace"), "Ctrl+B", tr("Build the ROS2 workspace")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("Launch"), "Ctrl+L", tr("Launch ROS2 nodes")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("Scan Running System"), "Ctrl+Shift+R", tr("Scan for running ROS2 nodes")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("System Mapping Panel"), "Ctrl+Shift+M", tr("Show system mapping panel")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("Topic Viewer"), "Ctrl+Shift+T", tr("Show topic viewer panel")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("TF Tree"), "Ctrl+T", tr("Show TF tree panel")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("ROS Logs"), "Ctrl+Shift+L", tr("Show ROS log viewer")};
  shortcuts_ << ShortcutInfo{"ROS2", tr("Launch RViz2"), "Ctrl+Shift+V", tr("Launch RViz2 visualization")};

  // External Tools
  shortcuts_ << ShortcutInfo{"Tools", tr("Open in VS Code"), "Ctrl+Shift+E", tr("Open project in VS Code")};

  // Help
  shortcuts_ << ShortcutInfo{"Help", tr("Context Help"), "F1", tr("Show help for current widget")};
  shortcuts_ << ShortcutInfo{"Help", tr("Getting Started"), "Ctrl+F1", tr("Open Getting Started guide")};
  shortcuts_ << ShortcutInfo{"Help", tr("Keyboard Shortcuts"), "Ctrl+/", tr("Show this shortcuts reference")};

  // Canvas Navigation
  shortcuts_ << ShortcutInfo{"Canvas", tr("Pan Canvas"), tr("Middle-click + Drag"), tr("Pan the canvas view")};
  shortcuts_ << ShortcutInfo{"Canvas", tr("Pan Canvas (Alt)"), tr("Space + Drag"), tr("Alternative pan method")};
  shortcuts_ << ShortcutInfo{"Canvas", tr("Zoom Canvas"), tr("Mouse Wheel"), tr("Zoom in/out on canvas")};
  shortcuts_ << ShortcutInfo{"Canvas", tr("Select Item"), tr("Click"), tr("Select a single item")};
  shortcuts_ << ShortcutInfo{"Canvas", tr("Multi-Select"), tr("Ctrl + Click"), tr("Add/remove from selection")};
  shortcuts_ << ShortcutInfo{"Canvas", tr("Deselect All"), tr("Click Empty"), tr("Click empty area to deselect")};
}

void KeyboardShortcutsDialog::filterShortcuts(const QString& filter) {
  shortcutsTree_->clear();

  QString lowerFilter = filter.toLower();
  QMap<QString, QTreeWidgetItem*> categoryItems;
  int visibleCount = 0;

  for (const ShortcutInfo& shortcut : shortcuts_) {
    // Apply filter
    if (!filter.isEmpty()) {
      bool match = shortcut.action.toLower().contains(lowerFilter) ||
                   shortcut.shortcut.toLower().contains(lowerFilter) ||
                   shortcut.category.toLower().contains(lowerFilter) ||
                   shortcut.description.toLower().contains(lowerFilter);
      if (!match) {
        continue;
      }
    }

    // Get or create category item
    QTreeWidgetItem* categoryItem = categoryItems.value(shortcut.category, nullptr);
    if (!categoryItem) {
      categoryItem = new QTreeWidgetItem(shortcutsTree_);
      categoryItem->setText(0, shortcut.category);
      categoryItem->setExpanded(true);

      // Style category headers
      QFont font = categoryItem->font(0);
      font.setBold(true);
      categoryItem->setFont(0, font);
      categoryItem->setForeground(0, QColor("#2c3e50"));

      categoryItems[shortcut.category] = categoryItem;
    }

    // Add shortcut item
    QTreeWidgetItem* item = new QTreeWidgetItem(categoryItem);
    item->setText(0, shortcut.action);
    item->setText(1, shortcut.shortcut);
    item->setToolTip(0, shortcut.description);
    item->setToolTip(1, shortcut.description);

    // Style shortcut column
    QFont shortcutFont = item->font(1);
    shortcutFont.setFamily("Monaco, Menlo, Consolas, monospace");
    item->setFont(1, shortcutFont);
    item->setForeground(1, QColor("#3498db"));

    visibleCount++;
  }

  // Update count label
  if (filter.isEmpty()) {
    countLabel_->setText(tr("%1 shortcuts").arg(visibleCount));
  } else {
    countLabel_->setText(tr("%1 shortcuts found").arg(visibleCount));
  }

  // Expand all
  shortcutsTree_->expandAll();
}

void KeyboardShortcutsDialog::onSearchTextChanged(const QString& text) {
  filterShortcuts(text);
}

}  // namespace ros_weaver
