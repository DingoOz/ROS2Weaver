#include "ros_weaver/widgets/node_templates_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QMimeData>
#include <QDrag>
#include <QApplication>

namespace ros_weaver {

NodeTemplatesPanel::NodeTemplatesPanel(QWidget* parent)
  : QWidget(parent)
{
  setupUi();
  populateTemplates();

  // Connect to template manager signals
  connect(&NodeTemplatesManager::instance(), &NodeTemplatesManager::templatesChanged,
          this, &NodeTemplatesPanel::refreshTemplates);
}

void NodeTemplatesPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Search bar
  searchEdit_ = new QLineEdit(this);
  searchEdit_->setPlaceholderText(tr("Search templates..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &NodeTemplatesPanel::onSearchTextChanged);
  mainLayout->addWidget(searchEdit_);

  // Splitter for tree and details
  QSplitter* splitter = new QSplitter(Qt::Vertical, this);

  // Template tree
  templateTree_ = new QTreeWidget(this);
  templateTree_->setHeaderHidden(true);
  templateTree_->setRootIsDecorated(true);
  templateTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  templateTree_->setDragEnabled(true);
  templateTree_->setAnimated(true);
  templateTree_->setIndentation(16);
  templateTree_->setStyleSheet(R"(
    QTreeWidget {
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    QTreeWidget::item {
      padding: 4px 2px;
    }
    QTreeWidget::item:hover {
      background-color: #e8f0fe;
    }
    QTreeWidget::item:selected {
      background-color: #1a73e8;
      color: white;
    }
  )");

  connect(templateTree_, &QTreeWidget::itemClicked,
          this, &NodeTemplatesPanel::onTemplateClicked);
  connect(templateTree_, &QTreeWidget::itemDoubleClicked,
          this, &NodeTemplatesPanel::onTemplateDoubleClicked);

  splitter->addWidget(templateTree_);

  // Details panel
  QWidget* detailsWidget = new QWidget(this);
  QVBoxLayout* detailsLayout = new QVBoxLayout(detailsWidget);
  detailsLayout->setContentsMargins(0, 0, 0, 0);
  detailsLayout->setSpacing(4);

  previewLabel_ = new QLabel(tr("Select a template to see details"), this);
  previewLabel_->setStyleSheet("color: #666; font-style: italic;");
  previewLabel_->setAlignment(Qt::AlignCenter);
  detailsLayout->addWidget(previewLabel_);

  detailsBrowser_ = new QTextBrowser(this);
  detailsBrowser_->setOpenExternalLinks(false);
  detailsBrowser_->setStyleSheet(R"(
    QTextBrowser {
      border: 1px solid #ccc;
      border-radius: 4px;
      background-color: #fafafa;
    }
  )");
  detailsBrowser_->hide();
  detailsLayout->addWidget(detailsBrowser_);

  splitter->addWidget(detailsWidget);
  splitter->setStretchFactor(0, 2);
  splitter->setStretchFactor(1, 1);

  mainLayout->addWidget(splitter, 1);

  // Button row
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->setSpacing(4);

  addButton_ = new QPushButton(tr("Add to Canvas"), this);
  addButton_->setEnabled(false);
  addButton_->setStyleSheet(R"(
    QPushButton {
      background-color: #1a73e8;
      color: white;
      border: none;
      border-radius: 4px;
      padding: 6px 12px;
      font-weight: bold;
    }
    QPushButton:hover {
      background-color: #1557b0;
    }
    QPushButton:disabled {
      background-color: #ccc;
    }
  )");
  connect(addButton_, &QPushButton::clicked,
          this, &NodeTemplatesPanel::onAddButtonClicked);
  buttonLayout->addWidget(addButton_);

  buttonLayout->addStretch();

  createButton_ = new QPushButton(tr("+"), this);
  createButton_->setToolTip(tr("Create custom template"));
  createButton_->setFixedWidth(30);
  connect(createButton_, &QPushButton::clicked,
          this, &NodeTemplatesPanel::onCreateCustomClicked);
  buttonLayout->addWidget(createButton_);

  importButton_ = new QPushButton(tr("Import"), this);
  importButton_->setToolTip(tr("Import templates from file"));
  connect(importButton_, &QPushButton::clicked,
          this, &NodeTemplatesPanel::onImportClicked);
  buttonLayout->addWidget(importButton_);

  exportButton_ = new QPushButton(tr("Export"), this);
  exportButton_->setToolTip(tr("Export custom templates to file"));
  connect(exportButton_, &QPushButton::clicked,
          this, &NodeTemplatesPanel::onExportClicked);
  buttonLayout->addWidget(exportButton_);

  mainLayout->addLayout(buttonLayout);

  setMinimumWidth(250);
}

void NodeTemplatesPanel::populateTemplates() {
  populateWithFilter(QString());
}

void NodeTemplatesPanel::populateWithFilter(const QString& filter) {
  templateTree_->clear();
  categoryItems_.clear();

  auto& manager = NodeTemplatesManager::instance();
  QList<NodeTemplate> templates;

  if (filter.isEmpty()) {
    templates = manager.allTemplates();
  } else {
    templates = manager.searchTemplates(filter);
  }

  for (const auto& tmpl : templates) {
    QTreeWidgetItem* categoryItem = findOrCreateCategoryItem(tmpl.category);

    QTreeWidgetItem* templateItem = new QTreeWidgetItem(categoryItem);
    templateItem->setText(0, tmpl.displayName);
    templateItem->setToolTip(0, tmpl.description);
    templateItem->setData(0, Qt::UserRole, tmpl.id.toString());

    // Set icon based on category
    QString iconText = NodeTemplate::categoryToIcon(tmpl.category);
    templateItem->setText(0, QString("%1  %2").arg(iconText).arg(tmpl.displayName));
  }

  // Expand all categories
  templateTree_->expandAll();
}

QTreeWidgetItem* NodeTemplatesPanel::findOrCreateCategoryItem(TemplateCategory category) {
  if (categoryItems_.contains(category)) {
    return categoryItems_[category];
  }

  QTreeWidgetItem* item = new QTreeWidgetItem(templateTree_);
  QString categoryName = NodeTemplate::categoryToString(category);
  QString categoryIcon = NodeTemplate::categoryToIcon(category);
  item->setText(0, QString("%1  %2").arg(categoryIcon).arg(categoryName));
  item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
  item->setExpanded(true);

  QFont font = item->font(0);
  font.setBold(true);
  item->setFont(0, font);

  categoryItems_[category] = item;
  return item;
}

void NodeTemplatesPanel::refreshTemplates() {
  QString currentFilter = searchEdit_->text();
  populateWithFilter(currentFilter);
}

void NodeTemplatesPanel::setFilterText(const QString& text) {
  searchEdit_->setText(text);
}

void NodeTemplatesPanel::onSearchTextChanged(const QString& text) {
  populateWithFilter(text);
}

void NodeTemplatesPanel::onTemplateClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString idStr = item->data(0, Qt::UserRole).toString();
  if (idStr.isEmpty()) {
    // Category item clicked
    clearDetails();
    return;
  }

  QUuid id(idStr);
  auto* tmpl = NodeTemplatesManager::instance().findTemplate(id);
  if (tmpl) {
    selectedTemplateId_ = id;
    showTemplateDetails(*tmpl);
    addButton_->setEnabled(true);
  }
}

void NodeTemplatesPanel::onTemplateDoubleClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString idStr = item->data(0, Qt::UserRole).toString();
  if (idStr.isEmpty()) {
    return;
  }

  QUuid id(idStr);
  auto* tmpl = NodeTemplatesManager::instance().findTemplate(id);
  if (tmpl) {
    emit templateSelected(*tmpl);
  }
}

void NodeTemplatesPanel::onAddButtonClicked() {
  if (selectedTemplateId_.isNull()) return;

  auto* tmpl = NodeTemplatesManager::instance().findTemplate(selectedTemplateId_);
  if (tmpl) {
    emit templateSelected(*tmpl);
  }
}

void NodeTemplatesPanel::showTemplateDetails(const NodeTemplate& tmpl) {
  previewLabel_->hide();
  detailsBrowser_->show();

  QString html = QString(R"(
    <html>
    <head>
    <style>
      body { font-family: sans-serif; font-size: 11px; margin: 8px; }
      h2 { color: #1a73e8; margin: 0 0 8px 0; font-size: 14px; }
      h3 { color: #444; margin: 12px 0 4px 0; font-size: 12px; }
      p { margin: 4px 0; color: #333; }
      .tag { background-color: #e8f0fe; color: #1a73e8; padding: 2px 6px;
             border-radius: 10px; margin-right: 4px; font-size: 10px; }
      .pin { background-color: #f0f0f0; padding: 2px 6px; margin: 2px 0;
             border-radius: 3px; display: block; }
      .input { border-left: 3px solid #4caf50; }
      .output { border-left: 3px solid #2196f3; }
      .param { background-color: #fff3e0; padding: 2px 6px; margin: 2px 0;
               border-radius: 3px; display: block; border-left: 3px solid #ff9800; }
      table { width: 100%; border-collapse: collapse; }
      td { padding: 2px 4px; vertical-align: top; }
      .label { color: #666; width: 80px; }
    </style>
    </head>
    <body>
    <h2>%1</h2>
    <p>%2</p>
  )").arg(tmpl.displayName).arg(tmpl.description);

  // Tags
  if (!tmpl.tags.isEmpty()) {
    html += "<p>";
    for (const auto& tag : tmpl.tags) {
      html += QString("<span class='tag'>%1</span>").arg(tag);
    }
    html += "</p>";
  }

  // Input pins
  if (!tmpl.inputPins.isEmpty()) {
    html += "<h3>Inputs</h3>";
    for (const auto& pin : tmpl.inputPins) {
      html += QString("<div class='pin input'><b>%1</b> (%2)<br/>%3</div>")
        .arg(pin.name).arg(pin.messageType).arg(pin.description);
    }
  }

  // Output pins
  if (!tmpl.outputPins.isEmpty()) {
    html += "<h3>Outputs</h3>";
    for (const auto& pin : tmpl.outputPins) {
      html += QString("<div class='pin output'><b>%1</b> (%2)<br/>%3</div>")
        .arg(pin.name).arg(pin.messageType).arg(pin.description);
    }
  }

  // Parameters
  if (!tmpl.parameters.isEmpty()) {
    html += "<h3>Parameters</h3>";
    for (const auto& param : tmpl.parameters) {
      QString required = param.required ? " *" : "";
      html += QString("<div class='param'><b>%1</b>%2 (%3)<br/>Default: %4<br/>%5</div>")
        .arg(param.name).arg(required).arg(param.type)
        .arg(param.defaultValue.toString()).arg(param.description);
    }
  }

  // Metadata
  html += "<h3>Info</h3><table>";
  html += QString("<tr><td class='label'>Base class:</td><td>%1</td></tr>").arg(tmpl.baseClass);
  html += QString("<tr><td class='label'>Version:</td><td>%1</td></tr>").arg(tmpl.version);
  html += QString("<tr><td class='label'>Author:</td><td>%1</td></tr>").arg(tmpl.author);

  if (!tmpl.requiredPackages.isEmpty()) {
    html += QString("<tr><td class='label'>Packages:</td><td>%1</td></tr>")
      .arg(tmpl.requiredPackages.join(", "));
  }

  html += "</table></body></html>";

  detailsBrowser_->setHtml(html);
}

void NodeTemplatesPanel::clearDetails() {
  selectedTemplateId_ = QUuid();
  detailsBrowser_->hide();
  previewLabel_->show();
  addButton_->setEnabled(false);
}

void NodeTemplatesPanel::onImportClicked() {
  QString filePath = QFileDialog::getOpenFileName(
    this,
    tr("Import Templates"),
    QString(),
    tr("JSON Files (*.json);;All Files (*)")
  );

  if (filePath.isEmpty()) return;

  if (NodeTemplatesManager::instance().loadCustomTemplates(filePath)) {
    QMessageBox::information(this, tr("Import Successful"),
      tr("Templates imported successfully."));
  } else {
    QMessageBox::warning(this, tr("Import Failed"),
      tr("Failed to import templates from the selected file."));
  }
}

void NodeTemplatesPanel::onExportClicked() {
  QString filePath = QFileDialog::getSaveFileName(
    this,
    tr("Export Templates"),
    "custom_templates.json",
    tr("JSON Files (*.json);;All Files (*)")
  );

  if (filePath.isEmpty()) return;

  if (NodeTemplatesManager::instance().saveCustomTemplates(filePath)) {
    QMessageBox::information(this, tr("Export Successful"),
      tr("Custom templates exported successfully."));
  } else {
    QMessageBox::warning(this, tr("Export Failed"),
      tr("Failed to export templates to the selected file."));
  }
}

void NodeTemplatesPanel::onCreateCustomClicked() {
  // Create a basic custom template
  NodeTemplate customTmpl;
  customTmpl.id = QUuid::createUuid();
  customTmpl.name = "custom_node";
  customTmpl.displayName = tr("Custom Node");
  customTmpl.description = tr("A custom user-defined node template");
  customTmpl.category = TemplateCategory::Custom;
  customTmpl.iconName = "custom";
  customTmpl.baseClass = "rclcpp::Node";
  customTmpl.author = "User";
  customTmpl.version = "1.0";

  customTmpl.inputPins = {
    {"input", "std_msgs/msg/String", false, "Input topic"}
  };
  customTmpl.outputPins = {
    {"output", "std_msgs/msg/String", true, "Output topic"}
  };

  NodeTemplatesManager::instance().addTemplate(customTmpl);

  QMessageBox::information(this, tr("Template Created"),
    tr("A new custom template has been created. You can modify it by exporting and editing the JSON file."));
}

}  // namespace ros_weaver
