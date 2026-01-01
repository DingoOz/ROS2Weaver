#include "ros_weaver/widgets/schema_viewer_widget.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QApplication>
#include <QClipboard>
#include <QStyle>

namespace ros_weaver {

SchemaViewerWidget::SchemaViewerWidget(QWidget* parent)
    : QWidget(parent)
    , searchEdit_(nullptr)
    , typeFilter_(nullptr)
    , refreshButton_(nullptr)
    , interfaceList_(nullptr)
    , splitter_(nullptr)
    , schemaTree_(nullptr)
    , rawDefinition_(nullptr)
    , displayStack_(nullptr)
    , copyButton_(nullptr)
    , statusLabel_(nullptr)
{
  setupUi();
  populateInterfaceList();
}

void SchemaViewerWidget::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Title
  QLabel* titleLabel = new QLabel(tr("Message Schema Viewer"));
  titleLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
  mainLayout->addWidget(titleLabel);

  // Search and filter bar
  QHBoxLayout* searchLayout = new QHBoxLayout();

  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search interfaces..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &SchemaViewerWidget::onSearchTextChanged);

  typeFilter_ = new QComboBox();
  typeFilter_->addItem(tr("All Types"), static_cast<int>(TypeFilter::All));
  typeFilter_->addItem(tr("Messages"), static_cast<int>(TypeFilter::Messages));
  typeFilter_->addItem(tr("Services"), static_cast<int>(TypeFilter::Services));
  typeFilter_->addItem(tr("Actions"), static_cast<int>(TypeFilter::Actions));
  typeFilter_->setMinimumWidth(100);
  connect(typeFilter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SchemaViewerWidget::onTypeFilterChanged);

  refreshButton_ = new QPushButton();
  refreshButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  refreshButton_->setToolTip(tr("Refresh interface list"));
  refreshButton_->setFixedWidth(32);
  connect(refreshButton_, &QPushButton::clicked,
          this, &SchemaViewerWidget::onRefreshClicked);

  searchLayout->addWidget(searchEdit_, 1);
  searchLayout->addWidget(typeFilter_);
  searchLayout->addWidget(refreshButton_);
  mainLayout->addLayout(searchLayout);

  // Splitter for interface list and schema view
  splitter_ = new QSplitter(Qt::Horizontal);

  // Interface list
  interfaceList_ = new QTreeWidget();
  interfaceList_->setHeaderLabels({tr("Package"), tr("Type")});
  interfaceList_->setRootIsDecorated(true);
  interfaceList_->setAlternatingRowColors(true);
  interfaceList_->header()->setStretchLastSection(false);
  interfaceList_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  interfaceList_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  interfaceList_->setMinimumWidth(200);
  connect(interfaceList_, &QTreeWidget::itemClicked,
          this, &SchemaViewerWidget::onInterfaceSelected);

  // Schema display area
  QWidget* schemaPanel = new QWidget();
  QVBoxLayout* schemaPanelLayout = new QVBoxLayout(schemaPanel);
  schemaPanelLayout->setContentsMargins(0, 0, 0, 0);

  // Display stack (tree view or raw text)
  displayStack_ = new QStackedWidget();

  // Tree view for structured schema
  schemaTree_ = new QTreeWidget();
  schemaTree_->setHeaderLabels({tr("Field"), tr("Type"), tr("Default")});
  schemaTree_->setRootIsDecorated(true);
  schemaTree_->setAlternatingRowColors(true);
  schemaTree_->header()->setStretchLastSection(false);
  schemaTree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  schemaTree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  schemaTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

  // Raw definition view
  rawDefinition_ = new QTextBrowser();
  rawDefinition_->setFont(QFont("Monospace", 10));
  rawDefinition_->setStyleSheet("background-color: #f5f5f5;");

  displayStack_->addWidget(schemaTree_);
  displayStack_->addWidget(rawDefinition_);
  schemaPanelLayout->addWidget(displayStack_, 1);

  // Bottom bar with copy button and status
  QHBoxLayout* bottomLayout = new QHBoxLayout();

  copyButton_ = new QPushButton(tr("Copy Schema"));
  copyButton_->setIcon(style()->standardIcon(QStyle::SP_FileIcon));
  copyButton_->setEnabled(false);
  connect(copyButton_, &QPushButton::clicked,
          this, &SchemaViewerWidget::onCopyClicked);

  QPushButton* toggleViewButton = new QPushButton(tr("Toggle View"));
  connect(toggleViewButton, &QPushButton::clicked, this, [this]() {
    int currentIndex = displayStack_->currentIndex();
    displayStack_->setCurrentIndex(currentIndex == 0 ? 1 : 0);
  });

  statusLabel_ = new QLabel();
  statusLabel_->setStyleSheet("color: #666; font-size: 11px;");

  bottomLayout->addWidget(copyButton_);
  bottomLayout->addWidget(toggleViewButton);
  bottomLayout->addStretch();
  bottomLayout->addWidget(statusLabel_);
  schemaPanelLayout->addLayout(bottomLayout);

  splitter_->addWidget(interfaceList_);
  splitter_->addWidget(schemaPanel);
  splitter_->setSizes({250, 450});

  mainLayout->addWidget(splitter_, 1);
}

void SchemaViewerWidget::populateInterfaceList() {
  RosDocsProvider& provider = RosDocsProvider::instance();

  // Fetch all interface types
  allMessages_ = provider.getAvailableMessages();
  allServices_ = provider.getAvailableServices();
  allActions_ = provider.getAvailableActions();

  filterInterfaces();
}

void SchemaViewerWidget::filterInterfaces() {
  interfaceList_->clear();

  QString filter = searchEdit_->text().toLower();
  QMap<QString, QTreeWidgetItem*> packageItems;

  auto addInterfaces = [&](const QStringList& interfaces, const QString& typeLabel) {
    for (const QString& iface : interfaces) {
      if (!filter.isEmpty() && !iface.toLower().contains(filter)) {
        continue;
      }

      // Parse package name from interface (e.g., "std_msgs/msg/String")
      QStringList parts = iface.split('/');
      QString packageName = parts.isEmpty() ? "unknown" : parts[0];
      QString typeName = parts.size() >= 3 ? parts.mid(2).join('/') : iface;

      // Get or create package item
      QTreeWidgetItem* packageItem = packageItems.value(packageName, nullptr);
      if (!packageItem) {
        packageItem = new QTreeWidgetItem(interfaceList_);
        packageItem->setText(0, packageName);
        packageItem->setExpanded(false);
        QFont font = packageItem->font(0);
        font.setBold(true);
        packageItem->setFont(0, font);
        packageItems[packageName] = packageItem;
      }

      // Add interface item
      QTreeWidgetItem* item = new QTreeWidgetItem(packageItem);
      item->setText(0, typeName);
      item->setText(1, typeLabel);
      item->setData(0, Qt::UserRole, iface);  // Store full name

      // Color-code by type
      if (typeLabel == "msg") {
        item->setForeground(1, QColor("#27ae60"));
      } else if (typeLabel == "srv") {
        item->setForeground(1, QColor("#3498db"));
      } else if (typeLabel == "action") {
        item->setForeground(1, QColor("#9b59b6"));
      }
    }
  };

  TypeFilter tf = static_cast<TypeFilter>(typeFilter_->currentData().toInt());

  if (tf == TypeFilter::All || tf == TypeFilter::Messages) {
    addInterfaces(allMessages_, "msg");
  }
  if (tf == TypeFilter::All || tf == TypeFilter::Services) {
    addInterfaces(allServices_, "srv");
  }
  if (tf == TypeFilter::All || tf == TypeFilter::Actions) {
    addInterfaces(allActions_, "action");
  }

  // Update status
  int totalCount = 0;
  for (int i = 0; i < interfaceList_->topLevelItemCount(); ++i) {
    totalCount += interfaceList_->topLevelItem(i)->childCount();
  }
  statusLabel_->setText(tr("%1 interfaces in %2 packages")
                            .arg(totalCount)
                            .arg(interfaceList_->topLevelItemCount()));
}

void SchemaViewerWidget::setInterface(const QString& fullName) {
  currentInterface_ = fullName;

  RosDocsProvider& provider = RosDocsProvider::instance();
  InterfaceDoc doc = provider.getInterfaceDoc(fullName);

  displaySchema(doc);
  copyButton_->setEnabled(doc.isValid);

  emit interfaceSelected(fullName);
}

void SchemaViewerWidget::clear() {
  currentInterface_.clear();
  schemaTree_->clear();
  rawDefinition_->clear();
  copyButton_->setEnabled(false);
}

void SchemaViewerWidget::displaySchema(const InterfaceDoc& doc) {
  schemaTree_->clear();
  rawDefinition_->clear();

  if (!doc.isValid) {
    rawDefinition_->setHtml(
        QString("<p style='color: #c0392b;'><i>Could not load schema for %1</i></p>")
            .arg(doc.fullName));
    return;
  }

  // Build tree view
  buildSchemaTree(doc.fields);

  // Set raw definition
  rawDefinition_->setPlainText(doc.rawDefinition);

  // Expand all tree items
  schemaTree_->expandAll();
}

void SchemaViewerWidget::buildSchemaTree(const QList<FieldInfo>& fields) {
  QList<QTreeWidgetItem*> depthStack;

  for (const FieldInfo& field : fields) {
    QTreeWidgetItem* item = createFieldItem(field);

    // Find parent based on depth
    while (depthStack.size() > field.depth) {
      depthStack.removeLast();
    }

    if (depthStack.isEmpty()) {
      schemaTree_->addTopLevelItem(item);
    } else {
      depthStack.last()->addChild(item);
    }

    depthStack.append(item);
  }
}

QTreeWidgetItem* SchemaViewerWidget::createFieldItem(const FieldInfo& field) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, field.name);
  item->setText(1, field.type);
  item->setText(2, field.defaultValue);

  if (!field.comment.isEmpty()) {
    item->setToolTip(0, field.comment);
    item->setToolTip(1, field.comment);
  }

  // Style the type column
  QFont typeFont = item->font(1);
  typeFont.setFamily("Monospace");
  item->setFont(1, typeFont);

  // Color primitive types differently
  static QStringList primitiveTypes = {
      "bool", "byte", "char", "float32", "float64",
      "int8", "int16", "int32", "int64",
      "uint8", "uint16", "uint32", "uint64",
      "string", "wstring"
  };

  QString baseType = field.type;
  if (baseType.contains('[')) {
    baseType = baseType.left(baseType.indexOf('['));
  }

  if (primitiveTypes.contains(baseType)) {
    item->setForeground(1, QColor("#2980b9"));
  } else if (baseType.contains('/')) {
    item->setForeground(1, QColor("#8e44ad"));
  }

  return item;
}

void SchemaViewerWidget::onSearchTextChanged(const QString& /*text*/) {
  filterInterfaces();
}

void SchemaViewerWidget::onTypeFilterChanged(int /*index*/) {
  filterInterfaces();
}

void SchemaViewerWidget::onInterfaceSelected(QTreeWidgetItem* item, int /*column*/) {
  // Only handle leaf items (actual interfaces, not packages)
  if (item->childCount() > 0) {
    return;
  }

  QString fullName = item->data(0, Qt::UserRole).toString();
  if (!fullName.isEmpty()) {
    setInterface(fullName);
  }
}

void SchemaViewerWidget::onRefreshClicked() {
  RosDocsProvider::instance().clearCache();
  populateInterfaceList();

  if (!currentInterface_.isEmpty()) {
    setInterface(currentInterface_);
  }
}

void SchemaViewerWidget::onCopyClicked() {
  if (currentInterface_.isEmpty()) {
    return;
  }

  RosDocsProvider& provider = RosDocsProvider::instance();
  InterfaceDoc doc = provider.getInterfaceDoc(currentInterface_);

  if (doc.isValid) {
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(doc.rawDefinition);
  }
}

}  // namespace ros_weaver
