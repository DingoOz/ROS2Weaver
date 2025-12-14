#include "ros_weaver/widgets/lineage_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QApplication>
#include <QStyle>
#include <QClipboard>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QFileInfo>

namespace ros_weaver {

LineageDialog::LineageDialog(const DataLineage& lineage, QWidget* parent)
    : QDialog(parent)
    , lineage_(lineage)
    , headerLabel_(nullptr)
    , lineageTree_(nullptr)
    , openVsCodeButton_(nullptr)
    , copyPathButton_(nullptr)
    , closeButton_(nullptr) {
  setWindowTitle(tr("Data Origin"));
  setMinimumSize(500, 400);
  resize(550, 450);

  setupUi();
  populateTree();
}

LineageDialog::~LineageDialog() = default;

void LineageDialog::setLineage(const DataLineage& lineage) {
  lineage_ = lineage;
  populateTree();
}

void LineageDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(16, 16, 16, 16);
  mainLayout->setSpacing(12);

  // Header
  headerLabel_ = new QLabel();
  headerLabel_->setWordWrap(true);
  headerLabel_->setStyleSheet("font-size: 14px; font-weight: bold; color: #2c3e50;");
  mainLayout->addWidget(headerLabel_);

  // Description label
  QLabel* descLabel = new QLabel(tr("This data originates from the following sources:"));
  descLabel->setStyleSheet("color: #666;");
  mainLayout->addWidget(descLabel);

  // Lineage tree
  lineageTree_ = new QTreeWidget();
  lineageTree_->setHeaderLabels({tr("Property"), tr("Value")});
  lineageTree_->setRootIsDecorated(true);
  lineageTree_->setAlternatingRowColors(true);
  lineageTree_->setIndentation(20);
  lineageTree_->header()->setStretchLastSection(false);
  lineageTree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  lineageTree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  lineageTree_->setAnimated(true);
  lineageTree_->setSelectionMode(QAbstractItemView::SingleSelection);

  connect(lineageTree_, &QTreeWidget::itemDoubleClicked,
          this, &LineageDialog::onItemDoubleClicked);

  lineageTree_->setStyleSheet(R"(
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

  mainLayout->addWidget(lineageTree_, 1);

  // Info label
  QLabel* infoLabel = new QLabel(tr("Double-click a file path to open in VS Code"));
  infoLabel->setStyleSheet("color: #888; font-size: 11px; font-style: italic;");
  mainLayout->addWidget(infoLabel);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  openVsCodeButton_ = new QPushButton(tr("Open in VS Code"));
  openVsCodeButton_->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileDialogDetailedView));
  openVsCodeButton_->setEnabled(lineage_.hasEditableSource());
  connect(openVsCodeButton_, &QPushButton::clicked, this, &LineageDialog::onOpenInVsCode);
  buttonLayout->addWidget(openVsCodeButton_);

  copyPathButton_ = new QPushButton(tr("Copy Path"));
  copyPathButton_->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
  connect(copyPathButton_, &QPushButton::clicked, this, &LineageDialog::onCopyPath);
  buttonLayout->addWidget(copyPathButton_);

  buttonLayout->addStretch();

  closeButton_ = new QPushButton(tr("Close"));
  closeButton_->setDefault(true);
  connect(closeButton_, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton_);

  mainLayout->addLayout(buttonLayout);
}

void LineageDialog::populateTree() {
  lineageTree_->clear();

  // Set header based on data description
  if (!lineage_.dataDescription().isEmpty()) {
    headerLabel_->setText(lineage_.dataDescription());
  } else {
    headerLabel_->setText(tr("Data Lineage"));
  }

  // Primary source
  QTreeWidgetItem* primaryItem = new QTreeWidgetItem(lineageTree_);
  primaryItem->setText(0, tr("Primary Source"));
  primaryItem->setExpanded(true);

  QFont boldFont = primaryItem->font(0);
  boldFont.setBold(true);
  primaryItem->setFont(0, boldFont);
  primaryItem->setForeground(0, QColor("#2c3e50"));

  addLineageNodeToTree(primaryItem, lineage_.primarySource(), "");

  // Intermediate sources
  const auto& chain = lineage_.intermediateChain();
  if (!chain.isEmpty()) {
    QTreeWidgetItem* chainItem = new QTreeWidgetItem(lineageTree_);
    chainItem->setText(0, tr("Derived From"));
    chainItem->setExpanded(true);
    chainItem->setFont(0, boldFont);
    chainItem->setForeground(0, QColor("#2c3e50"));

    for (int i = 0; i < chain.size(); ++i) {
      QString label = QString("[%1]").arg(i + 1);
      addLineageNodeToTree(chainItem, chain[i], label);
    }
  }

  lineageTree_->expandAll();

  // Update button states
  openVsCodeButton_->setEnabled(lineage_.hasEditableSource());
}

void LineageDialog::addLineageNodeToTree(QTreeWidgetItem* parent,
                                          const LineageNode& node,
                                          const QString& label) {
  QTreeWidgetItem* nodeItem = new QTreeWidgetItem(parent);

  QString title = label.isEmpty() ? node.sourceTypeName()
                                   : QString("%1 %2").arg(label, node.sourceTypeName());
  nodeItem->setText(0, title);
  nodeItem->setExpanded(true);

  // Set icon based on source type
  QStyle::StandardPixmap icon = QStyle::SP_FileIcon;
  switch (node.sourceType) {
    case LineageSourceType::ProjectFile:
      icon = QStyle::SP_FileDialogContentsView;
      break;
    case LineageSourceType::YamlFile:
      icon = QStyle::SP_FileDialogDetailedView;
      break;
    case LineageSourceType::SourceCodeFile:
    case LineageSourceType::Generated:
      icon = QStyle::SP_FileIcon;
      break;
    case LineageSourceType::RosTopic:
    case LineageSourceType::RosParameter:
      icon = QStyle::SP_ComputerIcon;
      break;
    case LineageSourceType::UserInput:
      icon = QStyle::SP_DialogApplyButton;
      break;
    case LineageSourceType::SystemDiscovery:
      icon = QStyle::SP_DriveNetIcon;
      break;
    default:
      icon = QStyle::SP_MessageBoxQuestion;
      break;
  }
  nodeItem->setIcon(0, QApplication::style()->standardIcon(icon));

  // Add child items for node details
  auto addDetail = [nodeItem](const QString& key, const QString& value, bool isPath = false) {
    if (value.isEmpty()) return;

    QTreeWidgetItem* item = new QTreeWidgetItem(nodeItem);
    item->setText(0, key);
    item->setText(1, value);

    if (isPath) {
      item->setForeground(1, QColor("#3498db"));
      item->setToolTip(1, tr("Double-click to open in VS Code"));
      item->setData(0, Qt::UserRole, "path");

      // Store line number if available
      if (nodeItem->parent()) {
        // Find the line number in sibling items
      }
    }
  };

  if (!node.sourcePath.isEmpty()) {
    bool isEditableFile = node.isEditable();
    addDetail(tr("Location"), node.sourcePath, isEditableFile);

    // Store file path and line number in the tree item for double-click handling
    if (isEditableFile) {
      for (int i = 0; i < nodeItem->childCount(); ++i) {
        QTreeWidgetItem* child = nodeItem->child(i);
        if (child->data(0, Qt::UserRole).toString() == "path") {
          child->setData(1, Qt::UserRole, node.sourcePath);
          child->setData(1, Qt::UserRole + 1, node.lineNumber);
          break;
        }
      }
    }
  }

  if (node.lineNumber > 0) {
    addDetail(tr("Line"), QString::number(node.lineNumber));
  }

  if (!node.dataKey.isEmpty()) {
    addDetail(tr("Key/Field"), node.dataKey);
  }

  if (!node.description.isEmpty()) {
    addDetail(tr("Details"), node.description);
  }

  if (!node.timestamp.isEmpty()) {
    addDetail(tr("Timestamp"), node.timestamp);
  }

  if (node.dataValue.isValid()) {
    addDetail(tr("Value"), node.dataValue.toString());
  }
}

QString LineageDialog::getSelectedFilePath() const {
  QTreeWidgetItem* item = lineageTree_->currentItem();
  if (!item) {
    // No selection, use first editable source
    LineageNode editable = lineage_.firstEditableSource();
    return editable.sourcePath;
  }

  // Check if selected item has a path
  QString path = item->data(1, Qt::UserRole).toString();
  if (!path.isEmpty()) {
    return path;
  }

  // Check parent items
  QTreeWidgetItem* parent = item->parent();
  while (parent) {
    for (int i = 0; i < parent->childCount(); ++i) {
      QTreeWidgetItem* child = parent->child(i);
      path = child->data(1, Qt::UserRole).toString();
      if (!path.isEmpty()) {
        return path;
      }
    }
    parent = parent->parent();
  }

  // Fall back to first editable source
  LineageNode editable = lineage_.firstEditableSource();
  return editable.sourcePath;
}

int LineageDialog::getSelectedLineNumber() const {
  QTreeWidgetItem* item = lineageTree_->currentItem();
  if (!item) {
    LineageNode editable = lineage_.firstEditableSource();
    return editable.lineNumber;
  }

  // Check if selected item has a line number
  QVariant lineVar = item->data(1, Qt::UserRole + 1);
  if (lineVar.isValid()) {
    return lineVar.toInt();
  }

  // Check parent items
  QTreeWidgetItem* parent = item->parent();
  while (parent) {
    for (int i = 0; i < parent->childCount(); ++i) {
      QTreeWidgetItem* child = parent->child(i);
      lineVar = child->data(1, Qt::UserRole + 1);
      if (lineVar.isValid()) {
        return lineVar.toInt();
      }
    }
    parent = parent->parent();
  }

  LineageNode editable = lineage_.firstEditableSource();
  return editable.lineNumber;
}

void LineageDialog::onOpenInVsCode() {
  QString filePath = getSelectedFilePath();
  int lineNumber = getSelectedLineNumber();

  if (filePath.isEmpty()) {
    QMessageBox::warning(this, tr("Cannot Open"),
                         tr("No editable file found in the lineage chain."));
    return;
  }

  if (!QFileInfo::exists(filePath)) {
    QMessageBox::warning(this, tr("File Not Found"),
                         tr("The file does not exist:\n%1").arg(filePath));
    return;
  }

  if (!VsCodeIntegration::isAvailable()) {
    QMessageBox::warning(this, tr("VS Code Not Found"),
                         tr("VS Code (code, code-insiders, or codium) was not found in PATH."));
    return;
  }

  if (VsCodeIntegration::openFile(filePath, lineNumber)) {
    // Optionally close the dialog after opening
    // accept();
  } else {
    QMessageBox::warning(this, tr("Failed to Open"),
                         tr("Could not open VS Code."));
  }
}

void LineageDialog::onItemDoubleClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column);

  QString path = item->data(1, Qt::UserRole).toString();
  if (path.isEmpty()) {
    return;
  }

  int lineNumber = item->data(1, Qt::UserRole + 1).toInt();

  if (!QFileInfo::exists(path)) {
    return;
  }

  if (VsCodeIntegration::isAvailable()) {
    VsCodeIntegration::openFile(path, lineNumber);
  }
}

void LineageDialog::onCopyPath() {
  QString filePath = getSelectedFilePath();
  if (!filePath.isEmpty()) {
    QApplication::clipboard()->setText(filePath);
  }
}

// LineageContextMenu implementation

QAction* LineageContextMenu::addLineageAction(QMenu* menu, const DataLineage& lineage,
                                              QWidget* parent) {
  QAction* action = menu->addAction(
      QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation),
      QObject::tr("Show Data Origin..."));

  QObject::connect(action, &QAction::triggered, [lineage, parent]() {
    showLineageDialog(lineage, parent);
  });

  return action;
}

void LineageContextMenu::showContextMenu(const QPoint& globalPos,
                                          const DataLineage& lineage,
                                          QWidget* parent) {
  QMenu menu(parent);

  // Add header showing what data this is
  if (!lineage.dataDescription().isEmpty()) {
    QAction* headerAction = menu.addAction(lineage.dataDescription());
    headerAction->setEnabled(false);
    QFont font = headerAction->font();
    font.setBold(true);
    headerAction->setFont(font);
    menu.addSeparator();
  }

  // Show Data Origin action
  addLineageAction(&menu, lineage, parent);

  // If there's an editable source, add Open in VS Code action
  if (lineage.hasEditableSource()) {
    menu.addSeparator();

    LineageNode editable = lineage.firstEditableSource();
    QString label = QObject::tr("Open in VS Code");
    if (editable.lineNumber > 0) {
      label = QObject::tr("Open in VS Code (line %1)").arg(editable.lineNumber);
    }

    QAction* vsCodeAction = menu.addAction(
        QApplication::style()->standardIcon(QStyle::SP_FileDialogDetailedView),
        label);

    QObject::connect(vsCodeAction, &QAction::triggered, [editable]() {
      if (VsCodeIntegration::isAvailable()) {
        VsCodeIntegration::openFile(editable.sourcePath, editable.lineNumber);
      }
    });

    vsCodeAction->setEnabled(VsCodeIntegration::isAvailable());
  }

  // Copy path action
  const LineageNode& primary = lineage.primarySource();
  if (!primary.sourcePath.isEmpty()) {
    QAction* copyAction = menu.addAction(
        QApplication::style()->standardIcon(QStyle::SP_FileIcon),
        QObject::tr("Copy Path/Location"));

    QObject::connect(copyAction, &QAction::triggered, [primary]() {
      QApplication::clipboard()->setText(primary.sourcePath);
    });
  }

  menu.exec(globalPos);
}

void LineageContextMenu::showLineageDialog(const DataLineage& lineage, QWidget* parent) {
  LineageDialog* dialog = new LineageDialog(lineage, parent);
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->show();
}

}  // namespace ros_weaver
