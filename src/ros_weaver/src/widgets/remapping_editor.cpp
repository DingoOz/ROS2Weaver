#include "ros_weaver/widgets/remapping_editor.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QMessageBox>

namespace ros_weaver {

RemappingEditor::RemappingEditor(QWidget* parent)
    : QWidget(parent)
    , namespaceEdit_(nullptr)
    , remappingTable_(nullptr)
    , typeCombo_(nullptr)
    , fromEdit_(nullptr)
    , toEdit_(nullptr)
    , addButton_(nullptr)
    , removeButton_(nullptr)
    , applyButton_(nullptr)
    , resetButton_(nullptr)
    , statusLabel_(nullptr)
    , currentBlock_(nullptr)
{
  setupUi();
}

void RemappingEditor::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(12);

  // Title
  QLabel* titleLabel = new QLabel(tr("Namespace & Remappings"));
  titleLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
  mainLayout->addWidget(titleLabel);

  // Namespace section
  QGroupBox* nsGroup = new QGroupBox(tr("Node Namespace"));
  QHBoxLayout* nsLayout = new QHBoxLayout(nsGroup);

  QLabel* nsLabel = new QLabel(tr("Namespace:"));
  namespaceEdit_ = new QLineEdit();
  namespaceEdit_->setPlaceholderText(tr("/my_robot"));
  connect(namespaceEdit_, &QLineEdit::textChanged, this, [this]() {
    emit namespaceChanged(namespaceEdit_->text());
  });

  nsLayout->addWidget(nsLabel);
  nsLayout->addWidget(namespaceEdit_, 1);
  mainLayout->addWidget(nsGroup);

  // Remappings table
  QGroupBox* remapGroup = new QGroupBox(tr("Topic/Service Remappings"));
  QVBoxLayout* remapLayout = new QVBoxLayout(remapGroup);

  remappingTable_ = new QTableWidget();
  remappingTable_->setColumnCount(3);
  remappingTable_->setHorizontalHeaderLabels({tr("Type"), tr("From"), tr("To")});
  remappingTable_->horizontalHeader()->setStretchLastSection(true);
  remappingTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  remappingTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  remappingTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  remappingTable_->setAlternatingRowColors(true);
  remapLayout->addWidget(remappingTable_);

  // Add remapping form
  QHBoxLayout* addLayout = new QHBoxLayout();

  typeCombo_ = new QComboBox();
  typeCombo_->addItems({"topic", "service", "action", "parameter", "node"});
  typeCombo_->setMinimumWidth(80);

  fromEdit_ = new QLineEdit();
  fromEdit_->setPlaceholderText(tr("From (e.g., /cmd_vel)"));

  toEdit_ = new QLineEdit();
  toEdit_->setPlaceholderText(tr("To (e.g., /robot/cmd_vel)"));

  addButton_ = new QPushButton(tr("+"));
  addButton_->setToolTip(tr("Add remapping"));
  addButton_->setFixedWidth(30);
  connect(addButton_, &QPushButton::clicked, this, &RemappingEditor::addRemapping);

  removeButton_ = new QPushButton(tr("-"));
  removeButton_->setToolTip(tr("Remove selected remapping"));
  removeButton_->setFixedWidth(30);
  connect(removeButton_, &QPushButton::clicked, this, &RemappingEditor::removeSelectedRemapping);

  addLayout->addWidget(typeCombo_);
  addLayout->addWidget(fromEdit_);
  addLayout->addWidget(toEdit_);
  addLayout->addWidget(addButton_);
  addLayout->addWidget(removeButton_);
  remapLayout->addLayout(addLayout);

  mainLayout->addWidget(remapGroup, 1);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  statusLabel_ = new QLabel();
  statusLabel_->setStyleSheet("color: #666; font-size: 11px;");

  applyButton_ = new QPushButton(tr("Apply"));
  applyButton_->setToolTip(tr("Apply changes to the block"));
  connect(applyButton_, &QPushButton::clicked, this, &RemappingEditor::applyChanges);

  resetButton_ = new QPushButton(tr("Reset"));
  resetButton_->setToolTip(tr("Reset to original values"));
  connect(resetButton_, &QPushButton::clicked, this, &RemappingEditor::resetChanges);

  buttonLayout->addWidget(statusLabel_, 1);
  buttonLayout->addWidget(resetButton_);
  buttonLayout->addWidget(applyButton_);
  mainLayout->addLayout(buttonLayout);

  // Initial state
  setEnabled(false);
}

void RemappingEditor::setBlock(PackageBlock* block) {
  currentBlock_ = block;
  setEnabled(block != nullptr);

  if (block) {
    loadFromBlock();
  } else {
    remappings_.clear();
    namespaceEdit_->clear();
    remappingTable_->setRowCount(0);
  }
}

void RemappingEditor::loadFromBlock() {
  if (!currentBlock_) return;

  remappings_.clear();
  // TODO: Load existing remappings from block if stored

  originalNamespace_ = ""; // Would load from block
  originalRemappings_ = remappings_;

  namespaceEdit_->setText(originalNamespace_);
  updateTable();

  statusLabel_->setText(tr("Editing: %1").arg(currentBlock_->packageName()));
}

void RemappingEditor::updateTable() {
  remappingTable_->setRowCount(remappings_.size());

  for (int i = 0; i < remappings_.size(); ++i) {
    const Remapping& remap = remappings_[i];

    QTableWidgetItem* typeItem = new QTableWidgetItem(remap.type);
    typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable);

    QTableWidgetItem* fromItem = new QTableWidgetItem(remap.fromName);
    QTableWidgetItem* toItem = new QTableWidgetItem(remap.toName);

    // Color code by type
    QColor typeColor;
    if (remap.type == "topic") {
      typeColor = QColor("#27ae60");
    } else if (remap.type == "service") {
      typeColor = QColor("#3498db");
    } else if (remap.type == "action") {
      typeColor = QColor("#9b59b6");
    } else if (remap.type == "parameter") {
      typeColor = QColor("#e67e22");
    } else {
      typeColor = QColor("#7f8c8d");
    }
    typeItem->setForeground(typeColor);

    remappingTable_->setItem(i, 0, typeItem);
    remappingTable_->setItem(i, 1, fromItem);
    remappingTable_->setItem(i, 2, toItem);
  }
}

QString RemappingEditor::nodeNamespace() const {
  return namespaceEdit_->text();
}

void RemappingEditor::setNodeNamespace(const QString& ns) {
  namespaceEdit_->setText(ns);
}

void RemappingEditor::addRemapping() {
  QString fromName = fromEdit_->text().trimmed();
  QString toName = toEdit_->text().trimmed();

  if (fromName.isEmpty() || toName.isEmpty()) {
    QMessageBox::warning(this, tr("Invalid Remapping"),
                         tr("Both 'From' and 'To' fields must be filled."));
    return;
  }

  Remapping remap;
  remap.type = typeCombo_->currentText();
  remap.fromName = fromName;
  remap.toName = toName;

  // Check for duplicates
  for (const Remapping& existing : remappings_) {
    if (existing.fromName == fromName && existing.type == remap.type) {
      QMessageBox::warning(this, tr("Duplicate Remapping"),
                           tr("A remapping for '%1' already exists.").arg(fromName));
      return;
    }
  }

  remappings_.append(remap);
  updateTable();

  // Clear inputs
  fromEdit_->clear();
  toEdit_->clear();

  emit remappingsChanged();
}

void RemappingEditor::removeSelectedRemapping() {
  QList<QTableWidgetItem*> selected = remappingTable_->selectedItems();
  if (selected.isEmpty()) {
    return;
  }

  // Get unique rows
  QSet<int> rows;
  for (QTableWidgetItem* item : selected) {
    rows.insert(item->row());
  }

  // Remove in reverse order to maintain indices
  QList<int> sortedRows = rows.values();
  std::sort(sortedRows.begin(), sortedRows.end(), std::greater<int>());

  for (int row : sortedRows) {
    if (row >= 0 && row < remappings_.size()) {
      remappings_.removeAt(row);
    }
  }

  updateTable();
  emit remappingsChanged();
}

void RemappingEditor::applyChanges() {
  if (!currentBlock_) return;

  // Store remappings on block
  // The actual storage mechanism depends on how you want to persist this data
  // For now, we emit signals to indicate changes

  originalNamespace_ = namespaceEdit_->text();
  originalRemappings_ = remappings_;

  statusLabel_->setText(tr("Changes applied"));
  emit remappingsChanged();
  emit namespaceChanged(namespaceEdit_->text());
}

void RemappingEditor::resetChanges() {
  namespaceEdit_->setText(originalNamespace_);
  remappings_ = originalRemappings_;
  updateTable();
  statusLabel_->setText(tr("Changes reset"));
}

}  // namespace ros_weaver
