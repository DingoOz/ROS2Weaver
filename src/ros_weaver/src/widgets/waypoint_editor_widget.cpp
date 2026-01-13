#include "ros_weaver/widgets/waypoint_editor_widget.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>

namespace ros_weaver {

WaypointEditorWidget::WaypointEditorWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();
}

void WaypointEditorWidget::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(8);

  // Basic properties group
  auto* basicGroup = new QGroupBox(tr("Waypoint Properties"));
  auto* basicLayout = new QFormLayout(basicGroup);

  nameEdit_ = new QLineEdit();
  nameEdit_->setPlaceholderText(tr("Waypoint name"));
  connect(nameEdit_, &QLineEdit::editingFinished, this, &WaypointEditorWidget::onNameChanged);
  basicLayout->addRow(tr("Name:"), nameEdit_);

  // Position
  auto* posLayout = new QHBoxLayout();
  xSpinBox_ = new QDoubleSpinBox();
  xSpinBox_->setRange(-10000, 10000);
  xSpinBox_->setDecimals(3);
  xSpinBox_->setSuffix(" m");
  connect(xSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &WaypointEditorWidget::onPositionChanged);

  ySpinBox_ = new QDoubleSpinBox();
  ySpinBox_->setRange(-10000, 10000);
  ySpinBox_->setDecimals(3);
  ySpinBox_->setSuffix(" m");
  connect(ySpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &WaypointEditorWidget::onPositionChanged);

  posLayout->addWidget(new QLabel("X:"));
  posLayout->addWidget(xSpinBox_);
  posLayout->addWidget(new QLabel("Y:"));
  posLayout->addWidget(ySpinBox_);
  basicLayout->addRow(tr("Position:"), posLayout);

  // Orientation
  auto* orientLayout = new QHBoxLayout();
  thetaSpinBox_ = new QDoubleSpinBox();
  thetaSpinBox_->setRange(-180, 180);
  thetaSpinBox_->setDecimals(1);
  thetaSpinBox_->setSuffix(QString::fromUtf8("\u00B0"));  // Degree symbol
  connect(thetaSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &WaypointEditorWidget::onOrientationChanged);

  orientationDial_ = new QDial();
  orientationDial_->setRange(-180, 180);
  orientationDial_->setNotchesVisible(true);
  orientationDial_->setWrapping(true);
  orientationDial_->setFixedSize(60, 60);
  connect(orientationDial_, &QDial::valueChanged, this, [this](int value) {
    if (updatingUi_) return;
    updatingUi_ = true;
    thetaSpinBox_->setValue(static_cast<double>(value));
    currentWaypoint_.setThetaDegrees(static_cast<double>(value));
    updatingUi_ = false;
    emitWaypointChanged();
  });

  orientLayout->addWidget(thetaSpinBox_);
  orientLayout->addWidget(orientationDial_);
  orientLayout->addStretch();
  basicLayout->addRow(tr("Orientation:"), orientLayout);

  // Tolerance
  toleranceSpinBox_ = new QDoubleSpinBox();
  toleranceSpinBox_->setRange(0.01, 10.0);
  toleranceSpinBox_->setDecimals(2);
  toleranceSpinBox_->setSuffix(" m");
  toleranceSpinBox_->setValue(0.3);
  connect(toleranceSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &WaypointEditorWidget::onToleranceChanged);
  basicLayout->addRow(tr("Tolerance:"), toleranceSpinBox_);

  // Color
  colorButton_ = new QPushButton(tr("Choose..."));
  colorButton_->setFixedWidth(100);
  connect(colorButton_, &QPushButton::clicked, this, &WaypointEditorWidget::onColorClicked);
  basicLayout->addRow(tr("Color:"), colorButton_);

  // Description
  descriptionEdit_ = new QTextEdit();
  descriptionEdit_->setMaximumHeight(60);
  descriptionEdit_->setPlaceholderText(tr("Optional description"));
  connect(descriptionEdit_, &QTextEdit::textChanged, this, &WaypointEditorWidget::onDescriptionChanged);
  basicLayout->addRow(tr("Description:"), descriptionEdit_);

  mainLayout->addWidget(basicGroup);

  // Behavior triggers group
  auto* triggersGroup = new QGroupBox(tr("Behavior Triggers"));
  auto* triggersLayout = new QVBoxLayout(triggersGroup);

  triggersList_ = new QListWidget();
  triggersList_->setMaximumHeight(80);
  triggersLayout->addWidget(triggersList_);

  auto* triggerButtonsLayout = new QHBoxLayout();
  addTriggerButton_ = new QPushButton(tr("Add"));
  editTriggerButton_ = new QPushButton(tr("Edit"));
  removeTriggerButton_ = new QPushButton(tr("Remove"));

  connect(addTriggerButton_, &QPushButton::clicked, this, &WaypointEditorWidget::onAddTrigger);
  connect(editTriggerButton_, &QPushButton::clicked, this, &WaypointEditorWidget::onEditTrigger);
  connect(removeTriggerButton_, &QPushButton::clicked, this, &WaypointEditorWidget::onRemoveTrigger);

  triggerButtonsLayout->addWidget(addTriggerButton_);
  triggerButtonsLayout->addWidget(editTriggerButton_);
  triggerButtonsLayout->addWidget(removeTriggerButton_);
  triggerButtonsLayout->addStretch();
  triggersLayout->addLayout(triggerButtonsLayout);

  mainLayout->addWidget(triggersGroup);

  // Delete button
  auto* actionLayout = new QHBoxLayout();
  actionLayout->addStretch();
  deleteWaypointButton_ = new QPushButton(tr("Delete Waypoint"));
  deleteWaypointButton_->setStyleSheet("QPushButton { color: #c0392b; }");
  connect(deleteWaypointButton_, &QPushButton::clicked, this, &WaypointEditorWidget::onDeleteWaypoint);
  actionLayout->addWidget(deleteWaypointButton_);

  mainLayout->addLayout(actionLayout);
  mainLayout->addStretch();

  // Initial state
  setEnabled(false);
}

void WaypointEditorWidget::setWaypoint(const Waypoint& waypoint) {
  currentWaypoint_ = waypoint;
  setEnabled(true);
  updateFromWaypoint();
}

Waypoint WaypointEditorWidget::getWaypoint() const {
  return currentWaypoint_;
}

void WaypointEditorWidget::clear() {
  currentWaypoint_ = Waypoint();
  setEnabled(false);
  updatingUi_ = true;
  nameEdit_->clear();
  xSpinBox_->setValue(0);
  ySpinBox_->setValue(0);
  thetaSpinBox_->setValue(0);
  orientationDial_->setValue(0);
  toleranceSpinBox_->setValue(0.3);
  descriptionEdit_->clear();
  triggersList_->clear();
  updatingUi_ = false;
}

void WaypointEditorWidget::setReadOnly(bool readOnly) {
  nameEdit_->setReadOnly(readOnly);
  xSpinBox_->setReadOnly(readOnly);
  ySpinBox_->setReadOnly(readOnly);
  thetaSpinBox_->setReadOnly(readOnly);
  orientationDial_->setEnabled(!readOnly);
  toleranceSpinBox_->setReadOnly(readOnly);
  colorButton_->setEnabled(!readOnly);
  descriptionEdit_->setReadOnly(readOnly);
  addTriggerButton_->setEnabled(!readOnly);
  editTriggerButton_->setEnabled(!readOnly);
  removeTriggerButton_->setEnabled(!readOnly);
  deleteWaypointButton_->setEnabled(!readOnly);
}

void WaypointEditorWidget::updateFromWaypoint() {
  updatingUi_ = true;

  nameEdit_->setText(currentWaypoint_.name);
  xSpinBox_->setValue(currentWaypoint_.x);
  ySpinBox_->setValue(currentWaypoint_.y);

  double degrees = currentWaypoint_.thetaDegrees();
  thetaSpinBox_->setValue(degrees);
  orientationDial_->setValue(static_cast<int>(degrees));

  toleranceSpinBox_->setValue(currentWaypoint_.tolerance);
  descriptionEdit_->setText(currentWaypoint_.description);

  // Update color button
  QString colorStyle = QString("background-color: %1;").arg(currentWaypoint_.color.name());
  colorButton_->setStyleSheet(colorStyle);

  updateTriggersList();

  updatingUi_ = false;
}

void WaypointEditorWidget::emitWaypointChanged() {
  if (!updatingUi_) {
    emit waypointChanged(currentWaypoint_);
  }
}

void WaypointEditorWidget::updateTriggersList() {
  triggersList_->clear();
  for (const auto& trigger : currentWaypoint_.triggers) {
    QString typeStr = BehaviorTrigger::triggerTypeToString(trigger.type);
    QString label = QString("%1: %2").arg(typeStr, trigger.behaviorTreeId);
    triggersList_->addItem(label);
  }
}

void WaypointEditorWidget::onNameChanged() {
  if (updatingUi_) return;
  currentWaypoint_.name = nameEdit_->text();
  emitWaypointChanged();
}

void WaypointEditorWidget::onPositionChanged() {
  if (updatingUi_) return;
  currentWaypoint_.x = xSpinBox_->value();
  currentWaypoint_.y = ySpinBox_->value();
  emitWaypointChanged();
}

void WaypointEditorWidget::onOrientationChanged() {
  if (updatingUi_) return;
  updatingUi_ = true;
  double degrees = thetaSpinBox_->value();
  currentWaypoint_.setThetaDegrees(degrees);
  orientationDial_->setValue(static_cast<int>(degrees));
  updatingUi_ = false;
  emitWaypointChanged();
}

void WaypointEditorWidget::onToleranceChanged() {
  if (updatingUi_) return;
  currentWaypoint_.tolerance = toleranceSpinBox_->value();
  emitWaypointChanged();
}

void WaypointEditorWidget::onColorClicked() {
  QColor color = QColorDialog::getColor(currentWaypoint_.color, this, tr("Select Waypoint Color"));
  if (color.isValid()) {
    currentWaypoint_.color = color;
    QString colorStyle = QString("background-color: %1;").arg(color.name());
    colorButton_->setStyleSheet(colorStyle);
    emitWaypointChanged();
  }
}

void WaypointEditorWidget::onDescriptionChanged() {
  if (updatingUi_) return;
  currentWaypoint_.description = descriptionEdit_->toPlainText();
  emitWaypointChanged();
}

void WaypointEditorWidget::onAddTrigger() {
  BehaviorTrigger newTrigger;
  BehaviorTriggerDialog dialog(newTrigger, this);
  if (dialog.exec() == QDialog::Accepted) {
    currentWaypoint_.triggers.append(dialog.getTrigger());
    updateTriggersList();
    emitWaypointChanged();
  }
}

void WaypointEditorWidget::onEditTrigger() {
  int row = triggersList_->currentRow();
  if (row < 0 || row >= currentWaypoint_.triggers.size()) return;

  BehaviorTriggerDialog dialog(currentWaypoint_.triggers[row], this);
  if (dialog.exec() == QDialog::Accepted) {
    currentWaypoint_.triggers[row] = dialog.getTrigger();
    updateTriggersList();
    emitWaypointChanged();
  }
}

void WaypointEditorWidget::onRemoveTrigger() {
  int row = triggersList_->currentRow();
  if (row < 0 || row >= currentWaypoint_.triggers.size()) return;

  currentWaypoint_.triggers.removeAt(row);
  updateTriggersList();
  emitWaypointChanged();
}

void WaypointEditorWidget::onDeleteWaypoint() {
  auto result = QMessageBox::question(this, tr("Delete Waypoint"),
      tr("Are you sure you want to delete waypoint '%1'?").arg(currentWaypoint_.name),
      QMessageBox::Yes | QMessageBox::No);

  if (result == QMessageBox::Yes) {
    emit deleteRequested(currentWaypoint_.id);
  }
}

// BehaviorTriggerDialog implementation

BehaviorTriggerDialog::BehaviorTriggerDialog(const BehaviorTrigger& trigger, QWidget* parent)
    : QDialog(parent), currentTrigger_(trigger) {
  setWindowTitle(tr("Edit Behavior Trigger"));
  setupUi();
  updateUiForTriggerType();
}

BehaviorTrigger BehaviorTriggerDialog::getTrigger() const {
  return currentTrigger_;
}

void BehaviorTriggerDialog::setupUi() {
  auto* layout = new QVBoxLayout(this);

  auto* formLayout = new QFormLayout();

  // Trigger type
  triggerTypeCombo_ = new QComboBox();
  triggerTypeCombo_->addItem(tr("On Arrival"), static_cast<int>(BehaviorTrigger::OnArrival));
  triggerTypeCombo_->addItem(tr("On Approach"), static_cast<int>(BehaviorTrigger::OnApproach));
  triggerTypeCombo_->addItem(tr("On Departure"), static_cast<int>(BehaviorTrigger::OnDeparture));
  triggerTypeCombo_->addItem(tr("While Navigating"), static_cast<int>(BehaviorTrigger::WhileNavigating));

  int typeIndex = triggerTypeCombo_->findData(static_cast<int>(currentTrigger_.type));
  if (typeIndex >= 0) {
    triggerTypeCombo_->setCurrentIndex(typeIndex);
  }

  connect(triggerTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &BehaviorTriggerDialog::onTriggerTypeChanged);
  formLayout->addRow(tr("Trigger Type:"), triggerTypeCombo_);

  // Approach distance
  approachDistanceLabel_ = new QLabel(tr("Approach Distance:"));
  approachDistanceSpinBox_ = new QDoubleSpinBox();
  approachDistanceSpinBox_->setRange(0.1, 100.0);
  approachDistanceSpinBox_->setDecimals(2);
  approachDistanceSpinBox_->setSuffix(" m");
  approachDistanceSpinBox_->setValue(currentTrigger_.approachDistance);
  connect(approachDistanceSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double value) { currentTrigger_.approachDistance = value; });
  formLayout->addRow(approachDistanceLabel_, approachDistanceSpinBox_);

  // Behavior tree selection
  auto* btLayout = new QHBoxLayout();
  behaviorTreeCombo_ = new QComboBox();
  behaviorTreeCombo_->setEditable(true);
  behaviorTreeCombo_->setMinimumWidth(200);
  behaviorTreeCombo_->setEditText(currentTrigger_.behaviorTreeId);
  loadBehaviorTrees();
  connect(behaviorTreeCombo_, &QComboBox::editTextChanged,
          this, [this](const QString& text) { currentTrigger_.behaviorTreeId = text; });

  browseBTButton_ = new QPushButton(tr("Browse..."));
  connect(browseBTButton_, &QPushButton::clicked, this, &BehaviorTriggerDialog::onBrowseBehaviorTree);

  btLayout->addWidget(behaviorTreeCombo_, 1);
  btLayout->addWidget(browseBTButton_);
  formLayout->addRow(tr("Behavior Tree:"), btLayout);

  // Behavior node (optional)
  behaviorNodeEdit_ = new QLineEdit();
  behaviorNodeEdit_->setText(currentTrigger_.behaviorNodeId);
  behaviorNodeEdit_->setPlaceholderText(tr("Optional: specific node ID"));
  connect(behaviorNodeEdit_, &QLineEdit::textChanged,
          this, [this](const QString& text) { currentTrigger_.behaviorNodeId = text; });
  formLayout->addRow(tr("Node ID:"), behaviorNodeEdit_);

  layout->addLayout(formLayout);

  // Buttons
  auto* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  auto* okButton = new QPushButton(tr("OK"));
  auto* cancelButton = new QPushButton(tr("Cancel"));
  connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
  connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);

  layout->addLayout(buttonLayout);

  resize(400, 250);
}

void BehaviorTriggerDialog::onTriggerTypeChanged(int index) {
  currentTrigger_.type = static_cast<BehaviorTrigger::TriggerType>(
      triggerTypeCombo_->itemData(index).toInt());
  updateUiForTriggerType();
}

void BehaviorTriggerDialog::updateUiForTriggerType() {
  bool showApproach = (currentTrigger_.type == BehaviorTrigger::OnApproach);
  approachDistanceLabel_->setVisible(showApproach);
  approachDistanceSpinBox_->setVisible(showApproach);
}

void BehaviorTriggerDialog::onBrowseBehaviorTree() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Select Behavior Tree"),
      QString(),
      tr("Behavior Tree XML (*.xml);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (!filePath.isEmpty()) {
    behaviorTreeCombo_->setEditText(filePath);
  }
}

void BehaviorTriggerDialog::loadBehaviorTrees() {
  // TODO: Scan workspace for .xml behavior tree files
  // For now, leave combo empty
}

}  // namespace ros_weaver
