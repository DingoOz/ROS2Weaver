#include "ros_weaver/widgets/slam_config_widget.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QInputDialog>
#include <QTextEdit>
#include <QHeaderView>
#include <QMessageBox>

namespace ros_weaver {

SlamConfigWidget::SlamConfigWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();
  setupConnections();
}

void SlamConfigWidget::setSlamPipelineManager(SlamPipelineManager* manager) {
  if (manager_) {
    disconnect(manager_, nullptr, this, nullptr);
  }

  manager_ = manager;

  if (manager_) {
    connect(manager_, &SlamPipelineManager::slamStarted, this, &SlamConfigWidget::onSlamStarted);
    connect(manager_, &SlamPipelineManager::slamStopped, this, &SlamConfigWidget::onSlamStopped);
    connect(manager_, &SlamPipelineManager::slamError, this, &SlamConfigWidget::onSlamError);
    connect(manager_, &SlamPipelineManager::parametersUpdated, this, &SlamConfigWidget::onParametersUpdated);
    connect(manager_, &SlamPipelineManager::presetLoaded, this, &SlamConfigWidget::onPresetLoaded);
    connect(manager_, &SlamPipelineManager::outputReceived, this, [this](const QString& output) {
      if (outputText_) {
        outputText_->append(output);
      }
    });

    populatePresetCombo();
    updateStatus();
  }
}

void SlamConfigWidget::selectPreset(const QString& name) {
  int index = presetCombo_->findText(name);
  if (index >= 0) {
    presetCombo_->setCurrentIndex(index);
  }
}

QString SlamConfigWidget::selectedPreset() const {
  return presetCombo_->currentText();
}

bool SlamConfigWidget::isSlamRunning() const {
  return manager_ && manager_->isRunning();
}

void SlamConfigWidget::onSlamStarted() {
  launchButton_->setEnabled(false);
  stopButton_->setEnabled(true);
  statusLabel_->setText(tr("Status: Running"));
  statusLabel_->setStyleSheet("color: green; font-weight: bold;");
  presetCombo_->setEnabled(false);
}

void SlamConfigWidget::onSlamStopped() {
  launchButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  statusLabel_->setText(tr("Status: Stopped"));
  statusLabel_->setStyleSheet("color: gray;");
  presetCombo_->setEnabled(true);
}

void SlamConfigWidget::onSlamError(const QString& error) {
  statusLabel_->setText(tr("Status: Error"));
  statusLabel_->setStyleSheet("color: red; font-weight: bold;");

  if (outputText_) {
    outputText_->append(QString("<span style='color: red;'>Error: %1</span>").arg(error));
  }
}

void SlamConfigWidget::onParametersUpdated() {
  populateParameterTree();
}

void SlamConfigWidget::onPresetLoaded(const QString& name) {
  // Update combo box selection
  int index = presetCombo_->findText(name);
  if (index >= 0) {
    presetCombo_->blockSignals(true);
    presetCombo_->setCurrentIndex(index);
    presetCombo_->blockSignals(false);
  }

  // Update parameter tree
  populateParameterTree();
}

void SlamConfigWidget::onPresetComboChanged(int index) {
  if (index < 0 || !manager_) return;

  QString presetName = presetCombo_->currentText();
  manager_->loadPreset(presetName);
  emit presetSelected(presetName);
}

void SlamConfigWidget::onLaunchClicked() {
  if (!manager_) return;

  // Get use_sim_time setting
  QMap<QString, QVariant> params = manager_->getAllParameters();
  params["use_sim_time"] = useSimTimeCheckBox_->isChecked();

  // Clear output
  if (outputText_) {
    outputText_->clear();
  }

  manager_->launchSlamToolboxAsync(params);
  emit launchRequested();
}

void SlamConfigWidget::onStopClicked() {
  if (manager_) {
    manager_->stopSlam();
    emit stopRequested();
  }
}

void SlamConfigWidget::onSavePresetClicked() {
  bool ok;
  QString name = QInputDialog::getText(this, tr("Save Preset"),
                                       tr("Preset name:"),
                                       QLineEdit::Normal,
                                       QString("Custom Preset"),
                                       &ok);
  if (ok && !name.isEmpty() && manager_) {
    bool ok2;
    QString desc = QInputDialog::getText(this, tr("Save Preset"),
                                         tr("Description (optional):"),
                                         QLineEdit::Normal, "", &ok2);
    manager_->savePreset(name, desc);
    populatePresetCombo();
    emit savePresetRequested(name);
  }
}

void SlamConfigWidget::onAutoRerunToggled(bool checked) {
  if (manager_) {
    manager_->enableAutoRerun(checked);
    emit autoRerunToggled(checked);
  }
}

void SlamConfigWidget::onParameterItemChanged(QTreeWidgetItem* item, int column) {
  if (column != ColValue || !manager_) return;

  QString paramName = item->text(ColName);
  QString typeStr = item->text(ColType);
  QString valueStr = item->text(ColValue);

  QVariant value;
  if (typeStr == "bool") {
    value = (valueStr.toLower() == "true" || valueStr == "1");
  } else if (typeStr == "int") {
    value = valueStr.toInt();
  } else if (typeStr == "double") {
    value = valueStr.toDouble();
  } else {
    value = valueStr;
  }

  manager_->setParameter(paramName, value);
  emit parameterChanged(paramName, value);
}

void SlamConfigWidget::onRefreshParamsClicked() {
  populateParameterTree();
}

void SlamConfigWidget::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(5, 5, 5, 5);
  mainLayout->setSpacing(10);

  // Preset selection group
  auto* presetGroup = new QGroupBox(tr("SLAM Preset"), this);
  auto* presetLayout = new QHBoxLayout(presetGroup);

  presetCombo_ = new QComboBox(this);
  presetCombo_->setMinimumWidth(200);
  presetLayout->addWidget(presetCombo_, 1);

  savePresetButton_ = new QPushButton(tr("Save..."), this);
  savePresetButton_->setToolTip(tr("Save current parameters as preset"));
  presetLayout->addWidget(savePresetButton_);

  mainLayout->addWidget(presetGroup);

  // Launch controls group
  auto* launchGroup = new QGroupBox(tr("SLAM Control"), this);
  auto* launchLayout = new QHBoxLayout(launchGroup);

  launchButton_ = new QPushButton(tr("Launch SLAM"), this);
  launchButton_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
  launchLayout->addWidget(launchButton_);

  stopButton_ = new QPushButton(tr("Stop SLAM"), this);
  stopButton_->setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; }");
  stopButton_->setEnabled(false);
  launchLayout->addWidget(stopButton_);

  statusLabel_ = new QLabel(tr("Status: Stopped"), this);
  statusLabel_->setStyleSheet("color: gray;");
  launchLayout->addWidget(statusLabel_);

  launchLayout->addStretch();
  mainLayout->addWidget(launchGroup);

  // Options
  auto* optionsLayout = new QHBoxLayout();

  useSimTimeCheckBox_ = new QCheckBox(tr("Use sim_time"), this);
  useSimTimeCheckBox_->setChecked(true);
  useSimTimeCheckBox_->setToolTip(tr("Enable sim_time for bag playback"));
  optionsLayout->addWidget(useSimTimeCheckBox_);

  autoRerunCheckBox_ = new QCheckBox(tr("Auto-rerun on param change"), this);
  autoRerunCheckBox_->setToolTip(tr("Automatically restart SLAM and replay bag when parameters change"));
  optionsLayout->addWidget(autoRerunCheckBox_);

  optionsLayout->addStretch();
  mainLayout->addLayout(optionsLayout);

  // Parameters group
  auto* paramsGroup = new QGroupBox(tr("Parameters"), this);
  auto* paramsLayout = new QVBoxLayout(paramsGroup);

  auto* paramsToolbar = new QHBoxLayout();
  refreshParamsButton_ = new QPushButton(tr("Refresh"), this);
  paramsToolbar->addWidget(refreshParamsButton_);
  paramsToolbar->addStretch();
  paramsLayout->addLayout(paramsToolbar);

  parameterTree_ = new QTreeWidget(this);
  parameterTree_->setHeaderLabels({tr("Parameter"), tr("Value"), tr("Type")});
  parameterTree_->setColumnWidth(ColName, 200);
  parameterTree_->setColumnWidth(ColValue, 120);
  parameterTree_->setColumnWidth(ColType, 60);
  parameterTree_->setAlternatingRowColors(true);
  parameterTree_->header()->setStretchLastSection(true);
  paramsLayout->addWidget(parameterTree_, 1);

  mainLayout->addWidget(paramsGroup, 1);

  // Output group
  auto* outputGroup = new QGroupBox(tr("SLAM Output"), this);
  auto* outputLayout = new QVBoxLayout(outputGroup);

  outputText_ = new QTextEdit(this);
  outputText_->setReadOnly(true);
  outputText_->setMaximumHeight(100);
  outputText_->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #d4d4d4; font-family: monospace; }");
  outputLayout->addWidget(outputText_);

  mainLayout->addWidget(outputGroup);
}

void SlamConfigWidget::setupConnections() {
  connect(presetCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SlamConfigWidget::onPresetComboChanged);
  connect(launchButton_, &QPushButton::clicked, this, &SlamConfigWidget::onLaunchClicked);
  connect(stopButton_, &QPushButton::clicked, this, &SlamConfigWidget::onStopClicked);
  connect(savePresetButton_, &QPushButton::clicked, this, &SlamConfigWidget::onSavePresetClicked);
  connect(autoRerunCheckBox_, &QCheckBox::toggled, this, &SlamConfigWidget::onAutoRerunToggled);
  connect(parameterTree_, &QTreeWidget::itemChanged, this, &SlamConfigWidget::onParameterItemChanged);
  connect(refreshParamsButton_, &QPushButton::clicked, this, &SlamConfigWidget::onRefreshParamsClicked);
}

void SlamConfigWidget::populatePresetCombo() {
  // Block signals to prevent deadlock from currentIndexChanged -> loadPreset -> getAllParameters
  presetCombo_->blockSignals(true);
  presetCombo_->clear();

  if (!manager_) {
    presetCombo_->blockSignals(false);
    return;
  }

  QStringList presets = manager_->availablePresets();
  for (const QString& preset : presets) {
    presetCombo_->addItem(preset);
  }

  // Select first preset if any
  if (presetCombo_->count() > 0) {
    presetCombo_->setCurrentIndex(0);
  }

  presetCombo_->blockSignals(false);
}

void SlamConfigWidget::populateParameterTree() {
  parameterTree_->clear();

  if (!manager_) return;

  QMap<QString, QVariant> params = manager_->getAllParameters();

  for (auto it = params.constBegin(); it != params.constEnd(); ++it) {
    QTreeWidgetItem* item = createParameterItem(it.key(), it.value());
    parameterTree_->addTopLevelItem(item);
  }
}

QTreeWidgetItem* SlamConfigWidget::createParameterItem(const QString& name, const QVariant& value) {
  auto* item = new QTreeWidgetItem();
  item->setFlags(item->flags() | Qt::ItemIsEditable);

  item->setText(ColName, name);

  QString typeStr;
  if (value.type() == QVariant::Bool) {
    typeStr = "bool";
    item->setText(ColValue, value.toBool() ? "true" : "false");
  } else if (value.type() == QVariant::Int) {
    typeStr = "int";
    item->setText(ColValue, QString::number(value.toInt()));
  } else if (value.type() == QVariant::Double) {
    typeStr = "double";
    item->setText(ColValue, QString::number(value.toDouble(), 'f', 3));
  } else {
    typeStr = "string";
    item->setText(ColValue, value.toString());
  }
  item->setText(ColType, typeStr);

  return item;
}

void SlamConfigWidget::updateStatus() {
  if (!manager_) {
    statusLabel_->setText(tr("Status: No manager"));
    return;
  }

  if (manager_->isRunning()) {
    onSlamStarted();
  } else {
    onSlamStopped();
  }
}

}  // namespace ros_weaver
