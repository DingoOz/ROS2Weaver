#include "ros_weaver/widgets/preset_selector_widget.hpp"
#include "ros_weaver/core/parameter_preset.hpp"

#include <QHBoxLayout>
#include <QFileDialog>
#include <QTextStream>

namespace ros_weaver {

PresetSelectorWidget::PresetSelectorWidget(QWidget* parent)
  : QWidget(parent)
{
  setupUI();
  refreshPresets();

  // Connect to preset manager signals
  connect(&PresetManager::instance(), &PresetManager::activePresetChanged,
          this, &PresetSelectorWidget::onActivePresetChanged);
  connect(&PresetManager::instance(), &PresetManager::presetAdded,
          this, [this](const ParameterPreset&) { refreshPresets(); });
  connect(&PresetManager::instance(), &PresetManager::presetRemoved,
          this, [this](const QUuid&) { refreshPresets(); });
}

PresetSelectorWidget::~PresetSelectorWidget() = default;

void PresetSelectorWidget::setupUI() {
  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(4, 2, 4, 2);
  layout->setSpacing(4);

  // Label
  label_ = new QLabel(tr("Preset:"));
  layout->addWidget(label_);

  // Indicator (colored circle)
  indicator_ = new QLabel();
  indicator_->setFixedSize(12, 12);
  indicator_->setStyleSheet(
    "background-color: #808080;"
    "border-radius: 6px;"
  );
  indicator_->setToolTip(tr("No active preset"));
  layout->addWidget(indicator_);

  // Combo box
  presetCombo_ = new QComboBox();
  presetCombo_->setMinimumWidth(150);
  presetCombo_->setToolTip(tr("Select a parameter preset"));
  connect(presetCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PresetSelectorWidget::onComboIndexChanged);
  layout->addWidget(presetCombo_);

  // Edit button
  editButton_ = new QPushButton(tr("Edit..."));
  editButton_->setToolTip(tr("Edit parameter presets"));
  editButton_->setMaximumWidth(60);
  connect(editButton_, &QPushButton::clicked, this, &PresetSelectorWidget::onEditClicked);
  layout->addWidget(editButton_);

  // Export button
  exportButton_ = new QPushButton();
  exportButton_->setIcon(QIcon::fromTheme("document-save"));
  exportButton_->setToolTip(tr("Export active preset to YAML"));
  exportButton_->setMaximumWidth(30);
  connect(exportButton_, &QPushButton::clicked, this, &PresetSelectorWidget::onExportClicked);
  layout->addWidget(exportButton_);

  // Import button
  importButton_ = new QPushButton();
  importButton_->setIcon(QIcon::fromTheme("document-open"));
  importButton_->setToolTip(tr("Import preset from file"));
  importButton_->setMaximumWidth(30);
  connect(importButton_, &QPushButton::clicked, this, &PresetSelectorWidget::onImportClicked);
  layout->addWidget(importButton_);
}

void PresetSelectorWidget::refreshPresets() {
  presetCombo_->blockSignals(true);
  presetCombo_->clear();

  // Add "None" option
  presetCombo_->addItem(tr("(None)"), QVariant());

  // Add presets
  for (const ParameterPreset& preset : PresetManager::instance().presets()) {
    presetCombo_->addItem(preset.name, preset.id.toString());
  }

  // Select active preset
  ParameterPreset* active = PresetManager::instance().activePreset();
  if (active) {
    int index = presetCombo_->findData(active->id.toString());
    if (index >= 0) {
      presetCombo_->setCurrentIndex(index);
    }
  } else {
    presetCombo_->setCurrentIndex(0);
  }

  presetCombo_->blockSignals(false);
  updateIndicator();
}

void PresetSelectorWidget::onActivePresetChanged(ParameterPreset* preset) {
  presetCombo_->blockSignals(true);
  if (preset) {
    int index = presetCombo_->findData(preset->id.toString());
    if (index >= 0) {
      presetCombo_->setCurrentIndex(index);
    }
  } else {
    presetCombo_->setCurrentIndex(0);
  }
  presetCombo_->blockSignals(false);
  updateIndicator();
}

void PresetSelectorWidget::onComboIndexChanged(int index) {
  if (index <= 0) {
    PresetManager::instance().clearActivePreset();
    emit presetActivated(QString());
  } else {
    QString idStr = presetCombo_->itemData(index).toString();
    QUuid id(idStr);
    PresetManager::instance().setActivePreset(id);

    ParameterPreset* preset = PresetManager::instance().findPreset(id);
    if (preset) {
      emit presetActivated(preset->name);
    }
  }
}

void PresetSelectorWidget::onEditClicked() {
  emit editPresetsRequested();
}

void PresetSelectorWidget::onExportClicked() {
  ParameterPreset* active = PresetManager::instance().activePreset();
  if (!active) {
    return;
  }

  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Export Preset"),
    active->name.toLower().replace(' ', '_') + ".yaml",
    tr("YAML Files (*.yaml);;All Files (*)"));

  if (!fileName.isEmpty()) {
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << active->toYaml();
      file.close();
    }
  }
}

void PresetSelectorWidget::onImportClicked() {
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Import Preset"),
    QString(),
    tr("YAML Files (*.yaml);;JSON Files (*.json);;All Files (*)"));

  if (!fileName.isEmpty()) {
    if (fileName.endsWith(".yaml") || fileName.endsWith(".yml")) {
      QFile file(fileName);
      if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QString yaml = file.readAll();
        file.close();
        ParameterPreset preset = ParameterPreset::fromYaml(yaml);
        if (!preset.id.isNull()) {
          PresetManager::instance().addPreset(preset);
        }
      }
    } else {
      ParameterPreset preset = PresetManager::instance().loadPreset(fileName);
      if (!preset.id.isNull()) {
        PresetManager::instance().addPreset(preset);
      }
    }
    refreshPresets();
  }
}

void PresetSelectorWidget::updateIndicator() {
  ParameterPreset* active = PresetManager::instance().activePreset();
  if (active) {
    indicator_->setStyleSheet(QString(
      "background-color: %1;"
      "border-radius: 6px;"
    ).arg(active->color));
    indicator_->setToolTip(tr("Active: %1").arg(active->name));
  } else {
    indicator_->setStyleSheet(
      "background-color: #808080;"
      "border-radius: 6px;"
    );
    indicator_->setToolTip(tr("No active preset"));
  }
}

}  // namespace ros_weaver
