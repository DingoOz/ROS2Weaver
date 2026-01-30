#include "ros_weaver/widgets/ekf_config_editor.hpp"
#include "ros_weaver/core/ekf_simulation_engine.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QHeaderView>
#include <QScrollArea>
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>

namespace ros_weaver {

EKFConfigEditor::EKFConfigEditor(QWidget* parent)
  : QWidget(parent)
{
  setupUi();
  setConfig(EKFConfig::defaultConfig());
}

EKFConfigEditor::~EKFConfigEditor() = default;

EKFConfig EKFConfigEditor::config() const {
  return config_;
}

void EKFConfigEditor::setConfig(const EKFConfig& config) {
  config_ = config;
  updateUiFromConfig();
}

bool EKFConfigEditor::loadFromYaml(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }

  QString yaml = file.readAll();
  file.close();

  return fromYaml(yaml);
}

bool EKFConfigEditor::saveToYaml(const QString& path) {
  QString yaml = toYaml();

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QTextStream stream(&file);
  stream << yaml;
  file.close();

  return true;
}

QString EKFConfigEditor::toYaml() const {
  return EKFSimulationEngine::configToYaml(config_);
}

bool EKFConfigEditor::fromYaml(const QString& yaml) {
  EKFConfig config = EKFSimulationEngine::yamlToConfig(yaml);
  if (config.isValid() || config.name.isEmpty()) {
    // Even invalid config might have parsed some fields
    setConfig(config);
    return true;
  }
  return false;
}

void EKFConfigEditor::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setupToolbar();

  tabWidget_ = new QTabWidget(this);
  mainLayout->addWidget(tabWidget_);

  setupGeneralTab();
  setupCovarianceTab();
  setupSensorsTab();
}

void EKFConfigEditor::setupToolbar() {
  auto* toolbar = new QHBoxLayout();

  loadButton_ = new QPushButton("Load YAML", this);
  saveButton_ = new QPushButton("Save YAML", this);

  connect(loadButton_, &QPushButton::clicked, this, &EKFConfigEditor::onLoadClicked);
  connect(saveButton_, &QPushButton::clicked, this, &EKFConfigEditor::onSaveClicked);

  toolbar->addWidget(loadButton_);
  toolbar->addWidget(saveButton_);
  toolbar->addStretch();

  static_cast<QVBoxLayout*>(layout())->addLayout(toolbar);
}

void EKFConfigEditor::setupGeneralTab() {
  generalTab_ = new QWidget(this);
  auto* layout = new QFormLayout(generalTab_);

  nameEdit_ = new QLineEdit(this);
  connect(nameEdit_, &QLineEdit::textChanged, this, &EKFConfigEditor::onGeneralParamChanged);
  layout->addRow("Node Name:", nameEdit_);

  frequencySpinBox_ = new QDoubleSpinBox(this);
  frequencySpinBox_->setRange(1.0, 500.0);
  frequencySpinBox_->setSuffix(" Hz");
  connect(frequencySpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &EKFConfigEditor::onGeneralParamChanged);
  layout->addRow("Frequency:", frequencySpinBox_);

  // Frame IDs
  auto* framesGroup = new QGroupBox("Frame IDs", this);
  auto* framesLayout = new QFormLayout(framesGroup);

  mapFrameEdit_ = new QLineEdit(this);
  connect(mapFrameEdit_, &QLineEdit::textChanged, this, &EKFConfigEditor::onGeneralParamChanged);
  framesLayout->addRow("Map Frame:", mapFrameEdit_);

  odomFrameEdit_ = new QLineEdit(this);
  connect(odomFrameEdit_, &QLineEdit::textChanged, this, &EKFConfigEditor::onGeneralParamChanged);
  framesLayout->addRow("Odom Frame:", odomFrameEdit_);

  baseFrameEdit_ = new QLineEdit(this);
  connect(baseFrameEdit_, &QLineEdit::textChanged, this, &EKFConfigEditor::onGeneralParamChanged);
  framesLayout->addRow("Base Frame:", baseFrameEdit_);

  worldFrameCombo_ = new QComboBox(this);
  worldFrameCombo_->addItem("odom");
  worldFrameCombo_->addItem("map");
  connect(worldFrameCombo_, &QComboBox::currentTextChanged,
          this, &EKFConfigEditor::onGeneralParamChanged);
  framesLayout->addRow("World Frame:", worldFrameCombo_);

  layout->addRow(framesGroup);

  // Options
  auto* optionsGroup = new QGroupBox("Options", this);
  auto* optionsLayout = new QVBoxLayout(optionsGroup);

  publishTfCheck_ = new QCheckBox("Publish TF", this);
  connect(publishTfCheck_, &QCheckBox::toggled, this, &EKFConfigEditor::onGeneralParamChanged);
  optionsLayout->addWidget(publishTfCheck_);

  twoDModeCheck_ = new QCheckBox("2D Mode (constrain to planar motion)", this);
  connect(twoDModeCheck_, &QCheckBox::toggled, this, &EKFConfigEditor::onGeneralParamChanged);
  optionsLayout->addWidget(twoDModeCheck_);

  layout->addRow(optionsGroup);

  // Transform settings
  auto* transformGroup = new QGroupBox("Transform Settings", this);
  auto* transformLayout = new QFormLayout(transformGroup);

  transformTimeoutSpinBox_ = new QDoubleSpinBox(this);
  transformTimeoutSpinBox_->setRange(0.0, 10.0);
  transformTimeoutSpinBox_->setDecimals(3);
  transformTimeoutSpinBox_->setSuffix(" s");
  connect(transformTimeoutSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &EKFConfigEditor::onGeneralParamChanged);
  transformLayout->addRow("Transform Timeout:", transformTimeoutSpinBox_);

  layout->addRow(transformGroup);

  tabWidget_->addTab(generalTab_, "General");
}

void EKFConfigEditor::setupCovarianceTab() {
  covarianceTab_ = new QWidget(this);
  auto* layout = new QVBoxLayout(covarianceTab_);

  // State labels for headers
  QStringList stateLabels = {
    "X", "Y", "Z", "Roll", "Pitch", "Yaw",
    "VX", "VY", "VZ", "VRoll", "VPitch", "VYaw",
    "AX", "AY", "AZ"
  };

  // Process noise covariance
  auto* processNoiseGroup = new QGroupBox("Process Noise Covariance (diagonal)", this);
  auto* processNoiseLayout = new QVBoxLayout(processNoiseGroup);

  processNoiseTable_ = new QTableWidget(EKF_STATE_SIZE, 2, this);
  processNoiseTable_->setHorizontalHeaderLabels({"State", "Value"});
  processNoiseTable_->horizontalHeader()->setStretchLastSection(true);
  processNoiseTable_->setColumnWidth(0, 80);

  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    auto* labelItem = new QTableWidgetItem(stateLabels[i]);
    labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
    processNoiseTable_->setItem(i, 0, labelItem);

    auto* valueItem = new QTableWidgetItem("0.0");
    processNoiseTable_->setItem(i, 1, valueItem);
  }

  connect(processNoiseTable_, &QTableWidget::cellChanged,
          this, &EKFConfigEditor::onProcessNoiseChanged);

  processNoiseLayout->addWidget(processNoiseTable_);
  layout->addWidget(processNoiseGroup);

  // Initial estimate covariance
  auto* initialCovGroup = new QGroupBox("Initial Estimate Covariance (diagonal)", this);
  auto* initialCovLayout = new QVBoxLayout(initialCovGroup);

  initialCovarianceTable_ = new QTableWidget(EKF_STATE_SIZE, 2, this);
  initialCovarianceTable_->setHorizontalHeaderLabels({"State", "Value"});
  initialCovarianceTable_->horizontalHeader()->setStretchLastSection(true);
  initialCovarianceTable_->setColumnWidth(0, 80);

  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    auto* labelItem = new QTableWidgetItem(stateLabels[i]);
    labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
    initialCovarianceTable_->setItem(i, 0, labelItem);

    auto* valueItem = new QTableWidgetItem("0.0");
    initialCovarianceTable_->setItem(i, 1, valueItem);
  }

  connect(initialCovarianceTable_, &QTableWidget::cellChanged,
          this, &EKFConfigEditor::onInitialCovarianceChanged);

  initialCovLayout->addWidget(initialCovarianceTable_);
  layout->addWidget(initialCovGroup);

  tabWidget_->addTab(covarianceTab_, "Covariance");
}

void EKFConfigEditor::setupSensorsTab() {
  sensorsTab_ = new QWidget(this);
  auto* layout = new QVBoxLayout(sensorsTab_);

  sensorTabs_ = new QTabWidget(this);

  // Odom sensors
  auto* odomScroll = new QScrollArea(this);
  odomScroll->setWidgetResizable(true);
  odomSensorsWidget_ = new QWidget(this);
  odomSensorsWidget_->setLayout(new QVBoxLayout());
  odomScroll->setWidget(odomSensorsWidget_);

  auto* odomContainer = new QWidget(this);
  auto* odomLayout = new QVBoxLayout(odomContainer);
  odomLayout->addWidget(odomScroll);

  auto* addOdomBtn = new QPushButton("Add Odometry Sensor", this);
  connect(addOdomBtn, &QPushButton::clicked, this, &EKFConfigEditor::onAddOdomSensor);
  odomLayout->addWidget(addOdomBtn);

  sensorTabs_->addTab(odomContainer, "Odometry");

  // IMU sensors
  auto* imuScroll = new QScrollArea(this);
  imuScroll->setWidgetResizable(true);
  imuSensorsWidget_ = new QWidget(this);
  imuSensorsWidget_->setLayout(new QVBoxLayout());
  imuScroll->setWidget(imuSensorsWidget_);

  auto* imuContainer = new QWidget(this);
  auto* imuLayout = new QVBoxLayout(imuContainer);
  imuLayout->addWidget(imuScroll);

  auto* addImuBtn = new QPushButton("Add IMU Sensor", this);
  connect(addImuBtn, &QPushButton::clicked, this, &EKFConfigEditor::onAddImuSensor);
  imuLayout->addWidget(addImuBtn);

  sensorTabs_->addTab(imuContainer, "IMU");

  // Pose sensors
  auto* poseScroll = new QScrollArea(this);
  poseScroll->setWidgetResizable(true);
  poseSensorsWidget_ = new QWidget(this);
  poseSensorsWidget_->setLayout(new QVBoxLayout());
  poseScroll->setWidget(poseSensorsWidget_);

  auto* poseContainer = new QWidget(this);
  auto* poseLayout = new QVBoxLayout(poseContainer);
  poseLayout->addWidget(poseScroll);

  auto* addPoseBtn = new QPushButton("Add Pose Sensor", this);
  connect(addPoseBtn, &QPushButton::clicked, this, &EKFConfigEditor::onAddPoseSensor);
  poseLayout->addWidget(addPoseBtn);

  sensorTabs_->addTab(poseContainer, "Pose");

  // Twist sensors
  auto* twistScroll = new QScrollArea(this);
  twistScroll->setWidgetResizable(true);
  twistSensorsWidget_ = new QWidget(this);
  twistSensorsWidget_->setLayout(new QVBoxLayout());
  twistScroll->setWidget(twistSensorsWidget_);

  auto* twistContainer = new QWidget(this);
  auto* twistLayout = new QVBoxLayout(twistContainer);
  twistLayout->addWidget(twistScroll);

  auto* addTwistBtn = new QPushButton("Add Twist Sensor", this);
  connect(addTwistBtn, &QPushButton::clicked, this, &EKFConfigEditor::onAddTwistSensor);
  twistLayout->addWidget(addTwistBtn);

  sensorTabs_->addTab(twistContainer, "Twist");

  layout->addWidget(sensorTabs_);
  tabWidget_->addTab(sensorsTab_, "Sensors");
}

QWidget* EKFConfigEditor::createSensorWidget(const QString& type, int index) {
  auto* group = new QGroupBox(QString("%1 Sensor %2").arg(type).arg(index), this);
  auto* layout = new QVBoxLayout(group);

  // Topic
  auto* topicLayout = new QHBoxLayout();
  topicLayout->addWidget(new QLabel("Topic:"));
  auto* topicEdit = new QLineEdit(this);
  topicEdit->setObjectName(QString("%1_%2_topic").arg(type).arg(index));
  connect(topicEdit, &QLineEdit::textChanged, this, &EKFConfigEditor::onSensorConfigChanged);
  topicLayout->addWidget(topicEdit);

  auto* enabledCheck = new QCheckBox("Enabled", this);
  enabledCheck->setObjectName(QString("%1_%2_enabled").arg(type).arg(index));
  connect(enabledCheck, &QCheckBox::toggled, this, &EKFConfigEditor::onSensorConfigChanged);
  topicLayout->addWidget(enabledCheck);

  layout->addLayout(topicLayout);

  // State fusion checkboxes
  auto* stateGroup = new QGroupBox("Fuse States", this);
  auto* stateLayout = new QGridLayout(stateGroup);

  QStringList stateLabels = {
    "X", "Y", "Z", "Roll", "Pitch", "Yaw",
    "VX", "VY", "VZ", "VRoll", "VPitch", "VYaw",
    "AX", "AY", "AZ"
  };

  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    auto* check = new QCheckBox(stateLabels[i], this);
    check->setObjectName(QString("%1_%2_state_%3").arg(type).arg(index).arg(i));
    connect(check, &QCheckBox::toggled, this, &EKFConfigEditor::onSensorConfigChanged);
    stateLayout->addWidget(check, i / 6, i % 6);
  }

  layout->addWidget(stateGroup);

  // Options
  auto* optionsLayout = new QHBoxLayout();

  auto* differentialCheck = new QCheckBox("Differential", this);
  differentialCheck->setObjectName(QString("%1_%2_differential").arg(type).arg(index));
  connect(differentialCheck, &QCheckBox::toggled, this, &EKFConfigEditor::onSensorConfigChanged);
  optionsLayout->addWidget(differentialCheck);

  auto* relativeCheck = new QCheckBox("Relative", this);
  relativeCheck->setObjectName(QString("%1_%2_relative").arg(type).arg(index));
  connect(relativeCheck, &QCheckBox::toggled, this, &EKFConfigEditor::onSensorConfigChanged);
  optionsLayout->addWidget(relativeCheck);

  optionsLayout->addStretch();

  auto* removeBtn = new QPushButton("Remove", this);
  removeBtn->setProperty("sensor_type", type);
  removeBtn->setProperty("sensor_index", index);
  connect(removeBtn, &QPushButton::clicked, this, &EKFConfigEditor::onRemoveSensor);
  optionsLayout->addWidget(removeBtn);

  layout->addLayout(optionsLayout);

  return group;
}

void EKFConfigEditor::updateSensorWidgets() {
  // Clear existing widgets
  auto clearLayout = [](QWidget* container) {
    auto* layout = container->layout();
    while (layout->count() > 0) {
      auto* item = layout->takeAt(0);
      if (item->widget()) {
        delete item->widget();
      }
      delete item;
    }
  };

  clearLayout(odomSensorsWidget_);
  clearLayout(imuSensorsWidget_);
  clearLayout(poseSensorsWidget_);
  clearLayout(twistSensorsWidget_);

  // Recreate for current config
  for (int i = 0; i < config_.odomSensors.size(); ++i) {
    auto* widget = createSensorWidget("odom", i);
    odomSensorsWidget_->layout()->addWidget(widget);

    // Populate values
    const auto& sensor = config_.odomSensors[i];
    widget->findChild<QLineEdit*>(QString("odom_%1_topic").arg(i))->setText(sensor.topic);
    widget->findChild<QCheckBox*>(QString("odom_%1_enabled").arg(i))->setChecked(sensor.enabled);
    widget->findChild<QCheckBox*>(QString("odom_%1_differential").arg(i))->setChecked(sensor.differential);
    widget->findChild<QCheckBox*>(QString("odom_%1_relative").arg(i))->setChecked(sensor.relative);

    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      auto* check = widget->findChild<QCheckBox*>(QString("odom_%1_state_%2").arg(i).arg(j));
      if (check) check->setChecked(sensor.config[j]);
    }
  }

  for (int i = 0; i < config_.imuSensors.size(); ++i) {
    auto* widget = createSensorWidget("imu", i);
    imuSensorsWidget_->layout()->addWidget(widget);

    const auto& sensor = config_.imuSensors[i];
    widget->findChild<QLineEdit*>(QString("imu_%1_topic").arg(i))->setText(sensor.topic);
    widget->findChild<QCheckBox*>(QString("imu_%1_enabled").arg(i))->setChecked(sensor.enabled);
    widget->findChild<QCheckBox*>(QString("imu_%1_differential").arg(i))->setChecked(sensor.differential);
    widget->findChild<QCheckBox*>(QString("imu_%1_relative").arg(i))->setChecked(sensor.relative);

    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      auto* check = widget->findChild<QCheckBox*>(QString("imu_%1_state_%2").arg(i).arg(j));
      if (check) check->setChecked(sensor.config[j]);
    }
  }

  for (int i = 0; i < config_.poseSensors.size(); ++i) {
    auto* widget = createSensorWidget("pose", i);
    poseSensorsWidget_->layout()->addWidget(widget);

    const auto& sensor = config_.poseSensors[i];
    widget->findChild<QLineEdit*>(QString("pose_%1_topic").arg(i))->setText(sensor.topic);
    widget->findChild<QCheckBox*>(QString("pose_%1_enabled").arg(i))->setChecked(sensor.enabled);
    widget->findChild<QCheckBox*>(QString("pose_%1_differential").arg(i))->setChecked(sensor.differential);
    widget->findChild<QCheckBox*>(QString("pose_%1_relative").arg(i))->setChecked(sensor.relative);

    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      auto* check = widget->findChild<QCheckBox*>(QString("pose_%1_state_%2").arg(i).arg(j));
      if (check) check->setChecked(sensor.config[j]);
    }
  }

  for (int i = 0; i < config_.twistSensors.size(); ++i) {
    auto* widget = createSensorWidget("twist", i);
    twistSensorsWidget_->layout()->addWidget(widget);

    const auto& sensor = config_.twistSensors[i];
    widget->findChild<QLineEdit*>(QString("twist_%1_topic").arg(i))->setText(sensor.topic);
    widget->findChild<QCheckBox*>(QString("twist_%1_enabled").arg(i))->setChecked(sensor.enabled);

    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      auto* check = widget->findChild<QCheckBox*>(QString("twist_%1_state_%2").arg(i).arg(j));
      if (check) check->setChecked(sensor.config[j]);
    }
  }

  // Add spacers
  static_cast<QVBoxLayout*>(odomSensorsWidget_->layout())->addStretch();
  static_cast<QVBoxLayout*>(imuSensorsWidget_->layout())->addStretch();
  static_cast<QVBoxLayout*>(poseSensorsWidget_->layout())->addStretch();
  static_cast<QVBoxLayout*>(twistSensorsWidget_->layout())->addStretch();
}

void EKFConfigEditor::updateUiFromConfig() {
  updatingUi_ = true;

  // General tab
  nameEdit_->setText(config_.name);
  frequencySpinBox_->setValue(config_.frequency);
  mapFrameEdit_->setText(config_.mapFrame);
  odomFrameEdit_->setText(config_.odomFrame);
  baseFrameEdit_->setText(config_.baseFrame);
  worldFrameCombo_->setCurrentText(config_.worldFrame);
  publishTfCheck_->setChecked(config_.publishTf);
  twoDModeCheck_->setChecked(config_.twoDMode);
  transformTimeoutSpinBox_->setValue(config_.transformTimeout);

  // Covariance tables
  populateCovarianceTable(processNoiseTable_, config_.processNoiseCovariance);
  populateCovarianceTable(initialCovarianceTable_, config_.initialEstimateCovariance);

  // Sensors
  updateSensorWidgets();

  updatingUi_ = false;
}

void EKFConfigEditor::updateConfigFromUi() {
  // General
  config_.name = nameEdit_->text();
  config_.frequency = frequencySpinBox_->value();
  config_.mapFrame = mapFrameEdit_->text();
  config_.odomFrame = odomFrameEdit_->text();
  config_.baseFrame = baseFrameEdit_->text();
  config_.worldFrame = worldFrameCombo_->currentText();
  config_.publishTf = publishTfCheck_->isChecked();
  config_.twoDMode = twoDModeCheck_->isChecked();
  config_.transformTimeout = transformTimeoutSpinBox_->value();

  // Covariances
  readCovarianceTable(processNoiseTable_, config_.processNoiseCovariance);
  readCovarianceTable(initialCovarianceTable_, config_.initialEstimateCovariance);
}

void EKFConfigEditor::populateCovarianceTable(QTableWidget* table,
                                               const std::array<double, EKF_STATE_SIZE>& values) {
  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    table->item(i, 1)->setText(QString::number(values[i], 'g', 6));
  }
}

void EKFConfigEditor::readCovarianceTable(QTableWidget* table,
                                           std::array<double, EKF_STATE_SIZE>& values) {
  for (int i = 0; i < EKF_STATE_SIZE; ++i) {
    values[i] = table->item(i, 1)->text().toDouble();
  }
}

void EKFConfigEditor::onGeneralParamChanged() {
  if (updatingUi_) return;
  updateConfigFromUi();
  emit configChanged();
}

void EKFConfigEditor::onProcessNoiseChanged(int row, int col) {
  if (updatingUi_ || col != 1) return;
  if (row >= 0 && row < EKF_STATE_SIZE) {
    config_.processNoiseCovariance[row] = processNoiseTable_->item(row, 1)->text().toDouble();
    emit configChanged();
  }
}

void EKFConfigEditor::onInitialCovarianceChanged(int row, int col) {
  if (updatingUi_ || col != 1) return;
  if (row >= 0 && row < EKF_STATE_SIZE) {
    config_.initialEstimateCovariance[row] = initialCovarianceTable_->item(row, 1)->text().toDouble();
    emit configChanged();
  }
}

void EKFConfigEditor::onSensorConfigChanged() {
  if (updatingUi_) return;

  // Update sensor configs from UI
  auto readSensorFromWidget = [this](const QString& type, int index, EKFSensorConfig& sensor) {
    auto* topicEdit = findChild<QLineEdit*>(QString("%1_%2_topic").arg(type).arg(index));
    auto* enabledCheck = findChild<QCheckBox*>(QString("%1_%2_enabled").arg(type).arg(index));
    auto* diffCheck = findChild<QCheckBox*>(QString("%1_%2_differential").arg(type).arg(index));
    auto* relCheck = findChild<QCheckBox*>(QString("%1_%2_relative").arg(type).arg(index));

    if (topicEdit) sensor.topic = topicEdit->text();
    if (enabledCheck) sensor.enabled = enabledCheck->isChecked();
    if (diffCheck) sensor.differential = diffCheck->isChecked();
    if (relCheck) sensor.relative = relCheck->isChecked();

    for (int j = 0; j < EKF_STATE_SIZE; ++j) {
      auto* check = findChild<QCheckBox*>(QString("%1_%2_state_%3").arg(type).arg(index).arg(j));
      if (check) sensor.config[j] = check->isChecked();
    }
  };

  for (int i = 0; i < config_.odomSensors.size(); ++i) {
    readSensorFromWidget("odom", i, config_.odomSensors[i]);
  }
  for (int i = 0; i < config_.imuSensors.size(); ++i) {
    readSensorFromWidget("imu", i, config_.imuSensors[i]);
  }
  for (int i = 0; i < config_.poseSensors.size(); ++i) {
    readSensorFromWidget("pose", i, config_.poseSensors[i]);
  }
  for (int i = 0; i < config_.twistSensors.size(); ++i) {
    readSensorFromWidget("twist", i, config_.twistSensors[i]);
  }

  emit configChanged();
}

void EKFConfigEditor::onAddOdomSensor() {
  EKFSensorConfig sensor;
  sensor.topic = QString("/odom%1").arg(config_.odomSensors.size());
  config_.odomSensors.append(sensor);
  updateSensorWidgets();
  emit configChanged();
}

void EKFConfigEditor::onAddImuSensor() {
  EKFSensorConfig sensor;
  sensor.topic = QString("/imu/data%1").arg(config_.imuSensors.isEmpty() ? "" :
                                            QString::number(config_.imuSensors.size()));
  config_.imuSensors.append(sensor);
  updateSensorWidgets();
  emit configChanged();
}

void EKFConfigEditor::onAddPoseSensor() {
  EKFSensorConfig sensor;
  sensor.topic = QString("/pose%1").arg(config_.poseSensors.size());
  config_.poseSensors.append(sensor);
  updateSensorWidgets();
  emit configChanged();
}

void EKFConfigEditor::onAddTwistSensor() {
  EKFSensorConfig sensor;
  sensor.topic = QString("/twist%1").arg(config_.twistSensors.size());
  config_.twistSensors.append(sensor);
  updateSensorWidgets();
  emit configChanged();
}

void EKFConfigEditor::onRemoveSensor() {
  auto* btn = qobject_cast<QPushButton*>(sender());
  if (!btn) return;

  QString type = btn->property("sensor_type").toString();
  int index = btn->property("sensor_index").toInt();

  if (type == "odom" && index < config_.odomSensors.size()) {
    config_.odomSensors.remove(index);
  } else if (type == "imu" && index < config_.imuSensors.size()) {
    config_.imuSensors.remove(index);
  } else if (type == "pose" && index < config_.poseSensors.size()) {
    config_.poseSensors.remove(index);
  } else if (type == "twist" && index < config_.twistSensors.size()) {
    config_.twistSensors.remove(index);
  }

  updateSensorWidgets();
  emit configChanged();
}

void EKFConfigEditor::onLoadClicked() {
  QString path = QFileDialog::getOpenFileName(this,
      "Load EKF Configuration", QString(), "YAML Files (*.yaml *.yml)");

  if (!path.isEmpty()) {
    if (loadFromYaml(path)) {
      emit loadRequested();
    } else {
      QMessageBox::warning(this, "Load Failed",
          "Failed to load configuration from file.");
    }
  }
}

void EKFConfigEditor::onSaveClicked() {
  QString path = QFileDialog::getSaveFileName(this,
      "Save EKF Configuration", "ekf_config.yaml", "YAML Files (*.yaml)");

  if (!path.isEmpty()) {
    if (saveToYaml(path)) {
      emit saveRequested();
    } else {
      QMessageBox::warning(this, "Save Failed",
          "Failed to save configuration to file.");
    }
  }
}

}  // namespace ros_weaver
