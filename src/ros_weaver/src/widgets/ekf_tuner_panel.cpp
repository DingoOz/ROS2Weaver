#include "ros_weaver/widgets/ekf_tuner_panel.hpp"
#include "ros_weaver/core/bag_manager.hpp"
#include "ros_weaver/core/ekf_simulation_engine.hpp"
#include "ros_weaver/core/ekf_metrics_analyzer.hpp"
#include "ros_weaver/core/ekf_parameter_sweep.hpp"
#include "ros_weaver/widgets/ekf_config_editor.hpp"
#include "ros_weaver/widgets/ekf_path_visualizer.hpp"
#include "ros_weaver/widgets/ekf_results_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QListWidget>

namespace ros_weaver {

EKFTunerPanel::EKFTunerPanel(QWidget* parent)
  : QWidget(parent)
{
  // Create core components
  bagManager_ = new BagManager(this);
  simulationEngine_ = new EKFSimulationEngine(this);
  metricsAnalyzer_ = new EKFMetricsAnalyzer(this);
  parameterSweep_ = new EKFParameterSweep(this);

  setupUi();
  setupConnections();
  updateUiState();
}

EKFTunerPanel::~EKFTunerPanel() {
  if (simulationEngine_->isRunning()) {
    simulationEngine_->stopSimulation();
  }
  if (parameterSweep_->isRunning()) {
    parameterSweep_->cancelSweep();
  }
}

void EKFTunerPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setupToolbar();
  mainLayout->addWidget(toolbar_);

  // Main splitter with three panels
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  setupLeftPanel();
  setupCenterPanel();
  setupRightPanel();

  mainSplitter_->addWidget(leftPanel_);
  mainSplitter_->addWidget(centerPanel_);
  mainSplitter_->addWidget(rightPanel_);

  // Set initial sizes (1:2:1 ratio)
  mainSplitter_->setSizes({250, 500, 250});

  mainLayout->addWidget(mainSplitter_, 1);
}

void EKFTunerPanel::setupToolbar() {
  toolbar_ = new QToolBar("EKF Tuner", this);
  toolbar_->setIconSize(QSize(16, 16));

  // Bag operations
  openBagButton_ = new QPushButton("Open Bag", this);
  connect(openBagButton_, &QPushButton::clicked, this, &EKFTunerPanel::onOpenBagClicked);
  toolbar_->addWidget(openBagButton_);

  toolbar_->addSeparator();

  // Config operations
  loadConfigButton_ = new QPushButton("Load Config", this);
  connect(loadConfigButton_, &QPushButton::clicked, this, &EKFTunerPanel::onLoadConfigClicked);
  toolbar_->addWidget(loadConfigButton_);

  saveConfigButton_ = new QPushButton("Save Config", this);
  connect(saveConfigButton_, &QPushButton::clicked, this, &EKFTunerPanel::onSaveConfigClicked);
  toolbar_->addWidget(saveConfigButton_);

  toolbar_->addSeparator();

  // Simulation controls
  runButton_ = new QPushButton("Run Simulation", this);
  connect(runButton_, &QPushButton::clicked, this, &EKFTunerPanel::onRunSimulationClicked);
  toolbar_->addWidget(runButton_);

  runSweepButton_ = new QPushButton("Parameter Sweep...", this);
  connect(runSweepButton_, &QPushButton::clicked, this, &EKFTunerPanel::onRunSweepClicked);
  toolbar_->addWidget(runSweepButton_);

  stopButton_ = new QPushButton("Stop", this);
  stopButton_->setEnabled(false);
  connect(stopButton_, &QPushButton::clicked, this, &EKFTunerPanel::onStopClicked);
  toolbar_->addWidget(stopButton_);

  toolbar_->addSeparator();

  // Status
  statusLabel_ = new QLabel("Ready", this);
  toolbar_->addWidget(statusLabel_);

  progressBar_ = new QProgressBar(this);
  progressBar_->setMaximumWidth(150);
  progressBar_->setVisible(false);
  toolbar_->addWidget(progressBar_);
}

void EKFTunerPanel::setupLeftPanel() {
  leftPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(leftPanel_);

  // Bag info
  auto* bagGroup = new QGroupBox("Input Bag", this);
  auto* bagLayout = new QVBoxLayout(bagGroup);

  bagInfoLabel_ = new QLabel("No bag loaded", this);
  bagInfoLabel_->setWordWrap(true);
  bagLayout->addWidget(bagInfoLabel_);

  layout->addWidget(bagGroup);

  // Config editor
  auto* configGroup = new QGroupBox("EKF Configuration", this);
  auto* configLayout = new QVBoxLayout(configGroup);

  configEditor_ = new EKFConfigEditor(this);
  configLayout->addWidget(configEditor_);

  layout->addWidget(configGroup, 1);
}

void EKFTunerPanel::setupCenterPanel() {
  centerPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(centerPanel_);
  layout->setContentsMargins(0, 0, 0, 0);

  pathVisualizer_ = new EKFPathVisualizer(this);
  layout->addWidget(pathVisualizer_);
}

void EKFTunerPanel::setupRightPanel() {
  rightPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(rightPanel_);
  layout->setContentsMargins(0, 0, 0, 0);

  resultsPanel_ = new EKFResultsPanel(this);
  layout->addWidget(resultsPanel_);
}

void EKFTunerPanel::setupConnections() {
  // Bag manager
  connect(bagManager_, &BagManager::bagOpened, this, &EKFTunerPanel::onBagOpened);
  connect(bagManager_, &BagManager::bagClosed, this, &EKFTunerPanel::onBagClosed);
  connect(bagManager_, &BagManager::errorOccurred, this, &EKFTunerPanel::onBagError);

  // Simulation engine
  connect(simulationEngine_, &EKFSimulationEngine::statusChanged,
          this, [this](EKFSimulationStatus status) {
            onSimulationStatusChanged(static_cast<int>(status));
          });
  connect(simulationEngine_, &EKFSimulationEngine::progressChanged,
          this, &EKFTunerPanel::onSimulationProgress);
  connect(simulationEngine_, &EKFSimulationEngine::simulationCompleted,
          this, &EKFTunerPanel::onSimulationFinished);
  connect(simulationEngine_, &EKFSimulationEngine::trajectoriesReady,
          this, &EKFTunerPanel::onTrajectoriesReady);
  connect(simulationEngine_, &EKFSimulationEngine::errorOccurred,
          this, [this](const QString& error) {
            QMessageBox::warning(this, "Simulation Error", error);
          });

  // Parameter sweep
  connect(parameterSweep_, &EKFParameterSweep::sweepProgress,
          this, &EKFTunerPanel::onSweepProgress);
  connect(parameterSweep_, &EKFParameterSweep::sweepCompleted,
          this, &EKFTunerPanel::onSweepFinished);
  connect(parameterSweep_, &EKFParameterSweep::configurationCompleted,
          this, &EKFTunerPanel::onSweepConfigCompleted);
  connect(parameterSweep_, &EKFParameterSweep::errorOccurred,
          this, [this](const QString& error) {
            QMessageBox::warning(this, "Sweep Error", error);
          });

  // Results panel
  connect(resultsPanel_, &EKFResultsPanel::resultSelected,
          this, &EKFTunerPanel::onResultSelected);
  connect(resultsPanel_, &EKFResultsPanel::exportBestRequested,
          this, &EKFTunerPanel::onExportBestRequested);
  connect(resultsPanel_, &EKFResultsPanel::exportAllRequested,
          this, &EKFTunerPanel::onExportAllRequested);

  // Config editor
  connect(configEditor_, &EKFConfigEditor::configChanged,
          this, &EKFTunerPanel::onConfigChanged);
}

bool EKFTunerPanel::openBag(const QString& path) {
  return bagManager_->openBag(path);
}

void EKFTunerPanel::closeBag() {
  bagManager_->closeBag();
}

bool EKFTunerPanel::isBagOpen() const {
  return bagManager_->isOpen();
}

QString EKFTunerPanel::currentBagPath() const {
  return currentBagPath_;
}

void EKFTunerPanel::setConfig(const EKFConfig& config) {
  configEditor_->setConfig(config);
}

EKFConfig EKFTunerPanel::config() const {
  return configEditor_->config();
}

bool EKFTunerPanel::loadConfig(const QString& path) {
  return configEditor_->loadFromYaml(path);
}

bool EKFTunerPanel::saveConfig(const QString& path) {
  return configEditor_->saveToYaml(path);
}

BagManager* EKFTunerPanel::bagManager() const {
  return bagManager_;
}

EKFSimulationEngine* EKFTunerPanel::simulationEngine() const {
  return simulationEngine_;
}

EKFConfigEditor* EKFTunerPanel::configEditor() const {
  return configEditor_;
}

EKFPathVisualizer* EKFTunerPanel::pathVisualizer() const {
  return pathVisualizer_;
}

EKFResultsPanel* EKFTunerPanel::resultsPanel() const {
  return resultsPanel_;
}

void EKFTunerPanel::updateUiState() {
  bool hasBag = bagManager_->isOpen();
  bool running = simulationRunning_ || sweepRunning_;

  runButton_->setEnabled(hasBag && !running);
  runSweepButton_->setEnabled(hasBag && !running);
  stopButton_->setEnabled(running);
  openBagButton_->setEnabled(!running);

  progressBar_->setVisible(running);
}

void EKFTunerPanel::onOpenBagClicked() {
  QString path = QFileDialog::getOpenFileName(this,
      "Open Rosbag", QString(),
      "Rosbag Files (*.db3 *.mcap);;All Files (*)");

  if (!path.isEmpty()) {
    // Handle folder-based sqlite3 bags
    if (path.endsWith(".db3")) {
      QFileInfo info(path);
      path = info.absolutePath();
    }
    openBag(path);
  }
}

void EKFTunerPanel::onLoadConfigClicked() {
  QString path = QFileDialog::getOpenFileName(this,
      "Load EKF Configuration", QString(),
      "YAML Files (*.yaml *.yml);;All Files (*)");

  if (!path.isEmpty()) {
    if (!loadConfig(path)) {
      QMessageBox::warning(this, "Load Error",
          "Failed to load configuration from file.");
    }
  }
}

void EKFTunerPanel::onSaveConfigClicked() {
  QString path = QFileDialog::getSaveFileName(this,
      "Save EKF Configuration", "ekf_config.yaml",
      "YAML Files (*.yaml);;All Files (*)");

  if (!path.isEmpty()) {
    if (!saveConfig(path)) {
      QMessageBox::warning(this, "Save Error",
          "Failed to save configuration to file.");
    }
  }
}

void EKFTunerPanel::onRunSimulationClicked() {
  if (!bagManager_->isOpen()) {
    QMessageBox::warning(this, "No Bag",
        "Please open a rosbag first.");
    return;
  }

  EKFConfig cfg = configEditor_->config();
  if (!cfg.isValid()) {
    QMessageBox::warning(this, "Invalid Config",
        "Please configure at least one sensor.");
    return;
  }

  simulationRunning_ = true;
  resultsPanel_->clearResults();
  pathVisualizer_->clearTrajectories();

  updateUiState();
  emit simulationStarted();

  simulationEngine_->startSimulation(cfg, currentBagPath_);
}

void EKFTunerPanel::onRunSweepClicked() {
  if (!bagManager_->isOpen()) {
    QMessageBox::warning(this, "No Bag",
        "Please open a rosbag first.");
    return;
  }

  showSweepDialog();
}

void EKFTunerPanel::onStopClicked() {
  if (simulationRunning_) {
    simulationEngine_->stopSimulation();
  }
  if (sweepRunning_) {
    parameterSweep_->cancelSweep();
  }
}

void EKFTunerPanel::showSweepDialog() {
  QDialog dialog(this);
  dialog.setWindowTitle("Parameter Sweep Configuration");
  dialog.setMinimumWidth(500);

  auto* layout = new QVBoxLayout(&dialog);

  // Parameter list
  auto* paramGroup = new QGroupBox("Parameters to Sweep", &dialog);
  auto* paramLayout = new QVBoxLayout(paramGroup);

  auto* paramList = new QListWidget(&dialog);

  // Common parameters to sweep
  QStringList params = {
    "processNoiseCovariance.0 (X position)",
    "processNoiseCovariance.1 (Y position)",
    "processNoiseCovariance.5 (Yaw)",
    "processNoiseCovariance.6 (VX)",
    "processNoiseCovariance.11 (VYaw)",
    "frequency"
  };

  for (const QString& param : params) {
    auto* item = new QListWidgetItem(param, paramList);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
  }

  paramLayout->addWidget(paramList);
  layout->addWidget(paramGroup);

  // Sweep settings
  auto* settingsGroup = new QGroupBox("Sweep Settings", &dialog);
  auto* settingsLayout = new QFormLayout(settingsGroup);

  auto* stepsSpinBox = new QSpinBox(&dialog);
  stepsSpinBox->setRange(2, 10);
  stepsSpinBox->setValue(3);
  settingsLayout->addRow("Steps per parameter:", stepsSpinBox);

  auto* logScaleCheck = new QCheckBox("Use logarithmic scale", &dialog);
  settingsLayout->addRow(logScaleCheck);

  layout->addWidget(settingsGroup);

  // Preview
  auto* previewLabel = new QLabel(&dialog);
  previewLabel->setText("Select parameters and click OK to start sweep");
  layout->addWidget(previewLabel);

  // Buttons
  auto* buttons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);

  // Update preview when selection changes
  connect(paramList, &QListWidget::itemChanged, [paramList, stepsSpinBox, previewLabel]() {
    int selected = 0;
    for (int i = 0; i < paramList->count(); ++i) {
      if (paramList->item(i)->checkState() == Qt::Checked) {
        selected++;
      }
    }

    int total = 1;
    for (int i = 0; i < selected; ++i) {
      total *= stepsSpinBox->value();
    }

    previewLabel->setText(QString("Total configurations: %1").arg(total));
  });

  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  // Build sweep parameters
  parameterSweep_->clearSweepParameters();
  parameterSweep_->setBaseConfig(configEditor_->config());
  parameterSweep_->setInputBagPath(currentBagPath_);

  for (int i = 0; i < paramList->count(); ++i) {
    if (paramList->item(i)->checkState() != Qt::Checked) {
      continue;
    }

    QString paramText = paramList->item(i)->text();
    QString paramName = paramText.split(" ").first();

    EKFSweepParameter param;
    param.name = paramName;
    param.steps = stepsSpinBox->value();
    param.logScale = logScaleCheck->isChecked();

    // Set reasonable ranges based on parameter
    if (paramName.startsWith("processNoiseCovariance")) {
      param.minValue = 0.001;
      param.maxValue = 1.0;
    } else if (paramName == "frequency") {
      param.minValue = 10.0;
      param.maxValue = 100.0;
      param.logScale = false;
    } else {
      param.minValue = 0.01;
      param.maxValue = 1.0;
    }

    parameterSweep_->addSweepParameter(param);
  }

  if (parameterSweep_->sweepParameters().isEmpty()) {
    QMessageBox::warning(this, "No Parameters",
        "Please select at least one parameter to sweep.");
    return;
  }

  // Start sweep
  sweepRunning_ = true;
  resultsPanel_->clearResults();
  pathVisualizer_->clearTrajectories();

  updateUiState();

  int total = parameterSweep_->totalConfigurations();
  emit sweepStarted(total);
  statusLabel_->setText(QString("Sweep: 0/%1").arg(total));

  parameterSweep_->startSweep();
}

void EKFTunerPanel::onBagOpened() {
  auto meta = bagManager_->metadata();
  currentBagPath_ = meta.path;

  bagInfoLabel_->setText(QString(
      "<b>%1</b><br>"
      "Duration: %2 s<br>"
      "Messages: %3<br>"
      "Topics: %4")
      .arg(QFileInfo(meta.path).fileName())
      .arg(meta.durationNs / 1e9, 0, 'f', 1)
      .arg(meta.messageCount)
      .arg(meta.topicCount));

  updateUiState();
  emit bagOpened(currentBagPath_);
}

void EKFTunerPanel::onBagClosed() {
  currentBagPath_.clear();
  bagInfoLabel_->setText("No bag loaded");
  updateUiState();
  emit bagClosed();
}

void EKFTunerPanel::onBagError(const QString& error) {
  QMessageBox::warning(this, "Bag Error", error);
}

void EKFTunerPanel::onSimulationStatusChanged(int status) {
  EKFSimulationStatus s = static_cast<EKFSimulationStatus>(status);

  switch (s) {
    case EKFSimulationStatus::Idle:
      statusLabel_->setText("Ready");
      break;
    case EKFSimulationStatus::Preparing:
      statusLabel_->setText("Preparing...");
      break;
    case EKFSimulationStatus::LaunchingEKF:
      statusLabel_->setText("Launching EKF...");
      break;
    case EKFSimulationStatus::Recording:
      statusLabel_->setText("Recording...");
      break;
    case EKFSimulationStatus::Playing:
      statusLabel_->setText("Playing bag...");
      break;
    case EKFSimulationStatus::Stopping:
      statusLabel_->setText("Stopping...");
      break;
    case EKFSimulationStatus::Analyzing:
      statusLabel_->setText("Analyzing...");
      break;
    case EKFSimulationStatus::Completed:
      statusLabel_->setText("Completed");
      break;
    case EKFSimulationStatus::Error:
      statusLabel_->setText("Error");
      break;
  }
}

void EKFTunerPanel::onSimulationProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  statusLabel_->setText(message);
}

void EKFTunerPanel::onSimulationFinished(bool success) {
  simulationRunning_ = false;
  updateUiState();

  if (success) {
    statusLabel_->setText("Simulation completed");
  } else {
    statusLabel_->setText("Simulation failed");
  }

  emit simulationCompleted(success);
}

void EKFTunerPanel::onTrajectoriesReady(const Trajectory& ekf,
                                         const Trajectory& rawOdom,
                                         const Trajectory& groundTruth) {
  lastGroundTruth_ = groundTruth;

  // Display trajectories
  pathVisualizer_->setTrajectories(ekf, rawOdom, groundTruth);

  // Compute and display metrics
  EKFMetrics metrics;
  if (!groundTruth.poses.isEmpty()) {
    metrics = metricsAnalyzer_->computeMetrics(ekf, groundTruth);
  } else {
    metrics = metricsAnalyzer_->computeRelativeMetrics(ekf, rawOdom);
  }

  resultsPanel_->showSingleResult(metrics);
}

void EKFTunerPanel::onSweepProgress(int completed, int total, const QString& message) {
  progressBar_->setMaximum(total);
  progressBar_->setValue(completed);
  statusLabel_->setText(QString("Sweep: %1/%2 - %3").arg(completed).arg(total).arg(message));
}

void EKFTunerPanel::onSweepFinished(bool success) {
  sweepRunning_ = false;
  updateUiState();

  if (success) {
    statusLabel_->setText(QString("Sweep completed: %1 configurations")
        .arg(parameterSweep_->results().size()));
  } else {
    statusLabel_->setText("Sweep cancelled/failed");
  }

  emit sweepCompleted(success);
}

void EKFTunerPanel::onSweepConfigCompleted(int /*index*/, const EKFSweepResult& result) {
  resultsPanel_->addResult(result);
}

void EKFTunerPanel::onResultSelected(int /*index*/, const EKFSweepResult& result) {
  // Show selected config
  configEditor_->setConfig(result.config);

  // Highlight in status
  statusLabel_->setText(QString("Selected: %1 (ATE RMSE: %2 m)")
      .arg(result.metrics.configName)
      .arg(result.metrics.ateRmse, 0, 'f', 4));
}

void EKFTunerPanel::onExportBestRequested() {
  QString path = QFileDialog::getSaveFileName(this,
      "Export Best Configuration", "best_ekf_config.yaml",
      "YAML Files (*.yaml);;All Files (*)");

  if (!path.isEmpty()) {
    if (parameterSweep_->exportBestConfig(path)) {
      QMessageBox::information(this, "Export Successful",
          "Best configuration exported successfully.");
    } else {
      QMessageBox::warning(this, "Export Failed",
          "Failed to export configuration.");
    }
  }
}

void EKFTunerPanel::onExportAllRequested() {
  QString path = QFileDialog::getSaveFileName(this,
      "Export All Results", "ekf_sweep_results.csv",
      "CSV Files (*.csv);;All Files (*)");

  if (!path.isEmpty()) {
    if (parameterSweep_->exportResultsCSV(path)) {
      QMessageBox::information(this, "Export Successful",
          "Results exported successfully.");
    } else {
      QMessageBox::warning(this, "Export Failed",
          "Failed to export results.");
    }
  }
}

void EKFTunerPanel::onConfigChanged() {
  // Config was modified - could trigger auto-validation here
}

}  // namespace ros_weaver
