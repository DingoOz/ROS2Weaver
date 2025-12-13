#include "ros_weaver/widgets/ollama_settings_widget.hpp"
#include "ros_weaver/widgets/local_ai_status_widget.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include <QStyle>
#include <QDesktopServices>
#include <QUrl>

namespace ros_weaver {

OllamaSettingsWidget::OllamaSettingsWidget(QWidget* parent)
    : QWidget(parent)
    , connectionTestTimer_(new QTimer(this)) {
  setupUi();
  connectSignals();

  // Setup connection test timeout timer
  connectionTestTimer_->setSingleShot(true);
  connect(connectionTestTimer_, &QTimer::timeout,
          this, &OllamaSettingsWidget::onTestConnectionTimeout);

  // Initialize state from manager
  resetToSaved();

  // Trigger initial status check
  OllamaManager::instance().checkOllamaStatus();
}

void OllamaSettingsWidget::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(16);

  // Enable checkbox at top
  enableOllamaCheck_ = new QCheckBox(tr("Enable Local LLM (Ollama)"), this);
  enableOllamaCheck_->setToolTip(tr("Enable or disable the local LLM integration"));
  mainLayout->addWidget(enableOllamaCheck_);

  // Connection Group
  connectionGroup_ = new QGroupBox(tr("Ollama Connection"), this);
  QVBoxLayout* connLayout = new QVBoxLayout(connectionGroup_);

  QHBoxLayout* statusLayout = new QHBoxLayout();
  statusIcon_ = new QLabel(this);
  statusIcon_->setFixedSize(16, 16);
  statusLabel_ = new QLabel(tr("Checking..."), this);
  statusLayout->addWidget(statusIcon_);
  statusLayout->addWidget(statusLabel_);
  statusLayout->addStretch();
  connLayout->addLayout(statusLayout);

  QFormLayout* endpointLayout = new QFormLayout();
  endpointEdit_ = new QLineEdit(this);
  endpointEdit_->setPlaceholderText("http://localhost:11434");
  endpointLayout->addRow(tr("Endpoint URL:"), endpointEdit_);
  connLayout->addLayout(endpointLayout);

  QHBoxLayout* connBtnLayout = new QHBoxLayout();
  testConnectionBtn_ = new QPushButton(tr("Test Connection"), this);
  connBtnLayout->addWidget(testConnectionBtn_);
  connBtnLayout->addStretch();
  connLayout->addLayout(connBtnLayout);

  mainLayout->addWidget(connectionGroup_);

  // Model Selection Group
  modelGroup_ = new QGroupBox(tr("Active Model"), this);
  QVBoxLayout* modelLayout = new QVBoxLayout(modelGroup_);

  QHBoxLayout* modelSelectLayout = new QHBoxLayout();
  modelCombo_ = new QComboBox(this);
  modelCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  modelCombo_->setMinimumWidth(200);
  refreshBtn_ = new QPushButton(tr("Refresh"), this);
  modelSelectLayout->addWidget(new QLabel(tr("Selected Model:"), this));
  modelSelectLayout->addWidget(modelCombo_);
  modelSelectLayout->addWidget(refreshBtn_);
  modelLayout->addLayout(modelSelectLayout);

  autoLoadCheck_ = new QCheckBox(tr("Automatically use this model on next launch"), this);
  autoLoadCheck_->setToolTip(tr("When enabled, the selected model will be automatically loaded when the application starts"));
  modelLayout->addWidget(autoLoadCheck_);

  mainLayout->addWidget(modelGroup_);

  // Installed Models Management Group
  managementGroup_ = new QGroupBox(tr("Installed Models"), this);
  QVBoxLayout* mgmtLayout = new QVBoxLayout(managementGroup_);

  installedModelsList_ = new QListWidget(this);
  installedModelsList_->setSelectionMode(QAbstractItemView::SingleSelection);
  installedModelsList_->setMinimumHeight(100);
  mgmtLayout->addWidget(installedModelsList_);

  QHBoxLayout* mgmtBtnLayout = new QHBoxLayout();
  deleteModelBtn_ = new QPushButton(tr("Delete Selected"), this);
  deleteModelBtn_->setEnabled(false);
  mgmtBtnLayout->addWidget(deleteModelBtn_);
  mgmtBtnLayout->addStretch();
  mgmtLayout->addLayout(mgmtBtnLayout);

  mainLayout->addWidget(managementGroup_);

  // Download Group
  downloadGroup_ = new QGroupBox(tr("Download New Model"), this);
  QVBoxLayout* dlLayout = new QVBoxLayout(downloadGroup_);

  QHBoxLayout* dlSelectLayout = new QHBoxLayout();
  downloadModelCombo_ = new QComboBox(this);
  downloadModelCombo_->setEditable(true);
  downloadModelCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  downloadModelCombo_->setMinimumWidth(200);

  // Populate with recommended models
  for (const QString& model : OllamaManager::recommendedModels()) {
    downloadModelCombo_->addItem(model);
  }

  dlSelectLayout->addWidget(new QLabel(tr("Model:"), this));
  dlSelectLayout->addWidget(downloadModelCombo_);
  dlLayout->addLayout(dlSelectLayout);

  QLabel* dlHintLabel = new QLabel(
      tr("Enter a model name from <a href=\"https://ollama.ai/library\">ollama.ai/library</a> "
         "or select from the recommended models above."), this);
  dlHintLabel->setOpenExternalLinks(true);
  dlHintLabel->setWordWrap(true);
  dlHintLabel->setStyleSheet("color: gray; font-size: 11px;");
  dlLayout->addWidget(dlHintLabel);

  QHBoxLayout* dlBtnLayout = new QHBoxLayout();
  downloadBtn_ = new QPushButton(tr("Download"), this);
  cancelDownloadBtn_ = new QPushButton(tr("Cancel"), this);
  cancelDownloadBtn_->setEnabled(false);
  dlBtnLayout->addWidget(downloadBtn_);
  dlBtnLayout->addWidget(cancelDownloadBtn_);
  dlBtnLayout->addStretch();
  dlLayout->addLayout(dlBtnLayout);

  downloadProgress_ = new QProgressBar(this);
  downloadProgress_->setRange(0, 100);
  downloadProgress_->setValue(0);
  downloadProgress_->setVisible(false);
  dlLayout->addWidget(downloadProgress_);

  downloadStatusLabel_ = new QLabel(this);
  downloadStatusLabel_->setStyleSheet("color: gray;");
  downloadStatusLabel_->setVisible(false);
  dlLayout->addWidget(downloadStatusLabel_);

  mainLayout->addWidget(downloadGroup_);

  // Status Bar Display Group
  statusBarGroup_ = new QGroupBox(tr("Status Bar Display"), this);
  QVBoxLayout* statusBarLayout = new QVBoxLayout(statusBarGroup_);

  showStatusCheck_ = new QCheckBox(tr("Show LocalAI status in status bar"), this);
  showStatusCheck_->setToolTip(tr("Display the Local AI connection status in the application status bar"));
  showStatusCheck_->setChecked(true);
  statusBarLayout->addWidget(showStatusCheck_);

  showModelNameCheck_ = new QCheckBox(tr("Show model name in status bar"), this);
  showModelNameCheck_->setToolTip(tr("Display the selected model name in the status bar when connected"));
  showModelNameCheck_->setChecked(true);
  statusBarLayout->addWidget(showModelNameCheck_);

  mainLayout->addWidget(statusBarGroup_);

  mainLayout->addStretch();

  // Initial state
  updateUiState();
}

void OllamaSettingsWidget::connectSignals() {
  OllamaManager& mgr = OllamaManager::instance();

  // Connect to manager signals
  connect(&mgr, &OllamaManager::ollamaStatusChanged, this, &OllamaSettingsWidget::onOllamaStatusChanged);
  connect(&mgr, &OllamaManager::localModelsUpdated, this, &OllamaSettingsWidget::onLocalModelsUpdated);
  connect(&mgr, &OllamaManager::pullProgress, this, &OllamaSettingsWidget::onPullProgress);
  connect(&mgr, &OllamaManager::pullCompleted, this, &OllamaSettingsWidget::onPullCompleted);
  connect(&mgr, &OllamaManager::modelDeleted, this, &OllamaSettingsWidget::onModelDeleted);

  // UI signals
  connect(enableOllamaCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::updateUiState);
  connect(enableOllamaCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);
  connect(endpointEdit_, &QLineEdit::editingFinished, this, &OllamaSettingsWidget::onEndpointChanged);
  connect(testConnectionBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onTestConnectionClicked);
  connect(refreshBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onRefreshClicked);
  connect(downloadBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onDownloadClicked);
  connect(cancelDownloadBtn_, &QPushButton::clicked, &mgr, &OllamaManager::cancelPull);
  connect(deleteModelBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onDeleteClicked);

  connect(modelCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &OllamaSettingsWidget::settingsChanged);
  connect(autoLoadCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);

  connect(installedModelsList_, &QListWidget::itemSelectionChanged, this, [this]() {
    deleteModelBtn_->setEnabled(installedModelsList_->currentItem() != nullptr);
  });

  // Status bar display settings
  connect(showStatusCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);
  connect(showModelNameCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);
}

void OllamaSettingsWidget::updateUiState() {
  bool enabled = enableOllamaCheck_->isChecked();
  bool ollamaRunning = OllamaManager::instance().isOllamaRunning();

  connectionGroup_->setEnabled(enabled);
  modelGroup_->setEnabled(enabled && ollamaRunning);
  managementGroup_->setEnabled(enabled && ollamaRunning);
  downloadGroup_->setEnabled(enabled && ollamaRunning && !isPulling_);

  downloadBtn_->setEnabled(enabled && ollamaRunning && !isPulling_);
  cancelDownloadBtn_->setEnabled(isPulling_);

  // Update status icon
  if (!enabled) {
    statusLabel_->setText(tr("Disabled"));
    statusIcon_->setStyleSheet("background-color: gray; border-radius: 8px;");
  } else if (ollamaRunning) {
    statusLabel_->setText(tr("Connected"));
    statusIcon_->setStyleSheet("background-color: #4CAF50; border-radius: 8px;");
  } else {
    statusLabel_->setText(tr("Not Connected - Is Ollama running?"));
    statusIcon_->setStyleSheet("background-color: #f44336; border-radius: 8px;");
  }
}

void OllamaSettingsWidget::onOllamaStatusChanged(bool running) {
  // If we were testing connection and it succeeded, stop the timer
  if (isTestingConnection_ && running) {
    connectionTestTimer_->stop();
    isTestingConnection_ = false;
    testConnectionBtn_->setEnabled(true);

    QMessageBox::information(this, tr("Connection Successful"),
                             tr("Successfully connected to Ollama at:\n%1")
                                 .arg(OllamaManager::instance().endpoint()));
  }

  updateUiState();
}

void OllamaSettingsWidget::setLocalAIStatusWidget(LocalAIStatusWidget* widget) {
  localAIStatusWidget_ = widget;
}

void OllamaSettingsWidget::onLocalModelsUpdated(const QList<OllamaModel>& models) {
  // Update installed models list
  installedModelsList_->clear();
  for (const OllamaModel& model : models) {
    QListWidgetItem* item = new QListWidgetItem(
        QString("%1 (%2)").arg(model.name, model.size));
    item->setData(Qt::UserRole, model.name);
    installedModelsList_->addItem(item);
  }

  // Update model combo
  populateModelCombo();
}

void OllamaSettingsWidget::populateModelCombo() {
  QString currentSelection = modelCombo_->currentText();
  modelCombo_->clear();

  const QList<OllamaModel>& models = OllamaManager::instance().localModels();
  for (const OllamaModel& model : models) {
    modelCombo_->addItem(model.name);
  }

  // Restore selection if possible
  int idx = modelCombo_->findText(currentSelection);
  if (idx >= 0) {
    modelCombo_->setCurrentIndex(idx);
  } else {
    // Try to select the saved model
    QString savedModel = OllamaManager::instance().selectedModel();
    idx = modelCombo_->findText(savedModel);
    if (idx >= 0) {
      modelCombo_->setCurrentIndex(idx);
    }
  }
}

void OllamaSettingsWidget::onPullProgress(const QString& modelName, double progress, const QString& status) {
  Q_UNUSED(modelName);

  downloadProgress_->setVisible(true);
  downloadStatusLabel_->setVisible(true);

  if (progress >= 0) {
    downloadProgress_->setValue(static_cast<int>(progress));
    downloadStatusLabel_->setText(QString("%1 - %2%").arg(status).arg(progress, 0, 'f', 1));
  } else {
    downloadProgress_->setRange(0, 0);  // Indeterminate
    downloadStatusLabel_->setText(status);
  }
}

void OllamaSettingsWidget::onPullCompleted(const QString& modelName, bool success, const QString& error) {
  isPulling_ = false;
  downloadProgress_->setVisible(false);
  downloadStatusLabel_->setVisible(false);
  downloadProgress_->setRange(0, 100);
  downloadProgress_->setValue(0);

  updateUiState();

  if (success) {
    QMessageBox::information(this, tr("Download Complete"),
                             tr("Model '%1' has been downloaded successfully.").arg(modelName));

    // Auto-select the newly downloaded model
    int idx = modelCombo_->findText(modelName);
    if (idx >= 0) {
      modelCombo_->setCurrentIndex(idx);
    }
  } else {
    QMessageBox::warning(this, tr("Download Failed"),
                         tr("Failed to download model '%1':\n%2").arg(modelName, error));
  }
}

void OllamaSettingsWidget::onModelDeleted(const QString& modelName, bool success) {
  if (success) {
    QMessageBox::information(this, tr("Model Deleted"),
                             tr("Model '%1' has been deleted.").arg(modelName));
  } else {
    QMessageBox::warning(this, tr("Delete Failed"),
                         tr("Failed to delete model '%1'.").arg(modelName));
  }
}

void OllamaSettingsWidget::onDownloadClicked() {
  QString modelName = downloadModelCombo_->currentText().trimmed();
  if (modelName.isEmpty()) {
    QMessageBox::warning(this, tr("No Model Selected"),
                         tr("Please enter or select a model name to download."));
    return;
  }

  isPulling_ = true;
  updateUiState();

  OllamaManager::instance().pullModel(modelName);
}

void OllamaSettingsWidget::onDeleteClicked() {
  QListWidgetItem* item = installedModelsList_->currentItem();
  if (!item) return;

  QString modelName = item->data(Qt::UserRole).toString();

  QMessageBox::StandardButton reply = QMessageBox::question(
      this, tr("Delete Model"),
      tr("Are you sure you want to delete the model '%1'?\n\n"
         "This action cannot be undone.").arg(modelName),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

  if (reply == QMessageBox::Yes) {
    OllamaManager::instance().deleteModel(modelName);
  }
}

void OllamaSettingsWidget::onRefreshClicked() {
  OllamaManager::instance().refreshLocalModels();
}

void OllamaSettingsWidget::onTestConnectionClicked() {
  QString endpoint = endpointEdit_->text().trimmed();
  if (endpoint.isEmpty()) {
    endpoint = "http://localhost:11434";
  }

  isTestingConnection_ = true;
  testConnectionBtn_->setEnabled(false);
  statusLabel_->setText(tr("Testing connection..."));
  statusIcon_->setStyleSheet("background-color: #2196F3; border-radius: 8px;");  // Blue for testing

  OllamaManager::instance().setEndpoint(endpoint);
  OllamaManager::instance().checkOllamaStatus();

  // Start timeout timer
  connectionTestTimer_->start(CONNECTION_TEST_TIMEOUT_MS);
}

void OllamaSettingsWidget::onTestConnectionTimeout() {
  if (!isTestingConnection_) return;

  isTestingConnection_ = false;
  testConnectionBtn_->setEnabled(true);

  // Check if connection succeeded in the meantime
  if (OllamaManager::instance().isOllamaRunning()) {
    updateUiState();
    return;
  }

  // Connection failed - show helpful dialog
  statusLabel_->setText(tr("Connection Failed"));
  statusIcon_->setStyleSheet("background-color: #f44336; border-radius: 8px;");

  QMessageBox msgBox(this);
  msgBox.setWindowTitle(tr("Ollama Connection Failed"));
  msgBox.setIcon(QMessageBox::Warning);
  msgBox.setText(tr("Could not connect to Ollama at:\n%1")
                     .arg(OllamaManager::instance().endpoint()));
  msgBox.setInformativeText(
      tr("<b>Possible solutions:</b><br><br>"
         "1. <b>Install Ollama</b> if not already installed:<br>"
         "&nbsp;&nbsp;&nbsp;Visit <a href='https://ollama.ai'>ollama.ai</a> to download<br>"
         "&nbsp;&nbsp;&nbsp;Or run: <code>curl -fsSL https://ollama.ai/install.sh | sh</code><br><br>"
         "2. <b>Start the Ollama service</b>:<br>"
         "&nbsp;&nbsp;&nbsp;Run: <code>ollama serve</code> in a terminal<br><br>"
         "3. <b>Check the endpoint URL</b>:<br>"
         "&nbsp;&nbsp;&nbsp;Default is http://localhost:11434<br><br>"
         "4. <b>Check firewall settings</b> if using a remote server"));
  msgBox.setTextFormat(Qt::RichText);

  QPushButton* openWebsiteBtn = msgBox.addButton(tr("Open Ollama Website"), QMessageBox::ActionRole);
  msgBox.addButton(QMessageBox::Ok);

  msgBox.exec();

  if (msgBox.clickedButton() == openWebsiteBtn) {
    QDesktopServices::openUrl(QUrl("https://ollama.ai"));
  }
}

void OllamaSettingsWidget::onEndpointChanged() {
  emit settingsChanged();
}

void OllamaSettingsWidget::applySettings() {
  OllamaManager& mgr = OllamaManager::instance();

  mgr.setEnabled(enableOllamaCheck_->isChecked());
  mgr.setEndpoint(endpointEdit_->text().trimmed().isEmpty()
                      ? "http://localhost:11434"
                      : endpointEdit_->text().trimmed());
  mgr.setSelectedModel(modelCombo_->currentText());
  mgr.setAutoLoadModel(autoLoadCheck_->isChecked());

  // Apply status bar display settings
  if (localAIStatusWidget_) {
    localAIStatusWidget_->setShowStatus(showStatusCheck_->isChecked());
    localAIStatusWidget_->setShowModelName(showModelNameCheck_->isChecked());
  }
}

void OllamaSettingsWidget::resetToSaved() {
  OllamaManager& mgr = OllamaManager::instance();

  enableOllamaCheck_->setChecked(mgr.isEnabled());
  endpointEdit_->setText(mgr.endpoint());
  autoLoadCheck_->setChecked(mgr.autoLoadModel());

  // Model combo will be populated when models are loaded
  QString savedModel = mgr.selectedModel();
  int idx = modelCombo_->findText(savedModel);
  if (idx >= 0) {
    modelCombo_->setCurrentIndex(idx);
  }

  // Load status bar display settings
  if (localAIStatusWidget_) {
    showStatusCheck_->setChecked(localAIStatusWidget_->isStatusVisible());
    showModelNameCheck_->setChecked(localAIStatusWidget_->isModelNameVisible());
  }

  updateUiState();
}

}  // namespace ros_weaver
