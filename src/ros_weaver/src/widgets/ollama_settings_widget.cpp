#include "ros_weaver/widgets/ollama_settings_widget.hpp"
#include "ros_weaver/widgets/local_ai_status_widget.hpp"
#include "ros_weaver/widgets/system_prompt_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include <QStyle>
#include <QDesktopServices>
#include <QUrl>
#include <QColorDialog>
#include <QGridLayout>

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
  mainLayout->setSpacing(12);

  // Enable checkbox at top (always visible)
  enableOllamaCheck_ = new QCheckBox(tr("Enable Local LLM (Ollama)"), this);
  enableOllamaCheck_->setToolTip(tr("Enable or disable the local LLM integration"));
  mainLayout->addWidget(enableOllamaCheck_);

  // Tab widget
  tabWidget_ = new QTabWidget(this);
  tabWidget_->addTab(createConnectionTab(), tr("Connection"));
  tabWidget_->addTab(createModelsTab(), tr("Models"));
  tabWidget_->addTab(createAppearanceTab(), tr("Appearance"));
  mainLayout->addWidget(tabWidget_, 1);

  // Initial state
  updateUiState();
}

QWidget* OllamaSettingsWidget::createConnectionTab() {
  QWidget* tab = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(tab);
  layout->setSpacing(12);

  // Connection Group
  connectionGroup_ = new QGroupBox(tr("Ollama Connection"), tab);
  QVBoxLayout* connLayout = new QVBoxLayout(connectionGroup_);

  QHBoxLayout* statusLayout = new QHBoxLayout();
  statusIcon_ = new QLabel(tab);
  statusIcon_->setFixedSize(16, 16);
  statusLabel_ = new QLabel(tr("Checking..."), tab);
  statusLayout->addWidget(statusIcon_);
  statusLayout->addWidget(statusLabel_);
  statusLayout->addStretch();
  connLayout->addLayout(statusLayout);

  QFormLayout* endpointLayout = new QFormLayout();
  endpointEdit_ = new QLineEdit(tab);
  endpointEdit_->setPlaceholderText("http://localhost:11434");
  endpointLayout->addRow(tr("Endpoint URL:"), endpointEdit_);
  connLayout->addLayout(endpointLayout);

  QHBoxLayout* connBtnLayout = new QHBoxLayout();
  testConnectionBtn_ = new QPushButton(tr("Test Connection"), tab);
  connBtnLayout->addWidget(testConnectionBtn_);
  connBtnLayout->addStretch();
  connLayout->addLayout(connBtnLayout);

  layout->addWidget(connectionGroup_);

  // Status Bar Display Group
  statusBarGroup_ = new QGroupBox(tr("Status Bar Display"), tab);
  QVBoxLayout* statusBarLayout = new QVBoxLayout(statusBarGroup_);

  showStatusCheck_ = new QCheckBox(tr("Show LocalAI status in status bar"), tab);
  showStatusCheck_->setToolTip(tr("Display the Local AI connection status in the application status bar"));
  showStatusCheck_->setChecked(true);
  statusBarLayout->addWidget(showStatusCheck_);

  showModelNameCheck_ = new QCheckBox(tr("Show model name in status bar"), tab);
  showModelNameCheck_->setToolTip(tr("Display the selected model name in the status bar when connected"));
  showModelNameCheck_->setChecked(true);
  statusBarLayout->addWidget(showModelNameCheck_);

  layout->addWidget(statusBarGroup_);

  // Performance Group
  performanceGroup_ = new QGroupBox(tr("Performance"), tab);
  QVBoxLayout* perfLayout = new QVBoxLayout(performanceGroup_);

  QHBoxLayout* threadsLayout = new QHBoxLayout();
  threadsLayout->addWidget(new QLabel(tr("CPU Threads:"), tab));
  cpuThreadsSpin_ = new QSpinBox(tab);
  cpuThreadsSpin_->setRange(0, 64);
  cpuThreadsSpin_->setSpecialValueText(tr("Auto"));
  cpuThreadsSpin_->setToolTip(tr("Number of CPU threads for inference (0 = auto)"));
  cpuThreadsSpin_->setFixedWidth(80);
  threadsLayout->addWidget(cpuThreadsSpin_);
  threadsLayout->addStretch();
  perfLayout->addLayout(threadsLayout);

  QLabel* perfHintLabel = new QLabel(
      tr("Set to 0 for automatic. Typical optimal range: 4-8 threads."),
      tab);
  perfHintLabel->setWordWrap(true);
  perfHintLabel->setStyleSheet("color: gray; font-size: 11px;");
  perfLayout->addWidget(perfHintLabel);

  layout->addWidget(performanceGroup_);

  // System Prompt Group (opens dialog)
  systemPromptGroup_ = new QGroupBox(tr("System Prompt"), tab);
  QVBoxLayout* promptLayout = new QVBoxLayout(systemPromptGroup_);

  systemPromptPreview_ = new QLabel(tab);
  systemPromptPreview_->setWordWrap(true);
  systemPromptPreview_->setStyleSheet(
      "color: gray; font-style: italic; padding: 8px; "
      "background-color: rgba(128,128,128,0.1); border-radius: 4px;");
  systemPromptPreview_->setMaximumHeight(60);
  promptLayout->addWidget(systemPromptPreview_);

  QHBoxLayout* promptBtnLayout = new QHBoxLayout();
  editSystemPromptBtn_ = new QPushButton(tr("Edit System Prompt..."), tab);
  editSystemPromptBtn_->setToolTip(tr("Open the system prompt editor in a new window"));
  promptBtnLayout->addWidget(editSystemPromptBtn_);
  promptBtnLayout->addStretch();
  promptLayout->addLayout(promptBtnLayout);

  layout->addWidget(systemPromptGroup_);

  layout->addStretch();
  return tab;
}

QWidget* OllamaSettingsWidget::createModelsTab() {
  QWidget* tab = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(tab);
  layout->setSpacing(12);

  // Model Selection Group
  modelGroup_ = new QGroupBox(tr("Active Model"), tab);
  QVBoxLayout* modelLayout = new QVBoxLayout(modelGroup_);

  QHBoxLayout* modelSelectLayout = new QHBoxLayout();
  modelCombo_ = new QComboBox(tab);
  modelCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  modelCombo_->setMinimumWidth(200);
  refreshBtn_ = new QPushButton(tr("Refresh"), tab);
  modelSelectLayout->addWidget(new QLabel(tr("Selected Model:"), tab));
  modelSelectLayout->addWidget(modelCombo_);
  modelSelectLayout->addWidget(refreshBtn_);
  modelLayout->addLayout(modelSelectLayout);

  autoLoadCheck_ = new QCheckBox(tr("Automatically use this model on next launch"), tab);
  autoLoadCheck_->setToolTip(tr("When enabled, the selected model will be automatically loaded when the application starts"));
  modelLayout->addWidget(autoLoadCheck_);

  layout->addWidget(modelGroup_);

  // Installed Models Management Group
  managementGroup_ = new QGroupBox(tr("Installed Models"), tab);
  QVBoxLayout* mgmtLayout = new QVBoxLayout(managementGroup_);

  installedModelsList_ = new QListWidget(tab);
  installedModelsList_->setSelectionMode(QAbstractItemView::SingleSelection);
  installedModelsList_->setMinimumHeight(100);
  mgmtLayout->addWidget(installedModelsList_);

  QHBoxLayout* mgmtBtnLayout = new QHBoxLayout();
  refreshModelsBtn_ = new QPushButton(tr("Refresh List"), tab);
  refreshModelsBtn_->setToolTip(tr("Refresh the list of installed models from Ollama"));
  mgmtBtnLayout->addWidget(refreshModelsBtn_);
  deleteModelBtn_ = new QPushButton(tr("Delete Selected"), tab);
  deleteModelBtn_->setEnabled(false);
  mgmtBtnLayout->addWidget(deleteModelBtn_);
  mgmtBtnLayout->addStretch();
  mgmtLayout->addLayout(mgmtBtnLayout);

  layout->addWidget(managementGroup_);

  // Download Group
  downloadGroup_ = new QGroupBox(tr("Download New Model"), tab);
  QVBoxLayout* dlLayout = new QVBoxLayout(downloadGroup_);

  QHBoxLayout* dlSelectLayout = new QHBoxLayout();
  downloadModelCombo_ = new QComboBox(tab);
  downloadModelCombo_->setEditable(true);
  downloadModelCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  downloadModelCombo_->setMinimumWidth(200);

  // Populate with recommended models
  for (const QString& model : OllamaManager::recommendedModels()) {
    downloadModelCombo_->addItem(model);
  }

  dlSelectLayout->addWidget(new QLabel(tr("Model:"), tab));
  dlSelectLayout->addWidget(downloadModelCombo_);
  dlLayout->addLayout(dlSelectLayout);

  QLabel* dlHintLabel = new QLabel(
      tr("Enter a model name from <a href=\"https://ollama.ai/library\">ollama.ai/library</a> "
         "or select from the recommended models above."), tab);
  dlHintLabel->setOpenExternalLinks(true);
  dlHintLabel->setWordWrap(true);
  dlHintLabel->setStyleSheet("color: gray; font-size: 11px;");
  dlLayout->addWidget(dlHintLabel);

  QHBoxLayout* dlBtnLayout = new QHBoxLayout();
  downloadBtn_ = new QPushButton(tr("Download"), tab);
  cancelDownloadBtn_ = new QPushButton(tr("Cancel"), tab);
  cancelDownloadBtn_->setEnabled(false);
  dlBtnLayout->addWidget(downloadBtn_);
  dlBtnLayout->addWidget(cancelDownloadBtn_);
  dlBtnLayout->addStretch();
  dlLayout->addLayout(dlBtnLayout);

  downloadProgress_ = new QProgressBar(tab);
  downloadProgress_->setRange(0, 100);
  downloadProgress_->setValue(0);
  downloadProgress_->setVisible(false);
  dlLayout->addWidget(downloadProgress_);

  downloadStatusLabel_ = new QLabel(tab);
  downloadStatusLabel_->setStyleSheet("color: gray;");
  downloadStatusLabel_->setVisible(false);
  dlLayout->addWidget(downloadStatusLabel_);

  layout->addWidget(downloadGroup_);

  layout->addStretch();
  return tab;
}

QWidget* OllamaSettingsWidget::createAppearanceTab() {
  QWidget* tab = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(tab);
  layout->setSpacing(12);

  // Chat Appearance Group
  appearanceGroup_ = new QGroupBox(tr("Chat Appearance"), tab);
  QVBoxLayout* appearanceLayout = new QVBoxLayout(appearanceGroup_);

  // Font selection row
  QHBoxLayout* fontLayout = new QHBoxLayout();
  fontLayout->addWidget(new QLabel(tr("Font:"), tab));
  fontFamilyCombo_ = new QFontComboBox(tab);
  fontFamilyCombo_->setCurrentFont(OllamaManager::defaultChatFont());
  fontLayout->addWidget(fontFamilyCombo_, 1);

  fontLayout->addWidget(new QLabel(tr("Size:"), tab));
  fontSizeSpin_ = new QSpinBox(tab);
  fontSizeSpin_->setRange(8, 24);
  fontSizeSpin_->setValue(OllamaManager::defaultChatFontSize());
  fontSizeSpin_->setFixedWidth(60);
  fontLayout->addWidget(fontSizeSpin_);

  fontBoldCheck_ = new QCheckBox(tr("Bold"), tab);
  fontLayout->addWidget(fontBoldCheck_);

  fontItalicCheck_ = new QCheckBox(tr("Italic"), tab);
  fontLayout->addWidget(fontItalicCheck_);

  appearanceLayout->addLayout(fontLayout);

  // Spacer
  appearanceLayout->addSpacing(12);

  // Color buttons in a grid
  QGridLayout* colorGrid = new QGridLayout();

  colorGrid->addWidget(new QLabel(tr("Your Messages:"), tab), 0, 0);
  userTextColorBtn_ = new QPushButton(tr("Text Color"), tab);
  userTextColorBtn_->setFixedWidth(100);
  connect(userTextColorBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onUserTextColorClicked);
  colorGrid->addWidget(userTextColorBtn_, 0, 1);

  userBgColorBtn_ = new QPushButton(tr("Background"), tab);
  userBgColorBtn_->setFixedWidth(100);
  connect(userBgColorBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onUserBgColorClicked);
  colorGrid->addWidget(userBgColorBtn_, 0, 2);

  colorGrid->addWidget(new QLabel(tr("AI Messages:"), tab), 1, 0);
  assistantTextColorBtn_ = new QPushButton(tr("Text Color"), tab);
  assistantTextColorBtn_->setFixedWidth(100);
  connect(assistantTextColorBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onAssistantTextColorClicked);
  colorGrid->addWidget(assistantTextColorBtn_, 1, 1);

  assistantBgColorBtn_ = new QPushButton(tr("Background"), tab);
  assistantBgColorBtn_->setFixedWidth(100);
  connect(assistantBgColorBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onAssistantBgColorClicked);
  colorGrid->addWidget(assistantBgColorBtn_, 1, 2);

  colorGrid->addWidget(new QLabel(tr("Code Blocks:"), tab), 2, 0);
  codeBgColorBtn_ = new QPushButton(tr("Background"), tab);
  codeBgColorBtn_->setFixedWidth(100);
  connect(codeBgColorBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onCodeBgColorClicked);
  colorGrid->addWidget(codeBgColorBtn_, 2, 1);

  colorGrid->setColumnStretch(3, 1);
  appearanceLayout->addLayout(colorGrid);

  // Preview label
  previewLabel_ = new QLabel(tab);
  previewLabel_->setMinimumHeight(60);
  previewLabel_->setWordWrap(true);
  previewLabel_->setAlignment(Qt::AlignCenter);
  previewLabel_->setText(tr("Preview: The quick brown fox jumps over the lazy dog."));
  appearanceLayout->addWidget(previewLabel_);

  // Reset button
  QHBoxLayout* resetAppearanceLayout = new QHBoxLayout();
  resetAppearanceBtn_ = new QPushButton(tr("Reset to Defaults"), tab);
  connect(resetAppearanceBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onResetAppearanceClicked);
  resetAppearanceLayout->addWidget(resetAppearanceBtn_);
  resetAppearanceLayout->addStretch();
  appearanceLayout->addLayout(resetAppearanceLayout);

  layout->addWidget(appearanceGroup_);

  layout->addStretch();
  return tab;
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
  connect(refreshModelsBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onRefreshClicked);

  connect(modelCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &OllamaSettingsWidget::settingsChanged);
  connect(autoLoadCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);

  connect(installedModelsList_, &QListWidget::itemSelectionChanged, this, [this]() {
    deleteModelBtn_->setEnabled(installedModelsList_->currentItem() != nullptr);
  });

  // Status bar display settings
  connect(showStatusCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);
  connect(showModelNameCheck_, &QCheckBox::toggled, this, &OllamaSettingsWidget::settingsChanged);

  // Performance settings
  connect(cpuThreadsSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &OllamaSettingsWidget::settingsChanged);

  // System prompt dialog
  connect(editSystemPromptBtn_, &QPushButton::clicked, this, &OllamaSettingsWidget::onEditSystemPromptClicked);

  // Appearance settings
  connect(fontFamilyCombo_, &QFontComboBox::currentFontChanged, this, [this]() {
    updatePreview();
    emit settingsChanged();
  });
  connect(fontSizeSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    updatePreview();
    emit settingsChanged();
  });
  connect(fontBoldCheck_, &QCheckBox::toggled, this, [this]() {
    updatePreview();
    emit settingsChanged();
  });
  connect(fontItalicCheck_, &QCheckBox::toggled, this, [this]() {
    updatePreview();
    emit settingsChanged();
  });
}

void OllamaSettingsWidget::updateUiState() {
  bool enabled = enableOllamaCheck_->isChecked();
  bool ollamaRunning = OllamaManager::instance().isOllamaRunning();

  tabWidget_->setEnabled(enabled);

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

    // Explicitly refresh models after the dialog is closed
    // to ensure the new model appears in the list
    OllamaManager::instance().refreshLocalModels();

    // Auto-select the newly downloaded model (will work after refresh signal)
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

void OllamaSettingsWidget::onEditSystemPromptClicked() {
  SystemPromptDialog dialog(this);
  dialog.setPrompt(currentSystemPrompt_);

  if (dialog.exec() == QDialog::Accepted) {
    currentSystemPrompt_ = dialog.prompt();

    // Update preview
    QString preview = currentSystemPrompt_;
    if (preview.length() > 100) {
      preview = preview.left(100) + "...";
    }
    systemPromptPreview_->setText(preview.isEmpty() ? tr("(No system prompt set)") : preview);

    emit settingsChanged();
  }
}

void OllamaSettingsWidget::applySettings() {
  OllamaManager& mgr = OllamaManager::instance();

  mgr.setEnabled(enableOllamaCheck_->isChecked());
  mgr.setEndpoint(endpointEdit_->text().trimmed().isEmpty()
                      ? "http://localhost:11434"
                      : endpointEdit_->text().trimmed());
  mgr.setSelectedModel(modelCombo_->currentText());
  mgr.setAutoLoadModel(autoLoadCheck_->isChecked());
  mgr.setSystemPrompt(currentSystemPrompt_);
  mgr.setNumThreads(cpuThreadsSpin_->value());

  // Apply status bar display settings
  if (localAIStatusWidget_) {
    localAIStatusWidget_->setShowStatus(showStatusCheck_->isChecked());
    localAIStatusWidget_->setShowModelName(showModelNameCheck_->isChecked());
  }

  // Apply chat appearance settings
  QFont chatFont = fontFamilyCombo_->currentFont();
  chatFont.setPointSize(fontSizeSpin_->value());
  chatFont.setBold(fontBoldCheck_->isChecked());
  chatFont.setItalic(fontItalicCheck_->isChecked());
  mgr.setChatFont(chatFont);
  mgr.setChatFontSize(fontSizeSpin_->value());
  mgr.setUserMessageColor(currentUserTextColor_);
  mgr.setUserBackgroundColor(currentUserBgColor_);
  mgr.setAssistantMessageColor(currentAssistantTextColor_);
  mgr.setAssistantBackgroundColor(currentAssistantBgColor_);
  mgr.setCodeBackgroundColor(currentCodeBgColor_);
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

  // Load system prompt
  currentSystemPrompt_ = mgr.systemPrompt();
  QString preview = currentSystemPrompt_;
  if (preview.length() > 100) {
    preview = preview.left(100) + "...";
  }
  systemPromptPreview_->setText(preview.isEmpty() ? tr("(No system prompt set)") : preview);

  // Load performance settings
  cpuThreadsSpin_->setValue(mgr.numThreads());

  // Load status bar display settings
  if (localAIStatusWidget_) {
    showStatusCheck_->setChecked(localAIStatusWidget_->isStatusVisible());
    showModelNameCheck_->setChecked(localAIStatusWidget_->isModelNameVisible());
  }

  // Load chat appearance settings
  QFont chatFont = mgr.chatFont();
  fontFamilyCombo_->setCurrentFont(chatFont);
  fontSizeSpin_->setValue(mgr.chatFontSize());
  fontBoldCheck_->setChecked(chatFont.bold());
  fontItalicCheck_->setChecked(chatFont.italic());

  currentUserTextColor_ = mgr.userMessageColor();
  currentUserBgColor_ = mgr.userBackgroundColor();
  currentAssistantTextColor_ = mgr.assistantMessageColor();
  currentAssistantBgColor_ = mgr.assistantBackgroundColor();
  currentCodeBgColor_ = mgr.codeBackgroundColor();

  setButtonColor(userTextColorBtn_, currentUserTextColor_);
  setButtonColor(userBgColorBtn_, currentUserBgColor_);
  setButtonColor(assistantTextColorBtn_, currentAssistantTextColor_);
  setButtonColor(assistantBgColorBtn_, currentAssistantBgColor_);
  setButtonColor(codeBgColorBtn_, currentCodeBgColor_);

  updatePreview();
  updateUiState();
}

void OllamaSettingsWidget::setButtonColor(QPushButton* button, const QColor& color) {
  QString textColor = (color.lightness() > 128) ? "black" : "white";
  button->setStyleSheet(QString(
      "QPushButton {"
      "  background-color: %1;"
      "  color: %2;"
      "  border: 1px solid #606060;"
      "  border-radius: 4px;"
      "  padding: 4px 8px;"
      "}"
      "QPushButton:hover { border-color: #808080; }"
  ).arg(color.name(), textColor));
}

void OllamaSettingsWidget::onUserTextColorClicked() {
  QColor color = QColorDialog::getColor(currentUserTextColor_, this, tr("Select User Text Color"));
  if (color.isValid()) {
    currentUserTextColor_ = color;
    setButtonColor(userTextColorBtn_, color);
    updatePreview();
    emit settingsChanged();
  }
}

void OllamaSettingsWidget::onUserBgColorClicked() {
  QColor color = QColorDialog::getColor(currentUserBgColor_, this, tr("Select User Background Color"));
  if (color.isValid()) {
    currentUserBgColor_ = color;
    setButtonColor(userBgColorBtn_, color);
    updatePreview();
    emit settingsChanged();
  }
}

void OllamaSettingsWidget::onAssistantTextColorClicked() {
  QColor color = QColorDialog::getColor(currentAssistantTextColor_, this, tr("Select AI Text Color"));
  if (color.isValid()) {
    currentAssistantTextColor_ = color;
    setButtonColor(assistantTextColorBtn_, color);
    updatePreview();
    emit settingsChanged();
  }
}

void OllamaSettingsWidget::onAssistantBgColorClicked() {
  QColor color = QColorDialog::getColor(currentAssistantBgColor_, this, tr("Select AI Background Color"));
  if (color.isValid()) {
    currentAssistantBgColor_ = color;
    setButtonColor(assistantBgColorBtn_, color);
    updatePreview();
    emit settingsChanged();
  }
}

void OllamaSettingsWidget::onCodeBgColorClicked() {
  QColor color = QColorDialog::getColor(currentCodeBgColor_, this, tr("Select Code Background Color"));
  if (color.isValid()) {
    currentCodeBgColor_ = color;
    setButtonColor(codeBgColorBtn_, color);
    updatePreview();
    emit settingsChanged();
  }
}

void OllamaSettingsWidget::onResetAppearanceClicked() {
  fontFamilyCombo_->setCurrentFont(OllamaManager::defaultChatFont());
  fontSizeSpin_->setValue(OllamaManager::defaultChatFontSize());
  fontBoldCheck_->setChecked(false);
  fontItalicCheck_->setChecked(false);

  currentUserTextColor_ = OllamaManager::defaultUserMessageColor();
  currentUserBgColor_ = OllamaManager::defaultUserBackgroundColor();
  currentAssistantTextColor_ = OllamaManager::defaultAssistantMessageColor();
  currentAssistantBgColor_ = OllamaManager::defaultAssistantBackgroundColor();
  currentCodeBgColor_ = OllamaManager::defaultCodeBackgroundColor();

  setButtonColor(userTextColorBtn_, currentUserTextColor_);
  setButtonColor(userBgColorBtn_, currentUserBgColor_);
  setButtonColor(assistantTextColorBtn_, currentAssistantTextColor_);
  setButtonColor(assistantBgColorBtn_, currentAssistantBgColor_);
  setButtonColor(codeBgColorBtn_, currentCodeBgColor_);

  updatePreview();
  emit settingsChanged();
}

void OllamaSettingsWidget::updatePreview() {
  QFont previewFont = fontFamilyCombo_->currentFont();
  previewFont.setPointSize(fontSizeSpin_->value());
  previewFont.setBold(fontBoldCheck_->isChecked());
  previewFont.setItalic(fontItalicCheck_->isChecked());

  previewLabel_->setFont(previewFont);
  previewLabel_->setStyleSheet(QString(
      "QLabel {"
      "  background-color: %1;"
      "  color: %2;"
      "  border-radius: 8px;"
      "  padding: 10px;"
      "}"
  ).arg(currentAssistantBgColor_.name(), currentAssistantTextColor_.name()));
}

}  // namespace ros_weaver
