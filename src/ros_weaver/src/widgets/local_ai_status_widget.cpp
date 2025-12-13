#include "ros_weaver/widgets/local_ai_status_widget.hpp"
#include "ros_weaver/core/ollama_manager.hpp"

#include <QHBoxLayout>

namespace ros_weaver {

LocalAIStatusWidget::LocalAIStatusWidget(QWidget* parent)
    : QWidget(parent)
    , statusIconLabel_(nullptr)
    , statusTextLabel_(nullptr)
    , separatorLabel_(nullptr)
    , showStatus_(true)
    , showModelName_(true) {
  setupUi();
  loadSettings();

  // Connect to OllamaManager signals
  OllamaManager& mgr = OllamaManager::instance();
  connect(&mgr, &OllamaManager::ollamaStatusChanged,
          this, &LocalAIStatusWidget::onOllamaStatusChanged);
  connect(&mgr, &OllamaManager::settingsChanged,
          this, &LocalAIStatusWidget::onOllamaSettingsChanged);
  connect(&mgr, &OllamaManager::localModelsUpdated,
          this, [this](const QList<OllamaModel>&) { updateDisplay(); });

  // Initial display update
  updateDisplay();
}

LocalAIStatusWidget::~LocalAIStatusWidget() {
  saveSettings();
}

void LocalAIStatusWidget::setupUi() {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(4, 0, 4, 0);
  layout->setSpacing(6);

  // Separator from ROS status
  separatorLabel_ = new QLabel("|", this);
  separatorLabel_->setStyleSheet("color: gray;");
  layout->addWidget(separatorLabel_);

  // Status icon (colored dot)
  statusIconLabel_ = new QLabel(this);
  statusIconLabel_->setFixedSize(12, 12);
  statusIconLabel_->setToolTip(tr("Local AI (Ollama) connection status"));
  layout->addWidget(statusIconLabel_);

  // Status text
  statusTextLabel_ = new QLabel(tr("LocalAI: Checking..."), this);
  statusTextLabel_->setToolTip(tr("Local AI (Ollama) connection status"));
  layout->addWidget(statusTextLabel_);

  setLayout(layout);
}

void LocalAIStatusWidget::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(SETTINGS_GROUP);
  showStatus_ = settings.value(KEY_SHOW_STATUS, true).toBool();
  showModelName_ = settings.value(KEY_SHOW_MODEL_NAME, true).toBool();
  settings.endGroup();
}

void LocalAIStatusWidget::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(SETTINGS_GROUP);
  settings.setValue(KEY_SHOW_STATUS, showStatus_);
  settings.setValue(KEY_SHOW_MODEL_NAME, showModelName_);
  settings.endGroup();
}

void LocalAIStatusWidget::setShowStatus(bool show) {
  if (showStatus_ != show) {
    showStatus_ = show;
    updateDisplay();
    saveSettings();
    emit settingsChanged();
  }
}

void LocalAIStatusWidget::setShowModelName(bool show) {
  if (showModelName_ != show) {
    showModelName_ = show;
    updateDisplay();
    saveSettings();
    emit settingsChanged();
  }
}

void LocalAIStatusWidget::refreshStatus() {
  OllamaManager::instance().checkOllamaStatus();
}

void LocalAIStatusWidget::onOllamaStatusChanged(bool running) {
  Q_UNUSED(running);
  updateDisplay();
}

void LocalAIStatusWidget::onOllamaSettingsChanged() {
  updateDisplay();
}

void LocalAIStatusWidget::updateDisplay() {
  OllamaManager& mgr = OllamaManager::instance();

  bool enabled = mgr.isEnabled();
  bool connected = mgr.isOllamaRunning();
  QString selectedModel = mgr.selectedModel();

  // Determine visibility
  bool showWidget = showStatus_ && enabled;
  setVisible(showWidget);
  separatorLabel_->setVisible(showWidget);

  if (!showWidget) {
    return;
  }

  // Update status icon and text
  QString iconStyle;
  QString statusText;
  QString tooltip;

  if (!enabled) {
    iconStyle = "background-color: #9E9E9E; border-radius: 6px;";  // Gray
    statusText = tr("LocalAI: Disabled");
    tooltip = tr("Local AI integration is disabled.\nEnable it in Settings > Local LLM.");
  } else if (connected) {
    if (selectedModel.isEmpty()) {
      iconStyle = "background-color: #FF9800; border-radius: 6px;";  // Orange
      statusText = tr("LocalAI: No Model");
      tooltip = tr("Ollama is running but no model is selected.\nSelect a model in Settings > Local LLM.");
    } else {
      iconStyle = "background-color: #4CAF50; border-radius: 6px;";  // Green
      if (showModelName_) {
        // Truncate long model names
        QString displayModel = selectedModel;
        if (displayModel.length() > 20) {
          displayModel = displayModel.left(17) + "...";
        }
        statusText = tr("LocalAI: %1").arg(displayModel);
      } else {
        statusText = tr("LocalAI: Ready");
      }
      tooltip = tr("Ollama is connected.\nModel: %1\nEndpoint: %2")
                    .arg(selectedModel, mgr.endpoint());
    }
  } else {
    iconStyle = "background-color: #F44336; border-radius: 6px;";  // Red
    statusText = tr("LocalAI: Offline");
    tooltip = tr("Ollama is not running or unreachable.\n"
                 "Start Ollama with: ollama serve\n"
                 "Endpoint: %1").arg(mgr.endpoint());
  }

  statusIconLabel_->setStyleSheet(iconStyle);
  statusTextLabel_->setText(statusText);
  statusIconLabel_->setToolTip(tooltip);
  statusTextLabel_->setToolTip(tooltip);
}

}  // namespace ros_weaver
