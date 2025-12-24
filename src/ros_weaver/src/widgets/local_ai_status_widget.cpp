#include "ros_weaver/widgets/local_ai_status_widget.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QHBoxLayout>
#include <QTimer>
#include <iostream>

namespace ros_weaver {

LocalAIStatusWidget::LocalAIStatusWidget(QWidget* parent)
    : QWidget(parent)
    , statusIconLabel_(nullptr)
    , statusTextLabel_(nullptr)
    , separatorLabel_(nullptr)
    , tokenSpeedLabel_(nullptr)
    , showStatus_(true)
    , showModelName_(true)
    , tokenCount_(0)
    , isGenerating_(false)
    , currentTokenSpeed_(0.0) {
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

  // Connect to generation signals for token speed tracking
  connect(&mgr, &OllamaManager::completionStarted,
          this, &LocalAIStatusWidget::onGenerationStarted);
  connect(&mgr, &OllamaManager::completionToken,
          this, &LocalAIStatusWidget::onTokenReceived);
  connect(&mgr, &OllamaManager::completionFinished,
          this, [this](const QString&) { onGenerationFinished(); });
  connect(&mgr, &OllamaManager::completionError,
          this, [this](const QString&) { onGenerationFinished(); });

  // Initial display update
  updateDisplay();
}

LocalAIStatusWidget::~LocalAIStatusWidget() {
  std::cerr << "LocalAIStatusWidget destructor: starting" << std::endl;
  saveSettings();
  std::cerr << "LocalAIStatusWidget destructor: complete" << std::endl;
}

void LocalAIStatusWidget::setupUi() {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(4, 2, 4, 2);
  layout->setSpacing(6);

  // Separator from ROS status
  separatorLabel_ = new QLabel(QString::fromUtf8("\u2022"), this);  // Bullet point
  separatorLabel_->setStyleSheet("color: #666; font-size: 8px;");
  layout->addWidget(separatorLabel_);

  // Status icon (colored dot)
  statusIconLabel_ = new QLabel(this);
  statusIconLabel_->setFixedSize(8, 8);
  statusIconLabel_->setToolTip(tr("Local AI (Ollama) connection status"));
  layout->addWidget(statusIconLabel_);

  // Status text
  statusTextLabel_ = new QLabel(tr("AI"), this);
  statusTextLabel_->setToolTip(tr("Local AI (Ollama) connection status"));
  layout->addWidget(statusTextLabel_);

  // Token speed label (shown during generation)
  tokenSpeedLabel_ = new QLabel(this);
  tokenSpeedLabel_->setToolTip(tr("Token generation speed"));
  tokenSpeedLabel_->setVisible(false);
  layout->addWidget(tokenSpeedLabel_);

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
  auto& theme = ThemeManager::instance();

  bool enabled = mgr.isEnabled();
  bool connected = mgr.isOllamaRunning();
  QString selectedModel = mgr.selectedModel();

  // Get colors from theme
  QColor successColor = theme.successColor();
  QColor errorColor = theme.errorColor();
  QColor warningColor = theme.warningColor();
  QColor textSecondary = theme.textSecondaryColor();

  // Determine visibility
  bool showWidget = showStatus_ && enabled;
  setVisible(showWidget);
  separatorLabel_->setVisible(showWidget);

  if (!showWidget) {
    return;
  }

  // Update status icon and text
  QString iconStyle;
  QString textStyle;
  QString statusText;
  QString tooltip;

  if (!enabled) {
    iconStyle = QString("background-color: %1; border-radius: 4px;").arg(textSecondary.name());
    textStyle = QString("color: %1;").arg(textSecondary.name());
    statusText = tr("AI");
    tooltip = tr("Local AI integration is disabled.\nEnable it in Settings > Local LLM.");
  } else if (connected) {
    if (selectedModel.isEmpty()) {
      iconStyle = QString("background-color: %1; border-radius: 4px;").arg(warningColor.name());
      textStyle = QString("color: %1; font-weight: 500;").arg(warningColor.name());
      statusText = tr("AI");
      tooltip = tr("Ollama is running but no model is selected.\nSelect a model in Settings > Local LLM.");
    } else {
      iconStyle = QString("background-color: %1; border-radius: 4px;").arg(successColor.name());
      textStyle = QString("color: %1; font-weight: 500;").arg(successColor.name());
      if (showModelName_) {
        // Truncate long model names - more compact
        QString displayModel = selectedModel;
        if (displayModel.length() > 12) {
          displayModel = displayModel.left(10) + "..";
        }
        statusText = displayModel;
      } else {
        statusText = tr("AI");
      }
      tooltip = tr("Ollama connected\nModel: %1\nEndpoint: %2")
                    .arg(selectedModel, mgr.endpoint());
    }
  } else {
    iconStyle = QString("background-color: %1; border-radius: 4px;").arg(errorColor.name());
    textStyle = QString("color: %1; font-weight: 500;").arg(errorColor.name());
    statusText = tr("AI");
    tooltip = tr("Ollama is offline.\n"
                 "Start with: ollama serve\n"
                 "Endpoint: %1").arg(mgr.endpoint());
  }

  statusIconLabel_->setStyleSheet(iconStyle);
  statusTextLabel_->setStyleSheet(textStyle);
  statusTextLabel_->setText(statusText);
  statusIconLabel_->setToolTip(tooltip);
  statusTextLabel_->setToolTip(tooltip);

  // Update token speed label style
  tokenSpeedLabel_->setStyleSheet(QString("color: %1; font-size: 11px;").arg(successColor.name()));
}

void LocalAIStatusWidget::onGenerationStarted() {
  isGenerating_ = true;
  tokenCount_ = 0;
  currentTokenSpeed_ = 0.0;
  tokenTimer_.start();
  tokenSpeedLabel_->setVisible(true);
  tokenSpeedLabel_->setText(tr("0 tok/s"));
}

void LocalAIStatusWidget::onTokenReceived() {
  if (!isGenerating_) return;

  tokenCount_++;
  updateTokenSpeed();
}

void LocalAIStatusWidget::updateTokenSpeed() {
  if (!isGenerating_ || !tokenTimer_.isValid()) return;

  qint64 elapsedMs = tokenTimer_.elapsed();
  if (elapsedMs > 0) {
    currentTokenSpeed_ = (tokenCount_ * 1000.0) / elapsedMs;
    tokenSpeedLabel_->setText(tr("%1 tok/s").arg(currentTokenSpeed_, 0, 'f', 1));
  }
}

void LocalAIStatusWidget::onGenerationFinished() {
  isGenerating_ = false;

  // Show final speed briefly before hiding
  if (tokenCount_ > 0 && tokenTimer_.elapsed() > 0) {
    currentTokenSpeed_ = (tokenCount_ * 1000.0) / tokenTimer_.elapsed();
    tokenSpeedLabel_->setText(tr("%1 tok/s (done)").arg(currentTokenSpeed_, 0, 'f', 1));

    // Hide after 3 seconds
    QTimer::singleShot(3000, this, [this]() {
      if (!isGenerating_) {
        tokenSpeedLabel_->setVisible(false);
      }
    });
  } else {
    tokenSpeedLabel_->setVisible(false);
  }

  tokenCount_ = 0;
}

}  // namespace ros_weaver
