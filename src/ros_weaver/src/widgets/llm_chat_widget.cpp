#include "ros_weaver/widgets/llm_chat_widget.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include <QHBoxLayout>
#include <QScrollBar>
#include <QFrame>
#include <QApplication>

namespace ros_weaver {

// ============== ChatMessageWidget ==============

ChatMessageWidget::ChatMessageWidget(Role role, const QString& message, QWidget* parent)
    : QWidget(parent) {
  setupUi(role, message);
}

void ChatMessageWidget::setupUi(Role role, const QString& message) {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(8, 4, 8, 4);

  QLabel* msgLabel = new QLabel(this);
  msgLabel->setWordWrap(true);
  msgLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  msgLabel->setText(message);

  // Style based on role
  QString bgColor, textColor, alignment;
  QString rolePrefix;

  switch (role) {
    case Role::User:
      bgColor = "#2a82da";
      textColor = "white";
      rolePrefix = "<b>You:</b> ";
      layout->addStretch();
      break;
    case Role::Assistant:
      bgColor = "#404040";
      textColor = "#e0e0e0";
      rolePrefix = "<b>AI:</b> ";
      break;
    case Role::System:
      bgColor = "#505050";
      textColor = "#aaaaaa";
      rolePrefix = "";
      break;
  }

  msgLabel->setText(rolePrefix + message.toHtmlEscaped().replace("\n", "<br>"));
  msgLabel->setStyleSheet(QString(
      "QLabel {"
      "  background-color: %1;"
      "  color: %2;"
      "  border-radius: 8px;"
      "  padding: 8px 12px;"
      "  max-width: 600px;"
      "}").arg(bgColor, textColor));

  layout->addWidget(msgLabel);

  if (role == Role::User) {
    // User messages align right - stretch is already added
  } else {
    // Assistant/System messages align left
    layout->addStretch();
  }
}

// ============== LLMChatWidget ==============

LLMChatWidget::LLMChatWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();

  // Connect to OllamaManager signals
  OllamaManager& mgr = OllamaManager::instance();
  connect(&mgr, &OllamaManager::ollamaStatusChanged,
          this, &LLMChatWidget::onOllamaStatusChanged);
  connect(&mgr, &OllamaManager::completionReceived,
          this, &LLMChatWidget::onCompletionReceived);
  connect(&mgr, &OllamaManager::completionError,
          this, &LLMChatWidget::onCompletionError);
  connect(&mgr, &OllamaManager::settingsChanged,
          this, &LLMChatWidget::onSettingsChanged);

  // Initial status update
  updateStatusDisplay();
}

LLMChatWidget::~LLMChatWidget() = default;

void LLMChatWidget::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  // Status bar at top
  QWidget* statusBar = new QWidget(this);
  QHBoxLayout* statusLayout = new QHBoxLayout(statusBar);
  statusLayout->setContentsMargins(8, 4, 8, 4);

  statusLabel_ = new QLabel(tr("Checking connection..."), this);
  statusLabel_->setStyleSheet("color: gray;");
  statusLayout->addWidget(statusLabel_);
  statusLayout->addStretch();

  clearBtn_ = new QPushButton(tr("Clear Chat"), this);
  clearBtn_->setFixedWidth(80);
  connect(clearBtn_, &QPushButton::clicked, this, &LLMChatWidget::clearChat);
  statusLayout->addWidget(clearBtn_);

  mainLayout->addWidget(statusBar);

  // Separator
  QFrame* separator = new QFrame(this);
  separator->setFrameShape(QFrame::HLine);
  separator->setStyleSheet("color: #404040;");
  mainLayout->addWidget(separator);

  // Chat history scroll area
  scrollArea_ = new QScrollArea(this);
  scrollArea_->setWidgetResizable(true);
  scrollArea_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scrollArea_->setStyleSheet("QScrollArea { border: none; background-color: #2a2a2a; }");

  chatContainer_ = new QWidget();
  chatLayout_ = new QVBoxLayout(chatContainer_);
  chatLayout_->setContentsMargins(4, 4, 4, 4);
  chatLayout_->setSpacing(8);
  chatLayout_->addStretch();  // Push messages to top initially

  scrollArea_->setWidget(chatContainer_);
  mainLayout->addWidget(scrollArea_, 1);

  // Input area
  QWidget* inputArea = new QWidget(this);
  QHBoxLayout* inputLayout = new QHBoxLayout(inputArea);
  inputLayout->setContentsMargins(8, 8, 8, 8);
  inputLayout->setSpacing(8);

  inputEdit_ = new QLineEdit(this);
  inputEdit_->setPlaceholderText(tr("Type a message... (Enter to send)"));
  inputEdit_->setMinimumHeight(32);
  connect(inputEdit_, &QLineEdit::returnPressed, this, &LLMChatWidget::onSendClicked);
  inputLayout->addWidget(inputEdit_);

  sendBtn_ = new QPushButton(tr("Send"), this);
  sendBtn_->setMinimumHeight(32);
  sendBtn_->setFixedWidth(70);
  connect(sendBtn_, &QPushButton::clicked, this, &LLMChatWidget::onSendClicked);
  inputLayout->addWidget(sendBtn_);

  mainLayout->addWidget(inputArea);

  // Add initial system message
  addMessage(ChatMessageWidget::Role::System,
             tr("Welcome to LocalLLM Chat. Configure your Ollama connection in Settings > Local LLM to get started."));
}

void LLMChatWidget::addMessage(ChatMessageWidget::Role role, const QString& message) {
  // Remove the stretch from the layout temporarily
  QLayoutItem* stretch = chatLayout_->takeAt(chatLayout_->count() - 1);

  // Add the new message widget
  ChatMessageWidget* msgWidget = new ChatMessageWidget(role, message, chatContainer_);
  chatLayout_->addWidget(msgWidget);

  // Re-add the stretch to push messages up
  chatLayout_->addStretch();

  // Scroll to bottom
  QTimer::singleShot(10, this, [this]() {
    scrollArea_->verticalScrollBar()->setValue(
        scrollArea_->verticalScrollBar()->maximum());
  });

  delete stretch;
}

void LLMChatWidget::clearChat() {
  // Remove all widgets except the stretch
  while (chatLayout_->count() > 1) {
    QLayoutItem* item = chatLayout_->takeAt(0);
    if (item->widget()) {
      delete item->widget();
    }
    delete item;
  }

  // Add welcome message again
  addMessage(ChatMessageWidget::Role::System,
             tr("Chat cleared. Start a new conversation."));
}

void LLMChatWidget::onSendClicked() {
  QString message = inputEdit_->text().trimmed();
  if (message.isEmpty()) return;

  OllamaManager& mgr = OllamaManager::instance();

  // Check if we can send
  if (!mgr.isEnabled()) {
    addMessage(ChatMessageWidget::Role::System,
               tr("Local LLM is disabled. Enable it in Settings > Local LLM."));
    return;
  }

  if (!mgr.isOllamaRunning()) {
    addMessage(ChatMessageWidget::Role::System,
               tr("Ollama is not connected. Check your connection in Settings > Local LLM."));
    return;
  }

  if (mgr.selectedModel().isEmpty()) {
    addMessage(ChatMessageWidget::Role::System,
               tr("No model selected. Select a model in Settings > Local LLM."));
    return;
  }

  if (isWaitingForResponse_) {
    return;  // Already waiting for a response
  }

  // Add user message to chat
  addMessage(ChatMessageWidget::Role::User, message);
  inputEdit_->clear();

  // Show waiting state
  isWaitingForResponse_ = true;
  setInputEnabled(false);
  statusLabel_->setText(tr("Generating response..."));

  // Send to Ollama
  QString systemPrompt = tr("You are a helpful AI assistant integrated into ROS Weaver, "
                            "a visual ROS2 package editor. Help users with ROS2 development, "
                            "robotics questions, and general programming assistance. "
                            "Be concise and helpful.");
  mgr.generateCompletion(message, systemPrompt);

  emit messageSent(message);
}

void LLMChatWidget::onOllamaStatusChanged(bool running) {
  Q_UNUSED(running);
  updateStatusDisplay();
}

void LLMChatWidget::onCompletionReceived(const QString& response) {
  isWaitingForResponse_ = false;
  setInputEnabled(true);
  updateStatusDisplay();

  addMessage(ChatMessageWidget::Role::Assistant, response);
}

void LLMChatWidget::onCompletionError(const QString& error) {
  isWaitingForResponse_ = false;
  setInputEnabled(true);
  updateStatusDisplay();

  addMessage(ChatMessageWidget::Role::System,
             tr("Error: %1").arg(error));
}

void LLMChatWidget::onSettingsChanged() {
  updateStatusDisplay();
}

void LLMChatWidget::updateStatusDisplay() {
  OllamaManager& mgr = OllamaManager::instance();

  if (!mgr.isEnabled()) {
    statusLabel_->setText(tr("Local LLM disabled"));
    statusLabel_->setStyleSheet("color: gray;");
    setInputEnabled(false);
  } else if (!mgr.isOllamaRunning()) {
    statusLabel_->setText(tr("Ollama not connected"));
    statusLabel_->setStyleSheet("color: #f44336;");
    setInputEnabled(false);
  } else if (mgr.selectedModel().isEmpty()) {
    statusLabel_->setText(tr("No model selected"));
    statusLabel_->setStyleSheet("color: #FF9800;");
    setInputEnabled(false);
  } else {
    statusLabel_->setText(tr("Connected: %1").arg(mgr.selectedModel()));
    statusLabel_->setStyleSheet("color: #4CAF50;");
    setInputEnabled(!isWaitingForResponse_);
  }
}

void LLMChatWidget::setInputEnabled(bool enabled) {
  inputEdit_->setEnabled(enabled);
  sendBtn_->setEnabled(enabled);
}

}  // namespace ros_weaver
