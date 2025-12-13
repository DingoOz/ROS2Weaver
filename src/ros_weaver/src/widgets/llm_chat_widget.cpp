#include "ros_weaver/widgets/llm_chat_widget.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include <QHBoxLayout>
#include <QScrollBar>
#include <QFrame>
#include <QApplication>
#include <QRegularExpression>
#include <QTimer>

namespace ros_weaver {

// ============== ChatMessageWidget ==============

ChatMessageWidget::ChatMessageWidget(Role role, const QString& message, QWidget* parent)
    : QWidget(parent)
    , role_(role)
    , currentText_(message)
    , textBrowser_(nullptr) {
  setupUi(role);
  updateDisplay();
}

void ChatMessageWidget::setupUi(Role role) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setSpacing(0);

  // Role label
  QLabel* roleLabel = new QLabel(this);
  roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; margin-bottom: 2px;");

  switch (role) {
    case Role::User:
      roleLabel->setText(tr("You"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #2a82da; margin-bottom: 2px;");
      break;
    case Role::Assistant:
      roleLabel->setText(tr("AI Assistant"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #4CAF50; margin-bottom: 2px;");
      break;
    case Role::System:
      roleLabel->setText(tr("System"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #888888; margin-bottom: 2px;");
      break;
  }
  layout->addWidget(roleLabel);

  // Text browser for markdown rendering
  textBrowser_ = new QTextBrowser(this);
  textBrowser_->setOpenExternalLinks(true);
  textBrowser_->setFrameShape(QFrame::NoFrame);
  textBrowser_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  textBrowser_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  // Style based on role - full width
  QString bgColor, textColor;
  switch (role) {
    case Role::User:
      bgColor = "#1e3a5f";
      textColor = "#e0e0e0";
      break;
    case Role::Assistant:
      bgColor = "#2d2d2d";
      textColor = "#e0e0e0";
      break;
    case Role::System:
      bgColor = "#3d3d3d";
      textColor = "#aaaaaa";
      break;
  }

  textBrowser_->setStyleSheet(QString(
      "QTextBrowser {"
      "  background-color: %1;"
      "  color: %2;"
      "  border-radius: 8px;"
      "  padding: 10px 14px;"
      "  font-size: 13px;"
      "  line-height: 1.4;"
      "}"
      "QTextBrowser a { color: #5dade2; }"
      "QTextBrowser code { background-color: #1a1a1a; padding: 2px 4px; border-radius: 3px; font-family: monospace; }"
      "QTextBrowser pre { background-color: #1a1a1a; padding: 10px; border-radius: 5px; font-family: monospace; }"
  ).arg(bgColor, textColor));

  layout->addWidget(textBrowser_);
}

void ChatMessageWidget::appendText(const QString& text) {
  currentText_ += text;
  updateDisplay();
}

void ChatMessageWidget::updateDisplay() {
  QString html = convertMarkdownToHtml(currentText_);
  textBrowser_->setHtml(html);

  // Adjust height to content
  QTextDocument* doc = textBrowser_->document();
  doc->setTextWidth(textBrowser_->viewport()->width());
  int height = static_cast<int>(doc->size().height()) + 10;
  textBrowser_->setMinimumHeight(height);
  textBrowser_->setMaximumHeight(height);
}

QString ChatMessageWidget::convertMarkdownToHtml(const QString& markdown) {
  QString html = markdown;

  // Escape HTML first (but preserve our formatting)
  html.replace("&", "&amp;");
  html.replace("<", "&lt;");
  html.replace(">", "&gt;");

  // Code blocks (```...```)
  QRegularExpression codeBlockRe("```(\\w*)\\n([\\s\\S]*?)```");
  html.replace(codeBlockRe, "<pre><code>\\2</code></pre>");

  // Inline code (`...`)
  QRegularExpression inlineCodeRe("`([^`]+)`");
  html.replace(inlineCodeRe, "<code>\\1</code>");

  // Bold (**...** or __...__)
  QRegularExpression boldRe("\\*\\*([^*]+)\\*\\*");
  html.replace(boldRe, "<b>\\1</b>");
  QRegularExpression boldRe2("__([^_]+)__");
  html.replace(boldRe2, "<b>\\1</b>");

  // Italic (*...* or _..._)
  QRegularExpression italicRe("\\*([^*]+)\\*");
  html.replace(italicRe, "<i>\\1</i>");
  QRegularExpression italicRe2("(?<!_)_([^_]+)_(?!_)");
  html.replace(italicRe2, "<i>\\1</i>");

  // Headers
  QRegularExpression h3Re("^### (.+)$", QRegularExpression::MultilineOption);
  html.replace(h3Re, "<h4>\\1</h4>");
  QRegularExpression h2Re("^## (.+)$", QRegularExpression::MultilineOption);
  html.replace(h2Re, "<h3>\\1</h3>");
  QRegularExpression h1Re("^# (.+)$", QRegularExpression::MultilineOption);
  html.replace(h1Re, "<h2>\\1</h2>");

  // Bullet lists
  QRegularExpression bulletRe("^[*-] (.+)$", QRegularExpression::MultilineOption);
  html.replace(bulletRe, "<li>\\1</li>");

  // Numbered lists
  QRegularExpression numRe("^\\d+\\. (.+)$", QRegularExpression::MultilineOption);
  html.replace(numRe, "<li>\\1</li>");

  // Links [text](url)
  QRegularExpression linkRe("\\[([^\\]]+)\\]\\(([^)]+)\\)");
  html.replace(linkRe, "<a href=\"\\2\">\\1</a>");

  // Line breaks
  html.replace("\n", "<br>");

  // Clean up consecutive <br> after block elements
  html.replace(QRegularExpression("</pre><br>"), "</pre>");
  html.replace(QRegularExpression("</h[234]><br>"), "</h\\1>");

  return html;
}

// ============== LLMChatWidget ==============

LLMChatWidget::LLMChatWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();

  // Connect to OllamaManager signals
  OllamaManager& mgr = OllamaManager::instance();
  connect(&mgr, &OllamaManager::ollamaStatusChanged,
          this, &LLMChatWidget::onOllamaStatusChanged);
  connect(&mgr, &OllamaManager::completionStarted,
          this, &LLMChatWidget::onCompletionStarted);
  connect(&mgr, &OllamaManager::completionToken,
          this, &LLMChatWidget::onCompletionToken);
  connect(&mgr, &OllamaManager::completionFinished,
          this, &LLMChatWidget::onCompletionFinished);
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

  clearBtn_ = new QPushButton(tr("Clear"), this);
  clearBtn_->setFixedWidth(60);
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
  scrollArea_->setStyleSheet("QScrollArea { border: none; background-color: #1e1e1e; }");

  chatContainer_ = new QWidget();
  chatContainer_->setStyleSheet("background-color: #1e1e1e;");
  chatLayout_ = new QVBoxLayout(chatContainer_);
  chatLayout_->setContentsMargins(8, 8, 8, 8);
  chatLayout_->setSpacing(12);
  chatLayout_->addStretch();  // Push messages to top initially

  scrollArea_->setWidget(chatContainer_);
  mainLayout->addWidget(scrollArea_, 1);

  // Input area
  QWidget* inputArea = new QWidget(this);
  inputArea->setStyleSheet("background-color: #2a2a2a;");
  QHBoxLayout* inputLayout = new QHBoxLayout(inputArea);
  inputLayout->setContentsMargins(8, 8, 8, 8);
  inputLayout->setSpacing(8);

  inputEdit_ = new QLineEdit(this);
  inputEdit_->setPlaceholderText(tr("Type a message... (Enter to send)"));
  inputEdit_->setMinimumHeight(36);
  inputEdit_->setStyleSheet(
      "QLineEdit {"
      "  background-color: #3a3a3a;"
      "  border: 1px solid #505050;"
      "  border-radius: 4px;"
      "  padding: 8px;"
      "  color: white;"
      "}"
      "QLineEdit:focus { border-color: #2a82da; }");
  connect(inputEdit_, &QLineEdit::returnPressed, this, &LLMChatWidget::onSendClicked);
  inputLayout->addWidget(inputEdit_);

  sendBtn_ = new QPushButton(tr("Send"), this);
  sendBtn_->setMinimumHeight(36);
  sendBtn_->setFixedWidth(70);
  sendBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #2a82da;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-weight: bold;"
      "}"
      "QPushButton:hover { background-color: #3d9ae8; }"
      "QPushButton:disabled { background-color: #555555; }");
  connect(sendBtn_, &QPushButton::clicked, this, &LLMChatWidget::onSendClicked);
  inputLayout->addWidget(sendBtn_);

  stopBtn_ = new QPushButton(tr("Stop"), this);
  stopBtn_->setMinimumHeight(36);
  stopBtn_->setFixedWidth(60);
  stopBtn_->setVisible(false);
  stopBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #c0392b;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-weight: bold;"
      "}"
      "QPushButton:hover { background-color: #e74c3c; }");
  connect(stopBtn_, &QPushButton::clicked, this, []() {
    OllamaManager::instance().cancelCompletion();
  });
  inputLayout->addWidget(stopBtn_);

  mainLayout->addWidget(inputArea);

  // Add initial system message
  addMessage(ChatMessageWidget::Role::System,
             tr("Welcome to LocalLLM Chat. Configure your Ollama connection in Settings > Local LLM to get started."));
}

ChatMessageWidget* LLMChatWidget::addMessage(ChatMessageWidget::Role role, const QString& message) {
  // Remove the stretch from the layout temporarily
  QLayoutItem* stretch = chatLayout_->takeAt(chatLayout_->count() - 1);

  // Add the new message widget
  ChatMessageWidget* msgWidget = new ChatMessageWidget(role, message, chatContainer_);
  chatLayout_->addWidget(msgWidget);

  // Re-add the stretch to push messages up
  chatLayout_->addStretch();

  // Scroll to bottom
  scrollToBottom();

  delete stretch;
  return msgWidget;
}

void LLMChatWidget::scrollToBottom() {
  QTimer::singleShot(10, this, [this]() {
    scrollArea_->verticalScrollBar()->setValue(
        scrollArea_->verticalScrollBar()->maximum());
  });
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

  currentStreamingMessage_ = nullptr;

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
  sendBtn_->setVisible(false);
  stopBtn_->setVisible(true);
  statusLabel_->setText(tr("Generating..."));

  // Send to Ollama
  QString systemPrompt = tr("You are a helpful AI assistant integrated into ROS Weaver, "
                            "a visual ROS2 package editor. Help users with ROS2 development, "
                            "robotics questions, and general programming assistance. "
                            "Be concise and helpful. Use markdown formatting for code blocks and lists.");
  mgr.generateCompletion(message, systemPrompt);

  emit messageSent(message);
}

void LLMChatWidget::onOllamaStatusChanged(bool running) {
  Q_UNUSED(running);
  updateStatusDisplay();
}

void LLMChatWidget::onCompletionStarted() {
  // Create empty assistant message that will be filled in via streaming
  currentStreamingMessage_ = addMessage(ChatMessageWidget::Role::Assistant, "");
}

void LLMChatWidget::onCompletionToken(const QString& token) {
  if (currentStreamingMessage_) {
    currentStreamingMessage_->appendText(token);
    scrollToBottom();
  }
}

void LLMChatWidget::onCompletionFinished(const QString& fullResponse) {
  Q_UNUSED(fullResponse);

  isWaitingForResponse_ = false;
  currentStreamingMessage_ = nullptr;
  setInputEnabled(true);
  sendBtn_->setVisible(true);
  stopBtn_->setVisible(false);
  updateStatusDisplay();
}

void LLMChatWidget::onCompletionError(const QString& error) {
  isWaitingForResponse_ = false;
  currentStreamingMessage_ = nullptr;
  setInputEnabled(true);
  sendBtn_->setVisible(true);
  stopBtn_->setVisible(false);
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
    statusLabel_->setText(tr("Ready: %1").arg(mgr.selectedModel()));
    statusLabel_->setStyleSheet("color: #4CAF50;");
    setInputEnabled(!isWaitingForResponse_);
  }
}

void LLMChatWidget::setInputEnabled(bool enabled) {
  inputEdit_->setEnabled(enabled);
  sendBtn_->setEnabled(enabled);
}

}  // namespace ros_weaver
