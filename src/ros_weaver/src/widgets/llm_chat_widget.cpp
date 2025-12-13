#include "ros_weaver/widgets/llm_chat_widget.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include <QHBoxLayout>
#include <QScrollBar>
#include <QFrame>
#include <QApplication>
#include <QRegularExpression>
#include <QTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QMimeDatabase>
#include <QImageReader>
#include <QClipboard>
#include <QMimeData>
#include <QBuffer>
#include <QKeyEvent>
#include <QColor>

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
  QHBoxLayout* outerLayout = new QHBoxLayout(this);
  outerLayout->setContentsMargins(8, 4, 8, 4);
  outerLayout->setSpacing(0);

  // Container for the message content
  QWidget* contentWidget = new QWidget(this);
  QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);
  contentLayout->setContentsMargins(0, 0, 0, 0);
  contentLayout->setSpacing(0);

  // Role label
  QLabel* roleLabel = new QLabel(contentWidget);

  // Get appearance settings from OllamaManager
  OllamaManager& mgr = OllamaManager::instance();
  QFont chatFont = mgr.chatFont();
  int fontSize = mgr.chatFontSize();
  QColor userTextColor = mgr.userMessageColor();
  QColor userBgColor = mgr.userBackgroundColor();
  QColor assistantTextColor = mgr.assistantMessageColor();
  QColor assistantBgColor = mgr.assistantBackgroundColor();
  QColor codeBgColor = mgr.codeBackgroundColor();

  // Use defaults if colors are invalid
  if (!userTextColor.isValid()) userTextColor = OllamaManager::defaultUserMessageColor();
  if (!userBgColor.isValid()) userBgColor = OllamaManager::defaultUserBackgroundColor();
  if (!assistantTextColor.isValid()) assistantTextColor = OllamaManager::defaultAssistantMessageColor();
  if (!assistantBgColor.isValid()) assistantBgColor = OllamaManager::defaultAssistantBackgroundColor();
  if (!codeBgColor.isValid()) codeBgColor = OllamaManager::defaultCodeBackgroundColor();

  // Style and alignment based on role
  QString bgColor, textColor;
  switch (role) {
    case Role::User:
      roleLabel->setText(tr("You"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #2a82da; margin-bottom: 2px;");
      roleLabel->setAlignment(Qt::AlignRight);
      bgColor = userBgColor.name();
      textColor = userTextColor.name();
      // User: 80% width, right-aligned (spacer on left)
      outerLayout->addStretch(1);
      outerLayout->addWidget(contentWidget, 4);
      break;
    case Role::Assistant:
      roleLabel->setText(tr("AI Assistant"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #4CAF50; margin-bottom: 2px;");
      bgColor = assistantBgColor.name();
      textColor = assistantTextColor.name();
      // Assistant: 80% width, left-aligned (spacer on right)
      outerLayout->addWidget(contentWidget, 4);
      outerLayout->addStretch(1);
      break;
    case Role::System:
      roleLabel->setText(tr("System"));
      roleLabel->setStyleSheet("font-weight: bold; font-size: 11px; color: #888888; margin-bottom: 2px;");
      roleLabel->setAlignment(Qt::AlignCenter);
      bgColor = "#3d3d3d";
      textColor = "#aaaaaa";
      // System: centered, 80% width
      outerLayout->addStretch(1);
      outerLayout->addWidget(contentWidget, 4);
      outerLayout->addStretch(1);
      break;
  }
  contentLayout->addWidget(roleLabel);

  // Text browser for markdown rendering
  textBrowser_ = new QTextBrowser(contentWidget);
  textBrowser_->setOpenExternalLinks(true);
  textBrowser_->setFrameShape(QFrame::NoFrame);
  textBrowser_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  textBrowser_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  // Apply custom font
  if (chatFont.family().isEmpty()) {
    chatFont = OllamaManager::defaultChatFont();
  }
  chatFont.setPointSize(fontSize > 0 ? fontSize : OllamaManager::defaultChatFontSize());
  textBrowser_->setFont(chatFont);

  textBrowser_->setStyleSheet(QString(
      "QTextBrowser {"
      "  background-color: %1;"
      "  color: %2;"
      "  border-radius: 8px;"
      "  padding: 10px 14px;"
      "  font-size: %3px;"
      "  line-height: 1.4;"
      "}"
      "QTextBrowser a { color: #5dade2; }"
      "QTextBrowser code { background-color: %4; padding: 2px 4px; border-radius: 3px; font-family: monospace; }"
      "QTextBrowser pre { background-color: %4; padding: 10px; border-radius: 5px; font-family: monospace; }"
  ).arg(bgColor, textColor, QString::number(fontSize), codeBgColor.name()));

  contentLayout->addWidget(textBrowser_);
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
    : QWidget(parent)
    , quickQuestionsCombo_(nullptr)
    , bottomSpacer_(nullptr)
    , tokenCount_(0) {
  setupUi();

  // Install event filter for paste handling
  inputEdit_->installEventFilter(this);

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

  // Quick questions dropdown
  QWidget* quickQuestionsBar = new QWidget(this);
  QHBoxLayout* quickQuestionsLayout = new QHBoxLayout(quickQuestionsBar);
  quickQuestionsLayout->setContentsMargins(8, 4, 8, 4);

  QLabel* quickLabel = new QLabel(tr("Quick:"), this);
  quickLabel->setStyleSheet("color: gray; font-size: 11px;");
  quickQuestionsLayout->addWidget(quickLabel);

  quickQuestionsCombo_ = new QComboBox(this);
  quickQuestionsCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  quickQuestionsCombo_->setStyleSheet(
      "QComboBox {"
      "  background-color: #3a3a3a;"
      "  border: 1px solid #505050;"
      "  border-radius: 4px;"
      "  padding: 4px 8px;"
      "  color: white;"
      "  font-size: 11px;"
      "}"
      "QComboBox:hover { border-color: #606060; }"
      "QComboBox::drop-down { border: none; width: 20px; }"
      "QComboBox::down-arrow { image: none; border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 5px solid #888; }"
      "QComboBox QAbstractItemView { background-color: #3a3a3a; color: white; selection-background-color: #2a82da; }");
  setupQuickQuestions();
  connect(quickQuestionsCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &LLMChatWidget::onQuickQuestionSelected);
  quickQuestionsLayout->addWidget(quickQuestionsCombo_);

  mainLayout->addWidget(quickQuestionsBar);

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

  // Add bottom spacer to prevent text from sitting too low during streaming
  bottomSpacer_ = new QWidget(chatContainer_);
  bottomSpacer_->setFixedHeight(60);
  bottomSpacer_->setStyleSheet("background-color: transparent;");
  chatLayout_->addWidget(bottomSpacer_);

  scrollArea_->setWidget(chatContainer_);
  mainLayout->addWidget(scrollArea_, 1);

  // Attachment indicator bar (hidden by default)
  attachmentBar_ = new QFrame(this);
  attachmentBar_->setStyleSheet(
      "QFrame {"
      "  background-color: #3a4a5a;"
      "  border-top: 1px solid #505050;"
      "  padding: 4px;"
      "}");
  attachmentBar_->setVisible(false);
  QHBoxLayout* attachBarLayout = new QHBoxLayout(attachmentBar_);
  attachBarLayout->setContentsMargins(8, 4, 8, 4);
  attachBarLayout->setSpacing(8);

  QLabel* attachIcon = new QLabel(QString::fromUtf8("\xF0\x9F\x93\x8E"), this);  // Paperclip emoji
  attachBarLayout->addWidget(attachIcon);

  attachmentLabel_ = new QLabel(this);
  attachmentLabel_->setStyleSheet("color: #e0e0e0; font-size: 12px;");
  attachBarLayout->addWidget(attachmentLabel_, 1);

  removeAttachmentBtn_ = new QPushButton(QString::fromUtf8("\xE2\x9C\x95"), this);  // X symbol
  removeAttachmentBtn_->setFixedSize(20, 20);
  removeAttachmentBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: transparent;"
      "  color: #aaaaaa;"
      "  border: none;"
      "  font-size: 12px;"
      "}"
      "QPushButton:hover { color: #ff6666; }");
  connect(removeAttachmentBtn_, &QPushButton::clicked, this, &LLMChatWidget::onRemoveAttachment);
  attachBarLayout->addWidget(removeAttachmentBtn_);

  mainLayout->addWidget(attachmentBar_);

  // Input area
  QWidget* inputArea = new QWidget(this);
  inputArea->setStyleSheet("background-color: #2a2a2a;");
  QHBoxLayout* inputLayout = new QHBoxLayout(inputArea);
  inputLayout->setContentsMargins(8, 8, 8, 8);
  inputLayout->setSpacing(8);

  // Attach button
  attachBtn_ = new QPushButton(QString::fromUtf8("\xF0\x9F\x93\x8E"), this);  // Paperclip emoji
  attachBtn_->setMinimumHeight(36);
  attachBtn_->setFixedWidth(40);
  attachBtn_->setToolTip(tr("Attach file (text or image)"));
  attachBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #3a3a3a;"
      "  border: 1px solid #505050;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-size: 16px;"
      "}"
      "QPushButton:hover { background-color: #4a4a4a; border-color: #606060; }"
      "QPushButton:disabled { background-color: #333333; color: #666666; }");
  connect(attachBtn_, &QPushButton::clicked, this, &LLMChatWidget::onAttachClicked);
  inputLayout->addWidget(attachBtn_);

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
  connect(stopBtn_, &QPushButton::clicked, this, &LLMChatWidget::onStopClicked);
  inputLayout->addWidget(stopBtn_);

  mainLayout->addWidget(inputArea);

  // Add initial system message
  addMessage(ChatMessageWidget::Role::System,
             tr("Welcome to LocalLLM Chat. Configure your Ollama connection in Settings > Local LLM to get started."));
}

ChatMessageWidget* LLMChatWidget::addMessage(ChatMessageWidget::Role role, const QString& message) {
  // Remove the bottom spacer and stretch from the layout temporarily
  // Layout order: [stretch, messages..., bottomSpacer]
  QLayoutItem* spacerItem = chatLayout_->takeAt(chatLayout_->count() - 1);  // bottomSpacer
  QLayoutItem* stretch = chatLayout_->takeAt(chatLayout_->count() - 1);      // stretch

  // Add the new message widget
  ChatMessageWidget* msgWidget = new ChatMessageWidget(role, message, chatContainer_);
  chatLayout_->addWidget(msgWidget);

  // Re-add the stretch and bottom spacer
  chatLayout_->addStretch();
  chatLayout_->addItem(spacerItem);

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
  // Remove all widgets except the stretch and bottom spacer
  // Layout order: [stretch, messages..., bottomSpacer]
  while (chatLayout_->count() > 2) {
    // Remove items between stretch (index 0) and bottomSpacer (last item)
    QLayoutItem* item = chatLayout_->takeAt(1);
    if (item->widget() && item->widget() != bottomSpacer_) {
      delete item->widget();
    }
    delete item;
  }

  currentStreamingMessage_ = nullptr;

  // Clear any pending attachment
  onRemoveAttachment();

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

  // Build the prompt with file attachment if present
  QString fullPrompt = message;
  QStringList images;
  QString displayMessage = message;

  if (!attachedFilePath_.isEmpty()) {
    QFileInfo fileInfo(attachedFilePath_);

    if (attachedIsImage_) {
      // For images, send via the images parameter
      images.append(attachedImageBase64_);
      displayMessage = QString("[Image: %1]\n%2").arg(fileInfo.fileName(), message);
    } else {
      // For text files, prepend content to the prompt
      fullPrompt = QString("Here is the content of file '%1':\n```\n%2\n```\n\n%3")
                       .arg(fileInfo.fileName(), attachedFileContent_, message);
      displayMessage = QString("[File: %1]\n%2").arg(fileInfo.fileName(), message);
    }

    // Clear attachment after use
    onRemoveAttachment();
  }

  // Add user message to chat (show what user typed + attachment indicator)
  addMessage(ChatMessageWidget::Role::User, displayMessage);
  inputEdit_->clear();

  // Show waiting state
  isWaitingForResponse_ = true;
  setInputEnabled(false);
  sendBtn_->setVisible(false);
  stopBtn_->setVisible(true);
  statusLabel_->setText(tr("Generating..."));

  // Send to Ollama with the configured system prompt and any images
  mgr.generateCompletion(fullPrompt, mgr.systemPrompt(), images);

  emit messageSent(message);
}

void LLMChatWidget::onStopClicked() {
  // Cancel the completion
  OllamaManager::instance().cancelCompletion();

  // Immediately reset UI state so user can type
  isWaitingForResponse_ = false;
  currentStreamingMessage_ = nullptr;
  setInputEnabled(true);
  sendBtn_->setVisible(true);
  stopBtn_->setVisible(false);
  updateStatusDisplay();

  // Focus the input field for immediate typing
  inputEdit_->setFocus();
}

void LLMChatWidget::sendMessage(const QString& message) {
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
    addMessage(ChatMessageWidget::Role::System,
               tr("Please wait for the current response to complete."));
    return;
  }

  // Add user message to chat
  addMessage(ChatMessageWidget::Role::User, message);

  // Show waiting state
  isWaitingForResponse_ = true;
  setInputEnabled(false);
  sendBtn_->setVisible(false);
  stopBtn_->setVisible(true);
  statusLabel_->setText(tr("Generating..."));

  // Send to Ollama with the configured system prompt
  mgr.generateCompletion(message, mgr.systemPrompt(), QStringList());

  emit messageSent(message);
}

void LLMChatWidget::onAttachClicked() {
  // Build file filter
  QString imageFormats;
  for (const QByteArray& format : QImageReader::supportedImageFormats()) {
    imageFormats += "*." + QString(format).toLower() + " ";
  }

  QString filter = tr("All Supported Files (*.txt *.md *.py *.cpp *.hpp *.c *.h *.js *.ts *.json *.xml *.yaml *.yml *.cmake *.sh *.bash *.launch *.urdf *.xacro *.msg *.srv *.action %1);;"
                      "Text Files (*.txt *.md *.py *.cpp *.hpp *.c *.h *.js *.ts *.json *.xml *.yaml *.yml *.cmake *.sh *.bash);;"
                      "ROS Files (*.launch *.urdf *.xacro *.msg *.srv *.action);;"
                      "Images (%1);;"
                      "All Files (*)").arg(imageFormats.trimmed());

  QString filePath = QFileDialog::getOpenFileName(
      this, tr("Attach File"), QString(), filter);

  if (filePath.isEmpty()) {
    return;
  }

  QFileInfo fileInfo(filePath);
  if (!fileInfo.exists()) {
    addMessage(ChatMessageWidget::Role::System, tr("File not found: %1").arg(filePath));
    return;
  }

  // Check file size (limit to 1MB for text, 10MB for images)
  qint64 maxSize = 1024 * 1024;  // 1MB default
  QMimeDatabase mimeDb;
  QMimeType mimeType = mimeDb.mimeTypeForFile(filePath);
  bool isImage = mimeType.name().startsWith("image/");

  if (isImage) {
    maxSize = 10 * 1024 * 1024;  // 10MB for images
  }

  if (fileInfo.size() > maxSize) {
    addMessage(ChatMessageWidget::Role::System,
               tr("File too large. Maximum size: %1 MB").arg(maxSize / (1024 * 1024)));
    return;
  }

  // Clear previous attachment
  onRemoveAttachment();

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    addMessage(ChatMessageWidget::Role::System, tr("Could not open file: %1").arg(filePath));
    return;
  }

  attachedFilePath_ = filePath;
  attachedIsImage_ = isImage;

  if (isImage) {
    // Check if current model supports vision
    OllamaManager& mgr = OllamaManager::instance();
    if (!OllamaManager::isVisionModel(mgr.selectedModel())) {
      addMessage(ChatMessageWidget::Role::System,
                 tr("Note: Current model '%1' may not support images. Consider using a vision model like 'llava' or 'llama3.2-vision'.").arg(mgr.selectedModel()));
    }

    // Read and base64 encode the image
    QByteArray imageData = file.readAll();
    attachedImageBase64_ = QString::fromLatin1(imageData.toBase64());
    attachedFileContent_.clear();
  } else {
    // Read text content
    QTextStream stream(&file);
    attachedFileContent_ = stream.readAll();
    attachedImageBase64_.clear();

    // Truncate if too long (show warning)
    if (attachedFileContent_.length() > 50000) {
      attachedFileContent_ = attachedFileContent_.left(50000);
      addMessage(ChatMessageWidget::Role::System,
                 tr("File truncated to 50,000 characters."));
    }
  }
  file.close();

  // Update UI to show attachment
  QString displayName = fileInfo.fileName();
  QString sizeStr;
  if (fileInfo.size() < 1024) {
    sizeStr = QString::number(fileInfo.size()) + " B";
  } else if (fileInfo.size() < 1024 * 1024) {
    sizeStr = QString::number(fileInfo.size() / 1024.0, 'f', 1) + " KB";
  } else {
    sizeStr = QString::number(fileInfo.size() / (1024.0 * 1024.0), 'f', 1) + " MB";
  }

  QString typeStr = isImage ? tr("Image") : tr("Text");
  attachmentLabel_->setText(QString("%1 (%2, %3)").arg(displayName, typeStr, sizeStr));
  attachmentBar_->setVisible(true);
}

void LLMChatWidget::onRemoveAttachment() {
  attachedFilePath_.clear();
  attachedFileContent_.clear();
  attachedImageBase64_.clear();
  attachedIsImage_ = false;
  attachmentBar_->setVisible(false);
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
  attachBtn_->setEnabled(enabled);
  if (quickQuestionsCombo_) {
    quickQuestionsCombo_->setEnabled(enabled);
  }
}

void LLMChatWidget::setupQuickQuestions() {
  quickQuestionsCombo_->clear();
  quickQuestionsCombo_->addItem(tr("-- Select a quick question --"));

  // Diagnostics
  quickQuestionsCombo_->addItem(tr("🔍 What ROS2 topics are active?"));
  quickQuestionsCombo_->addItem(tr("🔍 What ROS2 nodes are running?"));
  quickQuestionsCombo_->addItem(tr("🔍 Show the TF tree structure"));
  quickQuestionsCombo_->addItem(tr("🔍 Are there any unconnected topics?"));
  quickQuestionsCombo_->addItem(tr("🔍 Analyze my robot's current state"));

  // Help & Learning
  quickQuestionsCombo_->addItem(tr("📚 Explain ROS2 topics vs services"));
  quickQuestionsCombo_->addItem(tr("📚 How do I create a launch file?"));
  quickQuestionsCombo_->addItem(tr("📚 What is tf2 and how does it work?"));
  quickQuestionsCombo_->addItem(tr("📚 Best practices for ROS2 node design"));

  // Code Generation
  quickQuestionsCombo_->addItem(tr("💻 Generate a simple publisher node"));
  quickQuestionsCombo_->addItem(tr("💻 Generate a subscriber node"));
  quickQuestionsCombo_->addItem(tr("💻 Create a service server example"));
  quickQuestionsCombo_->addItem(tr("💻 Write a basic launch file"));

  // Debugging
  quickQuestionsCombo_->addItem(tr("🐛 Why might my robot not be moving?"));
  quickQuestionsCombo_->addItem(tr("🐛 Common TF tree issues and fixes"));
  quickQuestionsCombo_->addItem(tr("🐛 Debug nav2 navigation problems"));
}

void LLMChatWidget::onQuickQuestionSelected(int index) {
  if (index <= 0) return;  // Skip placeholder

  QString question = quickQuestionsCombo_->itemText(index);

  // Remove emoji prefix if present (first 2-4 characters before space)
  int spaceIdx = question.indexOf(' ');
  if (spaceIdx > 0 && spaceIdx <= 4) {
    question = question.mid(spaceIdx + 1);
  }

  // Check if this is a diagnostics question that should include ROS context
  bool includeContext = question.contains("topics") ||
                        question.contains("nodes") ||
                        question.contains("TF tree") ||
                        question.contains("robot's current state") ||
                        question.contains("unconnected");

  if (includeContext) {
    QString context = gatherROSContext();
    if (!context.isEmpty()) {
      question = context + "\n\n" + question;
    }
  }

  inputEdit_->setText(question);
  inputEdit_->setFocus();

  // Reset combo to placeholder
  quickQuestionsCombo_->setCurrentIndex(0);
}

bool LLMChatWidget::eventFilter(QObject* obj, QEvent* event) {
  if (obj == inputEdit_ && event->type() == QEvent::KeyPress) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);

    // Check for Ctrl+V (paste)
    if (keyEvent->matches(QKeySequence::Paste)) {
      handleClipboardPaste();
      return true;  // Event handled
    }
  }
  return QWidget::eventFilter(obj, event);
}

void LLMChatWidget::handleClipboardPaste() {
  QClipboard* clipboard = QApplication::clipboard();
  const QMimeData* mimeData = clipboard->mimeData();

  if (mimeData->hasImage()) {
    // Paste image from clipboard
    QImage image = qvariant_cast<QImage>(mimeData->imageData());
    if (!image.isNull()) {
      attachImageFromClipboard(image);
      return;
    }
  }

  // Fall back to default text paste behavior
  if (mimeData->hasText()) {
    inputEdit_->insert(mimeData->text());
  }
}

void LLMChatWidget::attachImageFromClipboard(const QImage& image) {
  // Check if current model supports vision
  OllamaManager& mgr = OllamaManager::instance();
  if (!OllamaManager::isVisionModel(mgr.selectedModel())) {
    addMessage(ChatMessageWidget::Role::System,
               tr("Note: Current model '%1' may not support images. Consider using a vision model like 'llava' or 'llama3.2-vision'.").arg(mgr.selectedModel()));
  }

  // Clear previous attachment
  onRemoveAttachment();

  // Convert image to base64 PNG
  QByteArray imageData;
  QBuffer buffer(&imageData);
  buffer.open(QIODevice::WriteOnly);
  image.save(&buffer, "PNG");
  buffer.close();

  attachedImageBase64_ = QString::fromLatin1(imageData.toBase64());
  attachedFilePath_ = tr("Clipboard Image");
  attachedIsImage_ = true;
  attachedFileContent_.clear();

  // Calculate size
  QString sizeStr;
  if (imageData.size() < 1024) {
    sizeStr = QString::number(imageData.size()) + " B";
  } else if (imageData.size() < 1024 * 1024) {
    sizeStr = QString::number(imageData.size() / 1024.0, 'f', 1) + " KB";
  } else {
    sizeStr = QString::number(imageData.size() / (1024.0 * 1024.0), 'f', 1) + " MB";
  }

  // Update UI
  attachmentLabel_->setText(tr("Pasted Image (%1x%2, %3)")
                                .arg(image.width())
                                .arg(image.height())
                                .arg(sizeStr));
  attachmentBar_->setVisible(true);
}

QString LLMChatWidget::gatherROSContext() {
  // This function gathers available ROS2 system information
  // to provide context to the AI for diagnostics questions

  QString context;
  context += "=== Current ROS2 System State ===\n";

  // Note: In a full implementation, this would query the ROS2 system
  // For now, we provide a placeholder that can be expanded
  // The actual data would come from the ROS2 node integration

  // TODO: Integrate with the existing ROS2 scanning functionality
  // to get actual topics, nodes, and TF tree information

  context += "\n[ROS2 context data would be gathered here from the running system]\n";
  context += "Please provide general ROS2 assistance based on the question.\n";

  return context;
}

}  // namespace ros_weaver
