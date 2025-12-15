#ifndef ROS_WEAVER_LLM_CHAT_WIDGET_HPP
#define ROS_WEAVER_LLM_CHAT_WIDGET_HPP

#include <QWidget>
#include <QTextBrowser>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QFrame>
#include <QComboBox>
#include <QElapsedTimer>
#include <QJsonObject>
#include "ros_weaver/core/ollama_manager.hpp"

namespace ros_weaver {

class PackageBlock;
class ConnectionLine;
class NodeGroup;
class WeaverCanvas;

// Individual chat message widget with markdown support
class ChatMessageWidget : public QWidget {
  Q_OBJECT

public:
  enum class Role { User, Assistant, System };

  ChatMessageWidget(Role role, const QString& message, QWidget* parent = nullptr);

  // For streaming: append text to this message
  void appendText(const QString& text);

  // Get current text
  QString text() const { return currentText_; }

  // Update display (rerender markdown)
  void updateDisplay();

private:
  void setupUi(Role role);
  QString convertMarkdownToHtml(const QString& markdown);

  Role role_;
  QString currentText_;
  QTextBrowser* textBrowser_;
};

// Main LLM Chat Widget
class LLMChatWidget : public QWidget {
  Q_OBJECT

public:
  explicit LLMChatWidget(QWidget* parent = nullptr);
  ~LLMChatWidget() override;

  // Clear chat history
  void clearChat();

  // Send a message programmatically (useful for automated analysis)
  void sendMessage(const QString& message);

  // Set the canvas for AI tool registration and context
  void setCanvas(WeaverCanvas* canvas);

  // Ask AI about specific elements (triggered from context menu)
  void askAboutBlock(PackageBlock* block);
  void askAboutConnection(ConnectionLine* connection);
  void askAboutGroup(NodeGroup* group);
  void askAboutPin(PackageBlock* block, int pinIndex, bool isOutput);

signals:
  void messageSent(const QString& message);
  // Signal to request loading an example (handled by MainWindow)
  void loadExampleRequested(const QString& exampleName);

protected:
  // Override to handle paste events for images
  bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
  void onSendClicked();
  void onStopClicked();
  void onAttachClicked();
  void onRemoveAttachment();
  void onOllamaStatusChanged(bool running);
  // Legacy /api/generate callbacks (for fallback)
  void onCompletionStarted();
  void onCompletionToken(const QString& token);
  void onCompletionFinished(const QString& fullResponse);
  void onCompletionError(const QString& error);
  void onSettingsChanged();
  void onQuickQuestionSelected(int index);
  void handleClipboardPaste();

  // Native chat API callbacks (/api/chat with tools)
  void onChatStarted();
  void onChatToken(const QString& token);
  void onChatFinished(const QString& fullResponse, const ChatMessage& assistantMessage);
  void onChatError(const QString& error);
  void onToolCallsReceived(const QList<OllamaToolCall>& toolCalls);

  // AI Tool callbacks
  void onAIPermissionRequired(const QString& toolName, const QString& description,
                               const QJsonObject& params);
  void onAIToolExecuted(const QString& toolName, bool success, const QString& message);
  void onAIActionUndone(const QString& description);
  void onUndoStackChanged(int size);

private:
  void setupUi();
  void setupQuickQuestions();
  void setupAITools();
  void setupNativeToolCalling();
  ChatMessageWidget* addMessage(ChatMessageWidget::Role role, const QString& message);
  void updateStatusDisplay();
  void setInputEnabled(bool enabled);
  void scrollToBottom();
  QString gatherROSContext();
  void attachImageFromClipboard(const QImage& image);
  void processToolCalls(const QString& response);  // Legacy text-based parsing
  void executeNativeToolCalls(const QList<OllamaToolCall>& toolCalls);  // Native tool execution
  void sendToolResultsToModel();  // Continue conversation with tool results
  void showUndoNotification(const QString& actionDescription);
  QString buildEnhancedSystemPrompt();
  QList<OllamaTool> buildToolsList();  // Convert AITools to OllamaTools

  // UI components
  QWidget* chatContainer_;
  QVBoxLayout* chatLayout_;
  QScrollArea* scrollArea_;
  QLineEdit* inputEdit_;
  QPushButton* sendBtn_;
  QPushButton* stopBtn_;
  QPushButton* clearBtn_;
  QPushButton* attachBtn_;
  QLabel* statusLabel_;
  QComboBox* quickQuestionsCombo_;
  QWidget* bottomSpacer_;  // Buffer at bottom for scrolling

  // Attachment UI
  QFrame* attachmentBar_;
  QLabel* attachmentLabel_;
  QPushButton* removeAttachmentBtn_;

  // Current streaming message
  ChatMessageWidget* currentStreamingMessage_ = nullptr;

  // State
  bool isWaitingForResponse_ = false;

  // Token generation tracking
  QElapsedTimer tokenTimer_;
  int tokenCount_ = 0;

  // Attachment state
  QString attachedFilePath_;
  QString attachedFileContent_;  // For text files
  QString attachedImageBase64_;  // For image files
  bool attachedIsImage_ = false;

  // Canvas reference for AI tools
  WeaverCanvas* canvas_ = nullptr;

  // Undo button for AI actions
  QPushButton* undoBtn_ = nullptr;

  // Native tool calling state
  QList<ChatMessage> conversationHistory_;
  QList<OllamaTool> currentTools_;
  QList<OllamaToolCall> pendingToolCalls_;
  QMap<QString, QString> toolResults_;  // toolCallId -> result
  bool useNativeToolCalling_ = true;    // Use native /api/chat with tools
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_LLM_CHAT_WIDGET_HPP
