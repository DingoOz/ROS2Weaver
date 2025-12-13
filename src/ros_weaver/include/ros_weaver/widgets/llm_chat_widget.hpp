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

namespace ros_weaver {

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

signals:
  void messageSent(const QString& message);

protected:
  // Override to handle paste events for images
  bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
  void onSendClicked();
  void onStopClicked();
  void onAttachClicked();
  void onRemoveAttachment();
  void onOllamaStatusChanged(bool running);
  void onCompletionStarted();
  void onCompletionToken(const QString& token);
  void onCompletionFinished(const QString& fullResponse);
  void onCompletionError(const QString& error);
  void onSettingsChanged();
  void onQuickQuestionSelected(int index);
  void handleClipboardPaste();

private:
  void setupUi();
  void setupQuickQuestions();
  ChatMessageWidget* addMessage(ChatMessageWidget::Role role, const QString& message);
  void updateStatusDisplay();
  void setInputEnabled(bool enabled);
  void scrollToBottom();
  QString gatherROSContext();
  void attachImageFromClipboard(const QImage& image);

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
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_LLM_CHAT_WIDGET_HPP
