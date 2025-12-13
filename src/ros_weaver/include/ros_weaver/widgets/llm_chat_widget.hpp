#ifndef ROS_WEAVER_LLM_CHAT_WIDGET_HPP
#define ROS_WEAVER_LLM_CHAT_WIDGET_HPP

#include <QWidget>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QScrollArea>

namespace ros_weaver {

// Individual chat message widget
class ChatMessageWidget : public QWidget {
  Q_OBJECT

public:
  enum class Role { User, Assistant, System };

  ChatMessageWidget(Role role, const QString& message, QWidget* parent = nullptr);

private:
  void setupUi(Role role, const QString& message);
};

// Main LLM Chat Widget
class LLMChatWidget : public QWidget {
  Q_OBJECT

public:
  explicit LLMChatWidget(QWidget* parent = nullptr);
  ~LLMChatWidget() override;

  // Clear chat history
  void clearChat();

signals:
  void messageSent(const QString& message);

private slots:
  void onSendClicked();
  void onOllamaStatusChanged(bool running);
  void onCompletionReceived(const QString& response);
  void onCompletionError(const QString& error);
  void onSettingsChanged();

private:
  void setupUi();
  void addMessage(ChatMessageWidget::Role role, const QString& message);
  void updateStatusDisplay();
  void setInputEnabled(bool enabled);

  // UI components
  QWidget* chatContainer_;
  QVBoxLayout* chatLayout_;
  QScrollArea* scrollArea_;
  QLineEdit* inputEdit_;
  QPushButton* sendBtn_;
  QPushButton* clearBtn_;
  QLabel* statusLabel_;

  // State
  bool isWaitingForResponse_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_LLM_CHAT_WIDGET_HPP
