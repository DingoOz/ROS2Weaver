#ifndef ROS_WEAVER_SYSTEM_PROMPT_DIALOG_HPP
#define ROS_WEAVER_SYSTEM_PROMPT_DIALOG_HPP

#include <QDialog>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>

namespace ros_weaver {

class SystemPromptDialog : public QDialog {
  Q_OBJECT

public:
  explicit SystemPromptDialog(QWidget* parent = nullptr);
  ~SystemPromptDialog() = default;

  void setPrompt(const QString& prompt);
  QString prompt() const;

private slots:
  void onResetToDefault();

private:
  void setupUi();

  QTextEdit* promptEdit_;
  QPushButton* resetBtn_;
  QPushButton* okBtn_;
  QPushButton* cancelBtn_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_SYSTEM_PROMPT_DIALOG_HPP
