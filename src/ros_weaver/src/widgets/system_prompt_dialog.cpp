#include "ros_weaver/widgets/system_prompt_dialog.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>

namespace ros_weaver {

SystemPromptDialog::SystemPromptDialog(QWidget* parent)
    : QDialog(parent) {
  setWindowTitle(tr("Edit System Prompt"));
  setMinimumSize(600, 400);
  resize(700, 500);
  setupUi();
}

void SystemPromptDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(12);

  // Description label
  QLabel* descLabel = new QLabel(
      tr("Customize the AI assistant's behavior. This prompt is sent with every message "
         "to guide how the AI responds to your questions."),
      this);
  descLabel->setWordWrap(true);
  descLabel->setStyleSheet("color: gray; margin-bottom: 8px;");
  mainLayout->addWidget(descLabel);

  // Prompt editor
  promptEdit_ = new QTextEdit(this);
  promptEdit_->setPlaceholderText(tr("Enter your custom system prompt here..."));
  promptEdit_->setAcceptRichText(false);
  mainLayout->addWidget(promptEdit_, 1);

  // Button layout
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  resetBtn_ = new QPushButton(tr("Reset to Default"), this);
  resetBtn_->setToolTip(tr("Reset the system prompt to the default ROS2-focused prompt"));
  connect(resetBtn_, &QPushButton::clicked, this, &SystemPromptDialog::onResetToDefault);
  buttonLayout->addWidget(resetBtn_);

  buttonLayout->addStretch();

  cancelBtn_ = new QPushButton(tr("Cancel"), this);
  connect(cancelBtn_, &QPushButton::clicked, this, &QDialog::reject);
  buttonLayout->addWidget(cancelBtn_);

  okBtn_ = new QPushButton(tr("Save"), this);
  okBtn_->setDefault(true);
  connect(okBtn_, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(okBtn_);

  mainLayout->addLayout(buttonLayout);
}

void SystemPromptDialog::setPrompt(const QString& prompt) {
  promptEdit_->setPlainText(prompt);
}

QString SystemPromptDialog::prompt() const {
  return promptEdit_->toPlainText();
}

void SystemPromptDialog::onResetToDefault() {
  promptEdit_->setPlainText(OllamaManager::defaultSystemPrompt());
}

}  // namespace ros_weaver
