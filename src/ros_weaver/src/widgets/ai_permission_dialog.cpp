#include "ros_weaver/widgets/ai_permission_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonValue>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>

namespace ros_weaver {

// ============== AIPermissionDialog ==============

AIPermissionDialog::AIPermissionDialog(QWidget* parent)
    : QDialog(parent)
    , approveAll_(false) {
  setupUi();
  setWindowTitle(tr("AI Action Permission"));
  setModal(true);
  setMinimumWidth(450);
}

void AIPermissionDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(16);
  mainLayout->setContentsMargins(20, 20, 20, 20);

  // Warning icon and title
  QHBoxLayout* headerLayout = new QHBoxLayout();

  QLabel* iconLabel = new QLabel(this);
  iconLabel->setText(QString::fromUtf8("\xE2\x9A\xA0\xEF\xB8\x8F"));  // Warning emoji
  iconLabel->setStyleSheet("font-size: 32px;");
  headerLayout->addWidget(iconLabel);

  titleLabel_ = new QLabel(tr("AI Action Request"), this);
  titleLabel_->setStyleSheet("font-size: 18px; font-weight: bold; color: #FF9800;");
  headerLayout->addWidget(titleLabel_, 1);

  mainLayout->addLayout(headerLayout);

  // Description
  descriptionLabel_ = new QLabel(this);
  descriptionLabel_->setWordWrap(true);
  descriptionLabel_->setStyleSheet("font-size: 14px; color: #e0e0e0;");
  mainLayout->addWidget(descriptionLabel_);

  // Details browser
  QLabel* detailsHeader = new QLabel(tr("Action Details:"), this);
  detailsHeader->setStyleSheet("font-weight: bold; color: #aaaaaa; margin-top: 8px;");
  mainLayout->addWidget(detailsHeader);

  detailsBrowser_ = new QTextBrowser(this);
  detailsBrowser_->setMaximumHeight(150);
  detailsBrowser_->setStyleSheet(
      "QTextBrowser {"
      "  background-color: #2a2a2a;"
      "  border: 1px solid #404040;"
      "  border-radius: 4px;"
      "  padding: 8px;"
      "  color: #e0e0e0;"
      "  font-family: monospace;"
      "}");
  mainLayout->addWidget(detailsBrowser_);

  // Approve all checkbox
  approveAllCheckbox_ = new QCheckBox(tr("Approve all AI actions for this session"), this);
  approveAllCheckbox_->setStyleSheet(
      "QCheckBox {"
      "  color: #aaaaaa;"
      "  font-size: 12px;"
      "}"
      "QCheckBox::indicator {"
      "  width: 16px;"
      "  height: 16px;"
      "}");
  mainLayout->addWidget(approveAllCheckbox_);

  // Note about undo
  QLabel* undoNote = new QLabel(
      tr("Note: You can undo this action after it's executed."), this);
  undoNote->setStyleSheet("font-size: 11px; color: #888888; font-style: italic;");
  mainLayout->addWidget(undoNote);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->setSpacing(12);

  denyBtn_ = new QPushButton(tr("Deny"), this);
  denyBtn_->setMinimumHeight(36);
  denyBtn_->setMinimumWidth(100);
  denyBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #c0392b;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-weight: bold;"
      "  padding: 8px 16px;"
      "}"
      "QPushButton:hover { background-color: #e74c3c; }");
  connect(denyBtn_, &QPushButton::clicked, this, &AIPermissionDialog::onDeny);
  buttonLayout->addWidget(denyBtn_);

  buttonLayout->addStretch();

  approveBtn_ = new QPushButton(tr("Approve"), this);
  approveBtn_->setMinimumHeight(36);
  approveBtn_->setMinimumWidth(100);
  approveBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #27ae60;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-weight: bold;"
      "  padding: 8px 16px;"
      "}"
      "QPushButton:hover { background-color: #2ecc71; }");
  approveBtn_->setDefault(true);
  connect(approveBtn_, &QPushButton::clicked, this, &AIPermissionDialog::onApprove);
  buttonLayout->addWidget(approveBtn_);

  mainLayout->addLayout(buttonLayout);

  // Dialog styling
  setStyleSheet(
      "QDialog {"
      "  background-color: #1e1e1e;"
      "  border: 1px solid #404040;"
      "}");
}

void AIPermissionDialog::setAction(const QString& toolName, const QString& description,
                                    const QJsonObject& params) {
  toolName_ = toolName;
  params_ = params;

  // Update title based on tool
  QString actionType;
  if (toolName == "load_example") actionType = tr("Load Example Project");
  else if (toolName == "add_block") actionType = tr("Add Block");
  else if (toolName == "remove_block") actionType = tr("Remove Block");
  else if (toolName == "set_parameter") actionType = tr("Modify Parameter");
  else if (toolName == "create_connection") actionType = tr("Create Connection");
  else if (toolName == "remove_connection") actionType = tr("Remove Connection");
  else if (toolName == "create_group") actionType = tr("Create Group");
  else actionType = toolName;

  titleLabel_->setText(tr("AI wants to: %1").arg(actionType));

  descriptionLabel_->setText(description);

  // Format parameters as readable text
  QString detailsText;
  detailsText += QString("Tool: %1\n\n").arg(toolName);
  detailsText += "Parameters:\n";

  for (const QString& key : params.keys()) {
    QJsonValue value = params[key];
    QString valueStr;

    if (value.isString()) {
      valueStr = value.toString();
    } else if (value.isDouble()) {
      valueStr = QString::number(value.toDouble());
    } else if (value.isArray()) {
      QStringList items;
      for (const QJsonValue& item : value.toArray()) {
        items << item.toString();
      }
      valueStr = QString("[%1]").arg(items.join(", "));
    } else {
      valueStr = QJsonDocument(QJsonObject{{key, value}}).toJson(QJsonDocument::Compact);
    }

    detailsText += QString("  %1: %2\n").arg(key, valueStr);
  }

  detailsBrowser_->setText(detailsText);
}

void AIPermissionDialog::onApprove() {
  approveAll_ = approveAllCheckbox_->isChecked();
  emit permissionGranted(approveAll_);
  accept();
}

void AIPermissionDialog::onDeny() {
  emit permissionDenied();
  reject();
}

// ============== AIUndoWidget ==============

AIUndoWidget::AIUndoWidget(const QString& actionDescription, QWidget* parent)
    : QWidget(parent) {
  setupUi(actionDescription);
}

void AIUndoWidget::setupUi(const QString& actionDescription) {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setSpacing(8);

  setStyleSheet(
      "QWidget {"
      "  background-color: #2d4a2d;"
      "  border: 1px solid #4a7c4a;"
      "  border-radius: 4px;"
      "}");

  // Checkmark icon
  QLabel* iconLabel = new QLabel(QString::fromUtf8("\xE2\x9C\x93"), this);
  iconLabel->setStyleSheet("color: #4CAF50; font-size: 14px; font-weight: bold;");
  layout->addWidget(iconLabel);

  // Description
  descriptionLabel_ = new QLabel(actionDescription, this);
  descriptionLabel_->setStyleSheet("color: #c0e0c0; font-size: 12px;");
  layout->addWidget(descriptionLabel_, 1);

  // Undo button
  undoBtn_ = new QPushButton(tr("Undo"), this);
  undoBtn_->setFixedSize(50, 24);
  undoBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: #5a5a5a;"
      "  border: 1px solid #707070;"
      "  border-radius: 3px;"
      "  color: white;"
      "  font-size: 11px;"
      "}"
      "QPushButton:hover { background-color: #707070; }");
  connect(undoBtn_, &QPushButton::clicked, this, &AIUndoWidget::undoRequested);
  layout->addWidget(undoBtn_);
}

// ============== AIActionNotification ==============

AIActionNotification::AIActionNotification(Type type, const QString& message,
                                            bool showUndo, QWidget* parent)
    : QWidget(parent) {
  setupUi(type, message, showUndo);

  // Set window flags for floating notification
  setWindowFlags(Qt::ToolTip | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);
  setAttribute(Qt::WA_ShowWithoutActivating);
}

void AIActionNotification::setupUi(Type type, const QString& message, bool showUndo) {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 8, 12, 8);
  layout->setSpacing(10);

  // Background color based on type
  QString bgColor, borderColor, iconText;
  switch (type) {
    case Type::Success:
      bgColor = "#2d4a2d";
      borderColor = "#4a7c4a";
      iconText = QString::fromUtf8("\xE2\x9C\x93");  // Checkmark
      break;
    case Type::Error:
      bgColor = "#4a2d2d";
      borderColor = "#7c4a4a";
      iconText = QString::fromUtf8("\xE2\x9C\x97");  // X mark
      break;
    case Type::Info:
    default:
      bgColor = "#2d3d4a";
      borderColor = "#4a6a7c";
      iconText = QString::fromUtf8("\xE2\x84\xB9");  // Info icon
      break;
  }

  setStyleSheet(QString(
      "QWidget {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 6px;"
      "}").arg(bgColor, borderColor));

  // Icon
  QLabel* iconLabel = new QLabel(iconText, this);
  iconLabel->setStyleSheet("font-size: 16px;");
  layout->addWidget(iconLabel);

  // Message
  QLabel* messageLabel = new QLabel(message, this);
  messageLabel->setStyleSheet("color: #e0e0e0; font-size: 12px;");
  messageLabel->setMaximumWidth(300);
  messageLabel->setWordWrap(true);
  layout->addWidget(messageLabel, 1);

  // Undo button (optional)
  if (showUndo) {
    undoBtn_ = new QPushButton(tr("Undo"), this);
    undoBtn_->setFixedSize(50, 24);
    undoBtn_->setStyleSheet(
        "QPushButton {"
        "  background-color: #5a5a5a;"
        "  border: 1px solid #707070;"
        "  border-radius: 3px;"
        "  color: white;"
        "  font-size: 11px;"
        "}"
        "QPushButton:hover { background-color: #707070; }");
    connect(undoBtn_, &QPushButton::clicked, this, &AIActionNotification::undoClicked);
    layout->addWidget(undoBtn_);
  }

  // Close button
  closeBtn_ = new QPushButton(QString::fromUtf8("\xC3\x97"), this);  // X symbol
  closeBtn_->setFixedSize(20, 20);
  closeBtn_->setStyleSheet(
      "QPushButton {"
      "  background-color: transparent;"
      "  border: none;"
      "  color: #808080;"
      "  font-size: 14px;"
      "}"
      "QPushButton:hover { color: #c0c0c0; }");
  connect(closeBtn_, &QPushButton::clicked, this, [this]() {
    emit dismissed();
    close();
  });
  layout->addWidget(closeBtn_);

  adjustSize();
}

void AIActionNotification::showFor(int milliseconds) {
  show();

  QTimer::singleShot(milliseconds, this, &AIActionNotification::fadeOut);
}

void AIActionNotification::fadeOut() {
  QGraphicsOpacityEffect* effect = new QGraphicsOpacityEffect(this);
  setGraphicsEffect(effect);

  QPropertyAnimation* anim = new QPropertyAnimation(effect, "opacity", this);
  anim->setDuration(300);
  anim->setStartValue(1.0);
  anim->setEndValue(0.0);

  connect(anim, &QPropertyAnimation::finished, this, [this]() {
    emit dismissed();
    close();
    deleteLater();
  });

  anim->start(QPropertyAnimation::DeleteWhenStopped);
}

}  // namespace ros_weaver
