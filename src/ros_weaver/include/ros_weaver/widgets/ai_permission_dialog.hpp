#ifndef ROS_WEAVER_AI_PERMISSION_DIALOG_HPP
#define ROS_WEAVER_AI_PERMISSION_DIALOG_HPP

#include <QDialog>
#include <QString>
#include <QJsonObject>
#include <QLabel>
#include <QTextBrowser>
#include <QPushButton>
#include <QCheckBox>

namespace ros_weaver {

// Dialog for asking user permission before AI executes a tool
class AIPermissionDialog : public QDialog {
  Q_OBJECT

public:
  explicit AIPermissionDialog(QWidget* parent = nullptr);

  // Set the action being requested
  void setAction(const QString& toolName, const QString& description,
                 const QJsonObject& params);

  // Check if "approve all" was selected
  bool approveAllSelected() const { return approveAll_; }

signals:
  void permissionGranted(bool approveAll);
  void permissionDenied();

private slots:
  void onApprove();
  void onDeny();

private:
  void setupUi();

  QLabel* titleLabel_;
  QLabel* descriptionLabel_;
  QTextBrowser* detailsBrowser_;
  QCheckBox* approveAllCheckbox_;
  QPushButton* approveBtn_;
  QPushButton* denyBtn_;

  QString toolName_;
  QJsonObject params_;
  bool approveAll_ = false;
};

// Widget shown in the chat when an action can be undone
class AIUndoWidget : public QWidget {
  Q_OBJECT

public:
  explicit AIUndoWidget(const QString& actionDescription, QWidget* parent = nullptr);

signals:
  void undoRequested();

private:
  void setupUi(const QString& actionDescription);

  QLabel* descriptionLabel_;
  QPushButton* undoBtn_;
};

// Floating notification for AI action results
class AIActionNotification : public QWidget {
  Q_OBJECT

public:
  enum class Type { Success, Error, Info };

  AIActionNotification(Type type, const QString& message,
                       bool showUndo = false, QWidget* parent = nullptr);

  void showFor(int milliseconds);

signals:
  void undoClicked();
  void dismissed();

private:
  void setupUi(Type type, const QString& message, bool showUndo);
  void fadeOut();

  QPushButton* undoBtn_ = nullptr;
  QPushButton* closeBtn_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_AI_PERMISSION_DIALOG_HPP
