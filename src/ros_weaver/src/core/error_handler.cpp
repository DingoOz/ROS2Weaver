#include "ros_weaver/core/error_handler.hpp"
#include "ros_weaver/widgets/toast_notification.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QStatusBar>

namespace ros_weaver {

void ErrorHandler::show(ErrorSeverity severity,
                        const QString& title,
                        const QString& message,
                        QWidget* parent) {
  switch (severity) {
    case ErrorSeverity::Info:
      showInfo(message);
      break;

    case ErrorSeverity::Warning:
      showWarning(message);
      break;

    case ErrorSeverity::Error:
      showError(message);
      break;

    case ErrorSeverity::Critical: {
      QWidget* dialogParent = parent ? parent : findMainWindow();
      QMessageBox::critical(dialogParent, title, message);
      break;
    }
  }
}

bool ErrorHandler::showCritical(const QString& title,
                                const QString& message,
                                const ErrorAction& action,
                                QWidget* parent) {
  QWidget* dialogParent = parent ? parent : findMainWindow();

  if (action.label.isEmpty() || !action.callback) {
    // No action, just show standard critical dialog
    QMessageBox::critical(dialogParent, title, message);
    return false;
  }

  // Create dialog with custom action button
  QMessageBox msgBox(dialogParent);
  msgBox.setIcon(QMessageBox::Critical);
  msgBox.setWindowTitle(title);
  msgBox.setText(message);

  QPushButton* actionButton = msgBox.addButton(action.label, QMessageBox::AcceptRole);
  msgBox.addButton(QMessageBox::Close);

  msgBox.exec();

  if (msgBox.clickedButton() == actionButton) {
    action.callback();
    return true;
  }
  return false;
}

bool ErrorHandler::confirm(const QString& title,
                           const QString& message,
                           const QString& confirmText,
                           QWidget* parent) {
  QWidget* dialogParent = parent ? parent : findMainWindow();

  QString buttonText = confirmText.isEmpty() ? tr("Continue") : confirmText;

  QMessageBox msgBox(dialogParent);
  msgBox.setIcon(QMessageBox::Warning);
  msgBox.setWindowTitle(title);
  msgBox.setText(message);

  QPushButton* confirmButton = msgBox.addButton(buttonText, QMessageBox::AcceptRole);
  msgBox.addButton(QMessageBox::Cancel);

  msgBox.exec();

  return msgBox.clickedButton() == confirmButton;
}

void ErrorHandler::showSuccess(const QString& message) {
  Toast.showSuccess(message);
}

void ErrorHandler::showInfo(const QString& message) {
  Toast.showInfo(message);
}

void ErrorHandler::showWarning(const QString& message) {
  Toast.showWarning(message);
}

void ErrorHandler::showError(const QString& message) {
  Toast.showError(message);
}

void ErrorHandler::showStatus(const QString& message, int durationMs) {
  QWidget* mainWindow = findMainWindow();
  if (QMainWindow* mw = qobject_cast<QMainWindow*>(mainWindow)) {
    if (QStatusBar* statusBar = mw->statusBar()) {
      statusBar->showMessage(message, durationMs);
    }
  }
}

QWidget* ErrorHandler::findMainWindow() {
  const QWidgetList& topLevelWidgets = QApplication::topLevelWidgets();
  for (QWidget* widget : topLevelWidgets) {
    if (QMainWindow* mainWindow = qobject_cast<QMainWindow*>(widget)) {
      if (mainWindow->isVisible()) {
        return mainWindow;
      }
    }
  }
  // Return first main window even if not visible
  for (QWidget* widget : topLevelWidgets) {
    if (qobject_cast<QMainWindow*>(widget)) {
      return widget;
    }
  }
  return nullptr;
}

}  // namespace ros_weaver
