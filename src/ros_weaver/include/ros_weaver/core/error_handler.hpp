#ifndef ROS_WEAVER_ERROR_HANDLER_HPP
#define ROS_WEAVER_ERROR_HANDLER_HPP

#include <QObject>
#include <QString>
#include <QMessageBox>
#include <functional>

namespace ros_weaver {

/**
 * @brief Error severity levels
 *
 * Guidelines for when to use each level:
 * - Info: Successful operations, status updates, hints
 * - Warning: Recoverable issues, validation errors, missing optional features
 * - Error: Failed operations that need attention but don't block workflow
 * - Critical: Blocking errors that require user acknowledgment or action
 */
enum class ErrorSeverity {
  Info,      // Uses toast, auto-dismisses
  Warning,   // Uses toast, longer duration
  Error,     // Uses toast with longer duration
  Critical   // Uses modal dialog requiring acknowledgment
};

/**
 * @brief Action to take after an error
 */
struct ErrorAction {
  QString label;
  std::function<void()> callback;
};

/**
 * @brief Unified error handler for the application
 *
 * Provides a consistent interface for showing errors with appropriate
 * UI based on severity. Use this instead of calling QMessageBox or
 * Toast directly for consistency.
 *
 * Usage:
 *   ErrorHandler::show(ErrorSeverity::Warning, "Save Error", "Failed to save file");
 *   ErrorHandler::showSuccess("File saved successfully");
 *   ErrorHandler::showError("Connection failed");
 */
class ErrorHandler : public QObject {
  Q_OBJECT

public:
  /**
   * @brief Show a notification with the specified severity
   * @param severity The severity level determining UI behavior
   * @param title Title for the notification (used in dialogs)
   * @param message The message to display
   * @param parent Parent widget for modal dialogs (optional)
   */
  static void show(ErrorSeverity severity,
                   const QString& title,
                   const QString& message,
                   QWidget* parent = nullptr);

  /**
   * @brief Show a critical error with optional action button
   * @param title Dialog title
   * @param message Error message
   * @param action Optional action button (e.g., "Retry", "Open Settings")
   * @param parent Parent widget for the dialog
   * @return true if action was clicked, false if dismissed
   */
  static bool showCritical(const QString& title,
                           const QString& message,
                           const ErrorAction& action = {},
                           QWidget* parent = nullptr);

  /**
   * @brief Show an error that requires user confirmation to proceed
   * @param title Dialog title
   * @param message Warning message
   * @param confirmText Text for the confirm button (default: "Continue")
   * @param parent Parent widget
   * @return true if user confirmed, false if cancelled
   */
  static bool confirm(const QString& title,
                      const QString& message,
                      const QString& confirmText = {},
                      QWidget* parent = nullptr);

  // Convenience methods for common cases

  /** @brief Show success toast */
  static void showSuccess(const QString& message);

  /** @brief Show info toast */
  static void showInfo(const QString& message);

  /** @brief Show warning toast */
  static void showWarning(const QString& message);

  /** @brief Show error toast (non-blocking) */
  static void showError(const QString& message);

  /** @brief Show error in status bar (for transient feedback) */
  static void showStatus(const QString& message, int durationMs = 3000);

private:
  ErrorHandler() = default;
  static QWidget* findMainWindow();
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_ERROR_HANDLER_HPP
