#ifndef ROS_WEAVER_TOAST_NOTIFICATION_HPP
#define ROS_WEAVER_TOAST_NOTIFICATION_HPP

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QList>

namespace ros_weaver {

/**
 * @brief Toast notification types with semantic meaning
 */
enum class ToastType {
  Success,
  Warning,
  Error,
  Info
};

/**
 * @brief Individual toast notification widget
 *
 * A non-blocking notification that appears briefly and auto-dismisses.
 * Can be manually dismissed by clicking.
 */
class ToastNotification : public QWidget {
  Q_OBJECT
  Q_PROPERTY(qreal opacity READ opacity WRITE setOpacity)

public:
  explicit ToastNotification(ToastType type, const QString& message,
                             int durationMs = 4000, QWidget* parent = nullptr);
  ~ToastNotification() override;

  qreal opacity() const;
  void setOpacity(qreal opacity);

  void show();
  void dismiss();

  ToastType type() const { return type_; }
  QString message() const { return message_; }

signals:
  void dismissed();
  void clicked();

protected:
  void mousePressEvent(QMouseEvent* event) override;
  void enterEvent(QEvent* event) override;
  void leaveEvent(QEvent* event) override;

private:
  void setupUi();
  QString getIconForType() const;
  QString getStyleForType() const;

  ToastType type_;
  QString message_;
  int durationMs_;

  QLabel* iconLabel_;
  QLabel* messageLabel_;
  QPushButton* closeButton_;
  QTimer* autoCloseTimer_;
  QGraphicsOpacityEffect* opacityEffect_;
  QPropertyAnimation* fadeAnimation_;
  bool isHovered_;
};

/**
 * @brief Notification manager singleton
 *
 * Manages toast notifications, handles stacking, and provides
 * a simple API for showing notifications.
 */
class NotificationManager : public QObject {
  Q_OBJECT

public:
  static NotificationManager& instance();

  // Show notifications with various types
  void showSuccess(const QString& message, int durationMs = 4000);
  void showWarning(const QString& message, int durationMs = 5000);
  void showError(const QString& message, int durationMs = 6000);
  void showInfo(const QString& message, int durationMs = 4000);

  // Generic show method
  void show(ToastType type, const QString& message, int durationMs = 4000);

  // Configuration
  void setParentWidget(QWidget* parent);
  void setMaxVisibleNotifications(int max);
  void setSpacing(int spacing);
  void setMargin(int margin);

  // Clear all notifications
  void clearAll();

private:
  explicit NotificationManager(QObject* parent = nullptr);
  ~NotificationManager() override;

  NotificationManager(const NotificationManager&) = delete;
  NotificationManager& operator=(const NotificationManager&) = delete;

  void repositionNotifications();
  void onNotificationDismissed();

  QWidget* parentWidget_;
  QList<ToastNotification*> activeNotifications_;
  int maxVisible_;
  int spacing_;
  int margin_;
};

// Convenience macros for quick access
#define Toast NotificationManager::instance()

}  // namespace ros_weaver

#endif  // ROS_WEAVER_TOAST_NOTIFICATION_HPP
