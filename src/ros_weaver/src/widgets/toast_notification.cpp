#include "ros_weaver/widgets/toast_notification.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QMouseEvent>
#include <QApplication>
#include <QScreen>

namespace ros_weaver {

// =============================================================================
// ToastNotification
// =============================================================================

ToastNotification::ToastNotification(ToastType type, const QString& message,
                                     int durationMs, QWidget* parent)
    : QWidget(parent)
    , type_(type)
    , message_(message)
    , durationMs_(durationMs)
    , iconLabel_(nullptr)
    , messageLabel_(nullptr)
    , closeButton_(nullptr)
    , autoCloseTimer_(nullptr)
    , opacityEffect_(nullptr)
    , fadeAnimation_(nullptr)
    , isHovered_(false) {
  setupUi();

  // Setup opacity effect for fade animations
  opacityEffect_ = new QGraphicsOpacityEffect(this);
  opacityEffect_->setOpacity(0.0);
  setGraphicsEffect(opacityEffect_);

  // Setup fade animation
  fadeAnimation_ = new QPropertyAnimation(this, "opacity", this);
  fadeAnimation_->setDuration(200);

  // Setup auto-close timer
  autoCloseTimer_ = new QTimer(this);
  autoCloseTimer_->setSingleShot(true);
  connect(autoCloseTimer_, &QTimer::timeout, this, &ToastNotification::dismiss);
}

ToastNotification::~ToastNotification() = default;

void ToastNotification::setupUi() {
  setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
  setAttribute(Qt::WA_TranslucentBackground);
  setAttribute(Qt::WA_ShowWithoutActivating);
  setFixedWidth(350);
  setCursor(Qt::PointingHandCursor);

  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 10, 12, 10);
  layout->setSpacing(10);

  // Container widget with styling
  QWidget* container = new QWidget(this);
  container->setObjectName("toastContainer");
  container->setStyleSheet(getStyleForType());

  QHBoxLayout* containerLayout = new QHBoxLayout(container);
  containerLayout->setContentsMargins(12, 10, 12, 10);
  containerLayout->setSpacing(10);

  // Icon
  iconLabel_ = new QLabel(container);
  iconLabel_->setText(getIconForType());
  iconLabel_->setStyleSheet("font-size: 16px; background: transparent;");
  iconLabel_->setFixedWidth(24);
  containerLayout->addWidget(iconLabel_);

  // Message
  messageLabel_ = new QLabel(container);
  messageLabel_->setText(message_);
  messageLabel_->setWordWrap(true);
  messageLabel_->setStyleSheet("background: transparent; font-size: 13px;");
  containerLayout->addWidget(messageLabel_, 1);

  // Close button
  closeButton_ = new QPushButton(container);
  closeButton_->setText(QString::fromUtf8("\xC3\x97")); // Unicode multiplication sign as X
  closeButton_->setFixedSize(20, 20);
  closeButton_->setCursor(Qt::PointingHandCursor);
  closeButton_->setStyleSheet(
      "QPushButton {"
      "  background: transparent;"
      "  border: none;"
      "  color: rgba(255,255,255,0.6);"
      "  font-size: 14px;"
      "  font-weight: bold;"
      "}"
      "QPushButton:hover {"
      "  color: rgba(255,255,255,1.0);"
      "}");
  connect(closeButton_, &QPushButton::clicked, this, &ToastNotification::dismiss);
  containerLayout->addWidget(closeButton_);

  // Add container to main layout
  layout->addWidget(container);

  adjustSize();
}

QString ToastNotification::getIconForType() const {
  switch (type_) {
    case ToastType::Success:
      return QString::fromUtf8("\xE2\x9C\x93"); // Checkmark
    case ToastType::Warning:
      return QString::fromUtf8("\xE2\x9A\xA0"); // Warning triangle
    case ToastType::Error:
      return QString::fromUtf8("\xE2\x9C\x95"); // X mark
    case ToastType::Info:
    default:
      return QString::fromUtf8("\xE2\x84\xB9"); // Info symbol
  }
}

QString ToastNotification::getStyleForType() const {
  QString bgColor, borderColor, textColor;

  switch (type_) {
    case ToastType::Success:
      bgColor = "#2d5a3d";
      borderColor = "#4CAF50";
      textColor = "#e8f5e9";
      break;
    case ToastType::Warning:
      bgColor = "#5a4a2d";
      borderColor = "#FF9800";
      textColor = "#fff3e0";
      break;
    case ToastType::Error:
      bgColor = "#5a2d2d";
      borderColor = "#f44336";
      textColor = "#ffebee";
      break;
    case ToastType::Info:
    default:
      bgColor = "#2d3a5a";
      borderColor = "#2196F3";
      textColor = "#e3f2fd";
      break;
  }

  return QString(
      "#toastContainer {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-left: 4px solid %2;"
      "  border-radius: 6px;"
      "  color: %3;"
      "}")
      .arg(bgColor, borderColor, textColor);
}

qreal ToastNotification::opacity() const {
  return opacityEffect_ ? opacityEffect_->opacity() : 1.0;
}

void ToastNotification::setOpacity(qreal opacity) {
  if (opacityEffect_) {
    opacityEffect_->setOpacity(opacity);
  }
}

void ToastNotification::show() {
  QWidget::show();

  // Fade in
  fadeAnimation_->stop();
  fadeAnimation_->setStartValue(0.0);
  fadeAnimation_->setEndValue(1.0);
  fadeAnimation_->start();

  // Start auto-close timer
  if (durationMs_ > 0) {
    autoCloseTimer_->start(durationMs_);
  }
}

void ToastNotification::dismiss() {
  autoCloseTimer_->stop();

  // Fade out
  fadeAnimation_->stop();
  fadeAnimation_->setStartValue(opacity());
  fadeAnimation_->setEndValue(0.0);
  connect(fadeAnimation_, &QPropertyAnimation::finished, this, [this]() {
    emit dismissed();
    deleteLater();
  });
  fadeAnimation_->start();
}

void ToastNotification::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    emit clicked();
    dismiss();
  }
  QWidget::mousePressEvent(event);
}

void ToastNotification::enterEvent(QEvent* event) {
  isHovered_ = true;
  autoCloseTimer_->stop(); // Pause timer while hovered
  QWidget::enterEvent(event);
}

void ToastNotification::leaveEvent(QEvent* event) {
  isHovered_ = false;
  // Resume timer with remaining time or minimum 1 second
  if (durationMs_ > 0) {
    autoCloseTimer_->start(1000);
  }
  QWidget::leaveEvent(event);
}

// =============================================================================
// NotificationManager
// =============================================================================

NotificationManager& NotificationManager::instance() {
  static NotificationManager instance;
  return instance;
}

NotificationManager::NotificationManager(QObject* parent)
    : QObject(parent)
    , parentWidget_(nullptr)
    , maxVisible_(5)
    , spacing_(10)
    , margin_(20) {}

NotificationManager::~NotificationManager() {
  clearAll();
}

void NotificationManager::setParentWidget(QWidget* parent) {
  parentWidget_ = parent;
}

void NotificationManager::setMaxVisibleNotifications(int max) {
  maxVisible_ = max;
}

void NotificationManager::setSpacing(int spacing) {
  spacing_ = spacing;
}

void NotificationManager::setMargin(int margin) {
  margin_ = margin;
}

void NotificationManager::showSuccess(const QString& message, int durationMs) {
  show(ToastType::Success, message, durationMs);
}

void NotificationManager::showWarning(const QString& message, int durationMs) {
  show(ToastType::Warning, message, durationMs);
}

void NotificationManager::showError(const QString& message, int durationMs) {
  show(ToastType::Error, message, durationMs);
}

void NotificationManager::showInfo(const QString& message, int durationMs) {
  show(ToastType::Info, message, durationMs);
}

void NotificationManager::show(ToastType type, const QString& message, int durationMs) {
  // Remove oldest if at max
  while (activeNotifications_.size() >= maxVisible_) {
    if (!activeNotifications_.isEmpty()) {
      activeNotifications_.first()->dismiss();
    }
  }

  // Create new notification
  ToastNotification* toast = new ToastNotification(type, message, durationMs, parentWidget_);

  connect(toast, &ToastNotification::dismissed, this, &NotificationManager::onNotificationDismissed);

  activeNotifications_.append(toast);
  repositionNotifications();
  toast->show();
}

void NotificationManager::clearAll() {
  for (ToastNotification* toast : activeNotifications_) {
    toast->dismiss();
  }
  activeNotifications_.clear();
}

void NotificationManager::repositionNotifications() {
  if (!parentWidget_) {
    // Use primary screen if no parent
    QScreen* screen = QApplication::primaryScreen();
    if (!screen) return;

    QRect screenGeometry = screen->availableGeometry();
    int x = screenGeometry.right() - 350 - margin_;
    int y = screenGeometry.bottom() - margin_;

    for (int i = activeNotifications_.size() - 1; i >= 0; --i) {
      ToastNotification* toast = activeNotifications_[i];
      y -= toast->height();
      toast->move(x, y);
      y -= spacing_;
    }
  } else {
    // Position relative to parent widget
    QPoint parentPos = parentWidget_->mapToGlobal(QPoint(0, 0));
    int x = parentPos.x() + parentWidget_->width() - 350 - margin_;
    int y = parentPos.y() + parentWidget_->height() - margin_;

    for (int i = activeNotifications_.size() - 1; i >= 0; --i) {
      ToastNotification* toast = activeNotifications_[i];
      y -= toast->height();
      toast->move(x, y);
      y -= spacing_;
    }
  }
}

void NotificationManager::onNotificationDismissed() {
  ToastNotification* toast = qobject_cast<ToastNotification*>(sender());
  if (toast) {
    activeNotifications_.removeAll(toast);
    repositionNotifications();
  }
}

}  // namespace ros_weaver
