#include "ros_weaver/widgets/ros_status_widget.hpp"

#include <QHBoxLayout>
#include <QFont>
#include <QToolTip>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

RosStatusWidget::RosStatusWidget(QWidget* parent)
  : QWidget(parent)
  , statusIconLabel_(nullptr)
  , statusTextLabel_(nullptr)
  , domainIdLabel_(nullptr)
  , ros2Status_(Ros2Status::Disconnected)
  , domainId_(0)
  , showRos2Status_(true)
  , showDomainId_(true)
  , highlightNonDefaultDomain_(true)
  , displayLocation_(StatusDisplayLocation::StatusBarOnly)
  , healthCheckTimer_(nullptr)
{
  setupUi();
  loadSettings();

  // Set up health check timer
  healthCheckTimer_ = new QTimer(this);
  connect(healthCheckTimer_, &QTimer::timeout, this, &RosStatusWidget::onHealthCheckTimer);
  healthCheckTimer_->start(HEALTH_CHECK_INTERVAL_MS);

  // Initial status check
  refreshStatus();
}

RosStatusWidget::~RosStatusWidget() {
  if (healthCheckTimer_) {
    healthCheckTimer_->stop();
  }
  saveSettings();
}

void RosStatusWidget::setupUi() {
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(4, 0, 4, 0);
  layout->setSpacing(8);

  // Status icon (colored dot)
  statusIconLabel_ = new QLabel(this);
  statusIconLabel_->setFixedSize(12, 12);
  statusIconLabel_->setToolTip(tr("ROS2 connection status"));
  layout->addWidget(statusIconLabel_);

  // Status text
  statusTextLabel_ = new QLabel(tr("ROS2: Checking..."), this);
  statusTextLabel_->setToolTip(tr("ROS2 connection status"));
  layout->addWidget(statusTextLabel_);

  // Separator
  QLabel* separator = new QLabel("|", this);
  separator->setStyleSheet("color: gray;");
  layout->addWidget(separator);

  // Domain ID
  domainIdLabel_ = new QLabel(tr("Domain ID: --"), this);
  domainIdLabel_->setToolTip(tr("Current ROS_DOMAIN_ID value"));
  layout->addWidget(domainIdLabel_);

  setLayout(layout);
}

void RosStatusWidget::loadSettings() {
  QSettings settings;
  settings.beginGroup(SETTINGS_GROUP);

  showRos2Status_ = settings.value(KEY_SHOW_ROS_STATUS, true).toBool();
  showDomainId_ = settings.value(KEY_SHOW_DOMAIN_ID, true).toBool();
  highlightNonDefaultDomain_ = settings.value(KEY_HIGHLIGHT_NON_DEFAULT, true).toBool();
  displayLocation_ = static_cast<StatusDisplayLocation>(
    settings.value(KEY_DISPLAY_LOCATION, static_cast<int>(StatusDisplayLocation::StatusBarOnly)).toInt()
  );

  settings.endGroup();
}

void RosStatusWidget::saveSettings() {
  QSettings settings;
  settings.beginGroup(SETTINGS_GROUP);

  settings.setValue(KEY_SHOW_ROS_STATUS, showRos2Status_);
  settings.setValue(KEY_SHOW_DOMAIN_ID, showDomainId_);
  settings.setValue(KEY_HIGHLIGHT_NON_DEFAULT, highlightNonDefaultDomain_);
  settings.setValue(KEY_DISPLAY_LOCATION, static_cast<int>(displayLocation_));

  settings.endGroup();
}

void RosStatusWidget::setShowRos2Status(bool show) {
  if (showRos2Status_ != show) {
    showRos2Status_ = show;
    updateDisplay();
    updateTitleBarSuffix();
    saveSettings();
    emit settingsChanged();
  }
}

void RosStatusWidget::setShowDomainId(bool show) {
  if (showDomainId_ != show) {
    showDomainId_ = show;
    updateDisplay();
    updateTitleBarSuffix();
    saveSettings();
    emit settingsChanged();
  }
}

void RosStatusWidget::setHighlightNonDefaultDomain(bool highlight) {
  if (highlightNonDefaultDomain_ != highlight) {
    highlightNonDefaultDomain_ = highlight;
    updateDisplay();
    saveSettings();
    emit settingsChanged();
  }
}

void RosStatusWidget::setDisplayLocation(StatusDisplayLocation location) {
  if (displayLocation_ != location) {
    displayLocation_ = location;

    // Update visibility based on location
    bool showInStatusBar = (location == StatusDisplayLocation::StatusBarOnly ||
                            location == StatusDisplayLocation::Both);
    setVisible(showInStatusBar);

    updateTitleBarSuffix();
    saveSettings();
    emit settingsChanged();
  }
}

void RosStatusWidget::refreshStatus() {
  Ros2Status newStatus = checkRos2Status();
  int newDomainId = readDomainId();

  bool statusChanged = (newStatus != ros2Status_);
  bool domainChanged = (newDomainId != domainId_);

  ros2Status_ = newStatus;
  domainId_ = newDomainId;

  if (statusChanged) {
    emit ros2StatusChanged(ros2Status_);
  }
  if (domainChanged) {
    emit domainIdChanged(domainId_);
  }

  updateDisplay();

  if (statusChanged || domainChanged) {
    updateTitleBarSuffix();
  }
}

void RosStatusWidget::onHealthCheckTimer() {
  refreshStatus();
}

Ros2Status RosStatusWidget::checkRos2Status() {
  // Check if ROS2 context is active
  if (rclcpp::ok()) {
    return Ros2Status::Connected;
  }

  // Check if we can create a context (not already shutdown)
  // If rclcpp::ok() returns false but we're in the application,
  // it might be disconnected or in error state
  try {
    // Try to check daemon availability by testing if init was done
    // Since we're in a Qt app that uses ROS2, if rclcpp::ok() is false,
    // it's likely disconnected/shutdown
    return Ros2Status::Disconnected;
  } catch (...) {
    return Ros2Status::Error;
  }
}

int RosStatusWidget::readDomainId() {
  // Read ROS_DOMAIN_ID from environment
  const char* domainIdEnv = std::getenv("ROS_DOMAIN_ID");
  if (domainIdEnv) {
    try {
      return std::stoi(domainIdEnv);
    } catch (...) {
      return 0;  // Default to 0 if invalid
    }
  }
  return 0;  // Default domain ID
}

void RosStatusWidget::updateDisplay() {
  // Update status icon and text
  QString iconStyle;
  QString statusText;
  QString tooltip;

  switch (ros2Status_) {
    case Ros2Status::Connected:
      iconStyle = "background-color: #4CAF50; border-radius: 6px;";  // Green
      statusText = tr("ROS2: Connected");
      tooltip = tr("ROS2 context is active and healthy");
      break;
    case Ros2Status::Disconnected:
      iconStyle = "background-color: #9E9E9E; border-radius: 6px;";  // Gray
      statusText = tr("ROS2: Disconnected");
      tooltip = tr("ROS2 not initialized or shutdown");
      break;
    case Ros2Status::Error:
      iconStyle = "background-color: #F44336; border-radius: 6px;";  // Red
      statusText = tr("ROS2: Error");
      tooltip = tr("ROS2 initialization failed or daemon unreachable");
      break;
    case Ros2Status::Warning:
      iconStyle = "background-color: #FF9800; border-radius: 6px;";  // Yellow/Orange
      statusText = tr("ROS2: Warning");
      tooltip = tr("Partial connectivity or unusual state");
      break;
  }

  statusIconLabel_->setStyleSheet(iconStyle);
  statusTextLabel_->setText(statusText);
  statusIconLabel_->setToolTip(tooltip);
  statusTextLabel_->setToolTip(tooltip);

  // Update visibility
  statusIconLabel_->setVisible(showRos2Status_);
  statusTextLabel_->setVisible(showRos2Status_);

  // Update domain ID display
  QString domainText = tr("Domain ID: %1").arg(domainId_);
  domainIdLabel_->setText(domainText);

  // Highlight non-default domain ID
  if (highlightNonDefaultDomain_ && domainId_ != 0) {
    domainIdLabel_->setStyleSheet("color: #FF9800; font-weight: bold;");  // Orange highlight
    domainIdLabel_->setToolTip(tr("Non-default ROS_DOMAIN_ID (default is 0)\nSource: ROS_DOMAIN_ID environment variable"));
  } else {
    domainIdLabel_->setStyleSheet("");
    domainIdLabel_->setToolTip(tr("Current ROS_DOMAIN_ID value\nSource: %1")
      .arg(std::getenv("ROS_DOMAIN_ID") ? "ROS_DOMAIN_ID environment variable" : "default (0)"));
  }

  domainIdLabel_->setVisible(showDomainId_);

  // Find and update separator visibility
  for (QObject* child : children()) {
    QLabel* label = qobject_cast<QLabel*>(child);
    if (label && label->text() == "|") {
      label->setVisible(showRos2Status_ && showDomainId_);
      break;
    }
  }
}

QString RosStatusWidget::titleBarSuffix() const {
  if (displayLocation_ == StatusDisplayLocation::StatusBarOnly) {
    return QString();
  }

  QStringList parts;

  if (showRos2Status_) {
    QString statusChar;
    switch (ros2Status_) {
      case Ros2Status::Connected:
        statusChar = QString::fromUtf8("\u25CF");  // Filled circle
        break;
      case Ros2Status::Disconnected:
        statusChar = QString::fromUtf8("\u25CB");  // Empty circle
        break;
      case Ros2Status::Error:
        statusChar = QString::fromUtf8("\u2716");  // X mark
        break;
      case Ros2Status::Warning:
        statusChar = QString::fromUtf8("\u26A0");  // Warning sign
        break;
    }
    parts << QString("ROS2 %1").arg(statusChar);
  }

  if (showDomainId_) {
    parts << QString("Domain: %1").arg(domainId_);
  }

  if (parts.isEmpty()) {
    return QString();
  }

  return QString(" [%1]").arg(parts.join(" | "));
}

void RosStatusWidget::updateTitleBarSuffix() {
  emit titleBarUpdateRequested(titleBarSuffix());
}

}  // namespace ros_weaver
