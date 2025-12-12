#ifndef ROS_WEAVER_ROS_STATUS_WIDGET_HPP
#define ROS_WEAVER_ROS_STATUS_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QSettings>

namespace ros_weaver {

// ROS2 connection status states
enum class Ros2Status {
  Connected,    // ROS2 context is active and healthy
  Disconnected, // ROS2 not initialized or shutdown
  Error,        // ROS2 initialization failed or daemon unreachable
  Warning       // Partial connectivity or unusual state
};

// Display location options
enum class StatusDisplayLocation {
  StatusBarOnly,
  TitleBarOnly,
  Both
};

class RosStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit RosStatusWidget(QWidget* parent = nullptr);
  ~RosStatusWidget() override;

  // Status accessors
  Ros2Status ros2Status() const { return ros2Status_; }
  int domainId() const { return domainId_; }

  // Settings
  bool isRos2StatusVisible() const { return showRos2Status_; }
  bool isDomainIdVisible() const { return showDomainId_; }
  bool isHighlightNonDefaultDomain() const { return highlightNonDefaultDomain_; }
  StatusDisplayLocation displayLocation() const { return displayLocation_; }

  // Get formatted title bar suffix for display location options
  QString titleBarSuffix() const;

public slots:
  void setShowRos2Status(bool show);
  void setShowDomainId(bool show);
  void setHighlightNonDefaultDomain(bool highlight);
  void setDisplayLocation(StatusDisplayLocation location);

  // Manual refresh
  void refreshStatus();

signals:
  // Emitted when status changes
  void ros2StatusChanged(Ros2Status status);
  void domainIdChanged(int domainId);

  // Emitted when title bar needs update (for display location options)
  void titleBarUpdateRequested(const QString& suffix);

  // Emitted when settings change
  void settingsChanged();

private slots:
  void onHealthCheckTimer();

private:
  void setupUi();
  void loadSettings();
  void saveSettings();
  void updateDisplay();
  void updateTitleBarSuffix();

  // Check ROS2 system status
  Ros2Status checkRos2Status();
  int readDomainId();

  // UI components
  QLabel* statusIconLabel_;
  QLabel* statusTextLabel_;
  QLabel* domainIdLabel_;

  // State
  Ros2Status ros2Status_;
  int domainId_;

  // Settings
  bool showRos2Status_;
  bool showDomainId_;
  bool highlightNonDefaultDomain_;
  StatusDisplayLocation displayLocation_;

  // Health check timer
  QTimer* healthCheckTimer_;

  // Settings key constants
  static constexpr const char* SETTINGS_GROUP = "RosStatusDisplay";
  static constexpr const char* KEY_SHOW_ROS_STATUS = "showRos2Status";
  static constexpr const char* KEY_SHOW_DOMAIN_ID = "showDomainId";
  static constexpr const char* KEY_HIGHLIGHT_NON_DEFAULT = "highlightNonDefaultDomain";
  static constexpr const char* KEY_DISPLAY_LOCATION = "displayLocation";

  // Health check interval in milliseconds
  static constexpr int HEALTH_CHECK_INTERVAL_MS = 1500;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_ROS_STATUS_WIDGET_HPP
