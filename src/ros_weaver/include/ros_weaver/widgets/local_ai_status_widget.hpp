#ifndef ROS_WEAVER_LOCAL_AI_STATUS_WIDGET_HPP
#define ROS_WEAVER_LOCAL_AI_STATUS_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QSettings>
#include <QElapsedTimer>

namespace ros_weaver {

class LocalAIStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit LocalAIStatusWidget(QWidget* parent = nullptr);
  ~LocalAIStatusWidget() override;

  // Settings
  bool isStatusVisible() const { return showStatus_; }
  bool isModelNameVisible() const { return showModelName_; }

public slots:
  void setShowStatus(bool show);
  void setShowModelName(bool show);

  // Manual refresh
  void refreshStatus();

  // Token generation speed tracking
  void onGenerationStarted();
  void onTokenReceived();
  void onGenerationFinished();

signals:
  void settingsChanged();

private slots:
  void onOllamaStatusChanged(bool running);
  void onOllamaSettingsChanged();
  void updateTokenSpeed();

private:
  void setupUi();
  void loadSettings();
  void saveSettings();
  void updateDisplay();

  // UI components
  QLabel* statusIconLabel_;
  QLabel* statusTextLabel_;
  QLabel* separatorLabel_;
  QLabel* tokenSpeedLabel_;

  // Settings
  bool showStatus_;
  bool showModelName_;

  // Token generation tracking
  QElapsedTimer tokenTimer_;
  int tokenCount_ = 0;
  bool isGenerating_ = false;
  double currentTokenSpeed_ = 0.0;

  // Settings key constants
  static constexpr const char* SETTINGS_GROUP = "LocalAIStatusDisplay";
  static constexpr const char* KEY_SHOW_STATUS = "showStatus";
  static constexpr const char* KEY_SHOW_MODEL_NAME = "showModelName";
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_LOCAL_AI_STATUS_WIDGET_HPP
