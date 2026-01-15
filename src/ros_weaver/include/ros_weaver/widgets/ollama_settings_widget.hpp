#ifndef ROS_WEAVER_OLLAMA_SETTINGS_WIDGET_HPP
#define ROS_WEAVER_OLLAMA_SETTINGS_WIDGET_HPP

#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QListWidget>
#include <QGroupBox>
#include <QTimer>
#include <QSpinBox>
#include <QFontComboBox>
#include <QTabWidget>
#include "ros_weaver/core/ollama_manager.hpp"

namespace ros_weaver {
class LocalAIStatusWidget;
}

namespace ros_weaver {

class OllamaSettingsWidget : public QWidget {
  Q_OBJECT

public:
  explicit OllamaSettingsWidget(QWidget* parent = nullptr);
  ~OllamaSettingsWidget() = default;

  // Apply current settings to the manager
  void applySettings();

  // Reset to current saved values
  void resetToSaved();

  // Set reference to status widget for applying settings
  void setLocalAIStatusWidget(LocalAIStatusWidget* widget);

signals:
  void settingsChanged();

private slots:
  void onOllamaStatusChanged(bool running);
  void onLocalModelsUpdated(const QList<OllamaModel>& models);
  void onPullProgress(const QString& modelName, double progress, const QString& status);
  void onPullCompleted(const QString& modelName, bool success, const QString& error);
  void onModelDeleted(const QString& modelName, bool success);
  void onDownloadClicked();
  void onDeleteClicked();
  void onRefreshClicked();
  void onTestConnectionClicked();
  void onTestConnectionTimeout();
  void onEndpointChanged();
  void onEditSystemPromptClicked();

  // Appearance slots
  void onUserTextColorClicked();
  void onUserBgColorClicked();
  void onAssistantTextColorClicked();
  void onAssistantBgColorClicked();
  void onCodeBgColorClicked();
  void onResetAppearanceClicked();
  void updatePreview();

private:
  void setupUi();
  void connectSignals();
  void updateUiState();
  void populateModelCombo();
  void setButtonColor(QPushButton* button, const QColor& color);

  // Tab pages
  QWidget* createConnectionTab();
  QWidget* createModelsTab();
  QWidget* createAppearanceTab();

  // Main layout widgets
  QTabWidget* tabWidget_;

  // Enable/Disable (always visible at top)
  QCheckBox* enableOllamaCheck_;

  // Connection Tab widgets
  QGroupBox* connectionGroup_;
  QLineEdit* endpointEdit_;
  QPushButton* testConnectionBtn_;
  QLabel* statusLabel_;
  QLabel* statusIcon_;

  // Status Bar Display
  QGroupBox* statusBarGroup_;
  QCheckBox* showStatusCheck_;
  QCheckBox* showModelNameCheck_;

  // Performance
  QGroupBox* performanceGroup_;
  QSpinBox* cpuThreadsSpin_;
  QCheckBox* toolCallingCheck_;

  // System Prompt (opens dialog)
  QGroupBox* systemPromptGroup_;
  QLabel* systemPromptPreview_;
  QPushButton* editSystemPromptBtn_;
  QString currentSystemPrompt_;

  // Models Tab widgets
  QGroupBox* modelGroup_;
  QComboBox* modelCombo_;
  QCheckBox* autoLoadCheck_;
  QPushButton* refreshBtn_;

  // Model Management
  QGroupBox* managementGroup_;
  QListWidget* installedModelsList_;
  QPushButton* deleteModelBtn_;
  QPushButton* refreshModelsBtn_;

  // Download
  QGroupBox* downloadGroup_;
  QComboBox* downloadModelCombo_;
  QPushButton* downloadBtn_;
  QPushButton* cancelDownloadBtn_;
  QProgressBar* downloadProgress_;
  QLabel* downloadStatusLabel_;

  // Appearance Tab widgets
  QGroupBox* appearanceGroup_;
  QFontComboBox* fontFamilyCombo_;
  QSpinBox* fontSizeSpin_;
  QCheckBox* fontBoldCheck_;
  QCheckBox* fontItalicCheck_;
  QPushButton* userTextColorBtn_;
  QPushButton* userBgColorBtn_;
  QPushButton* assistantTextColorBtn_;
  QPushButton* assistantBgColorBtn_;
  QPushButton* codeBgColorBtn_;
  QPushButton* resetAppearanceBtn_;
  QLabel* previewLabel_;

  // Current color values for display
  QColor currentUserTextColor_;
  QColor currentUserBgColor_;
  QColor currentAssistantTextColor_;
  QColor currentAssistantBgColor_;
  QColor currentCodeBgColor_;

  // Connection test timeout
  QTimer* connectionTestTimer_;
  bool isTestingConnection_ = false;

  // Reference to status widget
  LocalAIStatusWidget* localAIStatusWidget_ = nullptr;

  // State
  bool isPulling_ = false;

  // Connection test timeout in ms
  static constexpr int CONNECTION_TEST_TIMEOUT_MS = 5000;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_OLLAMA_SETTINGS_WIDGET_HPP
