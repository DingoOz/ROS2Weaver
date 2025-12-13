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
#include "ros_weaver/core/ollama_manager.hpp"

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
  void onEndpointChanged();

private:
  void setupUi();
  void connectSignals();
  void updateUiState();
  void populateModelCombo();

  // Connection Group
  QGroupBox* connectionGroup_;
  QLineEdit* endpointEdit_;
  QPushButton* testConnectionBtn_;
  QLabel* statusLabel_;
  QLabel* statusIcon_;

  // Model Selection Group
  QGroupBox* modelGroup_;
  QComboBox* modelCombo_;
  QCheckBox* autoLoadCheck_;
  QPushButton* refreshBtn_;

  // Model Management Group
  QGroupBox* managementGroup_;
  QListWidget* installedModelsList_;
  QPushButton* deleteModelBtn_;

  // Download Group
  QGroupBox* downloadGroup_;
  QComboBox* downloadModelCombo_;
  QPushButton* downloadBtn_;
  QPushButton* cancelDownloadBtn_;
  QProgressBar* downloadProgress_;
  QLabel* downloadStatusLabel_;

  // Enable/Disable
  QCheckBox* enableOllamaCheck_;

  // State
  bool isPulling_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_OLLAMA_SETTINGS_WIDGET_HPP
