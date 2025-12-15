#ifndef ROS_WEAVER_WIDGETS_SLAM_CONFIG_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_SLAM_CONFIG_WIDGET_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QComboBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTextEdit>

#include "ros_weaver/core/slam_pipeline_manager.hpp"

namespace ros_weaver {

/**
 * @brief Widget for SLAM configuration and parameter tuning
 *
 * Provides UI for selecting SLAM presets, launching/stopping SLAM,
 * and tuning parameters with live update support.
 */
class SlamConfigWidget : public QWidget {
  Q_OBJECT

public:
  explicit SlamConfigWidget(QWidget* parent = nullptr);
  ~SlamConfigWidget() override = default;

  // Manager connection
  void setSlamPipelineManager(SlamPipelineManager* manager);

  // Preset operations
  void selectPreset(const QString& name);
  QString selectedPreset() const;

  // State access
  bool isSlamRunning() const;

signals:
  void launchRequested();
  void stopRequested();
  void parameterChanged(const QString& name, const QVariant& value);
  void presetSelected(const QString& name);
  void savePresetRequested(const QString& name);
  void autoRerunToggled(bool enabled);

public slots:
  void onSlamStarted();
  void onSlamStopped();
  void onSlamError(const QString& error);
  void onParametersUpdated();
  void onPresetLoaded(const QString& name);

private slots:
  void onPresetComboChanged(int index);
  void onLaunchClicked();
  void onStopClicked();
  void onSavePresetClicked();
  void onAutoRerunToggled(bool checked);
  void onParameterItemChanged(QTreeWidgetItem* item, int column);
  void onRefreshParamsClicked();

private:
  void setupUi();
  void setupConnections();
  void populatePresetCombo();
  void populateParameterTree();
  QTreeWidgetItem* createParameterItem(const QString& name, const QVariant& value);
  void updateStatus();

  SlamPipelineManager* manager_ = nullptr;

  // Preset selection
  QComboBox* presetCombo_ = nullptr;
  QPushButton* savePresetButton_ = nullptr;

  // Launch controls
  QPushButton* launchButton_ = nullptr;
  QPushButton* stopButton_ = nullptr;
  QLabel* statusLabel_ = nullptr;

  // Options
  QCheckBox* autoRerunCheckBox_ = nullptr;
  QCheckBox* useSimTimeCheckBox_ = nullptr;

  // Parameter tree
  QTreeWidget* parameterTree_ = nullptr;
  QPushButton* refreshParamsButton_ = nullptr;

  // Output
  QTextEdit* outputText_ = nullptr;

  // Tree columns
  enum Column {
    ColName = 0,
    ColValue,
    ColType
  };
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_SLAM_CONFIG_WIDGET_HPP
