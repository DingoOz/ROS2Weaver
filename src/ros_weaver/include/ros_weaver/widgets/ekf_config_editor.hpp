#ifndef ROS_WEAVER_WIDGETS_EKF_CONFIG_EDITOR_HPP
#define ROS_WEAVER_WIDGETS_EKF_CONFIG_EDITOR_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QTableWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QGroupBox>
#include <QTabWidget>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

/**
 * @brief Widget for editing EKF configuration parameters
 *
 * Provides:
 * - Hierarchical tree view for general parameters
 * - Table views for covariance matrices with state labels
 * - Sensor configuration panels
 * - YAML import/export functionality
 */
class EKFConfigEditor : public QWidget {
  Q_OBJECT

public:
  explicit EKFConfigEditor(QWidget* parent = nullptr);
  ~EKFConfigEditor() override;

  // Configuration access
  EKFConfig config() const;
  void setConfig(const EKFConfig& config);

  // YAML import/export
  bool loadFromYaml(const QString& path);
  bool saveToYaml(const QString& path);
  QString toYaml() const;
  bool fromYaml(const QString& yaml);

signals:
  void configChanged();
  void loadRequested();
  void saveRequested();

private slots:
  void onGeneralParamChanged();
  void onProcessNoiseChanged(int row, int col);
  void onInitialCovarianceChanged(int row, int col);
  void onSensorConfigChanged();
  void onAddOdomSensor();
  void onAddImuSensor();
  void onAddPoseSensor();
  void onAddTwistSensor();
  void onRemoveSensor();
  void onLoadClicked();
  void onSaveClicked();

private:
  void setupUi();
  void setupGeneralTab();
  void setupCovarianceTab();
  void setupSensorsTab();
  void setupToolbar();

  void updateUiFromConfig();
  void updateConfigFromUi();

  // Create sensor config widget
  QWidget* createSensorWidget(const QString& type, int index);
  void updateSensorWidgets();

  // Covariance table helpers
  void populateCovarianceTable(QTableWidget* table,
                               const std::array<double, EKF_STATE_SIZE>& values);
  void readCovarianceTable(QTableWidget* table,
                           std::array<double, EKF_STATE_SIZE>& values);

  // Current configuration
  EKFConfig config_;
  bool updatingUi_ = false;

  // Main layout
  QTabWidget* tabWidget_ = nullptr;

  // General parameters tab
  QWidget* generalTab_ = nullptr;
  QLineEdit* nameEdit_ = nullptr;
  QDoubleSpinBox* frequencySpinBox_ = nullptr;
  QLineEdit* mapFrameEdit_ = nullptr;
  QLineEdit* odomFrameEdit_ = nullptr;
  QLineEdit* baseFrameEdit_ = nullptr;
  QComboBox* worldFrameCombo_ = nullptr;
  QCheckBox* publishTfCheck_ = nullptr;
  QCheckBox* twoDModeCheck_ = nullptr;
  QDoubleSpinBox* transformTimeoutSpinBox_ = nullptr;

  // Covariance tab
  QWidget* covarianceTab_ = nullptr;
  QTableWidget* processNoiseTable_ = nullptr;
  QTableWidget* initialCovarianceTable_ = nullptr;

  // Sensors tab
  QWidget* sensorsTab_ = nullptr;
  QTabWidget* sensorTabs_ = nullptr;
  QWidget* odomSensorsWidget_ = nullptr;
  QWidget* imuSensorsWidget_ = nullptr;
  QWidget* poseSensorsWidget_ = nullptr;
  QWidget* twistSensorsWidget_ = nullptr;

  // Toolbar buttons
  QPushButton* loadButton_ = nullptr;
  QPushButton* saveButton_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_EKF_CONFIG_EDITOR_HPP
