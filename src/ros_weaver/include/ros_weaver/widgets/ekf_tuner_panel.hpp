#ifndef ROS_WEAVER_WIDGETS_EKF_TUNER_PANEL_HPP
#define ROS_WEAVER_WIDGETS_EKF_TUNER_PANEL_HPP

#include <QWidget>
#include <QSplitter>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QToolBar>
#include <QComboBox>
#include <QSpinBox>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

// Forward declarations
class BagManager;
class EKFSimulationEngine;
class EKFMetricsAnalyzer;
class EKFParameterSweep;
class EKFConfigEditor;
class EKFPathVisualizer;
class EKFResultsPanel;

/**
 * @brief Main panel for the EKF Tuner Workbench
 *
 * Integrates all EKF tuning components:
 * - BagManager for loading input bags
 * - EKFConfigEditor for parameter editing
 * - EKFPathVisualizer for trajectory display
 * - EKFResultsPanel for metrics comparison
 * - EKFSimulationEngine for running simulations
 * - EKFParameterSweep for batch tuning
 */
class EKFTunerPanel : public QWidget {
  Q_OBJECT

public:
  explicit EKFTunerPanel(QWidget* parent = nullptr);
  ~EKFTunerPanel() override;

  // Bag operations
  bool openBag(const QString& path);
  void closeBag();
  bool isBagOpen() const;
  QString currentBagPath() const;

  // Configuration
  void setConfig(const EKFConfig& config);
  EKFConfig config() const;

  // Load/save config
  bool loadConfig(const QString& path);
  bool saveConfig(const QString& path);

  // Component access
  BagManager* bagManager() const;
  EKFSimulationEngine* simulationEngine() const;
  EKFConfigEditor* configEditor() const;
  EKFPathVisualizer* pathVisualizer() const;
  EKFResultsPanel* resultsPanel() const;

signals:
  void bagOpened(const QString& path);
  void bagClosed();
  void simulationStarted();
  void simulationCompleted(bool success);
  void sweepStarted(int totalConfigs);
  void sweepCompleted(bool success);

public slots:
  void onOpenBagClicked();
  void onLoadConfigClicked();
  void onSaveConfigClicked();
  void onRunSimulationClicked();
  void onRunSweepClicked();
  void onStopClicked();

private slots:
  void onBagOpened();
  void onBagClosed();
  void onBagError(const QString& error);

  void onSimulationStatusChanged(int status);
  void onSimulationProgress(int percent, const QString& message);
  void onSimulationFinished(bool success);
  void onTrajectoriesReady(const Trajectory& ekf,
                           const Trajectory& rawOdom,
                           const Trajectory& groundTruth);

  void onSweepProgress(int completed, int total, const QString& message);
  void onSweepFinished(bool success);
  void onSweepConfigCompleted(int index, const EKFSweepResult& result);

  void onResultSelected(int index, const EKFSweepResult& result);
  void onExportBestRequested();
  void onExportAllRequested();

  void onConfigChanged();

private:
  void setupUi();
  void setupToolbar();
  void setupLeftPanel();
  void setupCenterPanel();
  void setupRightPanel();
  void setupConnections();

  void updateUiState();
  void showSweepDialog();

  // Core components
  BagManager* bagManager_ = nullptr;
  EKFSimulationEngine* simulationEngine_ = nullptr;
  EKFMetricsAnalyzer* metricsAnalyzer_ = nullptr;
  EKFParameterSweep* parameterSweep_ = nullptr;

  // UI - Main layout
  QSplitter* mainSplitter_ = nullptr;

  // UI - Toolbar
  QToolBar* toolbar_ = nullptr;
  QPushButton* openBagButton_ = nullptr;
  QPushButton* loadConfigButton_ = nullptr;
  QPushButton* saveConfigButton_ = nullptr;
  QPushButton* runButton_ = nullptr;
  QPushButton* runSweepButton_ = nullptr;
  QPushButton* stopButton_ = nullptr;
  QLabel* statusLabel_ = nullptr;
  QProgressBar* progressBar_ = nullptr;

  // UI - Left panel (config/bag)
  QWidget* leftPanel_ = nullptr;
  EKFConfigEditor* configEditor_ = nullptr;
  QLabel* bagInfoLabel_ = nullptr;

  // UI - Center panel (visualizer)
  QWidget* centerPanel_ = nullptr;
  EKFPathVisualizer* pathVisualizer_ = nullptr;

  // UI - Right panel (results)
  QWidget* rightPanel_ = nullptr;
  EKFResultsPanel* resultsPanel_ = nullptr;

  // State
  QString currentBagPath_;
  bool simulationRunning_ = false;
  bool sweepRunning_ = false;

  // Last trajectories for result display
  Trajectory lastGroundTruth_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_EKF_TUNER_PANEL_HPP
