#ifndef ROS_WEAVER_WIDGETS_SCENARIO_EDITOR_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_SCENARIO_EDITOR_WIDGET_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolBar>
#include <QListWidget>
#include <QStackedWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QGroupBox>
#include <QCheckBox>
#include <QFileDialog>
#include <QMessageBox>

#include "ros_weaver/core/scenario_player.hpp"
#include "ros_weaver/core/generic_message_handler.hpp"

namespace ros_weaver {

class ScenarioEditorWidget : public QWidget {
  Q_OBJECT

public:
  explicit ScenarioEditorWidget(QWidget* parent = nullptr);
  ~ScenarioEditorWidget() override;

  void setMessageHandler(GenericMessageHandler* handler);
  void setRosNode(rclcpp::Node::SharedPtr node);

  ScenarioPlayer* scenarioPlayer() { return player_; }

signals:
  void scenarioLoaded(const QString& name);
  void scenarioCompleted(bool success);
  void stepExecuted(int stepIndex, bool success);

private slots:
  void onNewScenario();
  void onOpenScenario();
  void onSaveScenario();
  void onExportToPytest();

  void onAddStep();
  void onRemoveStep();
  void onMoveStepUp();
  void onMoveStepDown();
  void onDuplicateStep();

  void onStepSelected(int row);
  void onStepTypeChanged(int index);

  void onPlay();
  void onPause();
  void onStop();
  void onStepForward();

  void onStartRecording();
  void onStopRecording();

  void onPlayerStateChanged(ScenarioState state);
  void onPlayerStepStarted(int stepIndex, const ScenarioStep& step);
  void onPlayerStepCompleted(int stepIndex, const StepResult& result);
  void onPlayerStepFailed(int stepIndex, const QString& error);
  void onPlayerProgressUpdated(double progress);

private:
  void setupUi();
  void setupToolbar();
  void setupStepList();
  void setupStepEditor();
  void setupPlaybackControls();
  void setupRecordingControls();

  void updateStepList();
  void updateStepEditor();
  void saveCurrentStep();
  void clearStepEditor();

  void updatePlaybackState();
  void setPlaybackEnabled(bool enabled);

  ScenarioStep createStepFromEditor();
  void populateEditorFromStep(const ScenarioStep& step);

  // UI Components
  QToolBar* toolbar_;
  QListWidget* stepList_;
  QStackedWidget* stepEditorStack_;

  // Step editor widgets
  QComboBox* stepTypeCombo_;
  QLineEdit* stepDescriptionEdit_;
  QCheckBox* stepEnabledCheck_;

  // Publish step widgets
  QComboBox* publishTopicCombo_;
  QComboBox* publishTypeCombo_;
  QTextEdit* publishMessageEdit_;

  // Wait for message step widgets
  QComboBox* waitTopicCombo_;
  QComboBox* waitTypeCombo_;
  QLineEdit* waitConditionEdit_;
  QSpinBox* waitTimeoutSpin_;

  // Wait time step widgets
  QSpinBox* waitTimeSpin_;

  // Conditional step widgets
  QLineEdit* conditionExprEdit_;

  // Playback controls
  QPushButton* playBtn_;
  QPushButton* pauseBtn_;
  QPushButton* stopBtn_;
  QPushButton* stepBtn_;
  QProgressBar* progressBar_;
  QLabel* statusLabel_;

  // Recording controls
  QPushButton* recordBtn_;
  QPushButton* stopRecordBtn_;
  QListWidget* recordTopicsList_;

  // Core
  ScenarioPlayer* player_;
  GenericMessageHandler* messageHandler_;
  rclcpp::Node::SharedPtr node_;

  TestScenario currentScenario_;
  int currentStepIndex_;
  bool isModified_;
  QString currentFilePath_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_SCENARIO_EDITOR_WIDGET_HPP
