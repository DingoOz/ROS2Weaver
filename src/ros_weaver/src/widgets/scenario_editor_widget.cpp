#include "ros_weaver/widgets/scenario_editor_widget.hpp"

#include <QSplitter>
#include <QFormLayout>
#include <QJsonDocument>

namespace ros_weaver {

ScenarioEditorWidget::ScenarioEditorWidget(QWidget* parent)
    : QWidget(parent)
    , toolbar_(nullptr)
    , stepList_(nullptr)
    , stepEditorStack_(nullptr)
    , stepTypeCombo_(nullptr)
    , stepDescriptionEdit_(nullptr)
    , stepEnabledCheck_(nullptr)
    , publishTopicCombo_(nullptr)
    , publishTypeCombo_(nullptr)
    , publishMessageEdit_(nullptr)
    , waitTopicCombo_(nullptr)
    , waitTypeCombo_(nullptr)
    , waitConditionEdit_(nullptr)
    , waitTimeoutSpin_(nullptr)
    , waitTimeSpin_(nullptr)
    , conditionExprEdit_(nullptr)
    , playBtn_(nullptr)
    , pauseBtn_(nullptr)
    , stopBtn_(nullptr)
    , stepBtn_(nullptr)
    , progressBar_(nullptr)
    , statusLabel_(nullptr)
    , recordBtn_(nullptr)
    , stopRecordBtn_(nullptr)
    , recordTopicsList_(nullptr)
    , player_(new ScenarioPlayer(this))
    , messageHandler_(nullptr)
    , currentStepIndex_(-1)
    , isModified_(false) {

  setupUi();

  // Connect player signals
  connect(player_, &ScenarioPlayer::stateChanged,
          this, &ScenarioEditorWidget::onPlayerStateChanged);
  connect(player_, &ScenarioPlayer::stepStarted,
          this, &ScenarioEditorWidget::onPlayerStepStarted);
  connect(player_, &ScenarioPlayer::stepCompleted,
          this, &ScenarioEditorWidget::onPlayerStepCompleted);
  connect(player_, &ScenarioPlayer::stepFailed,
          this, &ScenarioEditorWidget::onPlayerStepFailed);
  connect(player_, &ScenarioPlayer::progressUpdated,
          this, &ScenarioEditorWidget::onPlayerProgressUpdated);
  connect(player_, &ScenarioPlayer::scenarioCompleted,
          this, &ScenarioEditorWidget::scenarioCompleted);
}

ScenarioEditorWidget::~ScenarioEditorWidget() = default;

void ScenarioEditorWidget::setMessageHandler(GenericMessageHandler* handler) {
  messageHandler_ = handler;
  player_->setMessageHandler(handler);

  // Populate topic combos
  if (handler) {
    auto topics = handler->getAvailableTopics();
    QStringList topicNames = topics.keys();
    topicNames.sort();

    publishTopicCombo_->clear();
    publishTopicCombo_->addItems(topicNames);

    waitTopicCombo_->clear();
    waitTopicCombo_->addItems(topicNames);

    recordTopicsList_->clear();
    for (const QString& topic : topicNames) {
      auto* item = new QListWidgetItem(topic, recordTopicsList_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
    }
  }
}

void ScenarioEditorWidget::setRosNode(rclcpp::Node::SharedPtr node) {
  node_ = node;
  player_->setRosNode(node);
}

void ScenarioEditorWidget::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setupToolbar();
  mainLayout->addWidget(toolbar_);

  // Main splitter
  auto* splitter = new QSplitter(Qt::Horizontal, this);

  // Left panel: step list
  auto* leftPanel = new QWidget;
  auto* leftLayout = new QVBoxLayout(leftPanel);
  leftLayout->setContentsMargins(4, 4, 4, 4);

  auto* listLabel = new QLabel("Scenario Steps:");
  leftLayout->addWidget(listLabel);

  setupStepList();
  leftLayout->addWidget(stepList_);

  // Step list buttons
  auto* listBtnLayout = new QHBoxLayout;
  auto* addBtn = new QPushButton("+");
  auto* removeBtn = new QPushButton("-");
  auto* upBtn = new QPushButton("\u2191");
  auto* downBtn = new QPushButton("\u2193");
  auto* dupBtn = new QPushButton("Dup");

  addBtn->setToolTip("Add new step");
  removeBtn->setToolTip("Remove selected step");
  upBtn->setToolTip("Move step up");
  downBtn->setToolTip("Move step down");
  dupBtn->setToolTip("Duplicate step");

  addBtn->setMaximumWidth(30);
  removeBtn->setMaximumWidth(30);
  upBtn->setMaximumWidth(30);
  downBtn->setMaximumWidth(30);
  dupBtn->setMaximumWidth(40);

  connect(addBtn, &QPushButton::clicked, this, &ScenarioEditorWidget::onAddStep);
  connect(removeBtn, &QPushButton::clicked, this, &ScenarioEditorWidget::onRemoveStep);
  connect(upBtn, &QPushButton::clicked, this, &ScenarioEditorWidget::onMoveStepUp);
  connect(downBtn, &QPushButton::clicked, this, &ScenarioEditorWidget::onMoveStepDown);
  connect(dupBtn, &QPushButton::clicked, this, &ScenarioEditorWidget::onDuplicateStep);

  listBtnLayout->addWidget(addBtn);
  listBtnLayout->addWidget(removeBtn);
  listBtnLayout->addWidget(upBtn);
  listBtnLayout->addWidget(downBtn);
  listBtnLayout->addWidget(dupBtn);
  listBtnLayout->addStretch();
  leftLayout->addLayout(listBtnLayout);

  splitter->addWidget(leftPanel);

  // Right panel: step editor
  auto* rightPanel = new QWidget;
  auto* rightLayout = new QVBoxLayout(rightPanel);
  rightLayout->setContentsMargins(4, 4, 4, 4);

  setupStepEditor();
  rightLayout->addWidget(stepEditorStack_);

  setupPlaybackControls();
  setupRecordingControls();

  splitter->addWidget(rightPanel);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 2);

  mainLayout->addWidget(splitter);
}

void ScenarioEditorWidget::setupToolbar() {
  toolbar_ = new QToolBar(this);
  toolbar_->setIconSize(QSize(16, 16));

  auto* newAction = toolbar_->addAction("New");
  auto* openAction = toolbar_->addAction("Open");
  auto* saveAction = toolbar_->addAction("Save");
  toolbar_->addSeparator();
  auto* exportAction = toolbar_->addAction("Export to Pytest");

  connect(newAction, &QAction::triggered, this, &ScenarioEditorWidget::onNewScenario);
  connect(openAction, &QAction::triggered, this, &ScenarioEditorWidget::onOpenScenario);
  connect(saveAction, &QAction::triggered, this, &ScenarioEditorWidget::onSaveScenario);
  connect(exportAction, &QAction::triggered, this, &ScenarioEditorWidget::onExportToPytest);
}

void ScenarioEditorWidget::setupStepList() {
  stepList_ = new QListWidget;
  stepList_->setSelectionMode(QAbstractItemView::SingleSelection);
  stepList_->setDragDropMode(QAbstractItemView::InternalMove);

  connect(stepList_, &QListWidget::currentRowChanged,
          this, &ScenarioEditorWidget::onStepSelected);
}

void ScenarioEditorWidget::setupStepEditor() {
  stepEditorStack_ = new QStackedWidget;

  // Empty state
  auto* emptyWidget = new QWidget;
  auto* emptyLayout = new QVBoxLayout(emptyWidget);
  auto* emptyLabel = new QLabel("Select or add a step to edit");
  emptyLabel->setAlignment(Qt::AlignCenter);
  emptyLayout->addWidget(emptyLabel);
  stepEditorStack_->addWidget(emptyWidget);

  // Step editor
  auto* editorWidget = new QWidget;
  auto* editorLayout = new QVBoxLayout(editorWidget);

  // Common properties
  auto* commonGroup = new QGroupBox("Step Properties");
  auto* commonLayout = new QFormLayout(commonGroup);

  stepTypeCombo_ = new QComboBox;
  stepTypeCombo_->addItem("Publish Message", static_cast<int>(ScenarioStep::PublishMessage));
  stepTypeCombo_->addItem("Wait for Message", static_cast<int>(ScenarioStep::WaitForMessage));
  stepTypeCombo_->addItem("Wait Time", static_cast<int>(ScenarioStep::WaitTime));
  stepTypeCombo_->addItem("Conditional", static_cast<int>(ScenarioStep::Conditional));
  connect(stepTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ScenarioEditorWidget::onStepTypeChanged);

  stepDescriptionEdit_ = new QLineEdit;
  stepEnabledCheck_ = new QCheckBox("Enabled");
  stepEnabledCheck_->setChecked(true);

  commonLayout->addRow("Type:", stepTypeCombo_);
  commonLayout->addRow("Description:", stepDescriptionEdit_);
  commonLayout->addRow("", stepEnabledCheck_);

  editorLayout->addWidget(commonGroup);

  // Type-specific editors (stacked)
  auto* typeStack = new QStackedWidget;

  // Publish message editor
  auto* publishWidget = new QWidget;
  auto* publishLayout = new QFormLayout(publishWidget);
  publishTopicCombo_ = new QComboBox;
  publishTopicCombo_->setEditable(true);
  publishTypeCombo_ = new QComboBox;
  publishTypeCombo_->setEditable(true);
  publishMessageEdit_ = new QTextEdit;
  publishMessageEdit_->setPlaceholderText("Enter JSON message data...");
  publishLayout->addRow("Topic:", publishTopicCombo_);
  publishLayout->addRow("Type:", publishTypeCombo_);
  publishLayout->addRow("Message:", publishMessageEdit_);
  typeStack->addWidget(publishWidget);

  // Wait for message editor
  auto* waitMsgWidget = new QWidget;
  auto* waitMsgLayout = new QFormLayout(waitMsgWidget);
  waitTopicCombo_ = new QComboBox;
  waitTopicCombo_->setEditable(true);
  waitTypeCombo_ = new QComboBox;
  waitTypeCombo_->setEditable(true);
  waitConditionEdit_ = new QLineEdit;
  waitConditionEdit_->setPlaceholderText("e.g., msg.status == 1");
  waitTimeoutSpin_ = new QSpinBox;
  waitTimeoutSpin_->setRange(100, 60000);
  waitTimeoutSpin_->setValue(5000);
  waitTimeoutSpin_->setSuffix(" ms");
  waitMsgLayout->addRow("Topic:", waitTopicCombo_);
  waitMsgLayout->addRow("Type:", waitTypeCombo_);
  waitMsgLayout->addRow("Condition:", waitConditionEdit_);
  waitMsgLayout->addRow("Timeout:", waitTimeoutSpin_);
  typeStack->addWidget(waitMsgWidget);

  // Wait time editor
  auto* waitTimeWidget = new QWidget;
  auto* waitTimeLayout = new QFormLayout(waitTimeWidget);
  waitTimeSpin_ = new QSpinBox;
  waitTimeSpin_->setRange(1, 60000);
  waitTimeSpin_->setValue(1000);
  waitTimeSpin_->setSuffix(" ms");
  waitTimeLayout->addRow("Duration:", waitTimeSpin_);
  typeStack->addWidget(waitTimeWidget);

  // Conditional editor
  auto* condWidget = new QWidget;
  auto* condLayout = new QFormLayout(condWidget);
  conditionExprEdit_ = new QLineEdit;
  conditionExprEdit_->setPlaceholderText("e.g., msg.value > 10");
  condLayout->addRow("Expression:", conditionExprEdit_);
  typeStack->addWidget(condWidget);

  editorLayout->addWidget(typeStack);

  // Connect type combo to stack
  connect(stepTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          typeStack, &QStackedWidget::setCurrentIndex);

  // Playback controls group
  auto* playbackGroup = new QGroupBox("Playback");
  auto* playbackLayout = new QVBoxLayout(playbackGroup);

  auto* buttonLayout = new QHBoxLayout;
  playBtn_ = new QPushButton("Play");
  pauseBtn_ = new QPushButton("Pause");
  stopBtn_ = new QPushButton("Stop");
  stepBtn_ = new QPushButton("Step");

  connect(playBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onPlay);
  connect(pauseBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onPause);
  connect(stopBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onStop);
  connect(stepBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onStepForward);

  buttonLayout->addWidget(playBtn_);
  buttonLayout->addWidget(pauseBtn_);
  buttonLayout->addWidget(stopBtn_);
  buttonLayout->addWidget(stepBtn_);
  playbackLayout->addLayout(buttonLayout);

  progressBar_ = new QProgressBar;
  progressBar_->setRange(0, 100);
  progressBar_->setValue(0);
  playbackLayout->addWidget(progressBar_);

  statusLabel_ = new QLabel("Ready");
  playbackLayout->addWidget(statusLabel_);

  editorLayout->addWidget(playbackGroup);

  // Recording group
  auto* recordGroup = new QGroupBox("Recording");
  auto* recordLayout = new QVBoxLayout(recordGroup);

  recordTopicsList_ = new QListWidget;
  recordTopicsList_->setMaximumHeight(100);
  recordLayout->addWidget(new QLabel("Topics to record:"));
  recordLayout->addWidget(recordTopicsList_);

  auto* recordBtnLayout = new QHBoxLayout;
  recordBtn_ = new QPushButton("Start Recording");
  stopRecordBtn_ = new QPushButton("Stop Recording");
  stopRecordBtn_->setEnabled(false);

  connect(recordBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onStartRecording);
  connect(stopRecordBtn_, &QPushButton::clicked, this, &ScenarioEditorWidget::onStopRecording);

  recordBtnLayout->addWidget(recordBtn_);
  recordBtnLayout->addWidget(stopRecordBtn_);
  recordLayout->addLayout(recordBtnLayout);

  editorLayout->addWidget(recordGroup);
  editorLayout->addStretch();

  stepEditorStack_->addWidget(editorWidget);
  stepEditorStack_->setCurrentIndex(0);
}

void ScenarioEditorWidget::setupPlaybackControls() {
  updatePlaybackState();
}

void ScenarioEditorWidget::setupRecordingControls() {
  // Already set up in setupStepEditor
}

void ScenarioEditorWidget::onNewScenario() {
  if (isModified_) {
    auto result = QMessageBox::question(this, "Unsaved Changes",
                                         "Save changes before creating a new scenario?",
                                         QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (result == QMessageBox::Save) {
      onSaveScenario();
    } else if (result == QMessageBox::Cancel) {
      return;
    }
  }

  currentScenario_ = TestScenario();
  currentScenario_.name = "New Scenario";
  currentScenario_.createdDate = QDateTime::currentDateTime().toString(Qt::ISODate);
  currentFilePath_.clear();
  isModified_ = false;
  currentStepIndex_ = -1;

  updateStepList();
  clearStepEditor();

  emit scenarioLoaded(currentScenario_.name);
}

void ScenarioEditorWidget::onOpenScenario() {
  QString filePath = QFileDialog::getOpenFileName(
      this, "Open Scenario", QString(),
      "Scenario Files (*.scenario.json);;All Files (*)");

  if (filePath.isEmpty()) return;

  if (player_->loadScenarioFromFile(filePath)) {
    currentScenario_ = player_->currentScenario();
    currentFilePath_ = filePath;
    isModified_ = false;
    currentStepIndex_ = -1;

    updateStepList();
    clearStepEditor();

    emit scenarioLoaded(currentScenario_.name);
  } else {
    QMessageBox::warning(this, "Error", "Failed to load scenario file.");
  }
}

void ScenarioEditorWidget::onSaveScenario() {
  if (currentFilePath_.isEmpty()) {
    QString filePath = QFileDialog::getSaveFileName(
        this, "Save Scenario", currentScenario_.name + ".scenario.json",
        "Scenario Files (*.scenario.json);;All Files (*)");

    if (filePath.isEmpty()) return;
    currentFilePath_ = filePath;
  }

  saveCurrentStep();
  player_->loadScenario(currentScenario_);

  if (player_->saveScenario(currentFilePath_)) {
    isModified_ = false;
  } else {
    QMessageBox::warning(this, "Error", "Failed to save scenario file.");
  }
}

void ScenarioEditorWidget::onExportToPytest() {
  QString filePath = QFileDialog::getSaveFileName(
      this, "Export to Pytest",
      currentScenario_.name.toLower().replace(" ", "_") + "_test.py",
      "Python Files (*.py);;All Files (*)");

  if (filePath.isEmpty()) return;

  saveCurrentStep();
  player_->loadScenario(currentScenario_);

  QString pytestCode = player_->exportToPytest();

  QFile file(filePath);
  if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    file.write(pytestCode.toUtf8());
    file.close();
    QMessageBox::information(this, "Export Complete",
                              "Scenario exported to pytest script.");
  } else {
    QMessageBox::warning(this, "Error", "Failed to write pytest file.");
  }
}

void ScenarioEditorWidget::onAddStep() {
  ScenarioStep step;
  step.type = ScenarioStep::PublishMessage;
  step.description = QString("Step %1").arg(currentScenario_.steps.size() + 1);
  step.enabled = true;

  currentScenario_.steps.append(step);
  isModified_ = true;

  updateStepList();
  stepList_->setCurrentRow(currentScenario_.steps.size() - 1);
}

void ScenarioEditorWidget::onRemoveStep() {
  int row = stepList_->currentRow();
  if (row < 0 || row >= currentScenario_.steps.size()) return;

  currentScenario_.steps.removeAt(row);
  isModified_ = true;
  currentStepIndex_ = -1;

  updateStepList();

  if (row < currentScenario_.steps.size()) {
    stepList_->setCurrentRow(row);
  } else if (!currentScenario_.steps.isEmpty()) {
    stepList_->setCurrentRow(currentScenario_.steps.size() - 1);
  } else {
    clearStepEditor();
  }
}

void ScenarioEditorWidget::onMoveStepUp() {
  int row = stepList_->currentRow();
  if (row <= 0 || row >= currentScenario_.steps.size()) return;

  currentScenario_.steps.swapItemsAt(row, row - 1);
  isModified_ = true;

  updateStepList();
  stepList_->setCurrentRow(row - 1);
}

void ScenarioEditorWidget::onMoveStepDown() {
  int row = stepList_->currentRow();
  if (row < 0 || row >= currentScenario_.steps.size() - 1) return;

  currentScenario_.steps.swapItemsAt(row, row + 1);
  isModified_ = true;

  updateStepList();
  stepList_->setCurrentRow(row + 1);
}

void ScenarioEditorWidget::onDuplicateStep() {
  int row = stepList_->currentRow();
  if (row < 0 || row >= currentScenario_.steps.size()) return;

  ScenarioStep copy = currentScenario_.steps[row];
  copy.description += " (copy)";

  currentScenario_.steps.insert(row + 1, copy);
  isModified_ = true;

  updateStepList();
  stepList_->setCurrentRow(row + 1);
}

void ScenarioEditorWidget::onStepSelected(int row) {
  // Save previous step
  if (currentStepIndex_ >= 0 && currentStepIndex_ < currentScenario_.steps.size()) {
    saveCurrentStep();
  }

  currentStepIndex_ = row;

  if (row >= 0 && row < currentScenario_.steps.size()) {
    stepEditorStack_->setCurrentIndex(1);
    populateEditorFromStep(currentScenario_.steps[row]);
  } else {
    stepEditorStack_->setCurrentIndex(0);
  }
}

void ScenarioEditorWidget::onStepTypeChanged(int index) {
  Q_UNUSED(index)
  isModified_ = true;
}

void ScenarioEditorWidget::onPlay() {
  saveCurrentStep();
  player_->loadScenario(currentScenario_);
  player_->play();
}

void ScenarioEditorWidget::onPause() {
  player_->pause();
}

void ScenarioEditorWidget::onStop() {
  player_->stop();
}

void ScenarioEditorWidget::onStepForward() {
  if (player_->state() == ScenarioState::Idle) {
    saveCurrentStep();
    player_->loadScenario(currentScenario_);
  }
  player_->stepForward();
}

void ScenarioEditorWidget::onStartRecording() {
  QStringList topics;
  for (int i = 0; i < recordTopicsList_->count(); ++i) {
    auto* item = recordTopicsList_->item(i);
    if (item->checkState() == Qt::Checked) {
      topics.append(item->text());
    }
  }

  if (topics.isEmpty()) {
    QMessageBox::warning(this, "No Topics Selected",
                          "Please select at least one topic to record.");
    return;
  }

  player_->startRecording(topics);
  recordBtn_->setEnabled(false);
  stopRecordBtn_->setEnabled(true);
  setPlaybackEnabled(false);
}

void ScenarioEditorWidget::onStopRecording() {
  player_->stopRecording();

  TestScenario recorded = player_->getRecordedScenario();
  if (!recorded.steps.isEmpty()) {
    auto result = QMessageBox::question(this, "Recording Complete",
                                         QString("Recorded %1 messages. Replace current scenario?")
                                             .arg(recorded.steps.size()),
                                         QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes) {
      currentScenario_ = recorded;
      isModified_ = true;
      updateStepList();
    }
  }

  recordBtn_->setEnabled(true);
  stopRecordBtn_->setEnabled(false);
  setPlaybackEnabled(true);
}

void ScenarioEditorWidget::onPlayerStateChanged(ScenarioState state) {
  updatePlaybackState();

  switch (state) {
    case ScenarioState::Idle:
      statusLabel_->setText("Ready");
      break;
    case ScenarioState::Running:
      statusLabel_->setText("Running...");
      break;
    case ScenarioState::Paused:
      statusLabel_->setText("Paused");
      break;
    case ScenarioState::Completed:
      statusLabel_->setText("Completed");
      break;
    case ScenarioState::Failed:
      statusLabel_->setText("Failed");
      break;
    case ScenarioState::Cancelled:
      statusLabel_->setText("Cancelled");
      break;
  }
}

void ScenarioEditorWidget::onPlayerStepStarted(int stepIndex, const ScenarioStep& step) {
  Q_UNUSED(step)

  // Highlight current step in list
  if (stepIndex < stepList_->count()) {
    stepList_->item(stepIndex)->setBackground(QColor(100, 150, 200, 100));
  }
}

void ScenarioEditorWidget::onPlayerStepCompleted(int stepIndex, const StepResult& result) {
  if (stepIndex < stepList_->count()) {
    auto* item = stepList_->item(stepIndex);
    if (result.success) {
      item->setBackground(QColor(100, 200, 100, 100));
    } else {
      item->setBackground(QColor(200, 100, 100, 100));
    }
  }

  emit stepExecuted(stepIndex, result.success);
}

void ScenarioEditorWidget::onPlayerStepFailed(int stepIndex, const QString& error) {
  if (stepIndex < stepList_->count()) {
    stepList_->item(stepIndex)->setBackground(QColor(200, 100, 100, 100));
  }

  statusLabel_->setText(QString("Step %1 failed: %2").arg(stepIndex + 1).arg(error));
  emit stepExecuted(stepIndex, false);
}

void ScenarioEditorWidget::onPlayerProgressUpdated(double progress) {
  progressBar_->setValue(static_cast<int>(progress * 100));
}

void ScenarioEditorWidget::updateStepList() {
  stepList_->clear();

  for (int i = 0; i < currentScenario_.steps.size(); ++i) {
    const auto& step = currentScenario_.steps[i];

    QString typeStr;
    switch (step.type) {
      case ScenarioStep::PublishMessage:
        typeStr = "Pub";
        break;
      case ScenarioStep::WaitForMessage:
        typeStr = "Wait";
        break;
      case ScenarioStep::WaitTime:
        typeStr = "Delay";
        break;
      case ScenarioStep::Conditional:
        typeStr = "If";
        break;
    }

    QString text = QString("[%1] %2").arg(typeStr, step.description);
    auto* item = new QListWidgetItem(text, stepList_);

    if (!step.enabled) {
      item->setForeground(Qt::gray);
    }
  }
}

void ScenarioEditorWidget::updateStepEditor() {
  if (currentStepIndex_ >= 0 && currentStepIndex_ < currentScenario_.steps.size()) {
    populateEditorFromStep(currentScenario_.steps[currentStepIndex_]);
  }
}

void ScenarioEditorWidget::saveCurrentStep() {
  if (currentStepIndex_ < 0 || currentStepIndex_ >= currentScenario_.steps.size()) {
    return;
  }

  currentScenario_.steps[currentStepIndex_] = createStepFromEditor();
  isModified_ = true;
}

void ScenarioEditorWidget::clearStepEditor() {
  stepEditorStack_->setCurrentIndex(0);
  currentStepIndex_ = -1;
}

void ScenarioEditorWidget::updatePlaybackState() {
  ScenarioState state = player_->state();

  bool isIdle = (state == ScenarioState::Idle ||
                 state == ScenarioState::Completed ||
                 state == ScenarioState::Failed ||
                 state == ScenarioState::Cancelled);
  bool isRunning = (state == ScenarioState::Running);
  bool isPaused = (state == ScenarioState::Paused);

  playBtn_->setEnabled(isIdle || isPaused);
  pauseBtn_->setEnabled(isRunning);
  stopBtn_->setEnabled(isRunning || isPaused);
  stepBtn_->setEnabled(isIdle || isPaused);
}

void ScenarioEditorWidget::setPlaybackEnabled(bool enabled) {
  playBtn_->setEnabled(enabled);
  pauseBtn_->setEnabled(enabled);
  stopBtn_->setEnabled(enabled);
  stepBtn_->setEnabled(enabled);
}

ScenarioStep ScenarioEditorWidget::createStepFromEditor() {
  ScenarioStep step;

  step.type = static_cast<ScenarioStep::Type>(
      stepTypeCombo_->currentData().toInt());
  step.description = stepDescriptionEdit_->text();
  step.enabled = stepEnabledCheck_->isChecked();

  switch (step.type) {
    case ScenarioStep::PublishMessage:
      step.topicName = publishTopicCombo_->currentText();
      step.messageType = publishTypeCombo_->currentText();
      {
        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(
            publishMessageEdit_->toPlainText().toUtf8(), &error);
        if (error.error == QJsonParseError::NoError) {
          step.messageData = doc.object();
        }
      }
      break;

    case ScenarioStep::WaitForMessage:
      step.topicName = waitTopicCombo_->currentText();
      step.messageType = waitTypeCombo_->currentText();
      step.conditionExpression = waitConditionEdit_->text();
      step.timeoutMs = waitTimeoutSpin_->value();
      break;

    case ScenarioStep::WaitTime:
      step.waitTimeMs = waitTimeSpin_->value();
      break;

    case ScenarioStep::Conditional:
      step.conditionExpression = conditionExprEdit_->text();
      break;
  }

  return step;
}

void ScenarioEditorWidget::populateEditorFromStep(const ScenarioStep& step) {
  // Block signals while populating
  stepTypeCombo_->blockSignals(true);
  stepDescriptionEdit_->blockSignals(true);
  stepEnabledCheck_->blockSignals(true);

  // Common properties
  stepTypeCombo_->setCurrentIndex(static_cast<int>(step.type));
  stepDescriptionEdit_->setText(step.description);
  stepEnabledCheck_->setChecked(step.enabled);

  // Type-specific properties
  switch (step.type) {
    case ScenarioStep::PublishMessage:
      publishTopicCombo_->setCurrentText(step.topicName);
      publishTypeCombo_->setCurrentText(step.messageType);
      publishMessageEdit_->setPlainText(
          QJsonDocument(step.messageData).toJson(QJsonDocument::Indented));
      break;

    case ScenarioStep::WaitForMessage:
      waitTopicCombo_->setCurrentText(step.topicName);
      waitTypeCombo_->setCurrentText(step.messageType);
      waitConditionEdit_->setText(step.conditionExpression);
      waitTimeoutSpin_->setValue(step.timeoutMs);
      break;

    case ScenarioStep::WaitTime:
      waitTimeSpin_->setValue(step.waitTimeMs);
      break;

    case ScenarioStep::Conditional:
      conditionExprEdit_->setText(step.conditionExpression);
      break;
  }

  stepTypeCombo_->blockSignals(false);
  stepDescriptionEdit_->blockSignals(false);
  stepEnabledCheck_->blockSignals(false);
}

}  // namespace ros_weaver
