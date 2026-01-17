#include "ros_weaver/widgets/urdf_viewer_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QColorDialog>
#include <QMessageBox>
#include <QDebug>

namespace ros_weaver {

URDFViewerPanel::URDFViewerPanel(QWidget* parent)
  : QWidget(parent) {
  // Create core components
  parser_ = new URDFParser(this);
  jointController_ = new URDFJointController(parser_, this);

  setupUi();
  setupConnections();
  setupShortcuts();

  // Initial state
  updateRotationControlsState();
}

URDFViewerPanel::~URDFViewerPanel() {
  // Qt parent-child ownership handles cleanup
}

void URDFViewerPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  // Toolbar at the top
  setupToolbar();
  mainLayout->addWidget(toolbar_);

  // Main splitter: 3D view | side panel
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  // 3D View
  view3D_ = new URDF3DView(this);
  view3D_->setParser(parser_);
  mainSplitter_->addWidget(view3D_);

  // Side panel (tree view + rotation controls)
  setupSidePanel();
  mainSplitter_->addWidget(sidePanel_);

  // Set splitter proportions (70% 3D view, 30% side panel)
  mainSplitter_->setStretchFactor(0, 7);
  mainSplitter_->setStretchFactor(1, 3);
  mainSplitter_->setSizes({700, 300});

  mainLayout->addWidget(mainSplitter_, 1);

  // Status bar area
  QHBoxLayout* statusLayout = new QHBoxLayout();
  statusLayout->setContentsMargins(5, 2, 5, 2);

  filePathLabel_ = new QLabel(tr("No file loaded"));
  filePathLabel_->setStyleSheet("color: gray;");
  statusLayout->addWidget(filePathLabel_);

  statusLayout->addStretch();

  statusLabel_ = new QLabel();
  statusLayout->addWidget(statusLabel_);

  mainLayout->addLayout(statusLayout);
}

void URDFViewerPanel::setupToolbar() {
  toolbar_ = new QToolBar(tr("URDF Viewer"), this);
  toolbar_->setMovable(false);
  toolbar_->setIconSize(QSize(20, 20));

  // Load action
  loadAction_ = toolbar_->addAction(tr("Load"));
  loadAction_->setToolTip(tr("Load URDF file (Ctrl+Shift+U)"));
  loadAction_->setShortcut(QKeySequence("Ctrl+Shift+U"));

  // Save action
  saveAction_ = toolbar_->addAction(tr("Save"));
  saveAction_->setToolTip(tr("Save URDF file (Ctrl+S)"));
  saveAction_->setEnabled(false);

  // Save As action
  saveAsAction_ = toolbar_->addAction(tr("Save As"));
  saveAsAction_->setToolTip(tr("Save URDF to new file"));
  saveAsAction_->setEnabled(false);

  toolbar_->addSeparator();

  // Render mode combo
  QLabel* renderLabel = new QLabel(tr("Render:"));
  toolbar_->addWidget(renderLabel);

  renderModeCombo_ = new QComboBox();
  renderModeCombo_->addItem(tr("Full Lighting"), static_cast<int>(RenderMode::FullLighting));
  renderModeCombo_->addItem(tr("Basic Shading"), static_cast<int>(RenderMode::BasicShading));
  renderModeCombo_->addItem(tr("Wireframe"), static_cast<int>(RenderMode::Wireframe));
  toolbar_->addWidget(renderModeCombo_);

  toolbar_->addSeparator();

  // Shadows checkbox
  shadowsCheckBox_ = new QCheckBox(tr("Shadows"));
  shadowsCheckBox_->setChecked(false);
  toolbar_->addWidget(shadowsCheckBox_);

  // Grid checkbox
  gridCheckBox_ = new QCheckBox(tr("Grid"));
  gridCheckBox_->setChecked(true);
  toolbar_->addWidget(gridCheckBox_);

  toolbar_->addSeparator();

  // Background color button
  bgColorButton_ = new QPushButton(tr("BG Color"));
  bgColorButton_->setToolTip(tr("Change background color"));
  bgColorButton_->setMaximumWidth(80);
  toolbar_->addWidget(bgColorButton_);

  toolbar_->addSeparator();

  // Ambient light slider
  QLabel* lightLabel = new QLabel(tr("Light:"));
  toolbar_->addWidget(lightLabel);

  ambientLightSlider_ = new QSlider(Qt::Horizontal);
  ambientLightSlider_->setRange(0, 100);
  ambientLightSlider_->setValue(30);
  ambientLightSlider_->setMaximumWidth(100);
  ambientLightSlider_->setToolTip(tr("Ambient light intensity"));
  toolbar_->addWidget(ambientLightSlider_);
}

void URDFViewerPanel::setupSidePanel() {
  sidePanel_ = new QWidget();
  QVBoxLayout* sideLayout = new QVBoxLayout(sidePanel_);
  sideLayout->setContentsMargins(5, 5, 5, 5);
  sideLayout->setSpacing(10);

  // Tree view
  QLabel* treeLabel = new QLabel(tr("Model Hierarchy"));
  treeLabel->setStyleSheet("font-weight: bold;");
  sideLayout->addWidget(treeLabel);

  treeView_ = new URDFTreeView(this);
  treeView_->setMinimumHeight(200);
  sideLayout->addWidget(treeView_, 1);

  // Rotation controls
  rotationGroup_ = new QGroupBox(tr("Joint Rotation"));
  QVBoxLayout* rotLayout = new QVBoxLayout(rotationGroup_);

  // Rotation mode selection
  QHBoxLayout* modeLayout = new QHBoxLayout();
  snap90Radio_ = new QRadioButton(tr("90° Snap"));
  snap90Radio_->setChecked(true);
  snap90Radio_->setToolTip(tr("Rotate in 90-degree increments"));
  modeLayout->addWidget(snap90Radio_);

  freeRotationRadio_ = new QRadioButton(tr("Free"));
  freeRotationRadio_->setToolTip(tr("Rotate by custom angle"));
  modeLayout->addWidget(freeRotationRadio_);

  freeAngleSpinBox_ = new QDoubleSpinBox();
  freeAngleSpinBox_->setRange(1, 180);
  freeAngleSpinBox_->setValue(45);
  freeAngleSpinBox_->setSuffix("°");
  freeAngleSpinBox_->setEnabled(false);
  freeAngleSpinBox_->setToolTip(tr("Rotation angle for free mode"));
  modeLayout->addWidget(freeAngleSpinBox_);

  modeLayout->addStretch();
  rotLayout->addLayout(modeLayout);

  // Rotation buttons
  QGridLayout* buttonGrid = new QGridLayout();

  rotateXPosButton_ = new QPushButton(tr("+X"));
  rotateXPosButton_->setToolTip(tr("Rotate +90° around X axis (X)"));
  rotateXPosButton_->setStyleSheet("background-color: #ff6666;");
  buttonGrid->addWidget(rotateXPosButton_, 0, 0);

  rotateXNegButton_ = new QPushButton(tr("-X"));
  rotateXNegButton_->setToolTip(tr("Rotate -90° around X axis (Shift+X)"));
  rotateXNegButton_->setStyleSheet("background-color: #ff9999;");
  buttonGrid->addWidget(rotateXNegButton_, 0, 1);

  rotateYPosButton_ = new QPushButton(tr("+Y"));
  rotateYPosButton_->setToolTip(tr("Rotate +90° around Y axis (Y)"));
  rotateYPosButton_->setStyleSheet("background-color: #66ff66;");
  buttonGrid->addWidget(rotateYPosButton_, 1, 0);

  rotateYNegButton_ = new QPushButton(tr("-Y"));
  rotateYNegButton_->setToolTip(tr("Rotate -90° around Y axis (Shift+Y)"));
  rotateYNegButton_->setStyleSheet("background-color: #99ff99;");
  buttonGrid->addWidget(rotateYNegButton_, 1, 1);

  rotateZPosButton_ = new QPushButton(tr("+Z"));
  rotateZPosButton_->setToolTip(tr("Rotate +90° around Z axis (Z)"));
  rotateZPosButton_->setStyleSheet("background-color: #6666ff;");
  buttonGrid->addWidget(rotateZPosButton_, 2, 0);

  rotateZNegButton_ = new QPushButton(tr("-Z"));
  rotateZNegButton_->setToolTip(tr("Rotate -90° around Z axis (Shift+Z)"));
  rotateZNegButton_->setStyleSheet("background-color: #9999ff;");
  buttonGrid->addWidget(rotateZNegButton_, 2, 1);

  rotLayout->addLayout(buttonGrid);

  // Reset button
  resetOrientationButton_ = new QPushButton(tr("Reset Orientation"));
  resetOrientationButton_->setToolTip(tr("Reset joint orientation to identity (R)"));
  rotLayout->addWidget(resetOrientationButton_);

  sideLayout->addWidget(rotationGroup_);

  // Help text
  QLabel* helpLabel = new QLabel(tr(
      "Camera Controls:\n"
      "• Middle mouse: Orbit\n"
      "• Shift + Middle: Pan\n"
      "• Scroll wheel: Zoom\n"
      "• Home: Reset camera"));
  helpLabel->setStyleSheet("color: gray; font-size: 11px;");
  helpLabel->setWordWrap(true);
  sideLayout->addWidget(helpLabel);

  sideLayout->addStretch();
}

void URDFViewerPanel::setupShortcuts() {
  // X axis rotation
  rotXPosShortcut_ = new QShortcut(QKeySequence("X"), this);
  connect(rotXPosShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateXPosClicked);

  rotXNegShortcut_ = new QShortcut(QKeySequence("Shift+X"), this);
  connect(rotXNegShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateXNegClicked);

  // Y axis rotation
  rotYPosShortcut_ = new QShortcut(QKeySequence("Y"), this);
  connect(rotYPosShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateYPosClicked);

  rotYNegShortcut_ = new QShortcut(QKeySequence("Shift+Y"), this);
  connect(rotYNegShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateYNegClicked);

  // Z axis rotation
  rotZPosShortcut_ = new QShortcut(QKeySequence("Z"), this);
  connect(rotZPosShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateZPosClicked);

  rotZNegShortcut_ = new QShortcut(QKeySequence("Shift+Z"), this);
  connect(rotZNegShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onRotateZNegClicked);

  // Reset
  resetShortcut_ = new QShortcut(QKeySequence("R"), this);
  connect(resetShortcut_, &QShortcut::activated, this, &URDFViewerPanel::onResetOrientationClicked);

  // Select all
  selectAllShortcut_ = new QShortcut(QKeySequence("Ctrl+A"), this);
  connect(selectAllShortcut_, &QShortcut::activated, this, [this]() {
    if (parser_->isLoaded()) {
      QStringList allJoints = parser_->getJointNames();
      jointController_->setSelectedJoints(allJoints);
      view3D_->selectJoints(allJoints);
      treeView_->selectJoints(allJoints);
    }
  });

  // Clear selection
  escapeShortcut_ = new QShortcut(QKeySequence("Escape"), this);
  connect(escapeShortcut_, &QShortcut::activated, this, [this]() {
    jointController_->clearSelection();
    view3D_->clearSelection();
    treeView_->clearJointSelection();
  });
}

void URDFViewerPanel::setupConnections() {
  // Toolbar actions
  connect(loadAction_, &QAction::triggered, this, &URDFViewerPanel::onLoadClicked);
  connect(saveAction_, &QAction::triggered, this, &URDFViewerPanel::onSaveClicked);
  connect(saveAsAction_, &QAction::triggered, this, &URDFViewerPanel::onSaveAsClicked);

  // Render mode
  connect(renderModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &URDFViewerPanel::onRenderModeChanged);

  // View options
  connect(shadowsCheckBox_, &QCheckBox::toggled, this, &URDFViewerPanel::onShadowsToggled);
  connect(gridCheckBox_, &QCheckBox::toggled, this, &URDFViewerPanel::onGridToggled);
  connect(bgColorButton_, &QPushButton::clicked, this, &URDFViewerPanel::onBgColorClicked);
  connect(ambientLightSlider_, &QSlider::valueChanged, this, &URDFViewerPanel::onAmbientLightChanged);

  // Rotation mode
  connect(snap90Radio_, &QRadioButton::toggled, this, &URDFViewerPanel::onRotationModeChanged);
  connect(freeRotationRadio_, &QRadioButton::toggled, this, &URDFViewerPanel::onRotationModeChanged);

  // Rotation buttons
  connect(rotateXPosButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateXPosClicked);
  connect(rotateXNegButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateXNegClicked);
  connect(rotateYPosButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateYPosClicked);
  connect(rotateYNegButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateYNegClicked);
  connect(rotateZPosButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateZPosClicked);
  connect(rotateZNegButton_, &QPushButton::clicked, this, &URDFViewerPanel::onRotateZNegClicked);
  connect(resetOrientationButton_, &QPushButton::clicked, this, &URDFViewerPanel::onResetOrientationClicked);

  // Parser signals
  connect(parser_, &URDFParser::modelLoaded, this, [this](const URDFModel& model) {
    view3D_->setModel(model);
    treeView_->setModel(model);
    saveAction_->setEnabled(true);
    saveAsAction_->setEnabled(true);
    statusLabel_->setText(tr("Loaded: %1 links, %2 joints")
                              .arg(model.links.size())
                              .arg(model.joints.size()));
  });

  connect(parser_, &URDFParser::jointModified, this, &URDFViewerPanel::onJointModified);
  connect(parser_, &URDFParser::parseError, this, [this](const QString& error) {
    QMessageBox::warning(this, tr("URDF Parse Error"), error);
  });

  // Joint controller signals
  connect(jointController_, &URDFJointController::jointRotated,
          this, &URDFViewerPanel::onJointRotated);
  connect(jointController_, &URDFJointController::selectionChanged,
          this, [this](const QStringList& selected) {
    updateRotationControlsState();
  });

  // 3D View selection
  connect(view3D_, &URDF3DView::selectionChanged,
          this, &URDFViewerPanel::on3DViewSelectionChanged);

  // Tree view selection
  connect(treeView_, &URDFTreeView::jointSelectionChanged,
          this, &URDFViewerPanel::onTreeViewSelectionChanged);
}

void URDFViewerPanel::loadURDF(const QString& filePath) {
  if (parser_->loadFromFile(filePath)) {
    currentFilePath_ = filePath;
    filePathLabel_->setText(filePath);
    filePathLabel_->setStyleSheet("color: black;");
    emit urdfLoaded(filePath);
  }
}

void URDFViewerPanel::saveURDF(const QString& filePath) {
  if (parser_->exportToFile(filePath)) {
    currentFilePath_ = filePath;
    filePathLabel_->setText(filePath);
    statusLabel_->setText(tr("Saved successfully"));
    emit urdfSaved(filePath);
  } else {
    QMessageBox::warning(this, tr("Save Error"),
                         tr("Failed to save URDF to: %1").arg(filePath));
  }
}

void URDFViewerPanel::saveURDF() {
  if (!currentFilePath_.isEmpty()) {
    saveURDF(currentFilePath_);
  } else {
    onSaveAsClicked();
  }
}

void URDFViewerPanel::onLoadClicked() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open URDF File"),
      QString(),
      tr("URDF Files (*.urdf *.xacro);;XML Files (*.xml);;All Files (*)"));

  if (!filePath.isEmpty()) {
    loadURDF(filePath);
  }
}

void URDFViewerPanel::onSaveClicked() {
  saveURDF();
}

void URDFViewerPanel::onSaveAsClicked() {
  QString filePath = QFileDialog::getSaveFileName(
      this,
      tr("Save URDF File"),
      currentFilePath_.isEmpty() ? "robot.urdf" : currentFilePath_,
      tr("URDF Files (*.urdf);;XML Files (*.xml);;All Files (*)"));

  if (!filePath.isEmpty()) {
    saveURDF(filePath);
  }
}

void URDFViewerPanel::onRenderModeChanged(int index) {
  RenderMode mode = static_cast<RenderMode>(renderModeCombo_->itemData(index).toInt());
  view3D_->setRenderMode(mode);
}

void URDFViewerPanel::onShadowsToggled(bool enabled) {
  view3D_->setShadowsEnabled(enabled);
}

void URDFViewerPanel::onBgColorClicked() {
  QColor color = QColorDialog::getColor(view3D_->backgroundColor(), this, tr("Select Background Color"));
  if (color.isValid()) {
    view3D_->setBackgroundColor(color);
  }
}

void URDFViewerPanel::onRotationModeChanged() {
  if (snap90Radio_->isChecked()) {
    jointController_->setRotationMode(RotationMode::Snap90Degrees);
    freeAngleSpinBox_->setEnabled(false);
  } else {
    jointController_->setRotationMode(RotationMode::FreeRotation);
    freeAngleSpinBox_->setEnabled(true);
  }
}

void URDFViewerPanel::onRotateXPosClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::X, true);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::X, freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onRotateXNegClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::X, false);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::X, -freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onRotateYPosClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::Y, true);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::Y, freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onRotateYNegClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::Y, false);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::Y, -freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onRotateZPosClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::Z, true);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::Z, freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onRotateZNegClicked() {
  if (snap90Radio_->isChecked()) {
    jointController_->snapRotateSelectedJoints(RotationAxis::Z, false);
  } else {
    jointController_->freeRotateSelectedJoints(RotationAxis::Z, -freeAngleSpinBox_->value());
  }
}

void URDFViewerPanel::onResetOrientationClicked() {
  jointController_->resetSelectedJoints();
}

void URDFViewerPanel::onAmbientLightChanged(int value) {
  float intensity = value / 100.0f;
  view3D_->setAmbientLightIntensity(intensity);
}

void URDFViewerPanel::onGridToggled(bool visible) {
  view3D_->setGridVisible(visible);
}

void URDFViewerPanel::onJointModified(const QString& jointName) {
  // Update tree view
  URDFJoint* joint = parser_->getJoint(jointName);
  if (joint) {
    treeView_->updateJoint(jointName, *joint);
  }

  emit urdfModified();
}

void URDFViewerPanel::on3DViewSelectionChanged(const QStringList& selectedJoints) {
  if (blockSelectionSync_) return;

  blockSelectionSync_ = true;
  jointController_->setSelectedJoints(selectedJoints);
  treeView_->selectJoints(selectedJoints);
  blockSelectionSync_ = false;

  updateRotationControlsState();
}

void URDFViewerPanel::onTreeViewSelectionChanged(const QStringList& selectedJoints) {
  if (blockSelectionSync_) return;

  blockSelectionSync_ = true;
  jointController_->setSelectedJoints(selectedJoints);
  view3D_->selectJoints(selectedJoints);
  blockSelectionSync_ = false;

  updateRotationControlsState();
}

void URDFViewerPanel::onJointRotated(const QString& jointName, const QQuaternion& newOrientation) {
  view3D_->updateJointTransform(jointName, newOrientation);
  view3D_->updateAllTransforms();
}

void URDFViewerPanel::updateRotationControlsState() {
  bool hasSelection = !jointController_->selectedJoints().isEmpty();
  rotationGroup_->setEnabled(hasSelection);

  if (hasSelection) {
    int count = jointController_->selectedJoints().size();
    statusLabel_->setText(tr("%1 joint(s) selected").arg(count));
  } else if (parser_->isLoaded()) {
    statusLabel_->setText(tr("Select joints to edit"));
  }
}

void URDFViewerPanel::syncSelectionFrom3DView() {
  if (blockSelectionSync_) return;
  blockSelectionSync_ = true;
  treeView_->selectJoints(view3D_->selectedJoints());
  blockSelectionSync_ = false;
}

void URDFViewerPanel::syncSelectionFromTreeView() {
  if (blockSelectionSync_) return;
  blockSelectionSync_ = true;
  view3D_->selectJoints(treeView_->selectedJointNames());
  blockSelectionSync_ = false;
}

}  // namespace ros_weaver
