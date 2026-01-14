#include "ros_weaver/widgets/mission_planner_panel.hpp"
#include "ros_weaver/core/undo/mission_undo_commands.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QMenu>
#include <QShortcut>
#include <QKeySequence>
#include <cmath>

namespace ros_weaver {

MissionPlannerPanel::MissionPlannerPanel(QWidget* parent)
    : QWidget(parent) {
  undoStack_ = new MissionUndoStack(this);

  setupUi();
  setupConnections();

  // Setup keyboard shortcuts for undo/redo
  auto* undoShortcut = new QShortcut(QKeySequence::Undo, this);
  connect(undoShortcut, &QShortcut::activated, this, &MissionPlannerPanel::undo);

  auto* redoShortcut = new QShortcut(QKeySequence::Redo, this);
  connect(redoShortcut, &QShortcut::activated, this, &MissionPlannerPanel::redo);

  // Also support Ctrl+Shift+Z for redo (common alternative)
  auto* redoShortcut2 = new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_Z), this);
  connect(redoShortcut2, &QShortcut::activated, this, &MissionPlannerPanel::redo);

  // Initialize with default mission
  currentMission_.name = tr("Untitled Mission");
}

MissionPlannerPanel::~MissionPlannerPanel() = default;

void MissionPlannerPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  setupToolbar();
  mainLayout->addWidget(toolbar_);

  // Main splitter
  mainSplitter_ = new QSplitter(Qt::Horizontal);

  // Left side: Map view
  auto* mapContainer = new QWidget();
  auto* mapLayout = new QVBoxLayout(mapContainer);
  mapLayout->setContentsMargins(4, 4, 4, 4);

  setupMapView();
  mapLayout->addWidget(mapView_, 1);

  // Status bar below map
  auto* mapStatusLayout = new QHBoxLayout();
  coordinateLabel_ = new QLabel(tr("Position: --"));
  scaleLabel_ = new QLabel(tr("Scale: --"));
  modeLabel_ = new QLabel(tr("Mode: Normal"));
  modeLabel_->setStyleSheet("font-weight: bold; color: #2980b9;");

  mapStatusLayout->addWidget(coordinateLabel_);
  mapStatusLayout->addStretch();
  mapStatusLayout->addWidget(scaleLabel_);
  mapStatusLayout->addWidget(modeLabel_);
  mapLayout->addLayout(mapStatusLayout);

  mainSplitter_->addWidget(mapContainer);

  // Right side: Properties
  setupPropertiesPanel();
  mainSplitter_->addWidget(propertiesTabs_);

  // Set splitter sizes (70% map, 30% properties)
  mainSplitter_->setSizes({700, 300});

  mainLayout->addWidget(mainSplitter_, 1);
}

void MissionPlannerPanel::setupToolbar() {
  toolbar_ = new QToolBar();
  toolbar_->setMovable(false);
  toolbar_->setIconSize(QSize(20, 20));

  loadMapAction_ = toolbar_->addAction(tr("Load Map"));
  loadMapAction_->setToolTip(tr("Load a map image file"));

  loadNav2MapAction_ = toolbar_->addAction(tr("Load Nav2 Map"));
  loadNav2MapAction_->setToolTip(tr("Load a Nav2 map.yaml file"));

  toolbar_->addSeparator();

  calibrateScaleAction_ = toolbar_->addAction(tr("Calibrate Scale"));
  calibrateScaleAction_->setToolTip(tr("Calibrate map scale by measuring a known distance"));
  calibrateScaleAction_->setCheckable(true);

  toolbar_->addSeparator();

  addWaypointAction_ = toolbar_->addAction(tr("Add Waypoint"));
  addWaypointAction_->setToolTip(tr("Click on map to add waypoints"));
  addWaypointAction_->setCheckable(true);

  setStartPoseAction_ = toolbar_->addAction(tr("Set Start"));
  setStartPoseAction_->setToolTip(tr("Set robot start pose"));
  setStartPoseAction_->setCheckable(true);

  clearWaypointsAction_ = toolbar_->addAction(tr("Clear"));
  clearWaypointsAction_->setToolTip(tr("Clear all waypoints"));

  toolbar_->addSeparator();

  zoomFitAction_ = toolbar_->addAction(tr("Fit"));
  zoomFitAction_->setToolTip(tr("Fit map to view"));

  toolbar_->addSeparator();

  saveMissionAction_ = toolbar_->addAction(tr("Save"));
  saveMissionAction_->setToolTip(tr("Save mission"));

  loadMissionAction_ = toolbar_->addAction(tr("Open"));
  loadMissionAction_->setToolTip(tr("Open mission file"));

  exportNav2Action_ = toolbar_->addAction(tr("Export"));
  exportNav2Action_->setToolTip(tr("Export to Nav2 format"));
}

void MissionPlannerPanel::setupMapView() {
  mapView_ = new MissionMapView();
}

void MissionPlannerPanel::setupPropertiesPanel() {
  propertiesTabs_ = new QTabWidget();

  setupMissionTab();
  setupWaypointsTab();
  setupStartPoseTab();
}

void MissionPlannerPanel::setupMissionTab() {
  auto* missionWidget = new QWidget();
  auto* layout = new QVBoxLayout(missionWidget);

  auto* formLayout = new QFormLayout();

  missionNameEdit_ = new QLineEdit();
  missionNameEdit_->setPlaceholderText(tr("Mission name"));
  connect(missionNameEdit_, &QLineEdit::textChanged, this, &MissionPlannerPanel::onMissionSettingsChanged);
  formLayout->addRow(tr("Name:"), missionNameEdit_);

  missionDescriptionEdit_ = new QTextEdit();
  missionDescriptionEdit_->setMaximumHeight(60);
  missionDescriptionEdit_->setPlaceholderText(tr("Mission description"));
  connect(missionDescriptionEdit_, &QTextEdit::textChanged, this, &MissionPlannerPanel::onMissionSettingsChanged);
  formLayout->addRow(tr("Description:"), missionDescriptionEdit_);

  layout->addLayout(formLayout);

  // Loop settings
  auto* loopGroup = new QGroupBox(tr("Loop Settings"));
  auto* loopLayout = new QFormLayout(loopGroup);

  loopMissionCheckbox_ = new QCheckBox(tr("Loop mission"));
  connect(loopMissionCheckbox_, &QCheckBox::toggled, this, &MissionPlannerPanel::onMissionSettingsChanged);
  connect(loopMissionCheckbox_, &QCheckBox::toggled, this, [this](bool checked) {
    mapView_->setLoopMission(checked);
  });
  loopLayout->addRow(loopMissionCheckbox_);

  loopCountSpinBox_ = new QSpinBox();
  loopCountSpinBox_->setRange(-1, 1000);
  loopCountSpinBox_->setSpecialValueText(tr("Infinite"));
  loopCountSpinBox_->setValue(1);
  connect(loopCountSpinBox_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &MissionPlannerPanel::onMissionSettingsChanged);
  loopLayout->addRow(tr("Loop count:"), loopCountSpinBox_);

  layout->addWidget(loopGroup);

  // Default settings
  auto* defaultsGroup = new QGroupBox(tr("Default Settings"));
  auto* defaultsLayout = new QFormLayout(defaultsGroup);

  defaultSpeedSpinBox_ = new QDoubleSpinBox();
  defaultSpeedSpinBox_->setRange(0.1, 5.0);
  defaultSpeedSpinBox_->setSingleStep(0.1);
  defaultSpeedSpinBox_->setSuffix(" m/s");
  defaultSpeedSpinBox_->setValue(0.5);
  connect(defaultSpeedSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MissionPlannerPanel::onMissionSettingsChanged);
  defaultsLayout->addRow(tr("Speed:"), defaultSpeedSpinBox_);

  defaultToleranceSpinBox_ = new QDoubleSpinBox();
  defaultToleranceSpinBox_->setRange(0.01, 5.0);
  defaultToleranceSpinBox_->setSingleStep(0.05);
  defaultToleranceSpinBox_->setSuffix(" m");
  defaultToleranceSpinBox_->setValue(0.3);
  connect(defaultToleranceSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &MissionPlannerPanel::onMissionSettingsChanged);
  defaultsLayout->addRow(tr("Tolerance:"), defaultToleranceSpinBox_);

  layout->addWidget(defaultsGroup);
  layout->addStretch();

  propertiesTabs_->addTab(missionWidget, tr("Mission"));
}

void MissionPlannerPanel::setupWaypointsTab() {
  auto* waypointsWidget = new QWidget();
  auto* layout = new QVBoxLayout(waypointsWidget);

  // Waypoints list
  auto* listGroup = new QGroupBox(tr("Waypoints"));
  auto* listLayout = new QVBoxLayout(listGroup);

  waypointsList_ = new QListWidget();
  waypointsList_->setMaximumHeight(120);
  waypointsList_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  connect(waypointsList_, &QListWidget::currentRowChanged,
          this, &MissionPlannerPanel::onWaypointListItemSelected);
  listLayout->addWidget(waypointsList_);

  auto* listButtonsLayout = new QHBoxLayout();
  moveUpButton_ = new QPushButton(tr("Up"));
  moveDownButton_ = new QPushButton(tr("Down"));
  connect(moveUpButton_, &QPushButton::clicked, this, &MissionPlannerPanel::onMoveWaypointUp);
  connect(moveDownButton_, &QPushButton::clicked, this, &MissionPlannerPanel::onMoveWaypointDown);
  listButtonsLayout->addWidget(moveUpButton_);
  listButtonsLayout->addWidget(moveDownButton_);
  listButtonsLayout->addStretch();
  listLayout->addLayout(listButtonsLayout);

  layout->addWidget(listGroup);

  // Waypoint editor
  waypointEditor_ = new WaypointEditorWidget();
  connect(waypointEditor_, &WaypointEditorWidget::waypointChanged,
          this, &MissionPlannerPanel::onWaypointEditorChanged);
  connect(waypointEditor_, &WaypointEditorWidget::deleteRequested,
          this, &MissionPlannerPanel::onWaypointDeleteRequested);
  layout->addWidget(waypointEditor_, 1);

  propertiesTabs_->addTab(waypointsWidget, tr("Waypoints"));
}

void MissionPlannerPanel::setupStartPoseTab() {
  auto* startPoseWidget = new QWidget();
  auto* layout = new QVBoxLayout(startPoseWidget);

  auto* formLayout = new QFormLayout();

  // Position
  auto* posLayout = new QHBoxLayout();
  startXSpinBox_ = new QDoubleSpinBox();
  startXSpinBox_->setRange(-10000, 10000);
  startXSpinBox_->setDecimals(3);
  startXSpinBox_->setSuffix(" m");
  connect(startXSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this]() {
    currentMission_.startPose.x = startXSpinBox_->value();
    mapView_->setStartPose(currentMission_.startPose);
    markMissionModified();
  });

  startYSpinBox_ = new QDoubleSpinBox();
  startYSpinBox_->setRange(-10000, 10000);
  startYSpinBox_->setDecimals(3);
  startYSpinBox_->setSuffix(" m");
  connect(startYSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this]() {
    currentMission_.startPose.y = startYSpinBox_->value();
    mapView_->setStartPose(currentMission_.startPose);
    markMissionModified();
  });

  posLayout->addWidget(new QLabel("X:"));
  posLayout->addWidget(startXSpinBox_);
  posLayout->addWidget(new QLabel("Y:"));
  posLayout->addWidget(startYSpinBox_);
  formLayout->addRow(tr("Position:"), posLayout);

  // Orientation
  auto* orientLayout = new QHBoxLayout();
  startThetaSpinBox_ = new QDoubleSpinBox();
  startThetaSpinBox_->setRange(-180, 180);
  startThetaSpinBox_->setDecimals(1);
  startThetaSpinBox_->setSuffix(QString::fromUtf8("\u00B0"));
  connect(startThetaSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double degrees) {
    currentMission_.startPose.setThetaDegrees(degrees);
    startOrientationDial_->blockSignals(true);
    startOrientationDial_->setValue(static_cast<int>(degrees));
    startOrientationDial_->blockSignals(false);
    mapView_->setStartPose(currentMission_.startPose);
    markMissionModified();
  });

  startOrientationDial_ = new QDial();
  startOrientationDial_->setRange(-180, 180);
  startOrientationDial_->setNotchesVisible(true);
  startOrientationDial_->setWrapping(true);
  startOrientationDial_->setFixedSize(80, 80);
  connect(startOrientationDial_, &QDial::valueChanged, this, [this](int value) {
    startThetaSpinBox_->blockSignals(true);
    startThetaSpinBox_->setValue(static_cast<double>(value));
    startThetaSpinBox_->blockSignals(false);
    currentMission_.startPose.setThetaDegrees(static_cast<double>(value));
    mapView_->setStartPose(currentMission_.startPose);
    markMissionModified();
  });

  orientLayout->addWidget(startThetaSpinBox_);
  orientLayout->addWidget(startOrientationDial_);
  orientLayout->addStretch();
  formLayout->addRow(tr("Orientation:"), orientLayout);

  layout->addLayout(formLayout);
  layout->addStretch();

  propertiesTabs_->addTab(startPoseWidget, tr("Start Pose"));
}

void MissionPlannerPanel::setupConnections() {
  // Toolbar actions
  connect(loadMapAction_, &QAction::triggered, this, &MissionPlannerPanel::onLoadMap);
  connect(loadNav2MapAction_, &QAction::triggered, this, &MissionPlannerPanel::onLoadNav2Map);
  connect(calibrateScaleAction_, &QAction::triggered, this, &MissionPlannerPanel::onCalibrateScale);
  connect(addWaypointAction_, &QAction::triggered, this, &MissionPlannerPanel::onAddWaypoint);
  connect(setStartPoseAction_, &QAction::triggered, this, &MissionPlannerPanel::onSetStartPose);
  connect(clearWaypointsAction_, &QAction::triggered, this, &MissionPlannerPanel::onClearWaypoints);
  connect(zoomFitAction_, &QAction::triggered, this, &MissionPlannerPanel::onZoomFit);
  connect(saveMissionAction_, &QAction::triggered, this, &MissionPlannerPanel::saveMission);
  connect(loadMissionAction_, &QAction::triggered, this, &MissionPlannerPanel::loadMission);
  connect(exportNav2Action_, &QAction::triggered, this, &MissionPlannerPanel::onExportNav2);

  // Map view signals
  connect(mapView_, &MissionMapView::waypointAdded, this, &MissionPlannerPanel::onWaypointAdded);
  connect(mapView_, &MissionMapView::waypointSelected, this, &MissionPlannerPanel::onWaypointSelected);
  connect(mapView_, &MissionMapView::selectionChanged, this, &MissionPlannerPanel::onSelectionChanged);
  connect(mapView_, &MissionMapView::waypointMoved, this, &MissionPlannerPanel::onWaypointMoved);
  connect(mapView_, &MissionMapView::waypointOrientationChanged, this, &MissionPlannerPanel::onWaypointOrientationChanged);
  connect(mapView_, &MissionMapView::waypointDoubleClicked, this, &MissionPlannerPanel::onWaypointDoubleClicked);
  connect(mapView_, &MissionMapView::startPoseChanged, this, &MissionPlannerPanel::onStartPoseChanged);
  connect(mapView_, &MissionMapView::scaleCalibrated, this, &MissionPlannerPanel::onScaleCalibrated);
  connect(mapView_, &MissionMapView::coordinateHovered, this, &MissionPlannerPanel::onCoordinateHovered);
  connect(mapView_, &MissionMapView::modeChanged, this, &MissionPlannerPanel::onModeChanged);
}

void MissionPlannerPanel::setMission(const Mission& mission) {
  currentMission_ = mission;
  updateUiFromMission();
  missionModified_ = false;
}

Mission MissionPlannerPanel::getMission() const {
  return currentMission_;
}

void MissionPlannerPanel::setBehaviorTreeEditor(BehaviorTreePanel* editor) {
  behaviorTreeEditor_ = editor;
}

void MissionPlannerPanel::newMission() {
  if (missionModified_) {
    auto result = QMessageBox::question(this, tr("New Mission"),
        tr("Current mission has unsaved changes. Discard them?"),
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if (result != QMessageBox::Yes) {
      return;
    }
  }

  currentMission_ = Mission();
  currentMission_.name = tr("Untitled Mission");
  currentMissionPath_.clear();
  mapView_->clearMap();
  updateUiFromMission();
  missionModified_ = false;
}

void MissionPlannerPanel::loadMission() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open Mission"),
      QString(),
      tr("Mission Files (*.mission.json);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (filePath.isEmpty()) return;

  Mission mission = Mission::load(filePath);
  if (mission.name.isEmpty() && mission.waypoints.isEmpty()) {
    QMessageBox::warning(this, tr("Load Error"),
        tr("Failed to load mission file."));
    return;
  }

  currentMission_ = mission;
  currentMissionPath_ = filePath;

  // Load map if path is set
  if (!mission.mapImagePath.isEmpty()) {
    mapView_->loadMapImage(mission.mapImagePath);
  }

  mapView_->setScale(mission.scale);
  mapView_->setWaypoints(mission.waypoints);
  mapView_->setStartPose(mission.startPose);

  updateUiFromMission();
  missionModified_ = false;

  emit missionLoaded(filePath);
}

void MissionPlannerPanel::saveMission() {
  if (currentMissionPath_.isEmpty()) {
    saveMissionAs();
    return;
  }

  updateMissionFromUi();
  currentMission_.waypoints = mapView_->getWaypoints();
  currentMission_.startPose = mapView_->getStartPose();
  currentMission_.scale = mapView_->getScale();

  if (currentMission_.save(currentMissionPath_)) {
    missionModified_ = false;
    emit missionSaved(currentMissionPath_);
  } else {
    QMessageBox::warning(this, tr("Save Error"),
        tr("Failed to save mission file."));
  }
}

void MissionPlannerPanel::saveMissionAs() {
  QString filePath = QFileDialog::getSaveFileName(
      this,
      tr("Save Mission As"),
      currentMission_.name + ".mission.json",
      tr("Mission Files (*.mission.json);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (filePath.isEmpty()) return;

  currentMissionPath_ = filePath;
  saveMission();
}

void MissionPlannerPanel::onLoadMap() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open Map Image"),
      QString(),
      tr("Images (*.png *.jpg *.pgm *.bmp);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (!filePath.isEmpty()) {
    mapView_->loadMapImage(filePath);
    currentMission_.mapImagePath = filePath;
    markMissionModified();
  }
}

void MissionPlannerPanel::onLoadNav2Map() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open Nav2 Map"),
      QString(),
      tr("Nav2 Map (*.yaml);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (!filePath.isEmpty()) {
    mapView_->loadNav2Map(filePath);
    currentMission_.mapYamlPath = filePath;
    currentMission_.scale = mapView_->getScale();
    scaleLabel_->setText(tr("Scale: %1 m/px").arg(currentMission_.scale.metersPerPixel, 0, 'f', 4));
    markMissionModified();
  }
}

void MissionPlannerPanel::onCalibrateScale() {
  if (calibrateScaleAction_->isChecked()) {
    mapView_->enterScaleCalibrationMode();
    addWaypointAction_->setChecked(false);
    setStartPoseAction_->setChecked(false);
  } else {
    mapView_->exitScaleCalibrationMode();
  }
}

void MissionPlannerPanel::onScaleDistanceEntered() {
  // Called after user draws the calibration line
  bool ok;
  double distance = QInputDialog::getDouble(
      this,
      tr("Enter Distance"),
      tr("Enter the real-world distance in meters:"),
      1.0, 0.01, 10000.0, 2, &ok);

  if (ok) {
    MapScale scale = mapView_->getScale();
    scale.realWorldDistance = distance;
    scale.computeScale();
    mapView_->setScale(scale);
    currentMission_.scale = scale;

    scaleLabel_->setText(tr("Scale: %1 m/px").arg(scale.metersPerPixel, 0, 'f', 4));
    markMissionModified();
  }

  mapView_->exitScaleCalibrationMode();
  calibrateScaleAction_->setChecked(false);
}

void MissionPlannerPanel::onAddWaypoint() {
  if (addWaypointAction_->isChecked()) {
    mapView_->setMode(MissionMapView::AddWaypoint);
    calibrateScaleAction_->setChecked(false);
    setStartPoseAction_->setChecked(false);
  } else {
    mapView_->setMode(MissionMapView::Normal);
  }
}

void MissionPlannerPanel::onSetStartPose() {
  if (setStartPoseAction_->isChecked()) {
    mapView_->enterStartPoseMode();
    calibrateScaleAction_->setChecked(false);
    addWaypointAction_->setChecked(false);
  } else {
    mapView_->setMode(MissionMapView::Normal);
  }
}

void MissionPlannerPanel::onClearWaypoints() {
  if (currentMission_.waypoints.isEmpty()) {
    return;  // Nothing to clear
  }

  auto result = QMessageBox::question(this, tr("Clear Waypoints"),
      tr("Are you sure you want to clear all waypoints?"),
      QMessageBox::Yes | QMessageBox::No);

  if (result == QMessageBox::Yes) {
    // Save current waypoints for undo
    QList<Waypoint> savedWaypoints = currentMission_.waypoints;

    // Create undo command before clearing
    auto* cmd = new ClearWaypointsCommand(this, savedWaypoints);
    undoStack_->push(cmd);

    mapView_->clearWaypoints();
    currentMission_.waypoints.clear();
    updateWaypointsList();
    waypointEditor_->clear();
    markMissionModified();
  }
}

void MissionPlannerPanel::onZoomFit() {
  mapView_->zoomToFit();
}

void MissionPlannerPanel::onExportNav2() {
  // Show format selection menu
  QMenu menu;
  menu.addAction(tr("Waypoint Follower YAML"), this, [this]() {
    QString path = QFileDialog::getSaveFileName(this, tr("Export Waypoint Follower"),
        currentMission_.name + "_waypoints.yaml", tr("YAML (*.yaml)"),
        nullptr, QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty()) {
      updateMissionFromUi();
      currentMission_.waypoints = mapView_->getWaypoints();
      Nav2Exporter::saveToFile(currentMission_, Nav2Exporter::Nav2WaypointFollower, path);
    }
  });

  menu.addAction(tr("Behavior Tree XML"), this, [this]() {
    QString path = QFileDialog::getSaveFileName(this, tr("Export Behavior Tree"),
        currentMission_.name + "_nav.xml", tr("XML (*.xml)"),
        nullptr, QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty()) {
      updateMissionFromUi();
      currentMission_.waypoints = mapView_->getWaypoints();
      Nav2Exporter::saveToFile(currentMission_, Nav2Exporter::Nav2BtNavigator, path);
    }
  });

  menu.addAction(tr("Python Script"), this, [this]() {
    QString path = QFileDialog::getSaveFileName(this, tr("Export Python Script"),
        currentMission_.name + "_mission.py", tr("Python (*.py)"),
        nullptr, QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty()) {
      updateMissionFromUi();
      currentMission_.waypoints = mapView_->getWaypoints();
      Nav2Exporter::saveToFile(currentMission_, Nav2Exporter::PythonScript, path);
    }
  });

  menu.addAction(tr("Custom YAML"), this, [this]() {
    QString path = QFileDialog::getSaveFileName(this, tr("Export Custom YAML"),
        currentMission_.name + "_full.yaml", tr("YAML (*.yaml)"),
        nullptr, QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty()) {
      updateMissionFromUi();
      currentMission_.waypoints = mapView_->getWaypoints();
      Nav2Exporter::saveToFile(currentMission_, Nav2Exporter::CustomYAML, path);
    }
  });

  menu.exec(QCursor::pos());
}

void MissionPlannerPanel::onWaypointAdded(const Waypoint& waypoint) {
  // Add to mission data (map view already created the graphics item)
  currentMission_.waypoints.append(waypoint);

  // Create undo command to track this action
  auto* cmd = new AddWaypointCommand(this, waypoint);
  undoStack_->push(cmd);

  updateWaypointsList();
  markMissionModified();
}

void MissionPlannerPanel::onWaypointSelected(int waypointId) {
  // Find and select in list
  for (int i = 0; i < currentMission_.waypoints.size(); ++i) {
    if (currentMission_.waypoints[i].id == waypointId) {
      // Block signals to prevent feedback loop:
      // setCurrentRow would trigger onWaypointListItemSelected which would
      // call mapView_->selectWaypoint() and clear multi-selection
      waypointsList_->blockSignals(true);
      waypointsList_->setCurrentRow(i);
      waypointsList_->blockSignals(false);

      waypointEditor_->setWaypoint(currentMission_.waypoints[i]);
      propertiesTabs_->setCurrentIndex(1);  // Switch to waypoints tab

      // Store initial position and orientation for potential drag operations
      draggingWaypointId_ = waypointId;
      waypointDragStartPos_ = QPointF(currentMission_.waypoints[i].x,
                                       currentMission_.waypoints[i].y);
      waypointOrientationStart_ = currentMission_.waypoints[i].theta;
      break;
    }
  }
}

void MissionPlannerPanel::onWaypointMoved(int waypointId, const QPointF& newPositionMeters) {
  Waypoint* wp = currentMission_.findWaypoint(waypointId);
  if (wp) {
    QPointF oldPos(wp->x, wp->y);

    // Update mission data
    wp->x = newPositionMeters.x();
    wp->y = newPositionMeters.y();

    // Create undo command (consecutive moves will be merged)
    auto* cmd = new MoveWaypointCommand(this, waypointId, oldPos, newPositionMeters);
    undoStack_->push(cmd);

    // Update editor if this waypoint is selected
    int row = waypointsList_->currentRow();
    if (row >= 0 && row < currentMission_.waypoints.size() &&
        currentMission_.waypoints[row].id == waypointId) {
      waypointEditor_->setWaypoint(*wp);
    }

    markMissionModified();
  }
}

void MissionPlannerPanel::onWaypointOrientationChanged(int waypointId, double theta) {
  Waypoint* wp = currentMission_.findWaypoint(waypointId);
  if (wp) {
    double oldTheta = wp->theta;
    wp->theta = theta;

    // Create undo command (consecutive orientation changes will be merged)
    auto* cmd = new ChangeWaypointOrientationCommand(this, waypointId, oldTheta, theta);
    undoStack_->push(cmd);

    int row = waypointsList_->currentRow();
    if (row >= 0 && row < currentMission_.waypoints.size() &&
        currentMission_.waypoints[row].id == waypointId) {
      waypointEditor_->setWaypoint(*wp);
    }

    markMissionModified();
  }
}

void MissionPlannerPanel::onWaypointDoubleClicked(int waypointId) {
  onWaypointSelected(waypointId);
}

void MissionPlannerPanel::onStartPoseChanged(const RobotStartPose& pose) {
  RobotStartPose oldPose = currentMission_.startPose;
  currentMission_.startPose = pose;

  // Create undo command (consecutive pose changes will be merged)
  auto* cmd = new SetStartPoseCommand(this, oldPose, pose);
  undoStack_->push(cmd);

  startXSpinBox_->blockSignals(true);
  startYSpinBox_->blockSignals(true);
  startThetaSpinBox_->blockSignals(true);
  startOrientationDial_->blockSignals(true);

  startXSpinBox_->setValue(pose.x);
  startYSpinBox_->setValue(pose.y);
  startThetaSpinBox_->setValue(pose.thetaDegrees());
  startOrientationDial_->setValue(static_cast<int>(pose.thetaDegrees()));

  startXSpinBox_->blockSignals(false);
  startYSpinBox_->blockSignals(false);
  startThetaSpinBox_->blockSignals(false);
  startOrientationDial_->blockSignals(false);

  setStartPoseAction_->setChecked(false);
  markMissionModified();
}

void MissionPlannerPanel::onScaleCalibrated(const MapScale&) {
  // Prompt for distance
  onScaleDistanceEntered();
}

void MissionPlannerPanel::onCoordinateHovered(const QPointF& meters) {
  coordinateLabel_->setText(tr("X: %1m  Y: %2m")
      .arg(meters.x(), 0, 'f', 2)
      .arg(meters.y(), 0, 'f', 2));
}

void MissionPlannerPanel::onModeChanged(MissionMapView::Mode mode) {
  QString modeStr;
  switch (mode) {
    case MissionMapView::Normal:
      modeStr = tr("Normal");
      addWaypointAction_->setChecked(false);
      setStartPoseAction_->setChecked(false);
      calibrateScaleAction_->setChecked(false);
      break;
    case MissionMapView::AddWaypoint:
      modeStr = tr("Add Waypoint");
      break;
    case MissionMapView::ScaleCalibration:
      modeStr = tr("Calibrate Scale");
      break;
    case MissionMapView::SetStartPose:
      modeStr = tr("Set Start Pose");
      break;
    case MissionMapView::SetOrientation:
      modeStr = tr("Set Orientation");
      break;
  }
  modeLabel_->setText(tr("Mode: %1").arg(modeStr));
}

void MissionPlannerPanel::onWaypointListItemSelected() {
  int row = waypointsList_->currentRow();
  if (row >= 0 && row < currentMission_.waypoints.size()) {
    waypointEditor_->setWaypoint(currentMission_.waypoints[row]);
    mapView_->selectWaypoint(currentMission_.waypoints[row].id);
  }
}

void MissionPlannerPanel::onSelectionChanged(const QList<int>& waypointIds) {
  // Update the list widget to reflect the map selection
  // Block signals to prevent feedback loop
  waypointsList_->blockSignals(true);
  waypointsList_->clearSelection();

  for (int id : waypointIds) {
    // Find the row for this waypoint ID
    for (int i = 0; i < currentMission_.waypoints.size(); ++i) {
      if (currentMission_.waypoints[i].id == id) {
        if (QListWidgetItem* item = waypointsList_->item(i)) {
          item->setSelected(true);
        }
        break;
      }
    }
  }

  waypointsList_->blockSignals(false);
}

void MissionPlannerPanel::onMoveWaypointUp() {
  int row = waypointsList_->currentRow();
  if (row > 0) {
    // Create undo command
    auto* cmd = new ReorderWaypointCommand(this, row, row - 1);
    undoStack_->push(cmd);

    currentMission_.reorderWaypoints(row, row - 1);
    updateWaypointsList();
    waypointsList_->setCurrentRow(row - 1);
    mapView_->setWaypoints(currentMission_.waypoints);
    markMissionModified();
  }
}

void MissionPlannerPanel::onMoveWaypointDown() {
  int row = waypointsList_->currentRow();
  if (row >= 0 && row < currentMission_.waypoints.size() - 1) {
    // Create undo command
    auto* cmd = new ReorderWaypointCommand(this, row, row + 1);
    undoStack_->push(cmd);

    currentMission_.reorderWaypoints(row, row + 1);
    updateWaypointsList();
    waypointsList_->setCurrentRow(row + 1);
    mapView_->setWaypoints(currentMission_.waypoints);
    markMissionModified();
  }
}

void MissionPlannerPanel::onWaypointEditorChanged(const Waypoint& waypoint) {
  int row = waypointsList_->currentRow();
  if (row >= 0 && row < currentMission_.waypoints.size()) {
    currentMission_.waypoints[row] = waypoint;
    mapView_->updateWaypoint(waypoint);
    updateWaypointsList();
    waypointsList_->setCurrentRow(row);
    markMissionModified();
  }
}

void MissionPlannerPanel::onWaypointDeleteRequested(int waypointId) {
  for (int i = 0; i < currentMission_.waypoints.size(); ++i) {
    if (currentMission_.waypoints[i].id == waypointId) {
      // Save waypoint and index for undo
      Waypoint savedWaypoint = currentMission_.waypoints[i];

      // Create undo command before removing
      auto* cmd = new RemoveWaypointCommand(this, savedWaypoint, i);
      undoStack_->push(cmd);

      currentMission_.waypoints.removeAt(i);
      mapView_->removeWaypoint(waypointId);
      updateWaypointsList();
      waypointEditor_->clear();
      markMissionModified();
      break;
    }
  }
}

void MissionPlannerPanel::onMissionSettingsChanged() {
  updateMissionFromUi();
  markMissionModified();
}

void MissionPlannerPanel::updateWaypointsList() {
  waypointsList_->clear();
  for (int i = 0; i < currentMission_.waypoints.size(); ++i) {
    const auto& wp = currentMission_.waypoints[i];
    QString label = QString("%1. %2 (%3, %4)")
        .arg(i + 1)
        .arg(wp.name)
        .arg(wp.x, 0, 'f', 2)
        .arg(wp.y, 0, 'f', 2);
    waypointsList_->addItem(label);
  }
}

void MissionPlannerPanel::updateMissionFromUi() {
  currentMission_.name = missionNameEdit_->text();
  currentMission_.description = missionDescriptionEdit_->toPlainText();
  currentMission_.loopMission = loopMissionCheckbox_->isChecked();
  currentMission_.loopCount = loopCountSpinBox_->value();
  currentMission_.defaultSpeed = defaultSpeedSpinBox_->value();
  currentMission_.defaultTolerance = defaultToleranceSpinBox_->value();
}

void MissionPlannerPanel::updateUiFromMission() {
  missionNameEdit_->setText(currentMission_.name);
  missionDescriptionEdit_->setText(currentMission_.description);
  loopMissionCheckbox_->setChecked(currentMission_.loopMission);
  loopCountSpinBox_->setValue(currentMission_.loopCount);
  defaultSpeedSpinBox_->setValue(currentMission_.defaultSpeed);
  defaultToleranceSpinBox_->setValue(currentMission_.defaultTolerance);

  startXSpinBox_->setValue(currentMission_.startPose.x);
  startYSpinBox_->setValue(currentMission_.startPose.y);
  startThetaSpinBox_->setValue(currentMission_.startPose.thetaDegrees());
  startOrientationDial_->setValue(static_cast<int>(currentMission_.startPose.thetaDegrees()));

  if (currentMission_.scale.isValid()) {
    scaleLabel_->setText(tr("Scale: %1 m/px").arg(currentMission_.scale.metersPerPixel, 0, 'f', 4));
  }

  updateWaypointsList();
}

void MissionPlannerPanel::markMissionModified() {
  missionModified_ = true;
  emit missionChanged(currentMission_);
}

// --- Undo/Redo Support ---

void MissionPlannerPanel::undo() {
  if (undoStack_->canUndo()) {
    undoStack_->undo();
    markMissionModified();
  }
}

void MissionPlannerPanel::redo() {
  if (undoStack_->canRedo()) {
    undoStack_->redo();
    markMissionModified();
  }
}

void MissionPlannerPanel::addWaypointFromUndo(const Waypoint& waypoint) {
  // Add waypoint to mission data
  currentMission_.waypoints.append(waypoint);

  // Add to map view
  mapView_->setWaypoints(currentMission_.waypoints);
  updateWaypointsList();
}

void MissionPlannerPanel::insertWaypointFromUndo(const Waypoint& waypoint, int index) {
  if (index < 0) index = 0;
  if (index > currentMission_.waypoints.size()) {
    index = currentMission_.waypoints.size();
  }

  currentMission_.waypoints.insert(index, waypoint);
  mapView_->setWaypoints(currentMission_.waypoints);
  updateWaypointsList();
}

void MissionPlannerPanel::removeWaypointById(int waypointId) {
  for (int i = 0; i < currentMission_.waypoints.size(); ++i) {
    if (currentMission_.waypoints[i].id == waypointId) {
      currentMission_.waypoints.removeAt(i);
      mapView_->removeWaypoint(waypointId);
      updateWaypointsList();
      waypointEditor_->clear();
      break;
    }
  }
}

void MissionPlannerPanel::moveWaypointFromUndo(int waypointId, const QPointF& position) {
  Waypoint* wp = currentMission_.findWaypoint(waypointId);
  if (wp) {
    wp->x = position.x();
    wp->y = position.y();
    mapView_->updateWaypoint(*wp);

    // Update editor if this waypoint is selected
    int row = waypointsList_->currentRow();
    if (row >= 0 && row < currentMission_.waypoints.size() &&
        currentMission_.waypoints[row].id == waypointId) {
      waypointEditor_->setWaypoint(*wp);
    }
    updateWaypointsList();
  }
}

void MissionPlannerPanel::setWaypointOrientationFromUndo(int waypointId, double theta) {
  Waypoint* wp = currentMission_.findWaypoint(waypointId);
  if (wp) {
    wp->theta = theta;
    mapView_->updateWaypoint(*wp);

    int row = waypointsList_->currentRow();
    if (row >= 0 && row < currentMission_.waypoints.size() &&
        currentMission_.waypoints[row].id == waypointId) {
      waypointEditor_->setWaypoint(*wp);
    }
  }
}

void MissionPlannerPanel::setStartPoseFromUndo(const RobotStartPose& pose) {
  currentMission_.startPose = pose;
  mapView_->setStartPose(pose);

  // Update UI
  startXSpinBox_->blockSignals(true);
  startYSpinBox_->blockSignals(true);
  startThetaSpinBox_->blockSignals(true);
  startOrientationDial_->blockSignals(true);

  startXSpinBox_->setValue(pose.x);
  startYSpinBox_->setValue(pose.y);
  startThetaSpinBox_->setValue(pose.thetaDegrees());
  startOrientationDial_->setValue(static_cast<int>(pose.thetaDegrees()));

  startXSpinBox_->blockSignals(false);
  startYSpinBox_->blockSignals(false);
  startThetaSpinBox_->blockSignals(false);
  startOrientationDial_->blockSignals(false);
}

void MissionPlannerPanel::setWaypointsFromUndo(const QList<Waypoint>& waypoints) {
  currentMission_.waypoints = waypoints;
  mapView_->setWaypoints(waypoints);
  updateWaypointsList();
}

void MissionPlannerPanel::clearWaypointsFromUndo() {
  currentMission_.waypoints.clear();
  mapView_->clearWaypoints();
  updateWaypointsList();
  waypointEditor_->clear();
}

void MissionPlannerPanel::reorderWaypointFromUndo(int fromIndex, int toIndex) {
  if (fromIndex < 0 || fromIndex >= currentMission_.waypoints.size()) return;
  if (toIndex < 0 || toIndex >= currentMission_.waypoints.size()) return;

  currentMission_.reorderWaypoints(fromIndex, toIndex);
  mapView_->setWaypoints(currentMission_.waypoints);
  updateWaypointsList();
}

}  // namespace ros_weaver
