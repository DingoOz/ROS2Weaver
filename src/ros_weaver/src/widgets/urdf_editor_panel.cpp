#include "ros_weaver/widgets/urdf_editor_panel.hpp"
#include "ros_weaver/widgets/urdf_hierarchy_tree.hpp"
#include "ros_weaver/widgets/urdf_viewport_3d.hpp"
#include "ros_weaver/widgets/urdf_properties_panel.hpp"
#include "ros_weaver/core/urdf_parser.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStyle>

namespace ros_weaver {

URDFEditorPanel::URDFEditorPanel(QWidget* parent)
  : QWidget(parent) {
  setupUi();
  setupConnections();
  updateUiState();
}

URDFEditorPanel::~URDFEditorPanel() = default;

bool URDFEditorPanel::openFile(const QString& filePath) {
  if (!maybeSave()) {
    return false;
  }

  if (filePath.isEmpty()) {
    return false;
  }

  statusLabel_->setText(tr("Loading..."));

  auto newModel = parser_->loadFile(filePath);
  if (!newModel) {
    QMessageBox::warning(this, tr("Load Failed"),
                         tr("Failed to load URDF file:\n%1").arg(parser_->lastError()));
    statusLabel_->setText(tr("Load failed"));
    return false;
  }

  model_ = std::move(newModel);
  currentFilePath_ = filePath;
  undoStack_->clear();

  // Update all components with the new model
  hierarchyTree_->setModel(model_.get());
  viewport_->setModel(model_.get());
  propertiesPanel_->setModel(model_.get());

  updateUiState();
  statusLabel_->setText(tr("Loaded: %1").arg(QFileInfo(filePath).fileName()));

  emit fileOpened(filePath);
  return true;
}

bool URDFEditorPanel::saveFile(const QString& filePath) {
  QString savePath = filePath.isEmpty() ? currentFilePath_ : filePath;

  if (savePath.isEmpty()) {
    return saveFileAs();
  }

  if (!model_) {
    return false;
  }

  statusLabel_->setText(tr("Saving..."));

  if (!parser_->saveFile(*model_, savePath, true)) {
    QMessageBox::warning(this, tr("Save Failed"),
                         tr("Failed to save URDF file:\n%1").arg(parser_->lastError()));
    statusLabel_->setText(tr("Save failed"));
    return false;
  }

  currentFilePath_ = savePath;
  model_->setModified(false);
  undoStack_->setClean();

  updateUiState();
  statusLabel_->setText(tr("Saved: %1").arg(QFileInfo(savePath).fileName()));

  emit fileSaved(savePath);
  return true;
}

bool URDFEditorPanel::saveFileAs() {
  QString filePath = QFileDialog::getSaveFileName(
      this,
      tr("Save URDF File"),
      currentFilePath_.isEmpty() ? QString() : QFileInfo(currentFilePath_).absolutePath(),
      tr("URDF Files (*.urdf);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (filePath.isEmpty()) {
    return false;
  }

  // Ensure .urdf extension
  if (!filePath.endsWith(".urdf", Qt::CaseInsensitive)) {
    filePath += ".urdf";
  }

  return saveFile(filePath);
}

void URDFEditorPanel::closeFile() {
  if (!maybeSave()) {
    return;
  }

  model_.reset();
  currentFilePath_.clear();
  undoStack_->clear();

  hierarchyTree_->setModel(nullptr);
  viewport_->setModel(nullptr);
  propertiesPanel_->setModel(nullptr);

  updateUiState();
  statusLabel_->setText(tr("No file loaded"));

  emit fileClosed();
}

bool URDFEditorPanel::hasUnsavedChanges() const {
  return model_ && model_->isModified();
}

QString URDFEditorPanel::currentFilePath() const {
  return currentFilePath_;
}

URDFModel* URDFEditorPanel::model() const {
  return model_.get();
}

bool URDFEditorPanel::hasModel() const {
  return model_ != nullptr;
}

void URDFEditorPanel::onOpenFile() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open URDF/Xacro File"),
      QString(),
      tr("URDF/Xacro Files (*.urdf *.xacro *.urdf.xacro);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (!filePath.isEmpty()) {
    openFile(filePath);
  }
}

void URDFEditorPanel::onSaveFile() {
  saveFile();
}

void URDFEditorPanel::onSaveFileAs() {
  saveFileAs();
}

void URDFEditorPanel::onCloseFile() {
  closeFile();
}

void URDFEditorPanel::onReloadFile() {
  if (currentFilePath_.isEmpty()) {
    return;
  }

  if (hasUnsavedChanges()) {
    int ret = QMessageBox::question(this, tr("Reload File"),
        tr("You have unsaved changes. Reload anyway?"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret != QMessageBox::Yes) {
      return;
    }
  }

  QString path = currentFilePath_;
  model_.reset();
  openFile(path);
}

void URDFEditorPanel::onResetView() {
  viewport_->resetView();
}

void URDFEditorPanel::onToggleGrid(bool visible) {
  viewport_->setGridVisible(visible);
}

void URDFEditorPanel::onToggleWireframe(bool wireframe) {
  viewport_->setWireframeMode(wireframe);
}

void URDFEditorPanel::onFocusSelected() {
  viewport_->focusOnSelected();
}

void URDFEditorPanel::onHierarchySelectionChanged(const QString& name, bool isJoint) {
  selectElement(name, isJoint);
}

void URDFEditorPanel::onViewportSelectionChanged(const QString& name, bool isJoint) {
  selectElement(name, isJoint);
  hierarchyTree_->selectByName(name, isJoint);
}

void URDFEditorPanel::onPropertyChanged() {
  if (model_) {
    model_->setModified(true);
    viewport_->refresh();
    updateUiState();
    emit modelModified();
  }
}

void URDFEditorPanel::onModelModified() {
  updateUiState();
  emit modelModified();
}

void URDFEditorPanel::onParseProgress(int percent, const QString& message) {
  statusLabel_->setText(QString("%1% - %2").arg(percent).arg(message));
}

void URDFEditorPanel::onParseError(const QString& error) {
  emit statusMessage(tr("Error: %1").arg(error), 5000);
}

void URDFEditorPanel::onWorkspaceNotBuilt(const QString& workspacePath, const QString& buildCommand) {
  QString message = tr(
      "<p><b>The ROS2 workspace containing this file has not been built.</b></p>"
      "<p>Xacro files require the workspace to be built so that package paths can be resolved.</p>"
      "<p><b>Workspace path:</b><br><code>%1</code></p>"
      "<p><b>To build the workspace, run:</b></p>"
      "<pre>%2</pre>"
      "<p>After building, source the workspace:</p>"
      "<pre>source %1/install/setup.bash</pre>"
      "<p>Then restart ROS2Weaver or try loading the file again.</p>"
  ).arg(workspacePath, buildCommand);

  QMessageBox msgBox(this);
  msgBox.setWindowTitle(tr("Workspace Not Built"));
  msgBox.setIcon(QMessageBox::Warning);
  msgBox.setTextFormat(Qt::RichText);
  msgBox.setText(message);
  msgBox.setStandardButtons(QMessageBox::Ok);
  msgBox.exec();

  statusLabel_->setText(tr("Workspace not built - see instructions"));
}

void URDFEditorPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  // Create undo stack
  undoStack_ = new QUndoStack(this);

  // Create parser
  parser_ = new URDFParser(this);

  // Setup toolbar
  setupToolbar();
  mainLayout->addWidget(toolbar_);

  // Setup main splitter with three panels
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  setupLeftPanel();
  setupCenterPanel();
  setupRightPanel();

  mainSplitter_->addWidget(leftPanel_);
  mainSplitter_->addWidget(centerPanel_);
  mainSplitter_->addWidget(rightPanel_);

  // Set initial sizes (200:600:250)
  mainSplitter_->setSizes({200, 600, 250});

  mainLayout->addWidget(mainSplitter_, 1);
}

void URDFEditorPanel::setupToolbar() {
  toolbar_ = new QToolBar(this);
  toolbar_->setMovable(false);
  toolbar_->setFloatable(false);

  // File operations
  openButton_ = new QPushButton(tr("Open"));
  openButton_->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
  openButton_->setToolTip(tr("Open URDF/Xacro file"));
  toolbar_->addWidget(openButton_);

  saveButton_ = new QPushButton(tr("Save"));
  saveButton_->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
  saveButton_->setToolTip(tr("Save URDF file"));
  toolbar_->addWidget(saveButton_);

  saveAsButton_ = new QPushButton(tr("Save As"));
  saveAsButton_->setToolTip(tr("Save URDF file as..."));
  toolbar_->addWidget(saveAsButton_);

  reloadButton_ = new QPushButton(tr("Reload"));
  reloadButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  reloadButton_->setToolTip(tr("Reload file from disk"));
  toolbar_->addWidget(reloadButton_);

  toolbar_->addSeparator();

  // Undo/Redo
  undoButton_ = new QPushButton(tr("Undo"));
  undoButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
  undoButton_->setToolTip(tr("Undo last change (Ctrl+Z)"));
  undoButton_->setShortcut(QKeySequence::Undo);
  toolbar_->addWidget(undoButton_);

  redoButton_ = new QPushButton(tr("Redo"));
  redoButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
  redoButton_->setToolTip(tr("Redo last undone change (Ctrl+Y)"));
  redoButton_->setShortcut(QKeySequence::Redo);
  toolbar_->addWidget(redoButton_);

  toolbar_->addSeparator();

  // View controls
  resetViewButton_ = new QPushButton(tr("Reset View"));
  resetViewButton_->setToolTip(tr("Reset camera to default position"));
  toolbar_->addWidget(resetViewButton_);

  focusSelectedButton_ = new QPushButton(tr("Focus"));
  focusSelectedButton_->setToolTip(tr("Focus camera on selected element"));
  toolbar_->addWidget(focusSelectedButton_);

  toolbar_->addSeparator();

  // Overlay toggles
  showLabelsButton_ = new QPushButton(tr("Labels"));
  showLabelsButton_->setCheckable(true);
  showLabelsButton_->setToolTip(tr("Show/hide link and joint names"));
  toolbar_->addWidget(showLabelsButton_);

  showLinesButton_ = new QPushButton(tr("Lines"));
  showLinesButton_->setCheckable(true);
  showLinesButton_->setToolTip(tr("Show/hide hierarchy connection lines"));
  toolbar_->addWidget(showLinesButton_);

  showAxesButton_ = new QPushButton(tr("Axes"));
  showAxesButton_->setCheckable(true);
  showAxesButton_->setToolTip(tr("Show/hide local coordinate axes (RGB=XYZ)"));
  toolbar_->addWidget(showAxesButton_);

  toolbar_->addSeparator();

  // Stretch to push status to right
  QWidget* spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar_->addWidget(spacer);

  // Status
  statusLabel_ = new QLabel(tr("No file loaded"));
  toolbar_->addWidget(statusLabel_);
}

void URDFEditorPanel::setupLeftPanel() {
  leftPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(leftPanel_);
  layout->setContentsMargins(4, 4, 4, 4);

  QLabel* header = new QLabel(tr("Hierarchy"));
  header->setStyleSheet("font-weight: bold;");
  layout->addWidget(header);

  hierarchyTree_ = new URDFHierarchyTree(this);
  layout->addWidget(hierarchyTree_, 1);
}

void URDFEditorPanel::setupCenterPanel() {
  centerPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(centerPanel_);
  layout->setContentsMargins(0, 0, 0, 0);

  viewport_ = new URDFViewport3D(this);
  viewport_->setParser(parser_);  // Allow viewport to resolve package:// URIs
  layout->addWidget(viewport_, 1);
}

void URDFEditorPanel::setupRightPanel() {
  rightPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(rightPanel_);
  layout->setContentsMargins(4, 4, 4, 4);

  QLabel* header = new QLabel(tr("Properties"));
  header->setStyleSheet("font-weight: bold;");
  layout->addWidget(header);

  propertiesPanel_ = new URDFPropertiesPanel(this);
  propertiesPanel_->setUndoStack(undoStack_);
  layout->addWidget(propertiesPanel_, 1);
}

void URDFEditorPanel::setupConnections() {
  // Toolbar buttons
  connect(openButton_, &QPushButton::clicked, this, &URDFEditorPanel::onOpenFile);
  connect(saveButton_, &QPushButton::clicked, this, &URDFEditorPanel::onSaveFile);
  connect(saveAsButton_, &QPushButton::clicked, this, &URDFEditorPanel::onSaveFileAs);
  connect(reloadButton_, &QPushButton::clicked, this, &URDFEditorPanel::onReloadFile);

  connect(undoButton_, &QPushButton::clicked, undoStack_, &QUndoStack::undo);
  connect(redoButton_, &QPushButton::clicked, undoStack_, &QUndoStack::redo);

  connect(resetViewButton_, &QPushButton::clicked, this, &URDFEditorPanel::onResetView);
  connect(focusSelectedButton_, &QPushButton::clicked, this, &URDFEditorPanel::onFocusSelected);

  // Overlay toggles
  connect(showLabelsButton_, &QPushButton::toggled, viewport_, &URDFViewport3D::setLabelsVisible);
  connect(showLinesButton_, &QPushButton::toggled, viewport_, &URDFViewport3D::setHierarchyLinesVisible);
  connect(showAxesButton_, &QPushButton::toggled, viewport_, &URDFViewport3D::setLocalAxesVisible);

  // Undo stack state
  connect(undoStack_, &QUndoStack::canUndoChanged, undoButton_, &QPushButton::setEnabled);
  connect(undoStack_, &QUndoStack::canRedoChanged, redoButton_, &QPushButton::setEnabled);
  connect(undoStack_, &QUndoStack::cleanChanged, this, [this](bool clean) {
    if (model_ && !clean) {
      model_->setModified(true);
      updateUiState();
    }
  });

  // Parser signals
  connect(parser_, &URDFParser::parseProgress, this, &URDFEditorPanel::onParseProgress);
  connect(parser_, &URDFParser::parseError, this, &URDFEditorPanel::onParseError);
  connect(parser_, &URDFParser::workspaceNotBuilt, this, &URDFEditorPanel::onWorkspaceNotBuilt);

  // Hierarchy tree selection
  connect(hierarchyTree_, &URDFHierarchyTree::selectionChanged,
          this, &URDFEditorPanel::onHierarchySelectionChanged);

  // Viewport selection
  connect(viewport_, &URDFViewport3D::selectionChanged,
          this, &URDFEditorPanel::onViewportSelectionChanged);

  // Viewport transform changes
  connect(viewport_, &URDFViewport3D::transformChanged,
          this, &URDFEditorPanel::onPropertyChanged);

  // Properties panel changes
  connect(propertiesPanel_, &URDFPropertiesPanel::propertyChanged,
          this, &URDFEditorPanel::onPropertyChanged);
}

void URDFEditorPanel::updateUiState() {
  bool hasFile = model_ != nullptr;
  bool isModified = hasFile && model_->isModified();

  saveButton_->setEnabled(hasFile);
  saveAsButton_->setEnabled(hasFile);
  reloadButton_->setEnabled(!currentFilePath_.isEmpty());

  resetViewButton_->setEnabled(hasFile);
  focusSelectedButton_->setEnabled(hasFile && !selectedElementName_.isEmpty());

  undoButton_->setEnabled(undoStack_->canUndo());
  redoButton_->setEnabled(undoStack_->canRedo());

  // Update title indicator
  QString title = currentFilePath_.isEmpty() ? tr("Untitled") : QFileInfo(currentFilePath_).fileName();
  if (isModified) {
    title += "*";
  }
  // Could emit signal to update dock widget title here
}

void URDFEditorPanel::updateWindowTitle() {
  // This would update the parent dock widget title
  // For now, handled by statusLabel_
}

bool URDFEditorPanel::maybeSave() {
  if (!hasUnsavedChanges()) {
    return true;
  }

  int ret = QMessageBox::question(this, tr("Unsaved Changes"),
      tr("The URDF has been modified.\nDo you want to save your changes?"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Save);

  switch (ret) {
    case QMessageBox::Save:
      return saveFile();
    case QMessageBox::Discard:
      return true;
    case QMessageBox::Cancel:
    default:
      return false;
  }
}

void URDFEditorPanel::selectElement(const QString& name, bool isJoint) {
  selectedElementName_ = name;
  selectedIsJoint_ = isJoint;

  // Update viewport selection
  viewport_->selectElement(name, isJoint);

  // Update properties panel
  propertiesPanel_->showElement(name, isJoint);

  updateUiState();
  emit selectionChanged(name, isJoint);
}

}  // namespace ros_weaver
