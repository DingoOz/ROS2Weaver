#include "ros_weaver/widgets/canvas_tab_widget.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/node_group.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/undo/undo_stack.hpp"

#include <QInputDialog>
#include <QMessageBox>

namespace ros_weaver {

CanvasTabWidget::CanvasTabWidget(QWidget* parent)
    : QTabWidget(parent)
    , undoStack_(nullptr)
    , canvasCounter_(0)
{
  setupUI();
}

CanvasTabWidget::~CanvasTabWidget() {
  // Canvases are owned by the tab widget and deleted automatically
}

void CanvasTabWidget::setupUI() {
  setTabsClosable(true);
  setMovable(true);
  setDocumentMode(true);

  connect(this, &QTabWidget::tabCloseRequested,
          this, &CanvasTabWidget::onTabCloseRequested);
  connect(this, &QTabWidget::currentChanged,
          this, &CanvasTabWidget::onCurrentChanged);
  connect(this, &QTabWidget::tabBarDoubleClicked,
          this, &CanvasTabWidget::onTabDoubleClicked);

  // Add the first canvas by default
  addCanvas("Main");
}

WeaverCanvas* CanvasTabWidget::addCanvas(const QString& name) {
  QString uniqueName = generateUniqueName(name);

  WeaverCanvas* canvas = new WeaverCanvas(this);
  if (undoStack_) {
    canvas->setUndoStack(undoStack_);
  }

  // Generate unique ID for the canvas
  QUuid canvasId = QUuid::createUuid();
  canvases_[canvasId] = canvas;

  // Add to tab
  int index = addTab(canvas, uniqueName);

  // Store tab data
  CanvasTabData data;
  data.id = canvasId;
  data.name = uniqueName;
  tabData_[index] = data;

  // Connect signals
  connectCanvasSignals(canvas);

  // Switch to new tab
  setCurrentIndex(index);

  emit tabAdded(index);
  canvasCounter_++;

  return canvas;
}

void CanvasTabWidget::removeCanvas(int index) {
  if (count() <= 1) {
    // Don't remove the last canvas
    return;
  }

  WeaverCanvas* canvas = canvasAt(index);
  if (!canvas) return;

  // Find and remove from map
  QUuid idToRemove;
  for (auto it = canvases_.begin(); it != canvases_.end(); ++it) {
    if (it.value() == canvas) {
      idToRemove = it.key();
      break;
    }
  }

  if (!idToRemove.isNull()) {
    canvases_.remove(idToRemove);
  }

  // Remove tab data
  tabData_.remove(index);

  // Disconnect signals before removing
  disconnectCanvasSignals(canvas);

  // Remove the tab (this deletes the widget)
  removeTab(index);

  emit tabRemoved(index);
}

void CanvasTabWidget::closeCurrentCanvas() {
  if (count() > 1) {
    removeCanvas(currentIndex());
  }
}

WeaverCanvas* CanvasTabWidget::currentCanvas() const {
  return qobject_cast<WeaverCanvas*>(currentWidget());
}

WeaverCanvas* CanvasTabWidget::canvasAt(int index) const {
  return qobject_cast<WeaverCanvas*>(widget(index));
}

int CanvasTabWidget::canvasCount() const {
  return count();
}

WeaverCanvas* CanvasTabWidget::canvasById(const QUuid& id) const {
  return canvases_.value(id, nullptr);
}

QUuid CanvasTabWidget::currentCanvasId() const {
  WeaverCanvas* canvas = currentCanvas();
  if (!canvas) return QUuid();

  for (auto it = canvases_.begin(); it != canvases_.end(); ++it) {
    if (it.value() == canvas) {
      return it.key();
    }
  }
  return QUuid();
}

void CanvasTabWidget::renameTab(int index, const QString& name) {
  if (index < 0 || index >= count()) return;

  setTabText(index, name);
  if (tabData_.contains(index)) {
    tabData_[index].name = name;
  }

  emit tabRenamed(index, name);
}

void CanvasTabWidget::renameCurrentTab(const QString& name) {
  renameTab(currentIndex(), name);
}

QString CanvasTabWidget::tabName(int index) const {
  return tabText(index);
}

void CanvasTabWidget::exportToProject(Project& project) const {
  // Clear existing canvas data
  project.clearCanvases();

  // Export each canvas
  for (int i = 0; i < count(); ++i) {
    WeaverCanvas* canvas = canvasAt(i);
    if (!canvas) continue;

    // Create canvas data
    CanvasData canvasData;
    canvasData.id = tabData_.contains(i) ? tabData_[i].id : QUuid::createUuid();
    canvasData.name = tabText(i);
    canvasData.description = tabData_.contains(i) ? tabData_[i].description : "";
    canvasData.isActive = (i == currentIndex());

    // Export blocks and connections from canvas
    canvas->exportToProject(project);

    // Move project blocks/connections to canvas data
    for (const BlockData& block : project.blocks()) {
      canvasData.blocks.append(block);
    }
    for (const ConnectionData& conn : project.connections()) {
      canvasData.connections.append(conn);
    }
    for (const NodeGroupData& group : project.nodeGroups()) {
      canvasData.groups.append(group);
    }

    // Clear project-level data (it's now in canvas)
    project.clearBlocks();
    project.clearConnections();
    project.clearNodeGroups();

    project.addCanvas(canvasData);
  }

  // Set active canvas index
  project.setActiveCanvasIndex(currentIndex());
}

void CanvasTabWidget::importFromProject(const Project& project) {
  clearAll();

  const QList<CanvasData>& canvasList = project.canvases();

  if (canvasList.isEmpty()) {
    // Legacy project - single canvas
    WeaverCanvas* canvas = addCanvas("Main");
    canvas->importFromProject(project);
    return;
  }

  // Import each canvas
  bool first = true;
  for (const CanvasData& canvasData : canvasList) {
    WeaverCanvas* canvas;
    if (first) {
      // Reuse the default canvas
      canvas = canvasAt(0);
      renameTab(0, canvasData.name);
      first = false;

      // Update ID in maps
      QUuid oldId;
      for (auto it = canvases_.begin(); it != canvases_.end(); ++it) {
        if (it.value() == canvas) {
          oldId = it.key();
          break;
        }
      }
      if (!oldId.isNull()) {
        canvases_.remove(oldId);
      }
      canvases_[canvasData.id] = canvas;
      tabData_[0].id = canvasData.id;
      tabData_[0].name = canvasData.name;
      tabData_[0].description = canvasData.description;
    } else {
      canvas = addCanvas(canvasData.name);

      // Update ID
      int index = indexOf(canvas);
      QUuid oldId;
      for (auto it = canvases_.begin(); it != canvases_.end(); ++it) {
        if (it.value() == canvas) {
          oldId = it.key();
          break;
        }
      }
      if (!oldId.isNull()) {
        canvases_.remove(oldId);
      }
      canvases_[canvasData.id] = canvas;
      if (tabData_.contains(index)) {
        tabData_[index].id = canvasData.id;
        tabData_[index].description = canvasData.description;
      }
    }

    // Create a temporary project for this canvas
    Project tempProject;
    for (const BlockData& block : canvasData.blocks) {
      tempProject.addBlock(block);
    }
    for (const ConnectionData& conn : canvasData.connections) {
      tempProject.addConnection(conn);
    }
    for (const NodeGroupData& group : canvasData.groups) {
      tempProject.addNodeGroup(group);
    }

    canvas->importFromProject(tempProject);
  }

  // Set active canvas
  int activeIndex = project.activeCanvasIndex();
  if (activeIndex >= 0 && activeIndex < count()) {
    setCurrentIndex(activeIndex);
  }
}

void CanvasTabWidget::clearAll() {
  // Keep only one canvas and clear it
  while (count() > 1) {
    removeCanvas(count() - 1);
  }

  // Clear the remaining canvas
  WeaverCanvas* canvas = canvasAt(0);
  if (canvas) {
    canvas->clearCanvas();
  }

  // Reset the name
  renameTab(0, "Main");

  canvasCounter_ = 1;
}

void CanvasTabWidget::setUndoStack(UndoStack* undoStack) {
  undoStack_ = undoStack;

  // Set on all canvases
  for (WeaverCanvas* canvas : canvases_) {
    canvas->setUndoStack(undoStack);
  }
}

QList<WeaverCanvas*> CanvasTabWidget::allCanvases() const {
  QList<WeaverCanvas*> list;
  for (int i = 0; i < count(); ++i) {
    WeaverCanvas* canvas = canvasAt(i);
    if (canvas) {
      list.append(canvas);
    }
  }
  return list;
}

void CanvasTabWidget::duplicateCurrentCanvas() {
  WeaverCanvas* source = currentCanvas();
  if (!source) return;

  QString baseName = tabName(currentIndex());
  QString newName = baseName + " (Copy)";
  WeaverCanvas* newCanvas = addCanvas(newName);

  // Export source canvas to a temporary project and import into new canvas
  Project tempProject;
  source->exportToProject(tempProject);
  newCanvas->importFromProject(tempProject);
}

void CanvasTabWidget::onTabCloseRequested(int index) {
  if (count() <= 1) {
    QMessageBox::information(this, tr("Cannot Close"),
        tr("Cannot close the last canvas. At least one canvas must remain."));
    return;
  }

  // Confirm before closing
  int result = QMessageBox::question(this, tr("Close Canvas"),
      tr("Are you sure you want to close '%1'?").arg(tabText(index)),
      QMessageBox::Yes | QMessageBox::No);

  if (result == QMessageBox::Yes) {
    removeCanvas(index);
  }
}

void CanvasTabWidget::onCurrentChanged(int index) {
  WeaverCanvas* canvas = canvasAt(index);
  emit currentCanvasChanged(canvas);
}

void CanvasTabWidget::onTabDoubleClicked(int index) {
  if (index < 0) {
    // Double-clicked on empty area - add new canvas
    addCanvas("New Canvas");
    return;
  }

  // Rename tab
  bool ok;
  QString currentName = tabText(index);
  QString newName = QInputDialog::getText(this, tr("Rename Canvas"),
      tr("Canvas name:"), QLineEdit::Normal, currentName, &ok);

  if (ok && !newName.isEmpty() && newName != currentName) {
    renameTab(index, newName);
  }
}

void CanvasTabWidget::forwardBlockSelected(PackageBlock* block) {
  WeaverCanvas* canvas = qobject_cast<WeaverCanvas*>(sender());
  if (canvas == currentCanvas()) {
    emit blockSelected(block);
  }
}

void CanvasTabWidget::forwardBlockDoubleClicked(PackageBlock* block) {
  WeaverCanvas* canvas = qobject_cast<WeaverCanvas*>(sender());
  if (canvas == currentCanvas()) {
    emit blockDoubleClicked(block);
  }
}

void CanvasTabWidget::forwardConnectionCreated(ConnectionLine* connection) {
  WeaverCanvas* canvas = qobject_cast<WeaverCanvas*>(sender());
  if (canvas == currentCanvas()) {
    emit connectionCreated(connection);
  }
}

void CanvasTabWidget::forwardGroupCreated(NodeGroup* group) {
  WeaverCanvas* canvas = qobject_cast<WeaverCanvas*>(sender());
  if (canvas == currentCanvas()) {
    emit groupCreated(group);
  }
}

void CanvasTabWidget::forwardCanvasCleared() {
  WeaverCanvas* canvas = qobject_cast<WeaverCanvas*>(sender());
  if (canvas == currentCanvas()) {
    emit canvasCleared();
  }
}

void CanvasTabWidget::connectCanvasSignals(WeaverCanvas* canvas) {
  connect(canvas, &WeaverCanvas::blockSelected,
          this, &CanvasTabWidget::forwardBlockSelected);
  connect(canvas, &WeaverCanvas::blockDoubleClicked,
          this, &CanvasTabWidget::forwardBlockDoubleClicked);
  connect(canvas, &WeaverCanvas::connectionCreated,
          this, &CanvasTabWidget::forwardConnectionCreated);
  connect(canvas, &WeaverCanvas::groupCreated,
          this, &CanvasTabWidget::forwardGroupCreated);
  connect(canvas, &WeaverCanvas::canvasCleared,
          this, &CanvasTabWidget::forwardCanvasCleared);
  connect(canvas, &WeaverCanvas::blockYamlSourceChanged,
          this, &CanvasTabWidget::blockYamlSourceChanged);
  connect(canvas, &WeaverCanvas::openBlockInVSCodeRequested,
          this, &CanvasTabWidget::openBlockInVSCodeRequested);
  connect(canvas, &WeaverCanvas::askAIAboutBlock,
          this, &CanvasTabWidget::askAIAboutBlock);
  connect(canvas, &WeaverCanvas::askAIAboutConnection,
          this, &CanvasTabWidget::askAIAboutConnection);
  connect(canvas, &WeaverCanvas::askAIAboutGroup,
          this, &CanvasTabWidget::askAIAboutGroup);
  connect(canvas, &WeaverCanvas::askAIAboutPin,
          this, &CanvasTabWidget::askAIAboutPin);
}

void CanvasTabWidget::disconnectCanvasSignals(WeaverCanvas* canvas) {
  disconnect(canvas, nullptr, this, nullptr);
}

QString CanvasTabWidget::generateUniqueName(const QString& baseName) const {
  QString name = baseName;
  int suffix = 1;

  // Check if name already exists
  bool exists = true;
  while (exists) {
    exists = false;
    for (int i = 0; i < count(); ++i) {
      if (tabText(i) == name) {
        exists = true;
        name = QString("%1 %2").arg(baseName).arg(++suffix);
        break;
      }
    }
  }

  return name;
}

}  // namespace ros_weaver
