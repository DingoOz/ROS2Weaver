#include "ros_weaver/widgets/behavior_tree_panel.hpp"
#include <QMessageBox>
#include <QWheelEvent>
#include <QDebug>

namespace ros_weaver {

BehaviorTreePanel::BehaviorTreePanel(QWidget* parent)
    : QWidget(parent) {
  setupUi();
}

BehaviorTreePanel::~BehaviorTreePanel() = default;

void BehaviorTreePanel::setupUi() {
  mainLayout_ = new QVBoxLayout(this);
  mainLayout_->setContentsMargins(4, 4, 4, 4);
  mainLayout_->setSpacing(4);

  // Toolbar
  toolbarLayout_ = new QHBoxLayout();
  toolbarLayout_->setSpacing(4);

  loadButton_ = new QPushButton(tr("Load BT..."));
  loadButton_->setToolTip(tr("Load a BehaviorTree.CPP XML file"));
  connect(loadButton_, &QPushButton::clicked, this, &BehaviorTreePanel::onLoadFileClicked);

  refreshButton_ = new QPushButton(tr("Refresh"));
  refreshButton_->setToolTip(tr("Reload the current file"));
  refreshButton_->setEnabled(false);
  connect(refreshButton_, &QPushButton::clicked, this, &BehaviorTreePanel::onRefreshClicked);

  toolbarLayout_->addWidget(loadButton_);
  toolbarLayout_->addWidget(refreshButton_);
  toolbarLayout_->addStretch();

  zoomInButton_ = new QPushButton("+");
  zoomInButton_->setFixedWidth(30);
  zoomInButton_->setToolTip(tr("Zoom In"));
  connect(zoomInButton_, &QPushButton::clicked, this, &BehaviorTreePanel::onZoomInClicked);

  zoomOutButton_ = new QPushButton("-");
  zoomOutButton_->setFixedWidth(30);
  zoomOutButton_->setToolTip(tr("Zoom Out"));
  connect(zoomOutButton_, &QPushButton::clicked, this, &BehaviorTreePanel::onZoomOutClicked);

  zoomFitButton_ = new QPushButton(tr("Fit"));
  zoomFitButton_->setToolTip(tr("Fit tree to view"));
  connect(zoomFitButton_, &QPushButton::clicked, this, &BehaviorTreePanel::onZoomFitClicked);

  toolbarLayout_->addWidget(zoomOutButton_);
  toolbarLayout_->addWidget(zoomInButton_);
  toolbarLayout_->addWidget(zoomFitButton_);

  mainLayout_->addLayout(toolbarLayout_);

  // Tree name label
  treeNameLabel_ = new QLabel(tr("No behavior tree loaded"));
  treeNameLabel_->setStyleSheet("font-weight: bold; padding: 4px;");
  mainLayout_->addWidget(treeNameLabel_);

  // Graphics view for tree visualization
  scene_ = new QGraphicsScene(this);
  scene_->setBackgroundBrush(QColor(245, 245, 245));

  view_ = new QGraphicsView(scene_);
  view_->setRenderHint(QPainter::Antialiasing);
  view_->setDragMode(QGraphicsView::ScrollHandDrag);
  view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  view_->setResizeAnchor(QGraphicsView::AnchorViewCenter);
  view_->setMinimumHeight(200);

  mainLayout_->addWidget(view_, 1);

  // Status label
  statusLabel_ = new QLabel();
  statusLabel_->setStyleSheet("color: gray; font-size: 11px;");
  mainLayout_->addWidget(statusLabel_);

  updateStatusLabel();
}

bool BehaviorTreePanel::loadFromFile(const QString& filePath) {
  currentTree_ = parser_.parseFile(filePath);

  if (!currentTree_.isValid) {
    emit loadError(currentTree_.errorMessage);
    QMessageBox::warning(this, tr("Load Error"),
                         tr("Failed to load behavior tree:\n%1").arg(currentTree_.errorMessage));
    return false;
  }

  currentFilePath_ = filePath;
  refreshButton_->setEnabled(true);

  buildTreeVisualization();

  treeNameLabel_->setText(tr("Tree: %1").arg(currentTree_.name));
  updateStatusLabel();

  emit treeLoaded(currentTree_.name);
  return true;
}

bool BehaviorTreePanel::loadFromString(const QString& xmlContent) {
  currentTree_ = parser_.parseString(xmlContent);

  if (!currentTree_.isValid) {
    emit loadError(currentTree_.errorMessage);
    return false;
  }

  currentFilePath_.clear();
  refreshButton_->setEnabled(false);

  buildTreeVisualization();

  treeNameLabel_->setText(tr("Tree: %1").arg(currentTree_.name));
  updateStatusLabel();

  emit treeLoaded(currentTree_.name);
  return true;
}

void BehaviorTreePanel::clear() {
  scene_->clear();
  nodeItems_.clear();
  currentTree_ = BehaviorTree();
  currentFilePath_.clear();
  refreshButton_->setEnabled(false);
  treeNameLabel_->setText(tr("No behavior tree loaded"));
  updateStatusLabel();
}

void BehaviorTreePanel::setRosNode(rclcpp::Node::SharedPtr node) {
  rosNode_ = node;
  // Future: Subscribe to BT state topic here
}

void BehaviorTreePanel::onLoadFileClicked() {
  QString filePath = QFileDialog::getOpenFileName(
      this,
      tr("Open Behavior Tree"),
      QString(),
      tr("BehaviorTree XML (*.xml);;All Files (*)"),
      nullptr,
      QFileDialog::DontUseNativeDialog);

  if (!filePath.isEmpty()) {
    loadFromFile(filePath);
  }
}

void BehaviorTreePanel::onZoomInClicked() {
  view_->scale(1.2, 1.2);
}

void BehaviorTreePanel::onZoomOutClicked() {
  view_->scale(1.0 / 1.2, 1.0 / 1.2);
}

void BehaviorTreePanel::onZoomFitClicked() {
  if (scene_->items().isEmpty()) {
    return;
  }

  QRectF bounds = scene_->itemsBoundingRect();
  bounds.adjust(-20, -20, 20, 20);
  view_->fitInView(bounds, Qt::KeepAspectRatio);
}

void BehaviorTreePanel::onRefreshClicked() {
  if (!currentFilePath_.isEmpty()) {
    loadFromFile(currentFilePath_);
  }
}

void BehaviorTreePanel::buildTreeVisualization() {
  scene_->clear();
  nodeItems_.clear();

  if (!currentTree_.root) {
    return;
  }

  // Add all nodes recursively
  addNodeToScene(currentTree_.root, nullptr);

  // Fit view to content
  QTimer::singleShot(100, this, &BehaviorTreePanel::onZoomFitClicked);
}

void BehaviorTreePanel::addNodeToScene(std::shared_ptr<BTNode> node, BTNodeItem* parentItem) {
  // Create the node item
  BTNodeItem* nodeItem = new BTNodeItem(node);
  nodeItem->setPos(node->x, node->y);
  scene_->addItem(nodeItem);

  // Store reference
  nodeItems_[node->id] = nodeItem;

  // Add edge from parent
  if (parentItem) {
    BTEdgeItem* edge = new BTEdgeItem(parentItem, nodeItem);
    scene_->addItem(edge);
  }

  // Recursively add children
  for (const auto& child : node->children) {
    addNodeToScene(child, nodeItem);
  }
}

void BehaviorTreePanel::updateStatusLabel() {
  if (!currentTree_.isValid) {
    statusLabel_->setText(tr("No tree loaded"));
    return;
  }

  int nodeCount = nodeItems_.size();
  QString status = tr("%1 nodes").arg(nodeCount);

  if (!currentFilePath_.isEmpty()) {
    QFileInfo info(currentFilePath_);
    status += tr(" | File: %1").arg(info.fileName());
  }

  statusLabel_->setText(status);
}

}  // namespace ros_weaver
