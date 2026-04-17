#include "ros_weaver/widgets/urdf_hierarchy_tree.hpp"
#include "ros_weaver/core/urdf_model.hpp"

#include <QVBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QAction>
#include <QPainter>
#include <QPixmap>

namespace ros_weaver {

URDFHierarchyTree::URDFHierarchyTree(QWidget* parent)
  : QWidget(parent) {
  setupUi();
  setupConnections();
}

URDFHierarchyTree::~URDFHierarchyTree() = default;

void URDFHierarchyTree::setModel(URDFModel* model) {
  model_ = model;
  refreshTree();
}

void URDFHierarchyTree::refreshTree() {
  treeWidget_->clear();
  linkItems_.clear();
  jointItems_.clear();

  if (!model_) {
    return;
  }

  populateTree();
  treeWidget_->expandAll();
}

QString URDFHierarchyTree::selectedName() const {
  QTreeWidgetItem* item = treeWidget_->currentItem();
  if (!item) {
    return QString();
  }
  return item->data(0, NameRole).toString();
}

bool URDFHierarchyTree::isSelectedJoint() const {
  QTreeWidgetItem* item = treeWidget_->currentItem();
  if (!item) {
    return false;
  }
  return item->data(0, IsJointRole).toBool();
}

void URDFHierarchyTree::selectByName(const QString& name, bool isJoint) {
  treeWidget_->blockSignals(true);

  QTreeWidgetItem* item = nullptr;
  if (isJoint) {
    item = jointItems_.value(name, nullptr);
  } else {
    item = linkItems_.value(name, nullptr);
  }

  if (item) {
    treeWidget_->setCurrentItem(item);
    treeWidget_->scrollToItem(item);
  } else {
    treeWidget_->clearSelection();
  }

  treeWidget_->blockSignals(false);
}

void URDFHierarchyTree::clearSelection() {
  treeWidget_->clearSelection();
}

void URDFHierarchyTree::expandAll() {
  treeWidget_->expandAll();
}

void URDFHierarchyTree::collapseAll() {
  treeWidget_->collapseAll();
}

void URDFHierarchyTree::expandToSelected() {
  QTreeWidgetItem* item = treeWidget_->currentItem();
  if (!item) {
    return;
  }

  // Expand all parents
  QTreeWidgetItem* parent = item->parent();
  while (parent) {
    parent->setExpanded(true);
    parent = parent->parent();
  }

  treeWidget_->scrollToItem(item);
}

void URDFHierarchyTree::onFilterTextChanged(const QString& text) {
  applyFilter(text);
}

void URDFHierarchyTree::onExpandAll() {
  expandAll();
}

void URDFHierarchyTree::onCollapseAll() {
  collapseAll();
}

void URDFHierarchyTree::onTreeSelectionChanged() {
  QString name = selectedName();
  bool isJoint = isSelectedJoint();

  if (!name.isEmpty()) {
    emit selectionChanged(name, isJoint);
  }
}

void URDFHierarchyTree::onTreeItemDoubleClicked(QTreeWidgetItem* item, int /* column */) {
  if (!item) return;

  QString name = item->data(0, NameRole).toString();
  bool isJoint = item->data(0, IsJointRole).toBool();

  emit itemDoubleClicked(name, isJoint);
}

void URDFHierarchyTree::onContextMenuRequested(const QPoint& pos) {
  QTreeWidgetItem* item = treeWidget_->itemAt(pos);
  if (!item) return;

  QMenu menu(this);

  QAction* expandAction = menu.addAction(tr("Expand All Children"));
  connect(expandAction, &QAction::triggered, this, [item]() {
    std::function<void(QTreeWidgetItem*)> expandRecursive = [&](QTreeWidgetItem* it) {
      it->setExpanded(true);
      for (int i = 0; i < it->childCount(); ++i) {
        expandRecursive(it->child(i));
      }
    };
    expandRecursive(item);
  });

  QAction* collapseAction = menu.addAction(tr("Collapse All Children"));
  connect(collapseAction, &QAction::triggered, this, [item]() {
    std::function<void(QTreeWidgetItem*)> collapseRecursive = [&](QTreeWidgetItem* it) {
      it->setExpanded(false);
      for (int i = 0; i < it->childCount(); ++i) {
        collapseRecursive(it->child(i));
      }
    };
    collapseRecursive(item);
  });

  menu.exec(treeWidget_->viewport()->mapToGlobal(pos));
}

void URDFHierarchyTree::setupUi() {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  // Search/filter edit
  searchEdit_ = new QLineEdit(this);
  searchEdit_->setPlaceholderText(tr("Filter..."));
  searchEdit_->setClearButtonEnabled(true);
  layout->addWidget(searchEdit_);

  // Tree widget
  treeWidget_ = new QTreeWidget(this);
  treeWidget_->setHeaderHidden(true);
  treeWidget_->setRootIsDecorated(true);
  treeWidget_->setSelectionMode(QAbstractItemView::SingleSelection);
  treeWidget_->setContextMenuPolicy(Qt::CustomContextMenu);
  treeWidget_->setAnimated(true);

  layout->addWidget(treeWidget_, 1);
}

void URDFHierarchyTree::setupConnections() {
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &URDFHierarchyTree::onFilterTextChanged);

  connect(treeWidget_, &QTreeWidget::itemSelectionChanged,
          this, &URDFHierarchyTree::onTreeSelectionChanged);

  connect(treeWidget_, &QTreeWidget::itemDoubleClicked,
          this, &URDFHierarchyTree::onTreeItemDoubleClicked);

  connect(treeWidget_, &QTreeWidget::customContextMenuRequested,
          this, &URDFHierarchyTree::onContextMenuRequested);
}

void URDFHierarchyTree::populateTree() {
  if (!model_) return;

  // Find root link (link with no parent joint)
  QString rootName = model_->rootLinkName();
  if (rootName.isEmpty() && model_->linkCount() > 0) {
    rootName = model_->linkNames().first();
  }

  if (!rootName.isEmpty()) {
    addLinkToTree(rootName, nullptr);
  }
}

void URDFHierarchyTree::addLinkToTree(const QString& linkName, QTreeWidgetItem* parent) {
  if (!model_) return;

  // Cycle detection: skip if we've already added this link
  if (linkItems_.contains(linkName)) return;

  // Create link item
  QTreeWidgetItem* linkItem = createLinkItem(linkName);
  if (parent) {
    parent->addChild(linkItem);
  } else {
    treeWidget_->addTopLevelItem(linkItem);
  }
  linkItems_[linkName] = linkItem;

  // Find child joints/links
  const auto& joints = model_->joints();
  for (auto it = joints.constBegin(); it != joints.constEnd(); ++it) {
    if (it->parentLink == linkName) {
      // Create joint item as child of this link
      QTreeWidgetItem* jointItem = createJointItem(it->name);
      linkItem->addChild(jointItem);
      jointItems_[it->name] = jointItem;

      // Recursively add child link under the joint
      addLinkToTree(it->childLink, jointItem);
    }
  }
}

QTreeWidgetItem* URDFHierarchyTree::createLinkItem(const QString& linkName) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, linkName);
  item->setIcon(0, linkIcon());
  item->setData(0, NameRole, linkName);
  item->setData(0, IsJointRole, false);
  item->setToolTip(0, tr("Link: %1").arg(linkName));
  return item;
}

QTreeWidgetItem* URDFHierarchyTree::createJointItem(const QString& jointName) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, jointName);
  item->setData(0, NameRole, jointName);
  item->setData(0, IsJointRole, true);

  // Set icon based on joint type
  const URDFJoint* joint = model_->joint(jointName);
  if (joint) {
    switch (joint->type) {
      case URDFJointType::Revolute:
        item->setIcon(0, jointRevoluteIcon());
        item->setToolTip(0, tr("Revolute Joint: %1").arg(jointName));
        break;
      case URDFJointType::Continuous:
        item->setIcon(0, jointContinuousIcon());
        item->setToolTip(0, tr("Continuous Joint: %1").arg(jointName));
        break;
      case URDFJointType::Prismatic:
        item->setIcon(0, jointPrismaticIcon());
        item->setToolTip(0, tr("Prismatic Joint: %1").arg(jointName));
        break;
      case URDFJointType::Fixed:
        item->setIcon(0, jointFixedIcon());
        item->setToolTip(0, tr("Fixed Joint: %1").arg(jointName));
        break;
      case URDFJointType::Floating:
        item->setIcon(0, jointContinuousIcon());
        item->setToolTip(0, tr("Floating Joint: %1").arg(jointName));
        break;
      case URDFJointType::Planar:
        item->setIcon(0, jointPrismaticIcon());
        item->setToolTip(0, tr("Planar Joint: %1").arg(jointName));
        break;
    }
  }

  return item;
}

void URDFHierarchyTree::applyFilter(const QString& filter) {
  if (filter.isEmpty()) {
    // Show all items
    QTreeWidgetItemIterator it(treeWidget_);
    while (*it) {
      (*it)->setHidden(false);
      ++it;
    }
    return;
  }

  // Hide non-matching items, but keep parents of matching items visible
  QTreeWidgetItemIterator it(treeWidget_);
  while (*it) {
    bool matches = itemMatchesFilter(*it, filter);
    (*it)->setHidden(!matches);

    // If item matches, make sure all parents are visible
    if (matches) {
      QTreeWidgetItem* parent = (*it)->parent();
      while (parent) {
        parent->setHidden(false);
        parent->setExpanded(true);
        parent = parent->parent();
      }
    }

    ++it;
  }
}

bool URDFHierarchyTree::itemMatchesFilter(QTreeWidgetItem* item, const QString& filter) {
  QString name = item->data(0, NameRole).toString();
  return name.contains(filter, Qt::CaseInsensitive);
}

// Icons - using simple colored circles/shapes
// In production, these would be proper icon resources

QIcon URDFHierarchyTree::linkIcon() const {
  static QIcon cached;
  if (cached.isNull()) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(100, 149, 237));  // Cornflower blue
    p.setPen(Qt::NoPen);
    p.drawEllipse(2, 2, 12, 12);
    cached = QIcon(pm);
  }
  return cached;
}

QIcon URDFHierarchyTree::jointRevoluteIcon() const {
  static QIcon cached;
  if (cached.isNull()) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(255, 165, 0));  // Orange
    p.setPen(Qt::NoPen);
    // Draw a rotation arrow shape
    p.drawEllipse(3, 3, 10, 10);
    p.setBrush(Qt::white);
    p.drawEllipse(5, 5, 6, 6);
    cached = QIcon(pm);
  }
  return cached;
}

QIcon URDFHierarchyTree::jointPrismaticIcon() const {
  static QIcon cached;
  if (cached.isNull()) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(50, 205, 50));  // Lime green
    p.setPen(Qt::NoPen);
    // Draw an arrow shape for linear motion
    p.drawRect(4, 5, 8, 6);
    cached = QIcon(pm);
  }
  return cached;
}

QIcon URDFHierarchyTree::jointFixedIcon() const {
  static QIcon cached;
  if (cached.isNull()) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(128, 128, 128));  // Gray
    p.setPen(Qt::NoPen);
    p.drawRect(4, 4, 8, 8);
    cached = QIcon(pm);
  }
  return cached;
}

QIcon URDFHierarchyTree::jointContinuousIcon() const {
  static QIcon cached;
  if (cached.isNull()) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(138, 43, 226));  // Blue-violet
    p.setPen(Qt::NoPen);
    p.drawEllipse(2, 2, 12, 12);
    cached = QIcon(pm);
  }
  return cached;
}

}  // namespace ros_weaver
