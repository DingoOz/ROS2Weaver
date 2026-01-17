#include "ros_weaver/widgets/urdf_tree_view.hpp"

#include <QHeaderView>
#include <QMouseEvent>
#include <QContextMenuEvent>
#include <QPainter>
#include <QApplication>
#include <QDebug>

namespace ros_weaver {

URDFTreeView::URDFTreeView(QWidget* parent)
  : QTreeWidget(parent) {
  setupUi();

  connect(this, &QTreeWidget::itemSelectionChanged,
          this, &URDFTreeView::onItemSelectionChanged);
}

void URDFTreeView::setupUi() {
  // Set up columns
  setColumnCount(3);
  setHeaderLabels({tr("Name"), tr("Type"), tr("Parent Link")});

  // Configure appearance
  setAlternatingRowColors(true);
  setSelectionMode(QAbstractItemView::ExtendedSelection);
  setSelectionBehavior(QAbstractItemView::SelectRows);
  setExpandsOnDoubleClick(false);  // We handle double-click ourselves
  setRootIsDecorated(true);
  setAnimated(true);

  // Column sizing
  header()->setStretchLastSection(true);
  header()->setSectionResizeMode(COL_NAME, QHeaderView::Interactive);
  header()->setSectionResizeMode(COL_TYPE, QHeaderView::ResizeToContents);
  header()->setSectionResizeMode(COL_PARENT, QHeaderView::Stretch);
  setColumnWidth(COL_NAME, 200);

  // Create icons (simple colored circles for now)
  // In a full implementation, these would be loaded from resources
  QPixmap jointPix(16, 16);
  jointPix.fill(Qt::transparent);
  QPainter painter(&jointPix);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(QColor(100, 150, 255));
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(2, 2, 12, 12);
  jointIcon_ = QIcon(jointPix);

  QPixmap linkPix(16, 16);
  linkPix.fill(Qt::transparent);
  QPainter linkPainter(&linkPix);
  linkPainter.setRenderHint(QPainter::Antialiasing);
  linkPainter.setBrush(QColor(150, 200, 100));
  linkPainter.setPen(Qt::NoPen);
  linkPainter.drawRect(2, 2, 12, 12);
  linkIcon_ = QIcon(linkPix);

  // Joint type icons
  QPixmap fixedPix(16, 16);
  fixedPix.fill(Qt::transparent);
  QPainter fixedPainter(&fixedPix);
  fixedPainter.setRenderHint(QPainter::Antialiasing);
  fixedPainter.setBrush(QColor(150, 150, 150));
  fixedPainter.drawEllipse(2, 2, 12, 12);
  fixedJointIcon_ = QIcon(fixedPix);

  QPixmap revolutePix(16, 16);
  revolutePix.fill(Qt::transparent);
  QPainter revolutePainter(&revolutePix);
  revolutePainter.setRenderHint(QPainter::Antialiasing);
  revolutePainter.setBrush(QColor(255, 150, 50));
  revolutePainter.drawEllipse(2, 2, 12, 12);
  revoluteJointIcon_ = QIcon(revolutePix);

  QPixmap prismaticPix(16, 16);
  prismaticPix.fill(Qt::transparent);
  QPainter prismaticPainter(&prismaticPix);
  prismaticPainter.setRenderHint(QPainter::Antialiasing);
  prismaticPainter.setBrush(QColor(50, 200, 255));
  prismaticPainter.drawEllipse(2, 2, 12, 12);
  prismaticJointIcon_ = QIcon(prismaticPix);

  QPixmap continuousPix(16, 16);
  continuousPix.fill(Qt::transparent);
  QPainter continuousPainter(&continuousPix);
  continuousPainter.setRenderHint(QPainter::Antialiasing);
  continuousPainter.setBrush(QColor(200, 100, 255));
  continuousPainter.drawEllipse(2, 2, 12, 12);
  continuousJointIcon_ = QIcon(continuousPix);
}

void URDFTreeView::setModel(const URDFModel& model) {
  blockSelectionSignals_ = true;
  clear();
  jointItems_.clear();
  linkItems_.clear();

  buildTree(model);

  blockSelectionSignals_ = false;
  expandAll();
}

void URDFTreeView::buildTree(const URDFModel& model) {
  if (model.rootLink.isEmpty()) return;

  // Start building from root link
  buildTreeRecursive(model.rootLink, nullptr, model);
}

void URDFTreeView::buildTreeRecursive(const QString& linkName, QTreeWidgetItem* parent,
                                       const URDFModel& model) {
  // Find the link
  const URDFLink* link = nullptr;
  for (const URDFLink& l : model.links) {
    if (l.name == linkName) {
      link = &l;
      break;
    }
  }

  if (!link) return;

  // Create link item
  QTreeWidgetItem* linkItem;
  if (parent) {
    linkItem = new QTreeWidgetItem(parent);
  } else {
    linkItem = new QTreeWidgetItem(this);
  }

  populateLinkItem(linkItem, *link);
  linkItems_[link->name] = linkItem;

  // Find child joints (joints where this link is the parent)
  for (const URDFJoint& joint : model.joints) {
    if (joint.parentLink == linkName) {
      // Create joint item as child of link
      QTreeWidgetItem* jointItem = new QTreeWidgetItem(linkItem);
      populateJointItem(jointItem, joint);
      jointItems_[joint.name] = jointItem;

      // Recurse to child link
      buildTreeRecursive(joint.childLink, jointItem, model);
    }
  }
}

void URDFTreeView::populateLinkItem(QTreeWidgetItem* item, const URDFLink& link) {
  item->setText(COL_NAME, link.name);
  item->setText(COL_TYPE, "link");
  item->setIcon(COL_NAME, linkIcon_);
  item->setData(COL_NAME, Qt::UserRole, "link");
  item->setData(COL_NAME, Qt::UserRole + 1, link.name);

  // Tooltip with details
  QString tooltip = QString("Link: %1").arg(link.name);
  if (!link.meshPath.isEmpty()) {
    tooltip += QString("\nMesh: %1").arg(link.meshPath);
  }
  if (link.mass > 0) {
    tooltip += QString("\nMass: %1 kg").arg(link.mass);
  }
  item->setToolTip(COL_NAME, tooltip);
}

void URDFTreeView::populateJointItem(QTreeWidgetItem* item, const URDFJoint& joint) {
  item->setText(COL_NAME, joint.name);
  item->setText(COL_TYPE, joint.type);
  item->setText(COL_PARENT, joint.parentLink);
  item->setData(COL_NAME, Qt::UserRole, "joint");
  item->setData(COL_NAME, Qt::UserRole + 1, joint.name);

  // Select appropriate icon based on joint type
  if (joint.type == "fixed") {
    item->setIcon(COL_NAME, fixedJointIcon_);
  } else if (joint.type == "revolute") {
    item->setIcon(COL_NAME, revoluteJointIcon_);
  } else if (joint.type == "prismatic") {
    item->setIcon(COL_NAME, prismaticJointIcon_);
  } else if (joint.type == "continuous") {
    item->setIcon(COL_NAME, continuousJointIcon_);
  } else {
    item->setIcon(COL_NAME, jointIcon_);
  }

  // Tooltip with details
  QString tooltip = QString("Joint: %1\nType: %2\nParent: %3\nChild: %4")
                        .arg(joint.name, joint.type, joint.parentLink, joint.childLink);

  if (joint.type == "revolute" || joint.type == "prismatic") {
    tooltip += QString("\nLimits: [%1, %2]").arg(joint.lowerLimit).arg(joint.upperLimit);
  }

  tooltip += QString("\nAxis: (%1, %2, %3)")
                 .arg(joint.axis.x())
                 .arg(joint.axis.y())
                 .arg(joint.axis.z());

  item->setToolTip(COL_NAME, tooltip);
}

void URDFTreeView::updateJoint(const QString& jointName, const URDFJoint& joint) {
  QTreeWidgetItem* item = findJointItem(jointName);
  if (item) {
    populateJointItem(item, joint);
  }
}

QTreeWidgetItem* URDFTreeView::findJointItem(const QString& jointName) const {
  return jointItems_.value(jointName, nullptr);
}

QTreeWidgetItem* URDFTreeView::findLinkItem(const QString& linkName) const {
  return linkItems_.value(linkName, nullptr);
}

void URDFTreeView::selectJoint(const QString& jointName, bool addToSelection) {
  blockSelectionSignals_ = true;

  if (!addToSelection) {
    clearSelection();
  }

  QTreeWidgetItem* item = findJointItem(jointName);
  if (item) {
    item->setSelected(true);
    scrollToItem(item);
  }

  blockSelectionSignals_ = false;
}

void URDFTreeView::selectJoints(const QStringList& jointNames) {
  blockSelectionSignals_ = true;
  clearSelection();

  for (const QString& name : jointNames) {
    QTreeWidgetItem* item = findJointItem(name);
    if (item) {
      item->setSelected(true);
    }
  }

  blockSelectionSignals_ = false;
}

void URDFTreeView::clearJointSelection() {
  blockSelectionSignals_ = true;
  clearSelection();
  blockSelectionSignals_ = false;
}

QStringList URDFTreeView::selectedJointNames() const {
  QStringList names;
  for (QTreeWidgetItem* item : selectedItems()) {
    QString type = item->data(COL_NAME, Qt::UserRole).toString();
    if (type == "joint") {
      names.append(item->data(COL_NAME, Qt::UserRole + 1).toString());
    }
  }
  return names;
}

void URDFTreeView::expandToJoint(const QString& jointName) {
  QTreeWidgetItem* item = findJointItem(jointName);
  if (item) {
    // Expand all ancestors
    QTreeWidgetItem* parent = item->parent();
    while (parent) {
      parent->setExpanded(true);
      parent = parent->parent();
    }
    scrollToItem(item);
  }
}

void URDFTreeView::onItemSelectionChanged() {
  if (blockSelectionSignals_) return;

  QStringList selectedJoints = selectedJointNames();
  emit selectionChanged(selectedJoints);
}

void URDFTreeView::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) {
  QTreeWidget::selectionChanged(selected, deselected);

  if (blockSelectionSignals_) return;

  // Emit signal for first selected joint
  QList<QTreeWidgetItem*> items = selectedItems();
  for (QTreeWidgetItem* item : items) {
    QString type = item->data(COL_NAME, Qt::UserRole).toString();
    if (type == "joint") {
      QString jointName = item->data(COL_NAME, Qt::UserRole + 1).toString();
      bool ctrlPressed = QApplication::keyboardModifiers() & Qt::ControlModifier;
      emit jointSelected(jointName, ctrlPressed);
      break;
    }
  }
}

void URDFTreeView::mouseDoubleClickEvent(QMouseEvent* event) {
  QTreeWidgetItem* item = itemAt(event->pos());
  if (item) {
    QString type = item->data(COL_NAME, Qt::UserRole).toString();
    if (type == "joint") {
      QString jointName = item->data(COL_NAME, Qt::UserRole + 1).toString();
      emit jointDoubleClicked(jointName);
      return;
    }
  }

  QTreeWidget::mouseDoubleClickEvent(event);
}

void URDFTreeView::contextMenuEvent(QContextMenuEvent* event) {
  QTreeWidgetItem* item = itemAt(event->pos());
  if (item) {
    QString type = item->data(COL_NAME, Qt::UserRole).toString();
    if (type == "joint") {
      QString jointName = item->data(COL_NAME, Qt::UserRole + 1).toString();
      emit jointContextMenuRequested(jointName, event->globalPos());
      return;
    }
  }

  QTreeWidget::contextMenuEvent(event);
}

}  // namespace ros_weaver
