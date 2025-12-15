#include "ros_weaver/widgets/topic_selector_widget.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QInputDialog>
#include <QLabel>

namespace ros_weaver {

TopicSelectorWidget::TopicSelectorWidget(QWidget* parent)
    : QWidget(parent) {
  setupUi();
  setupConnections();
}

void TopicSelectorWidget::setBagManager(BagManager* manager) {
  if (bagManager_) {
    disconnect(bagManager_, nullptr, this, nullptr);
  }

  bagManager_ = manager;

  if (bagManager_) {
    connect(bagManager_, &BagManager::bagOpened, this, &TopicSelectorWidget::onBagOpened);
    connect(bagManager_, &BagManager::bagClosed, this, &TopicSelectorWidget::onBagClosed);

    // If bag is already open, populate
    if (bagManager_->isOpen()) {
      onBagOpened(bagManager_->metadata());
    }
  }
}

void TopicSelectorWidget::selectAllTopics() {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topicTree_->topLevelItem(i);
    if (!item->isHidden()) {
      item->setCheckState(ColEnabled, Qt::Checked);
    }
  }
}

void TopicSelectorWidget::deselectAllTopics() {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topicTree_->topLevelItem(i);
    if (!item->isHidden()) {
      item->setCheckState(ColEnabled, Qt::Unchecked);
    }
  }
}

void TopicSelectorWidget::invertSelection() {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topicTree_->topLevelItem(i);
    if (!item->isHidden()) {
      Qt::CheckState newState = (item->checkState(ColEnabled) == Qt::Checked)
          ? Qt::Unchecked : Qt::Checked;
      item->setCheckState(ColEnabled, newState);
    }
  }
}

QStringList TopicSelectorWidget::selectedTopics() const {
  QStringList selected;
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topicTree_->topLevelItem(i);
    if (item->checkState(ColEnabled) == Qt::Checked) {
      selected.append(item->text(ColTopic));
    }
  }
  return selected;
}

void TopicSelectorWidget::setFilter(const QString& filter) {
  filterEdit_->setText(filter);
}

void TopicSelectorWidget::onBagOpened(const BagMetadata& metadata) {
  Q_UNUSED(metadata)
  populateTopicTree();
}

void TopicSelectorWidget::onBagClosed() {
  topicTree_->clear();
  summaryLabel_->setText(tr("No bag loaded"));
}

void TopicSelectorWidget::onFilterChanged(const QString& text) {
  Q_UNUSED(text)
  applyFilter();
}

void TopicSelectorWidget::onSelectAllClicked() {
  selectAllTopics();
}

void TopicSelectorWidget::onDeselectAllClicked() {
  deselectAllTopics();
}

void TopicSelectorWidget::onInvertClicked() {
  invertSelection();
}

void TopicSelectorWidget::onItemChanged(QTreeWidgetItem* item, int column) {
  if (column == ColEnabled && bagManager_) {
    QString topic = item->text(ColTopic);
    bool enabled = (item->checkState(ColEnabled) == Qt::Checked);
    bagManager_->setTopicEnabled(topic, enabled);
    emit topicSelectionChanged(topic, enabled);

    // Update summary
    int selectedCount = selectedTopics().size();
    int totalCount = topicTree_->topLevelItemCount();
    summaryLabel_->setText(tr("%1 / %2 topics selected").arg(selectedCount).arg(totalCount));
  }
}

void TopicSelectorWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column) {
  if (column == ColRemap && bagManager_) {
    QString originalTopic = item->text(ColTopic);
    QString currentRemap = item->text(ColRemap);

    bool ok;
    QString newTopic = QInputDialog::getText(this, tr("Remap Topic"),
                                             tr("Remap %1 to:").arg(originalTopic),
                                             QLineEdit::Normal,
                                             currentRemap.isEmpty() ? originalTopic : currentRemap,
                                             &ok);
    if (ok && !newTopic.isEmpty()) {
      item->setText(ColRemap, newTopic);
      bagManager_->setTopicRemap(originalTopic, newTopic);
      emit topicRemapChanged(originalTopic, newTopic);
    }
  }
}

void TopicSelectorWidget::onContextMenuRequested(const QPoint& pos) {
  QTreeWidgetItem* item = topicTree_->itemAt(pos);
  if (!item) return;

  QString topic = item->text(ColTopic);

  QMenu menu(this);

  QAction* remapAction = menu.addAction(tr("Remap Topic..."));
  connect(remapAction, &QAction::triggered, this, [this, item]() {
    onItemDoubleClicked(item, ColRemap);
  });

  menu.addSeparator();

  QAction* plotAction = menu.addAction(tr("Plot Numeric Fields..."));
  connect(plotAction, &QAction::triggered, this, [this, topic]() {
    emit plotFieldRequested(topic, "");
  });

  menu.addSeparator();

  QAction* selectSimilarAction = menu.addAction(tr("Select Similar Types"));
  connect(selectSimilarAction, &QAction::triggered, this, [this, item]() {
    QString targetType = item->text(ColType);
    for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
      QTreeWidgetItem* it = topicTree_->topLevelItem(i);
      if (it->text(ColType) == targetType) {
        it->setCheckState(ColEnabled, Qt::Checked);
      }
    }
  });

  menu.exec(topicTree_->viewport()->mapToGlobal(pos));
}

void TopicSelectorWidget::setupUi() {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(5);

  // Filter bar
  auto* filterLayout = new QHBoxLayout();
  filterLayout->addWidget(new QLabel(tr("Filter:"), this));

  filterEdit_ = new QLineEdit(this);
  filterEdit_->setPlaceholderText(tr("Filter topics..."));
  filterEdit_->setClearButtonEnabled(true);
  filterLayout->addWidget(filterEdit_, 1);

  layout->addLayout(filterLayout);

  // Selection buttons
  auto* buttonLayout = new QHBoxLayout();

  selectAllButton_ = new QPushButton(tr("Select All"), this);
  buttonLayout->addWidget(selectAllButton_);

  deselectAllButton_ = new QPushButton(tr("Deselect All"), this);
  buttonLayout->addWidget(deselectAllButton_);

  invertButton_ = new QPushButton(tr("Invert"), this);
  buttonLayout->addWidget(invertButton_);

  buttonLayout->addStretch();
  layout->addLayout(buttonLayout);

  // Topic tree
  topicTree_ = new QTreeWidget(this);
  topicTree_->setHeaderLabels({tr(""), tr("Topic"), tr("Type"), tr("Messages"), tr("Remap")});
  topicTree_->setColumnWidth(ColEnabled, 30);
  topicTree_->setColumnWidth(ColTopic, 200);
  topicTree_->setColumnWidth(ColType, 150);
  topicTree_->setColumnWidth(ColMessageCount, 80);
  topicTree_->setColumnWidth(ColRemap, 150);
  topicTree_->setAlternatingRowColors(true);
  topicTree_->setContextMenuPolicy(Qt::CustomContextMenu);
  topicTree_->header()->setStretchLastSection(true);
  layout->addWidget(topicTree_, 1);

  // Summary label
  summaryLabel_ = new QLabel(tr("No bag loaded"), this);
  summaryLabel_->setStyleSheet("color: gray;");
  layout->addWidget(summaryLabel_);
}

void TopicSelectorWidget::setupConnections() {
  connect(filterEdit_, &QLineEdit::textChanged, this, &TopicSelectorWidget::onFilterChanged);
  connect(selectAllButton_, &QPushButton::clicked, this, &TopicSelectorWidget::onSelectAllClicked);
  connect(deselectAllButton_, &QPushButton::clicked, this, &TopicSelectorWidget::onDeselectAllClicked);
  connect(invertButton_, &QPushButton::clicked, this, &TopicSelectorWidget::onInvertClicked);
  connect(topicTree_, &QTreeWidget::itemChanged, this, &TopicSelectorWidget::onItemChanged);
  connect(topicTree_, &QTreeWidget::itemDoubleClicked, this, &TopicSelectorWidget::onItemDoubleClicked);
  connect(topicTree_, &QTreeWidget::customContextMenuRequested,
          this, &TopicSelectorWidget::onContextMenuRequested);
}

void TopicSelectorWidget::populateTopicTree() {
  topicTree_->clear();

  if (!bagManager_) return;

  QList<TopicConfig> configs = bagManager_->topicConfigs();

  for (const auto& config : configs) {
    QTreeWidgetItem* item = createTopicItem(config);
    topicTree_->addTopLevelItem(item);
  }

  int totalCount = topicTree_->topLevelItemCount();
  summaryLabel_->setText(tr("%1 / %1 topics selected").arg(totalCount));

  applyFilter();
}

void TopicSelectorWidget::applyFilter() {
  QString filter = filterEdit_->text().toLower();

  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = topicTree_->topLevelItem(i);
    bool visible = filter.isEmpty() ||
        item->text(ColTopic).toLower().contains(filter) ||
        item->text(ColType).toLower().contains(filter);
    item->setHidden(!visible);
  }
}

QTreeWidgetItem* TopicSelectorWidget::createTopicItem(const TopicConfig& config) {
  auto* item = new QTreeWidgetItem();

  item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
  item->setCheckState(ColEnabled, config.enabled ? Qt::Checked : Qt::Unchecked);
  item->setText(ColTopic, config.originalName);
  item->setText(ColType, config.messageType);
  item->setText(ColMessageCount, formatMessageCount(config.messageCount));

  if (config.remappedName != config.originalName) {
    item->setText(ColRemap, config.remappedName);
  }

  // Tooltip with full info
  QString tooltip = QString("Topic: %1\nType: %2\nMessages: %3\nQoS: %4")
      .arg(config.originalName)
      .arg(config.messageType)
      .arg(config.messageCount)
      .arg(config.qosReliable ? "Reliable" : "Best Effort");
  item->setToolTip(ColTopic, tooltip);

  return item;
}

QString TopicSelectorWidget::formatMessageCount(int64_t count) const {
  if (count >= 1000000) {
    return QString("%1M").arg(count / 1000000.0, 0, 'f', 1);
  } else if (count >= 1000) {
    return QString("%1K").arg(count / 1000.0, 0, 'f', 1);
  }
  return QString::number(count);
}

}  // namespace ros_weaver
