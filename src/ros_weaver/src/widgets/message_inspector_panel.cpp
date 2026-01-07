#include "ros_weaver/widgets/message_inspector_panel.hpp"
#include "ros_weaver/core/generic_message_handler.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QClipboard>
#include <QApplication>
#include <QMessageBox>
#include <QMenu>
#include <QAction>

#include <iostream>

namespace ros_weaver {

MessageInspectorPanel::MessageInspectorPanel(QWidget* parent)
  : QWidget(parent)
  , topicSelector_(nullptr)
  , refreshTopicsButton_(nullptr)
  , inspectButton_(nullptr)
  , messageTreeView_(nullptr)
  , currentTopicLabel_(nullptr)
  , messageTypeLabel_(nullptr)
  , rateLabel_(nullptr)
  , lastReceivedLabel_(nullptr)
  , pauseButton_(nullptr)
  , clearButton_(nullptr)
  , copyButton_(nullptr)
  , bufferSizeSpinBox_(nullptr)
  , messageHistoryList_(nullptr)
  , publisherGroup_(nullptr)
  , jsonEditor_(nullptr)
  , loadSchemaButton_(nullptr)
  , publishButton_(nullptr)
  , mainSplitter_(nullptr)
  , viewSplitter_(nullptr)
  , messageHandler_(nullptr)
  , canvas_(nullptr)
  , rosNode_(nullptr)
  , spinning_(false)
  , isPaused_(false)
  , maxBufferSize_(100)
  , rateTimer_(nullptr)
  , messageCountSinceLastUpdate_(0)
  , currentRate_(0.0)
{
  setupUi();
  setupConnections();
  initializeRosNode();

  // Rate calculation timer
  rateTimer_ = new QTimer(this);
  connect(rateTimer_, &QTimer::timeout, this, &MessageInspectorPanel::updateRateDisplay);
  rateTimer_->start(1000);  // Update rate every second
}

MessageInspectorPanel::~MessageInspectorPanel() {
  stopInspecting();
  shutdownRosNode();
}

void MessageInspectorPanel::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void MessageInspectorPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Topic selection toolbar
  QHBoxLayout* topicBarLayout = new QHBoxLayout();
  topicBarLayout->setSpacing(4);

  topicSelector_ = new QComboBox();
  topicSelector_->setMinimumWidth(200);
  topicSelector_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  topicSelector_->setPlaceholderText(tr("Select a topic..."));
  topicBarLayout->addWidget(topicSelector_);

  refreshTopicsButton_ = new QPushButton(tr("Refresh"));
  refreshTopicsButton_->setToolTip(tr("Refresh topic list"));
  refreshTopicsButton_->setMaximumWidth(70);
  topicBarLayout->addWidget(refreshTopicsButton_);

  inspectButton_ = new QToolButton();
  inspectButton_->setText(tr("Inspect"));
  inspectButton_->setCheckable(true);
  inspectButton_->setToolTip(tr("Start/stop inspecting selected topic"));
  topicBarLayout->addWidget(inspectButton_);

  mainLayout->addLayout(topicBarLayout);

  // Info bar
  QHBoxLayout* infoBarLayout = new QHBoxLayout();
  infoBarLayout->setSpacing(8);

  currentTopicLabel_ = new QLabel(tr("Topic: <none>"));
  currentTopicLabel_->setStyleSheet("font-weight: bold;");
  infoBarLayout->addWidget(currentTopicLabel_);

  messageTypeLabel_ = new QLabel();
  messageTypeLabel_->setStyleSheet("color: gray;");
  infoBarLayout->addWidget(messageTypeLabel_);

  infoBarLayout->addStretch();

  rateLabel_ = new QLabel(tr("Rate: --"));
  rateLabel_->setMinimumWidth(80);
  infoBarLayout->addWidget(rateLabel_);

  lastReceivedLabel_ = new QLabel(tr("Last: --"));
  lastReceivedLabel_->setMinimumWidth(100);
  infoBarLayout->addWidget(lastReceivedLabel_);

  mainLayout->addLayout(infoBarLayout);

  // Main splitter (vertical: view area / publisher)
  mainSplitter_ = new QSplitter(Qt::Vertical);

  // View splitter (horizontal: message tree / history)
  viewSplitter_ = new QSplitter(Qt::Horizontal);

  // Message tree view
  QWidget* treeContainer = new QWidget();
  QVBoxLayout* treeLayout = new QVBoxLayout(treeContainer);
  treeLayout->setContentsMargins(0, 0, 0, 0);
  treeLayout->setSpacing(2);

  // Tree controls
  QHBoxLayout* treeControlsLayout = new QHBoxLayout();

  pauseButton_ = new QToolButton();
  pauseButton_->setText(tr("Pause"));
  pauseButton_->setCheckable(true);
  pauseButton_->setToolTip(tr("Pause message reception"));
  treeControlsLayout->addWidget(pauseButton_);

  clearButton_ = new QPushButton(tr("Clear"));
  clearButton_->setToolTip(tr("Clear message view and history"));
  treeControlsLayout->addWidget(clearButton_);

  copyButton_ = new QPushButton(tr("Copy"));
  copyButton_->setToolTip(tr("Copy current message as JSON"));
  treeControlsLayout->addWidget(copyButton_);

  treeControlsLayout->addStretch();

  QLabel* bufferLabel = new QLabel(tr("Buffer:"));
  treeControlsLayout->addWidget(bufferLabel);

  bufferSizeSpinBox_ = new QSpinBox();
  bufferSizeSpinBox_->setRange(10, 1000);
  bufferSizeSpinBox_->setValue(maxBufferSize_);
  bufferSizeSpinBox_->setSuffix(tr(" msgs"));
  bufferSizeSpinBox_->setToolTip(tr("Maximum messages to keep in history"));
  treeControlsLayout->addWidget(bufferSizeSpinBox_);

  treeLayout->addLayout(treeControlsLayout);

  messageTreeView_ = new QTreeWidget();
  messageTreeView_->setHeaderLabels({tr("Field"), tr("Value"), tr("Type")});
  messageTreeView_->setAlternatingRowColors(true);
  messageTreeView_->setRootIsDecorated(true);
  messageTreeView_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  messageTreeView_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  messageTreeView_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  messageTreeView_->setContextMenuPolicy(Qt::CustomContextMenu);
  treeLayout->addWidget(messageTreeView_);

  viewSplitter_->addWidget(treeContainer);

  // Message history
  QWidget* historyContainer = new QWidget();
  QVBoxLayout* historyLayout = new QVBoxLayout(historyContainer);
  historyLayout->setContentsMargins(0, 0, 0, 0);
  historyLayout->setSpacing(2);

  QLabel* historyLabel = new QLabel(tr("History"));
  historyLabel->setStyleSheet("font-weight: bold;");
  historyLayout->addWidget(historyLabel);

  messageHistoryList_ = new QListWidget();
  messageHistoryList_->setToolTip(tr("Click to view historical message"));
  historyLayout->addWidget(messageHistoryList_);

  viewSplitter_->addWidget(historyContainer);
  viewSplitter_->setStretchFactor(0, 3);
  viewSplitter_->setStretchFactor(1, 1);

  mainSplitter_->addWidget(viewSplitter_);

  // Publisher section
  publisherGroup_ = new QGroupBox(tr("Publish Message"));
  QVBoxLayout* publisherLayout = new QVBoxLayout(publisherGroup_);
  publisherLayout->setSpacing(4);

  QHBoxLayout* publishControlsLayout = new QHBoxLayout();

  loadSchemaButton_ = new QPushButton(tr("Load Schema"));
  loadSchemaButton_->setToolTip(tr("Load default message template"));
  publishControlsLayout->addWidget(loadSchemaButton_);

  publishControlsLayout->addStretch();

  publishButton_ = new QPushButton(tr("Publish"));
  publishButton_->setToolTip(tr("Publish the message to the topic"));
  publishButton_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
  publishControlsLayout->addWidget(publishButton_);

  publisherLayout->addLayout(publishControlsLayout);

  jsonEditor_ = new QTextEdit();
  jsonEditor_->setPlaceholderText(tr("Enter message JSON here...\n{\n  \"field\": \"value\"\n}"));
  jsonEditor_->setFont(QFont("Monospace", 9));
  jsonEditor_->setMinimumHeight(80);
  publisherLayout->addWidget(jsonEditor_);

  mainSplitter_->addWidget(publisherGroup_);
  mainSplitter_->setStretchFactor(0, 3);
  mainSplitter_->setStretchFactor(1, 1);

  mainLayout->addWidget(mainSplitter_);

  updateUiState();
}

void MessageInspectorPanel::setupConnections() {
  connect(topicSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MessageInspectorPanel::onTopicSelected);
  connect(refreshTopicsButton_, &QPushButton::clicked,
          this, &MessageInspectorPanel::onRefreshClicked);
  connect(inspectButton_, &QToolButton::toggled, this, [this](bool checked) {
    if (checked) {
      int idx = topicSelector_->currentIndex();
      if (idx >= 0) {
        QString topic = topicSelector_->currentData().toString();
        QString type = topicTypes_.value(topic);
        if (!topic.isEmpty() && !type.isEmpty()) {
          inspectTopic(topic, type);
        }
      }
    } else {
      stopInspecting();
    }
  });

  connect(pauseButton_, &QToolButton::toggled,
          this, &MessageInspectorPanel::onPauseToggled);
  connect(clearButton_, &QPushButton::clicked,
          this, &MessageInspectorPanel::onClearClicked);
  connect(copyButton_, &QPushButton::clicked,
          this, &MessageInspectorPanel::onCopyMessageClicked);
  connect(bufferSizeSpinBox_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &MessageInspectorPanel::onBufferSizeChanged);

  connect(messageHistoryList_, &QListWidget::itemClicked,
          this, &MessageInspectorPanel::onHistoryItemClicked);

  connect(loadSchemaButton_, &QPushButton::clicked,
          this, &MessageInspectorPanel::onLoadSchemaClicked);
  connect(publishButton_, &QPushButton::clicked,
          this, &MessageInspectorPanel::onPublishClicked);

  // Context menu for tree
  connect(messageTreeView_, &QTreeWidget::customContextMenuRequested,
          this, [this](const QPoint& pos) {
    QMenu menu;
    QAction* copyFieldAction = menu.addAction(tr("Copy Field Value"));
    QAction* copyPathAction = menu.addAction(tr("Copy Field Path"));

    QAction* selected = menu.exec(messageTreeView_->viewport()->mapToGlobal(pos));
    QTreeWidgetItem* item = messageTreeView_->itemAt(pos);

    if (item) {
      if (selected == copyFieldAction) {
        QApplication::clipboard()->setText(item->text(1));
      } else if (selected == copyPathAction) {
        // Build path from root
        QStringList path;
        QTreeWidgetItem* current = item;
        while (current) {
          path.prepend(current->text(0));
          current = current->parent();
        }
        QApplication::clipboard()->setText(path.join("."));
      }
    }
  });
}

void MessageInspectorPanel::initializeRosNode() {
  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rosNode_ = std::make_shared<rclcpp::Node>("ros_weaver_message_inspector");

    messageHandler_ = new GenericMessageHandler(this);
    messageHandler_->setRosNode(rosNode_);

    connect(messageHandler_, &GenericMessageHandler::messageReceived,
            this, &MessageInspectorPanel::onMessageReceived);
    connect(messageHandler_, &GenericMessageHandler::subscriptionError,
            this, [this](const QString& topic, const QString& error) {
      QMessageBox::warning(this, tr("Subscription Error"),
                           tr("Failed to subscribe to %1:\n%2").arg(topic, error));
      inspectButton_->setChecked(false);
    });
    connect(messageHandler_, &GenericMessageHandler::publishSuccess,
            this, [this](const QString& topic) {
      lastReceivedLabel_->setText(tr("Published to %1").arg(topic));
    });
    connect(messageHandler_, &GenericMessageHandler::publishError,
            this, [this](const QString& topic, const QString& error) {
      QMessageBox::warning(this, tr("Publish Error"),
                           tr("Failed to publish to %1:\n%2").arg(topic, error));
    });

    // Start spinner thread
    spinning_ = true;
    spinThread_ = std::thread([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    // Initial topic refresh
    QTimer::singleShot(500, this, &MessageInspectorPanel::refreshTopics);

  } catch (const std::exception& e) {
    std::cerr << "MessageInspectorPanel: Failed to init ROS: " << e.what() << std::endl;
  }
}

void MessageInspectorPanel::shutdownRosNode() {
  spinning_ = false;
  if (spinThread_.joinable()) {
    spinThread_.join();
  }

  if (messageHandler_) {
    messageHandler_->unsubscribeAll();
  }

  rosNode_.reset();
}

void MessageInspectorPanel::inspectTopic(const QString& topicName,
                                          const QString& messageType) {
  // Stop any existing inspection
  if (!currentTopic_.isEmpty()) {
    messageHandler_->unsubscribeFromTopic(currentTopic_);
  }

  currentTopic_ = topicName;
  currentMessageType_ = messageType;

  // Update UI
  currentTopicLabel_->setText(tr("Topic: %1").arg(topicName));
  messageTypeLabel_->setText(tr("[%1]").arg(messageType));

  // Clear previous data
  onClearClicked();

  // Subscribe
  if (messageHandler_->subscribeToTopic(topicName, messageType)) {
    inspectButton_->setChecked(true);
    updateUiState();
  }
}

void MessageInspectorPanel::stopInspecting() {
  if (!currentTopic_.isEmpty()) {
    messageHandler_->unsubscribeFromTopic(currentTopic_);
  }

  currentTopic_.clear();
  currentMessageType_.clear();

  currentTopicLabel_->setText(tr("Topic: <none>"));
  messageTypeLabel_->clear();
  rateLabel_->setText(tr("Rate: --"));
  lastReceivedLabel_->setText(tr("Last: --"));

  inspectButton_->setChecked(false);
  updateUiState();
}

void MessageInspectorPanel::refreshTopics() {
  if (!messageHandler_) return;

  topicSelector_->clear();
  topicTypes_.clear();

  auto topics = messageHandler_->getAvailableTopics();

  for (auto it = topics.begin(); it != topics.end(); ++it) {
    QString displayName = QString("%1 [%2]").arg(it.key(), it.value());
    topicSelector_->addItem(displayName, it.key());
    topicTypes_[it.key()] = it.value();
  }

  topicSelector_->setCurrentIndex(-1);
}

void MessageInspectorPanel::onTopicSelected(int index) {
  Q_UNUSED(index);
  updateUiState();
}

void MessageInspectorPanel::onRefreshClicked() {
  refreshTopics();
}

void MessageInspectorPanel::onPauseToggled(bool paused) {
  isPaused_ = paused;
  pauseButton_->setText(paused ? tr("Resume") : tr("Pause"));
}

void MessageInspectorPanel::onClearClicked() {
  messageTreeView_->clear();
  messageHistoryList_->clear();
  messageBuffer_.clear();
  messageCountSinceLastUpdate_ = 0;
  currentRate_ = 0.0;
  rateLabel_->setText(tr("Rate: --"));
  lastReceivedLabel_->setText(tr("Last: --"));
}

void MessageInspectorPanel::onMessageReceived(const QString& topicName,
                                               const QJsonObject& message,
                                               qint64 timestamp) {
  if (topicName != currentTopic_ || isPaused_) {
    return;
  }

  messageCountSinceLastUpdate_++;

  // Update last received time
  lastReceivedLabel_->setText(tr("Last: %1").arg(formatTimestamp(timestamp)));

  // Add to history
  addToHistory(message, timestamp);

  // Update tree view
  messageTreeView_->clear();
  populateTreeFromJson(nullptr, message);
  messageTreeView_->expandAll();
}

void MessageInspectorPanel::onHistoryItemClicked(QListWidgetItem* item) {
  int index = messageHistoryList_->row(item);
  if (index >= 0 && index < messageBuffer_.size()) {
    messageTreeView_->clear();
    populateTreeFromJson(nullptr, messageBuffer_[index].second);
    messageTreeView_->expandAll();
  }
}

void MessageInspectorPanel::onLoadSchemaClicked() {
  if (currentMessageType_.isEmpty()) {
    QMessageBox::information(this, tr("No Topic Selected"),
                             tr("Please select and inspect a topic first."));
    return;
  }

  QJsonObject schema = messageHandler_->getMessageSchema(currentMessageType_);
  QJsonDocument doc(schema);
  jsonEditor_->setPlainText(doc.toJson(QJsonDocument::Indented));
}

void MessageInspectorPanel::onPublishClicked() {
  if (currentTopic_.isEmpty()) {
    QMessageBox::information(this, tr("No Topic Selected"),
                             tr("Please select a topic to publish to."));
    return;
  }

  QString json = jsonEditor_->toPlainText().trimmed();
  if (json.isEmpty()) {
    QMessageBox::warning(this, tr("Empty Message"),
                         tr("Please enter a JSON message to publish."));
    return;
  }

  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8(), &error);

  if (error.error != QJsonParseError::NoError) {
    QMessageBox::warning(this, tr("Invalid JSON"),
                         tr("JSON parse error at position %1:\n%2")
                             .arg(error.offset)
                             .arg(error.errorString()));
    return;
  }

  messageHandler_->publishMessage(currentTopic_, currentMessageType_, doc.object());
}

void MessageInspectorPanel::onCopyMessageClicked() {
  if (messageBuffer_.isEmpty()) {
    return;
  }

  QJsonDocument doc(messageBuffer_.last().second);
  QApplication::clipboard()->setText(doc.toJson(QJsonDocument::Indented));
}

void MessageInspectorPanel::onBufferSizeChanged(int size) {
  maxBufferSize_ = size;

  // Trim buffer if needed
  while (messageBuffer_.size() > maxBufferSize_) {
    messageBuffer_.removeFirst();
    messageHistoryList_->takeItem(0);
  }
}

void MessageInspectorPanel::updateRateDisplay() {
  currentRate_ = messageCountSinceLastUpdate_;
  messageCountSinceLastUpdate_ = 0;

  if (!currentTopic_.isEmpty() && inspectButton_->isChecked()) {
    rateLabel_->setText(tr("Rate: %1 Hz").arg(currentRate_, 0, 'f', 1));
  }
}

void MessageInspectorPanel::populateTreeFromJson(QTreeWidgetItem* parent,
                                                   const QJsonObject& obj) {
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, it.key());

    if (it.value().isObject()) {
      item->setText(1, "{...}");
      item->setText(2, "object");
      populateTreeFromJson(item, it.value().toObject());
    } else if (it.value().isArray()) {
      QJsonArray arr = it.value().toArray();
      item->setText(1, QString("[%1 items]").arg(arr.size()));
      item->setText(2, "array");
      populateTreeFromJsonArray(item, arr);
    } else {
      item->setText(1, it.value().toVariant().toString());
      item->setText(2, getJsonTypeName(it.value()));
    }

    if (parent) {
      parent->addChild(item);
    } else {
      messageTreeView_->addTopLevelItem(item);
    }
  }
}

void MessageInspectorPanel::populateTreeFromJsonArray(QTreeWidgetItem* parent,
                                                        const QJsonArray& arr) {
  for (int i = 0; i < arr.size(); ++i) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, QString("[%1]").arg(i));

    const QJsonValue& val = arr[i];
    if (val.isObject()) {
      item->setText(1, "{...}");
      item->setText(2, "object");
      populateTreeFromJson(item, val.toObject());
    } else if (val.isArray()) {
      item->setText(1, QString("[%1 items]").arg(val.toArray().size()));
      item->setText(2, "array");
      populateTreeFromJsonArray(item, val.toArray());
    } else {
      item->setText(1, val.toVariant().toString());
      item->setText(2, getJsonTypeName(val));
    }

    parent->addChild(item);
  }
}

QString MessageInspectorPanel::getJsonTypeName(const QJsonValue& value) {
  switch (value.type()) {
    case QJsonValue::Bool:
      return "bool";
    case QJsonValue::Double:
      return "number";
    case QJsonValue::String:
      return "string";
    case QJsonValue::Array:
      return "array";
    case QJsonValue::Object:
      return "object";
    case QJsonValue::Null:
      return "null";
    default:
      return "unknown";
  }
}

QString MessageInspectorPanel::formatTimestamp(qint64 timestamp) {
  QDateTime dt = QDateTime::fromMSecsSinceEpoch(timestamp);
  return dt.toString("HH:mm:ss.zzz");
}

void MessageInspectorPanel::updateUiState() {
  bool hasSelection = topicSelector_->currentIndex() >= 0;
  bool isInspecting = !currentTopic_.isEmpty();

  inspectButton_->setEnabled(hasSelection || isInspecting);
  pauseButton_->setEnabled(isInspecting);
  clearButton_->setEnabled(isInspecting);
  copyButton_->setEnabled(!messageBuffer_.isEmpty());
  publisherGroup_->setEnabled(isInspecting);
}

void MessageInspectorPanel::addToHistory(const QJsonObject& message, qint64 timestamp) {
  // Add to buffer
  messageBuffer_.append({timestamp, message});

  // Trim if needed
  while (messageBuffer_.size() > maxBufferSize_) {
    messageBuffer_.removeFirst();
    delete messageHistoryList_->takeItem(0);
  }

  // Add to list widget
  QString timeStr = formatTimestamp(timestamp);
  QString preview;

  // Try to create a meaningful preview
  if (message.contains("data")) {
    preview = message["data"].toVariant().toString();
    if (preview.length() > 30) {
      preview = preview.left(27) + "...";
    }
  } else {
    preview = QString("%1 fields").arg(message.size());
  }

  QListWidgetItem* item = new QListWidgetItem(
      QString("%1: %2").arg(timeStr, preview));
  messageHistoryList_->addItem(item);
  messageHistoryList_->scrollToBottom();
}

}  // namespace ros_weaver
