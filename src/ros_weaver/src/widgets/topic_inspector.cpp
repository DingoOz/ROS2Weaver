#include "ros_weaver/widgets/topic_inspector.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

#include <QPainter>
#include <QApplication>
#include <QScreen>
#include <QFocusEvent>

namespace ros_weaver {

// ============================================================================
// TopicInspectorPopup
// ============================================================================

TopicInspectorPopup::TopicInspectorPopup(QWidget* parent)
    : QFrame(parent, Qt::Popup | Qt::FramelessWindowHint)
    , currentConnection_(nullptr)
    , topicMonitor_(nullptr)
    , updateTimer_(new QTimer(this))
{
  setupUi();

  // Auto-hide timer for stale popups
  connect(updateTimer_, &QTimer::timeout, this, &TopicInspectorPopup::updateDisplay);

  // Install event filter for click-outside-to-close
  installEventFilter(this);

  // Add shadow effect
  auto* shadow = new QGraphicsDropShadowEffect(this);
  shadow->setBlurRadius(15);
  shadow->setOffset(2, 2);
  shadow->setColor(QColor(0, 0, 0, 100));
  setGraphicsEffect(shadow);
}

TopicInspectorPopup::~TopicInspectorPopup()
{
  updateTimer_->stop();
}

void TopicInspectorPopup::setupUi()
{
  setObjectName("TopicInspectorPopup");
  setStyleSheet(R"(
    #TopicInspectorPopup {
      background-color: #2d2d2d;
      border: 1px solid #555;
      border-radius: 8px;
    }
    QLabel {
      color: #eee;
    }
    QLabel#titleLabel {
      font-weight: bold;
      font-size: 12px;
      color: #fff;
    }
    QLabel#topicNameLabel {
      font-family: monospace;
      color: #6cf;
      font-size: 11px;
    }
    QLabel#messageTypeLabel {
      font-family: monospace;
      color: #aaa;
      font-size: 10px;
    }
    QLabel#rateLabel {
      font-weight: bold;
      color: #6f6;
      font-size: 14px;
    }
    QPushButton {
      background-color: #404040;
      border: 1px solid #555;
      border-radius: 4px;
      color: #eee;
      padding: 4px 10px;
      font-size: 10px;
    }
    QPushButton:hover {
      background-color: #505050;
      border-color: #666;
    }
    QPushButton:pressed {
      background-color: #353535;
    }
    QPushButton#closeButton {
      background-color: transparent;
      border: none;
      color: #888;
      font-weight: bold;
      padding: 2px;
    }
    QPushButton#closeButton:hover {
      color: #fff;
    }
    QTextEdit {
      background-color: #1e1e1e;
      border: 1px solid #444;
      border-radius: 4px;
      color: #ddd;
      font-family: monospace;
      font-size: 9px;
    }
  )");

  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(12, 8, 12, 12);
  mainLayout->setSpacing(6);

  // Header with title and close button
  auto* headerLayout = new QHBoxLayout();
  titleLabel_ = new QLabel("Topic Inspector");
  titleLabel_->setObjectName("titleLabel");
  headerLayout->addWidget(titleLabel_);
  headerLayout->addStretch();

  closeButton_ = new QPushButton("X");
  closeButton_->setObjectName("closeButton");
  closeButton_->setFixedSize(20, 20);
  connect(closeButton_, &QPushButton::clicked, this, &TopicInspectorPopup::onCloseClicked);
  headerLayout->addWidget(closeButton_);
  mainLayout->addLayout(headerLayout);

  // Topic name
  topicNameLabel_ = new QLabel("/topic_name");
  topicNameLabel_->setObjectName("topicNameLabel");
  topicNameLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  mainLayout->addWidget(topicNameLabel_);

  // Message type
  messageTypeLabel_ = new QLabel("std_msgs/msg/String");
  messageTypeLabel_->setObjectName("messageTypeLabel");
  mainLayout->addWidget(messageTypeLabel_);

  // Separator
  auto* separator = new QFrame();
  separator->setFrameShape(QFrame::HLine);
  separator->setStyleSheet("background-color: #444;");
  separator->setFixedHeight(1);
  mainLayout->addWidget(separator);

  // Stats section
  auto* statsLayout = new QHBoxLayout();

  // Rate display
  auto* rateBox = new QVBoxLayout();
  auto* rateTitle = new QLabel("Rate");
  rateTitle->setStyleSheet("color: #888; font-size: 9px;");
  rateBox->addWidget(rateTitle);
  rateLabel_ = new QLabel("0.0 Hz");
  rateLabel_->setObjectName("rateLabel");
  rateBox->addWidget(rateLabel_);
  statsLayout->addLayout(rateBox);

  statsLayout->addStretch();

  // Publishers
  auto* pubBox = new QVBoxLayout();
  auto* pubTitle = new QLabel("Publishers");
  pubTitle->setStyleSheet("color: #888; font-size: 9px;");
  pubBox->addWidget(pubTitle);
  publishersLabel_ = new QLabel("0");
  publishersLabel_->setStyleSheet("font-weight: bold; color: #6f6;");
  pubBox->addWidget(publishersLabel_);
  statsLayout->addLayout(pubBox);

  statsLayout->addSpacing(20);

  // Subscribers
  auto* subBox = new QVBoxLayout();
  auto* subTitle = new QLabel("Subscribers");
  subTitle->setStyleSheet("color: #888; font-size: 9px;");
  subBox->addWidget(subTitle);
  subscribersLabel_ = new QLabel("0");
  subscribersLabel_->setStyleSheet("font-weight: bold; color: #6cf;");
  subBox->addWidget(subscribersLabel_);
  statsLayout->addLayout(subBox);

  mainLayout->addLayout(statsLayout);

  // Statistics
  statsLabel_ = new QLabel("Total: 0 msgs | Peak: 0.0 Hz | Avg: 0.0 Hz");
  statsLabel_->setStyleSheet("color: #888; font-size: 9px;");
  mainLayout->addWidget(statsLabel_);

  // Separator
  auto* separator2 = new QFrame();
  separator2->setFrameShape(QFrame::HLine);
  separator2->setStyleSheet("background-color: #444;");
  separator2->setFixedHeight(1);
  mainLayout->addWidget(separator2);

  // Message preview (optional, can be hidden)
  messagePreview_ = new QTextEdit();
  messagePreview_->setReadOnly(true);
  messagePreview_->setMaximumHeight(80);
  messagePreview_->setPlaceholderText("No message preview available");
  messagePreview_->hide();  // Hidden by default
  mainLayout->addWidget(messagePreview_);

  // Action buttons
  auto* buttonLayout = new QHBoxLayout();
  buttonLayout->setSpacing(6);

  echoButton_ = new QPushButton("Echo");
  echoButton_->setToolTip("Echo topic messages to output panel");
  connect(echoButton_, &QPushButton::clicked, this, &TopicInspectorPopup::onEchoClicked);
  buttonLayout->addWidget(echoButton_);

  monitorButton_ = new QPushButton("Monitor");
  monitorButton_->setToolTip("Start/stop monitoring this topic");
  connect(monitorButton_, &QPushButton::clicked, this, &TopicInspectorPopup::onMonitorClicked);
  buttonLayout->addWidget(monitorButton_);

  showOnCanvasButton_ = new QPushButton("Show");
  showOnCanvasButton_->setToolTip("Highlight connected nodes on canvas");
  connect(showOnCanvasButton_, &QPushButton::clicked, this, &TopicInspectorPopup::onShowOnCanvasClicked);
  buttonLayout->addWidget(showOnCanvasButton_);

  mainLayout->addLayout(buttonLayout);

  setMinimumWidth(280);
  adjustSize();
}

void TopicInspectorPopup::showForConnection(ConnectionLine* connection, const QPoint& globalPos)
{
  currentConnection_ = connection;

  if (connection) {
    topicName_ = connection->topicName();
    messageType_ = connection->messageType();

    topicNameLabel_->setText(topicName_.isEmpty() ? "(unknown topic)" : topicName_);
    messageTypeLabel_->setText(messageType_.isEmpty() ? "(unknown type)" : messageType_);

    // Update rate from connection
    double rate = connection->messageRate();
    if (rate >= 1000) {
      rateLabel_->setText(QString("%1 kHz").arg(rate / 1000.0, 0, 'f', 1));
    } else if (rate >= 1) {
      rateLabel_->setText(QString("%1 Hz").arg(rate, 0, 'f', 1));
    } else if (rate > 0) {
      rateLabel_->setText(QString("%1 Hz").arg(rate, 0, 'f', 2));
    } else {
      rateLabel_->setText("-- Hz");
    }

    // Connect to topic monitor for live updates
    if (topicMonitor_ && !topicName_.isEmpty()) {
      connect(topicMonitor_, &TopicMonitor::topicStatsUpdated,
              this, &TopicInspectorPopup::onTopicStatsUpdated);
      updateTimer_->start(500);  // Update every 500ms
    }
  }

  // Position popup, ensuring it stays on screen
  QPoint pos = globalPos;
  QScreen* screen = QApplication::screenAt(globalPos);
  if (screen) {
    QRect screenGeom = screen->availableGeometry();
    if (pos.x() + width() > screenGeom.right()) {
      pos.setX(screenGeom.right() - width() - 10);
    }
    if (pos.y() + height() > screenGeom.bottom()) {
      pos.setY(screenGeom.bottom() - height() - 10);
    }
  }

  move(pos);
  show();
  raise();
  setFocus();
}

void TopicInspectorPopup::updateStats(const TopicStats& stats)
{
  // Update rate
  double rate = stats.currentRate;
  if (rate >= 1000) {
    rateLabel_->setText(QString("%1 kHz").arg(rate / 1000.0, 0, 'f', 1));
  } else if (rate >= 1) {
    rateLabel_->setText(QString("%1 Hz").arg(rate, 0, 'f', 1));
  } else if (rate > 0) {
    rateLabel_->setText(QString("%1 Hz").arg(rate, 0, 'f', 2));
  } else {
    rateLabel_->setText("-- Hz");
  }

  // Update stats label
  QString statsText = QString("Total: %1 msgs | Peak: %2 Hz | Avg: %3 Hz")
      .arg(stats.totalMessages)
      .arg(stats.peakRate, 0, 'f', 1)
      .arg(stats.averageRate, 0, 'f', 1);
  statsLabel_->setText(statsText);
}

void TopicInspectorPopup::setTopicMonitor(TopicMonitor* monitor)
{
  if (topicMonitor_) {
    disconnect(topicMonitor_, nullptr, this, nullptr);
  }
  topicMonitor_ = monitor;
}

void TopicInspectorPopup::paintEvent(QPaintEvent* event)
{
  Q_UNUSED(event)

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw rounded rectangle background
  QRectF rect = this->rect().adjusted(0.5, 0.5, -0.5, -0.5);
  painter.setBrush(QColor(45, 45, 45));
  painter.setPen(QPen(QColor(85, 85, 85), 1));
  painter.drawRoundedRect(rect, 8, 8);
}

void TopicInspectorPopup::focusOutEvent(QFocusEvent* event)
{
  Q_UNUSED(event)
  // Don't auto-close on focus out - let user click outside or press close
}

bool TopicInspectorPopup::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == QEvent::MouseButtonPress) {
    QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
    if (!rect().contains(mapFromGlobal(mouseEvent->globalPos()))) {
      hide();
      return true;
    }
  }
  return QFrame::eventFilter(obj, event);
}

void TopicInspectorPopup::onEchoClicked()
{
  if (!topicName_.isEmpty()) {
    emit echoTopicRequested(topicName_);
  }
  hide();
}

void TopicInspectorPopup::onMonitorClicked()
{
  if (!topicName_.isEmpty()) {
    emit monitorTopicRequested(topicName_);
  }
}

void TopicInspectorPopup::onShowOnCanvasClicked()
{
  if (!topicName_.isEmpty()) {
    emit showOnCanvasRequested(topicName_);
  }
  hide();
}

void TopicInspectorPopup::onCloseClicked()
{
  hide();
}

void TopicInspectorPopup::onTopicStatsUpdated(const QString& topicName, const TopicStats& stats)
{
  if (topicName == topicName_ && isVisible()) {
    updateStats(stats);
  }
}

void TopicInspectorPopup::updateDisplay()
{
  if (topicMonitor_ && !topicName_.isEmpty()) {
    TopicStats stats = topicMonitor_->getTopicStats(topicName_);
    updateStats(stats);

    // Update publisher/subscriber counts
    TopicInfo info = topicMonitor_->getTopicInfo(topicName_);
    publishersLabel_->setText(QString::number(info.publishers.size()));
    subscribersLabel_->setText(QString::number(info.subscribers.size()));
  }
}

// ============================================================================
// TopicTooltip
// ============================================================================

TopicTooltip::TopicTooltip(QWidget* parent)
    : QLabel(parent, Qt::ToolTip | Qt::FramelessWindowHint)
    , hideTimer_(new QTimer(this))
{
  setStyleSheet(R"(
    QLabel {
      background-color: #2d2d2d;
      border: 1px solid #555;
      border-radius: 4px;
      color: #eee;
      padding: 6px 10px;
      font-size: 10px;
    }
  )");

  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_ShowWithoutActivating);

  hideTimer_->setSingleShot(true);
  connect(hideTimer_, &QTimer::timeout, this, &TopicTooltip::hide);
}

void TopicTooltip::showAt(const QPoint& globalPos, const QString& topicName,
                          const QString& messageType, double rate)
{
  QString rateStr;
  if (rate >= 1000) {
    rateStr = QString("%1 kHz").arg(rate / 1000.0, 0, 'f', 1);
  } else if (rate >= 1) {
    rateStr = QString("%1 Hz").arg(rate, 0, 'f', 1);
  } else if (rate > 0) {
    rateStr = QString("%1 Hz").arg(rate, 0, 'f', 2);
  } else {
    rateStr = "-- Hz";
  }

  QString text = QString("<b>%1</b><br>"
                         "<span style='color: #888;'>%2</span><br>"
                         "<span style='color: #6f6;'>%3</span>")
      .arg(topicName.isEmpty() ? "(unknown)" : topicName)
      .arg(messageType.isEmpty() ? "" : messageType)
      .arg(rateStr);

  setText(text);
  adjustSize();

  // Position tooltip
  QPoint pos = globalPos + QPoint(15, 15);
  QScreen* screen = QApplication::screenAt(globalPos);
  if (screen) {
    QRect screenGeom = screen->availableGeometry();
    if (pos.x() + width() > screenGeom.right()) {
      pos.setX(globalPos.x() - width() - 15);
    }
    if (pos.y() + height() > screenGeom.bottom()) {
      pos.setY(globalPos.y() - height() - 15);
    }
  }

  move(pos);
  show();
  raise();

  // Auto-hide after 3 seconds
  hideTimer_->start(3000);
}

void TopicTooltip::hideTooltip()
{
  hideTimer_->stop();
  hide();
}

void TopicTooltip::paintEvent(QPaintEvent* event)
{
  Q_UNUSED(event)

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QRectF rect = this->rect().adjusted(0.5, 0.5, -0.5, -0.5);
  painter.setBrush(QColor(45, 45, 45, 240));
  painter.setPen(QPen(QColor(85, 85, 85), 1));
  painter.drawRoundedRect(rect, 4, 4);

  // Draw text
  painter.setPen(QColor(238, 238, 238));
  QTextDocument doc;
  doc.setHtml(text());
  doc.setDefaultFont(font());
  painter.translate(6, 4);
  doc.drawContents(&painter);
}

}  // namespace ros_weaver
