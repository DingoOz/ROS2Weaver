#include "ros_weaver/widgets/connection_stats_dialog.hpp"
#include "ros_weaver/widgets/rolling_digit_label.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFrame>

namespace ros_weaver {

ConnectionStatsDialog::ConnectionStatsDialog(ConnectionLine* connection, QWidget* parent)
  : QDialog(parent)
  , connection_(connection)
  , updateTimer_(new QTimer(this))
{
  setWindowTitle("Connection Statistics");
  setMinimumWidth(400);
  setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

  setupUI();

  // Update stats every 500ms
  connect(updateTimer_, &QTimer::timeout, this, &ConnectionStatsDialog::updateStats);
  updateTimer_->start(500);

  // Initial update
  updateStats();
}

ConnectionStatsDialog::~ConnectionStatsDialog() {
  updateTimer_->stop();
}

void ConnectionStatsDialog::setupUI() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(12);

  // Topic info section
  auto* topicGroup = new QGroupBox("Topic Information");
  auto* topicLayout = new QGridLayout(topicGroup);

  topicLayout->addWidget(new QLabel("Topic:"), 0, 0);
  topicNameLabel_ = new QLabel("-");
  topicNameLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  topicLayout->addWidget(topicNameLabel_, 0, 1);

  topicLayout->addWidget(new QLabel("Message Type:"), 1, 0);
  messageTypeLabel_ = new QLabel("-");
  messageTypeLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  topicLayout->addWidget(messageTypeLabel_, 1, 1);

  topicLayout->setColumnStretch(1, 1);
  mainLayout->addWidget(topicGroup);

  // Health status section
  auto* healthGroup = new QGroupBox("Connection Health");
  auto* healthLayout = new QVBoxLayout(healthGroup);

  auto* healthStatusLayout = new QHBoxLayout();
  healthStatusLayout->addWidget(new QLabel("Status:"));
  healthStatusLabel_ = new QLabel("-");
  healthStatusLabel_->setStyleSheet("font-weight: bold;");
  healthStatusLayout->addWidget(healthStatusLabel_);
  healthStatusLayout->addStretch();
  healthLayout->addLayout(healthStatusLayout);

  healthBar_ = new QProgressBar();
  healthBar_->setRange(0, 100);
  healthBar_->setValue(100);
  healthBar_->setTextVisible(false);
  healthBar_->setFixedHeight(8);
  healthLayout->addWidget(healthBar_);

  auto* expectedLayout = new QHBoxLayout();
  expectedLayout->addWidget(new QLabel("Expected Rate:"));
  expectedRateLabel_ = new QLabel("-");
  expectedLayout->addWidget(expectedRateLabel_);
  expectedLayout->addStretch();
  healthLayout->addLayout(expectedLayout);

  mainLayout->addWidget(healthGroup);

  // Statistics section
  auto* statsGroup = new QGroupBox("Live Statistics");
  auto* statsLayout = new QGridLayout(statsGroup);

  int row = 0;

  statsLayout->addWidget(new QLabel("Message Rate:"), row, 0);
  messageRateLabel_ = new RollingDigitLabel(this);
  messageRateLabel_->setTextColor(QColor("#64C864"));
  messageRateLabel_->setFont(QFont(font().family(), font().pointSize(), QFont::Bold));
  messageRateLabel_->setValue("-");
  statsLayout->addWidget(messageRateLabel_, row++, 1);

  statsLayout->addWidget(new QLabel("Bandwidth:"), row, 0);
  bandwidthLabel_ = new RollingDigitLabel(this);
  bandwidthLabel_->setTextColor(QColor("#6464FF"));
  bandwidthLabel_->setFont(QFont(font().family(), font().pointSize(), QFont::Bold));
  bandwidthLabel_->setValue("-");
  statsLayout->addWidget(bandwidthLabel_, row++, 1);

  statsLayout->addWidget(new QLabel("Avg Latency:"), row, 0);
  latencyLabel_ = new QLabel("-");
  statsLayout->addWidget(latencyLabel_, row++, 1);

  statsLayout->addWidget(new QLabel("Jitter:"), row, 0);
  jitterLabel_ = new QLabel("-");
  statsLayout->addWidget(jitterLabel_, row++, 1);

  // Separator
  auto* separator = new QFrame();
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);
  statsLayout->addWidget(separator, row++, 0, 1, 2);

  statsLayout->addWidget(new QLabel("Queue Depth:"), row, 0);
  queueDepthLabel_ = new QLabel("-");
  statsLayout->addWidget(queueDepthLabel_, row++, 1);

  statsLayout->addWidget(new QLabel("Dropped Messages:"), row, 0);
  droppedCountLabel_ = new QLabel("-");
  statsLayout->addWidget(droppedCountLabel_, row++, 1);

  statsLayout->setColumnStretch(1, 1);
  mainLayout->addWidget(statsGroup);

  // Totals section
  auto* totalsGroup = new QGroupBox("Cumulative Statistics");
  auto* totalsLayout = new QGridLayout(totalsGroup);

  totalsLayout->addWidget(new QLabel("Total Messages:"), 0, 0);
  totalMessagesLabel_ = new QLabel("-");
  totalsLayout->addWidget(totalMessagesLabel_, 0, 1);

  totalsLayout->addWidget(new QLabel("Total Data:"), 1, 0);
  totalBytesLabel_ = new QLabel("-");
  totalsLayout->addWidget(totalBytesLabel_, 1, 1);

  totalsLayout->addWidget(new QLabel("Last Message:"), 2, 0);
  lastMessageLabel_ = new QLabel("-");
  totalsLayout->addWidget(lastMessageLabel_, 2, 1);

  totalsLayout->setColumnStretch(1, 1);
  mainLayout->addWidget(totalsGroup);

  // Close button
  auto* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  auto* closeButton = new QPushButton("Close");
  connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton);
  mainLayout->addLayout(buttonLayout);
}

void ConnectionStatsDialog::updateStats() {
  if (!connection_) return;

  // Topic info
  QString topicName = connection_->topicName();
  topicNameLabel_->setText(topicName.isEmpty() ? "(not set)" : topicName);

  QString msgType = connection_->messageType();
  messageTypeLabel_->setText(msgType.isEmpty() ? "(not set)" : msgType);

  // Connection stats
  ConnectionStats stats = connection_->connectionStats();

  // Message rate (using rolling digit animation)
  double rate = connection_->messageRate();
  QString rateString;
  if (rate >= 1000) {
    rateString = QString("%1 kHz").arg(rate / 1000.0, 0, 'f', 2);
  } else if (rate >= 1) {
    rateString = QString("%1 Hz").arg(rate, 0, 'f', 1);
  } else if (rate > 0) {
    rateString = QString("%1 Hz").arg(rate, 0, 'f', 3);
  } else {
    rateString = "0 Hz";
  }
  messageRateLabel_->setValue(rateString);

  // Bandwidth (using rolling digit animation)
  QString bandwidthString;
  if (stats.bandwidthBps >= 1e9) {
    bandwidthString = QString("%1 GB/s").arg(stats.bandwidthBps / 1e9, 0, 'f', 2);
  } else if (stats.bandwidthBps >= 1e6) {
    bandwidthString = QString("%1 MB/s").arg(stats.bandwidthBps / 1e6, 0, 'f', 2);
  } else if (stats.bandwidthBps >= 1e3) {
    bandwidthString = QString("%1 KB/s").arg(stats.bandwidthBps / 1e3, 0, 'f', 1);
  } else {
    bandwidthString = QString("%1 B/s").arg(stats.bandwidthBps, 0, 'f', 0);
  }
  bandwidthLabel_->setValue(bandwidthString);

  // Latency and jitter
  if (stats.avgLatencyMs > 0) {
    latencyLabel_->setText(QString("%1 ms").arg(stats.avgLatencyMs, 0, 'f', 2));
  } else {
    latencyLabel_->setText("-");
  }

  if (stats.jitterMs > 0) {
    jitterLabel_->setText(QString("%1 ms").arg(stats.jitterMs, 0, 'f', 2));
  } else {
    jitterLabel_->setText("-");
  }

  // Queue and drops
  queueDepthLabel_->setText(QString::number(stats.queueDepth));

  if (stats.droppedCount > 0) {
    droppedCountLabel_->setText(QString("<span style='color: #FF5050;'>%1</span>").arg(stats.droppedCount));
    droppedCountLabel_->setTextFormat(Qt::RichText);
  } else {
    droppedCountLabel_->setText("0");
  }

  // Totals
  totalMessagesLabel_->setText(QString::number(stats.totalMessages));
  totalBytesLabel_->setText(formatBytes(stats.totalBytes));

  // Last message time
  if (stats.lastMessageTime.isValid()) {
    qint64 elapsed = stats.lastMessageTime.msecsTo(QDateTime::currentDateTime());
    lastMessageLabel_->setText(formatDuration(elapsed) + " ago");
  } else {
    lastMessageLabel_->setText("-");
  }

  // Expected rate
  double expected = connection_->expectedRate();
  if (expected > 0) {
    expectedRateLabel_->setText(QString("%1 Hz").arg(expected, 0, 'f', 1));
  } else {
    expectedRateLabel_->setText("(not set)");
  }

  // Update health indicator
  updateHealthIndicator();
}

void ConnectionStatsDialog::updateHealthIndicator() {
  if (!connection_) return;

  TopicActivityState state = connection_->activityState();

  QString statusText;
  QString statusColor;
  int healthPercent = 100;

  switch (state) {
    case TopicActivityState::HighRate:
      statusText = "Excellent (High Rate)";
      statusColor = "#32FF32";
      healthPercent = 100;
      break;
    case TopicActivityState::Active:
      statusText = "Good (Active)";
      statusColor = "#64C864";
      healthPercent = 100;
      break;
    case TopicActivityState::Degraded:
      statusText = "Warning (Degraded)";
      statusColor = "#FFC832";
      healthPercent = 50;
      break;
    case TopicActivityState::Stale:
      statusText = "Error (Stale)";
      statusColor = "#FF5050";
      healthPercent = 10;
      break;
    case TopicActivityState::Inactive:
      statusText = "Inactive";
      statusColor = "#969696";
      healthPercent = 0;
      break;
    case TopicActivityState::Unknown:
    default:
      statusText = "Unknown";
      statusColor = "#969696";
      healthPercent = 0;
      break;
  }

  healthStatusLabel_->setText(statusText);
  healthStatusLabel_->setStyleSheet(QString("font-weight: bold; color: %1;").arg(statusColor));

  healthBar_->setValue(healthPercent);
  healthBar_->setStyleSheet(QString(
    "QProgressBar { background: #3C3C3C; border: none; border-radius: 4px; }"
    "QProgressBar::chunk { background: %1; border-radius: 4px; }"
  ).arg(statusColor));
}

QString ConnectionStatsDialog::formatBytes(qint64 bytes) const {
  if (bytes >= 1e12) {
    return QString("%1 TB").arg(bytes / 1e12, 0, 'f', 2);
  } else if (bytes >= 1e9) {
    return QString("%1 GB").arg(bytes / 1e9, 0, 'f', 2);
  } else if (bytes >= 1e6) {
    return QString("%1 MB").arg(bytes / 1e6, 0, 'f', 2);
  } else if (bytes >= 1e3) {
    return QString("%1 KB").arg(bytes / 1e3, 0, 'f', 1);
  } else {
    return QString("%1 B").arg(bytes);
  }
}

QString ConnectionStatsDialog::formatDuration(qint64 milliseconds) const {
  if (milliseconds < 1000) {
    return QString("%1 ms").arg(milliseconds);
  } else if (milliseconds < 60000) {
    return QString("%1 s").arg(milliseconds / 1000.0, 0, 'f', 1);
  } else if (milliseconds < 3600000) {
    int minutes = milliseconds / 60000;
    int seconds = (milliseconds % 60000) / 1000;
    return QString("%1m %2s").arg(minutes).arg(seconds);
  } else {
    int hours = milliseconds / 3600000;
    int minutes = (milliseconds % 3600000) / 60000;
    return QString("%1h %2m").arg(hours).arg(minutes);
  }
}

}  // namespace ros_weaver
