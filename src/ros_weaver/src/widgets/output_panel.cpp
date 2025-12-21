#include "ros_weaver/widgets/output_panel.hpp"
#include "ros_weaver/widgets/llm_chat_widget.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <iostream>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QScrollBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QApplication>

namespace ros_weaver {

// =============================================================================
// RosLogViewer
// =============================================================================

RosLogViewer::RosLogViewer(QWidget* parent)
  : QWidget(parent)
  , isListening_(false)
  , shouldStop_(false)
{
  // Register LogEntry type for queued signal/slot connections
  qRegisterMetaType<LogEntry>("LogEntry");

  setupUi();

  // Set up queue processing timer
  queueTimer_ = new QTimer(this);
  connect(queueTimer_, &QTimer::timeout, this, &RosLogViewer::processLogQueue);

  // Connect internal signal for thread-safe log handling
  connect(this, &RosLogViewer::logReceived,
          this, &RosLogViewer::onLogReceived, Qt::QueuedConnection);
}

RosLogViewer::~RosLogViewer() {
  std::cerr << "RosLogViewer destructor: starting" << std::endl;
  stopListening();
  std::cerr << "RosLogViewer destructor: complete" << std::endl;
}

void RosLogViewer::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(4);

  // Filter bar
  QHBoxLayout* filterLayout = new QHBoxLayout();

  // Level filter
  QLabel* levelLabel = new QLabel(tr("Level:"));
  levelFilter_ = new QComboBox();
  levelFilter_->addItem(tr("All Levels"), -1);
  levelFilter_->addItem(tr("FATAL"), 50);
  levelFilter_->addItem(tr("ERROR"), 40);
  levelFilter_->addItem(tr("WARN"), 30);
  levelFilter_->addItem(tr("INFO"), 20);
  levelFilter_->addItem(tr("DEBUG"), 10);
  levelFilter_->setCurrentIndex(0);
  connect(levelFilter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &RosLogViewer::onFilterChanged);

  // Node filter
  QLabel* nodeLabel = new QLabel(tr("Node:"));
  nodeFilter_ = new QLineEdit();
  nodeFilter_->setPlaceholderText(tr("Filter by node name..."));
  nodeFilter_->setClearButtonEnabled(true);
  connect(nodeFilter_, &QLineEdit::textChanged, this, &RosLogViewer::onFilterChanged);

  // Message filter
  QLabel* msgLabel = new QLabel(tr("Message:"));
  messageFilter_ = new QLineEdit();
  messageFilter_->setPlaceholderText(tr("Filter by message content..."));
  messageFilter_->setClearButtonEnabled(true);
  connect(messageFilter_, &QLineEdit::textChanged, this, &RosLogViewer::onFilterChanged);

  filterLayout->addWidget(levelLabel);
  filterLayout->addWidget(levelFilter_);
  filterLayout->addWidget(nodeLabel);
  filterLayout->addWidget(nodeFilter_);
  filterLayout->addWidget(msgLabel);
  filterLayout->addWidget(messageFilter_);
  filterLayout->addStretch();

  layout->addLayout(filterLayout);

  // Log tree
  logTree_ = new QTreeWidget();
  logTree_->setHeaderLabels({tr("Time"), tr("Level"), tr("Node"), tr("Message")});
  logTree_->setRootIsDecorated(false);
  logTree_->setAlternatingRowColors(true);
  logTree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  logTree_->header()->setStretchLastSection(true);
  logTree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  logTree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  logTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  layout->addWidget(logTree_);

  // Control bar
  QHBoxLayout* controlLayout = new QHBoxLayout();

  startStopButton_ = new QPushButton(tr("Start Listening"));
  startStopButton_->setCheckable(true);
  connect(startStopButton_, &QPushButton::clicked, this, [this](bool checked) {
    if (checked) {
      startListening();
    } else {
      stopListening();
    }
  });

  clearButton_ = new QPushButton(tr("Clear"));
  connect(clearButton_, &QPushButton::clicked, this, &RosLogViewer::onClearClicked);

  exportButton_ = new QPushButton(tr("Export..."));
  connect(exportButton_, &QPushButton::clicked, this, &RosLogViewer::onExportClicked);

  analyzeErrorsButton_ = new QPushButton(tr("Analyze Errors"));
  analyzeErrorsButton_->setToolTip(tr("Send current errors to LocalLLM for analysis"));
  auto& theme = ThemeManager::instance();
  analyzeErrorsButton_->setStyleSheet(theme.destructiveButtonStyle());
  connect(analyzeErrorsButton_, &QPushButton::clicked, this, &RosLogViewer::onAnalyzeErrorsClicked);

  autoScrollCheck_ = new QCheckBox(tr("Auto-scroll"));
  autoScrollCheck_->setChecked(true);

  statusLabel_ = new QLabel(tr("Not connected"));
  statusLabel_->setStyleSheet("color: gray;");

  controlLayout->addWidget(startStopButton_);
  controlLayout->addWidget(clearButton_);
  controlLayout->addWidget(exportButton_);
  controlLayout->addWidget(analyzeErrorsButton_);
  controlLayout->addWidget(autoScrollCheck_);
  controlLayout->addStretch();
  controlLayout->addWidget(statusLabel_);

  layout->addLayout(controlLayout);
}

void RosLogViewer::startListening() {
  if (isListening_) return;

  shouldStop_ = false;

  try {
    // Initialize ROS2 if not already done
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create node with unique name to avoid conflicts
    std::string nodeName = "ros_weaver_log_viewer_" +
                           std::to_string(QDateTime::currentMSecsSinceEpoch());
    rosNode_ = std::make_shared<rclcpp::Node>(nodeName);

    // Use the rosout-specific QoS profile for best compatibility
    // /rosout publishers use: RELIABLE + TRANSIENT_LOCAL
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1000))
      .reliable()
      .transient_local();

    // Subscribe to /rosout_agg (aggregated logs) which is more reliable
    // Fall back to /rosout if /rosout_agg doesn't exist
    logSubscription_ = rosNode_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", qos,
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        LogEntry entry;
        entry.timestamp = QDateTime::currentDateTime();

        // Map log level
        switch (msg->level) {
          case rcl_interfaces::msg::Log::DEBUG: entry.level = "DEBUG"; break;
          case rcl_interfaces::msg::Log::INFO:  entry.level = "INFO"; break;
          case rcl_interfaces::msg::Log::WARN:  entry.level = "WARN"; break;
          case rcl_interfaces::msg::Log::ERROR: entry.level = "ERROR"; break;
          case rcl_interfaces::msg::Log::FATAL: entry.level = "FATAL"; break;
          default: entry.level = "UNKNOWN"; break;
        }

        entry.nodeName = QString::fromStdString(msg->name);
        entry.message = QString::fromStdString(msg->msg);
        entry.file = QString::fromStdString(msg->file);
        entry.function = QString::fromStdString(msg->function);
        entry.line = msg->line;

        // Add to thread-safe queue
        {
          QMutexLocker locker(&queueMutex_);
          logQueue_.push(entry);
          receivedCount_++;
        }
      });

    // Start spin thread BEFORE setting isListening_ to ensure it's running
    isListening_ = true;
    spinThread_ = std::thread(&RosLogViewer::rosSpinThread, this);

    queueTimer_->start(50);  // Process queue every 50ms for faster updates

    startStopButton_->setText(tr("Stop Listening"));
    startStopButton_->setChecked(true);
    statusLabel_->setText(tr("Connected to /rosout"));
    statusLabel_->setStyleSheet("color: green;");

    emit connectionStatusChanged(true);

  } catch (const std::exception& e) {
    QMessageBox::warning(this, tr("ROS2 Error"),
      tr("Failed to start ROS2 log listener:\n%1").arg(e.what()));
    stopListening();
  }
}

void RosLogViewer::stopListening() {
  std::cerr << "RosLogViewer::stopListening() - isListening_=" << isListening_ << std::endl;
  if (!isListening_) return;

  shouldStop_ = true;
  queueTimer_->stop();

  // Wait for spin thread to finish
  if (spinThread_.joinable()) {
    std::cerr << "RosLogViewer: joining spin thread..." << std::endl;
    spinThread_.join();
    std::cerr << "RosLogViewer: spin thread joined" << std::endl;
  }

  // Clean up subscription and node
  logSubscription_.reset();
  rosNode_.reset();

  isListening_ = false;

  startStopButton_->setText(tr("Start Listening"));
  startStopButton_->setChecked(false);
  statusLabel_->setText(tr("Disconnected"));
  statusLabel_->setStyleSheet("color: gray;");

  emit connectionStatusChanged(false);
  std::cerr << "RosLogViewer::stopListening() complete" << std::endl;
}

void RosLogViewer::rosSpinThread() {
  while (!shouldStop_ && rclcpp::ok()) {
    rclcpp::spin_some(rosNode_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void RosLogViewer::processLogQueue() {
  QMutexLocker locker(&queueMutex_);

  // Process up to 100 logs per timer tick to avoid UI lag
  int processed = 0;
  while (!logQueue_.empty() && processed < 100) {
    LogEntry entry = logQueue_.front();
    logQueue_.pop();
    locker.unlock();

    emit logReceived(entry);

    locker.relock();
    processed++;
  }

  // Update status with message count (useful for debugging)
  int received = receivedCount_.load();
  int displayed = displayedCount_.load();
  if (isListening_) {
    locker.unlock();
    statusLabel_->setText(tr("Connected - Received: %1, Displayed: %2")
                          .arg(received).arg(displayed));
  }
}

void RosLogViewer::onLogReceived(const LogEntry& entry) {
  addLogEntry(entry);
}

void RosLogViewer::addLogEntry(const LogEntry& entry) {
  // Store entry for later re-filtering
  allLogEntries_.push_back(entry);

  // Enforce max stored entries
  while (static_cast<int>(allLogEntries_.size()) > maxLogEntries_) {
    allLogEntries_.erase(allLogEntries_.begin());
  }

  // Check filter for display
  if (!passesFilter(entry)) return;

  // Create tree item
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, entry.timestamp.toString("hh:mm:ss.zzz"));

  // Add icon and level text
  QString levelWithIcon = QString("%1 %2").arg(getIconTextForLevel(entry.level), entry.level);
  item->setText(1, levelWithIcon);
  item->setData(1, Qt::UserRole, entry.level);  // Store raw level for color lookup
  item->setText(2, entry.nodeName);
  item->setText(3, entry.message);

  // Set tooltip with full info
  QString tooltip = QString("File: %1:%2\nFunction: %3")
    .arg(entry.file).arg(entry.line).arg(entry.function);
  item->setToolTip(3, tooltip);

  // Color code by level - only color the level column for cleaner look
  QColor levelColor = getColorForLevel(entry.level);
  item->setForeground(1, levelColor);

  // Use default text color for other columns, but slightly muted for timestamp
  auto& theme = ThemeManager::instance();
  item->setForeground(0, theme.textSecondaryColor());
  item->setForeground(2, theme.textPrimaryColor());
  item->setForeground(3, theme.textPrimaryColor());

  // Bold for errors and fatals
  if (entry.level == "ERROR" || entry.level == "FATAL") {
    QFont boldFont = item->font(1);
    boldFont.setBold(true);
    item->setFont(1, boldFont);
    item->setFont(3, boldFont);
  }

  // Add to tree
  logTree_->addTopLevelItem(item);
  displayedCount_++;

  // Enforce max displayed entries
  while (logTree_->topLevelItemCount() > maxLogEntries_) {
    delete logTree_->takeTopLevelItem(0);
  }

  // Auto-scroll
  if (autoScrollCheck_->isChecked()) {
    logTree_->scrollToBottom();
  }
}

QColor RosLogViewer::getColorForLevel(const QString& level) const {
  return colorForLevel(level);
}

QColor RosLogViewer::colorForLevel(const QString& level) const {
  auto& theme = ThemeManager::instance();

  // Use theme colors when available, fall back to configured colors
  if (level == "DEBUG") return logColors_.debugColor;
  if (level == "INFO")  return theme.textPrimaryColor();
  if (level == "WARN")  return theme.warningColor();
  if (level == "ERROR") return theme.errorColor();
  if (level == "FATAL") return theme.errorColor().darker(120);
  return logColors_.unknownColor;
}

void RosLogViewer::setLogLevelColors(const LogLevelColors& colors) {
  logColors_ = colors;
  // Refresh existing items with new colors
  for (int i = 0; i < logTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = logTree_->topLevelItem(i);
    QString level = item->data(1, Qt::UserRole).toString();  // Get raw level, not display text with icon
    QColor color = colorForLevel(level);
    for (int j = 0; j < 4; ++j) {
      item->setForeground(j, color);
    }
  }
}

void RosLogViewer::setColorForLevel(const QString& level, const QColor& color) {
  if (level == "DEBUG") logColors_.debugColor = color;
  else if (level == "INFO") logColors_.infoColor = color;
  else if (level == "WARN") logColors_.warnColor = color;
  else if (level == "ERROR") logColors_.errorColor = color;
  else if (level == "FATAL") logColors_.fatalColor = color;

  // Refresh items of this level
  for (int i = 0; i < logTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = logTree_->topLevelItem(i);
    if (item->data(1, Qt::UserRole).toString() == level) {  // Compare raw level, not display text with icon
      for (int j = 0; j < 4; ++j) {
        item->setForeground(j, color);
      }
    }
  }
}

QString RosLogViewer::getIconTextForLevel(const QString& level) const {
  // Unicode icons for log levels
  if (level == "DEBUG") return QString::fromUtf8("\xF0\x9F\x94\x8D");  // Magnifying glass
  if (level == "INFO")  return QString::fromUtf8("\xE2\x84\xB9");      // Info symbol
  if (level == "WARN")  return QString::fromUtf8("\xE2\x9A\xA0");      // Warning triangle
  if (level == "ERROR") return QString::fromUtf8("\xE2\x9C\x95");      // X mark
  if (level == "FATAL") return QString::fromUtf8("\xF0\x9F\x92\x80");  // Skull
  return QString::fromUtf8("\xE2\x9D\x93");  // Question mark
}

QIcon RosLogViewer::getIconForLevel(const QString& level) const {
  Q_UNUSED(level)
  return QIcon();
}

bool RosLogViewer::passesFilter(const LogEntry& entry) const {
  // Level filter
  int minLevel = levelFilter_->currentData().toInt();
  if (minLevel > 0) {
    int entryLevel = 0;
    if (entry.level == "DEBUG") entryLevel = 10;
    else if (entry.level == "INFO") entryLevel = 20;
    else if (entry.level == "WARN") entryLevel = 30;
    else if (entry.level == "ERROR") entryLevel = 40;
    else if (entry.level == "FATAL") entryLevel = 50;

    if (entryLevel < minLevel) return false;
  }

  // Node name filter
  QString nodeFilterText = nodeFilter_->text();
  if (!nodeFilterText.isEmpty()) {
    if (!entry.nodeName.contains(nodeFilterText, Qt::CaseInsensitive)) {
      return false;
    }
  }

  // Message filter
  QString msgFilterText = messageFilter_->text();
  if (!msgFilterText.isEmpty()) {
    if (!entry.message.contains(msgFilterText, Qt::CaseInsensitive)) {
      return false;
    }
  }

  return true;
}

void RosLogViewer::onFilterChanged() {
  // Re-filter all stored entries when filter changes
  logTree_->clear();
  displayedCount_ = 0;

  auto& theme = ThemeManager::instance();

  for (const LogEntry& entry : allLogEntries_) {
    if (!passesFilter(entry)) continue;

    // Create tree item
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, entry.timestamp.toString("hh:mm:ss.zzz"));

    // Add icon and level text
    QString levelWithIcon = QString("%1 %2").arg(getIconTextForLevel(entry.level), entry.level);
    item->setText(1, levelWithIcon);
    item->setData(1, Qt::UserRole, entry.level);  // Store raw level for color lookup
    item->setText(2, entry.nodeName);
    item->setText(3, entry.message);

    // Set tooltip with full info
    QString tooltip = QString("File: %1:%2\nFunction: %3")
      .arg(entry.file).arg(entry.line).arg(entry.function);
    item->setToolTip(3, tooltip);

    // Color code by level - only color the level column for cleaner look
    QColor levelColor = getColorForLevel(entry.level);
    item->setForeground(1, levelColor);
    item->setForeground(0, theme.textSecondaryColor());
    item->setForeground(2, theme.textPrimaryColor());
    item->setForeground(3, theme.textPrimaryColor());

    // Bold for errors and fatals
    if (entry.level == "ERROR" || entry.level == "FATAL") {
      QFont boldFont = item->font(1);
      boldFont.setBold(true);
      item->setFont(1, boldFont);
      item->setFont(3, boldFont);
    }

    logTree_->addTopLevelItem(item);
    displayedCount_++;
  }

  // Scroll to bottom if auto-scroll enabled
  if (autoScrollCheck_->isChecked()) {
    logTree_->scrollToBottom();
  }
}

void RosLogViewer::onClearClicked() {
  clear();
}

void RosLogViewer::clear() {
  logTree_->clear();
  allLogEntries_.clear();
  receivedCount_ = 0;
  displayedCount_ = 0;
}

void RosLogViewer::onAutoScrollToggled(bool checked) {
  Q_UNUSED(checked)
}

void RosLogViewer::onExportClicked() {
  QString fileName = QFileDialog::getSaveFileName(
    this, tr("Export Logs"), QString(), tr("Text Files (*.txt);;All Files (*)"));

  if (fileName.isEmpty()) return;

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Export Error"),
      tr("Could not open file for writing."));
    return;
  }

  QTextStream stream(&file);
  stream << exportToText();
  file.close();

  QMessageBox::information(this, tr("Export Complete"),
    tr("Logs exported to %1").arg(fileName));
}

QString RosLogViewer::exportToText() const {
  QString output;
  QTextStream stream(&output);

  stream << "ROS2 Log Export - " << QDateTime::currentDateTime().toString() << "\n";
  stream << QString("=").repeated(80) << "\n\n";

  for (int i = 0; i < logTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = logTree_->topLevelItem(i);
    stream << item->text(0) << " [" << item->text(1) << "] "
           << item->text(2) << ": " << item->text(3) << "\n";
  }

  return output;
}

QString RosLogViewer::getErrorLogs() const {
  QString output;
  QTextStream stream(&output);

  int errorCount = 0;
  for (const LogEntry& entry : allLogEntries_) {
    if (entry.level == "ERROR" || entry.level == "FATAL") {
      errorCount++;
      stream << entry.timestamp.toString("hh:mm:ss.zzz") << " [" << entry.level << "] "
             << entry.nodeName << ": " << entry.message;
      if (!entry.file.isEmpty()) {
        stream << " (File: " << entry.file << ":" << entry.line << ")";
      }
      stream << "\n";
    }
  }

  if (errorCount == 0) {
    return QString();
  }

  return output;
}

bool RosLogViewer::hasErrors() const {
  for (const LogEntry& entry : allLogEntries_) {
    if (entry.level == "ERROR" || entry.level == "FATAL") {
      return true;
    }
  }
  return false;
}

void RosLogViewer::onAnalyzeErrorsClicked() {
  QString errorLogs = getErrorLogs();
  if (errorLogs.isEmpty()) {
    QMessageBox::information(this, tr("No Errors"),
                             tr("There are no ERROR or FATAL level logs to analyze."));
    return;
  }

  emit analyzeErrorsRequested(errorLogs);
}

// =============================================================================
// TerminalWidget
// =============================================================================

TerminalWidget::TerminalWidget(QWidget* parent)
  : QWidget(parent)
  , process_(nullptr)
{
  setupUi();
}

TerminalWidget::~TerminalWidget() {
  stopCommand();
}

void TerminalWidget::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(4);

  // Output view
  outputView_ = new QTextEdit();
  outputView_->setReadOnly(true);
  outputView_->setFont(QFont("Monospace", 10));
  outputView_->setStyleSheet(
    "QTextEdit { background-color: #1e1e1e; color: #d4d4d4; }"
  );
  layout->addWidget(outputView_);

  // Input bar
  QHBoxLayout* inputLayout = new QHBoxLayout();

  QLabel* promptLabel = new QLabel("$");
  promptLabel->setStyleSheet("font-weight: bold; color: #4ec9b0;");

  inputEdit_ = new QLineEdit();
  inputEdit_->setPlaceholderText(tr("Enter command..."));
  inputEdit_->setFont(QFont("Monospace", 10));
  connect(inputEdit_, &QLineEdit::returnPressed, this, &TerminalWidget::onInputEntered);

  runButton_ = new QPushButton(tr("Run"));
  connect(runButton_, &QPushButton::clicked, this, &TerminalWidget::onInputEntered);

  stopButton_ = new QPushButton(tr("Stop"));
  stopButton_->setEnabled(false);
  connect(stopButton_, &QPushButton::clicked, this, &TerminalWidget::stopCommand);

  clearButton_ = new QPushButton(tr("Clear"));
  connect(clearButton_, &QPushButton::clicked, this, &TerminalWidget::clear);

  inputLayout->addWidget(promptLabel);
  inputLayout->addWidget(inputEdit_);
  inputLayout->addWidget(runButton_);
  inputLayout->addWidget(stopButton_);
  inputLayout->addWidget(clearButton_);

  layout->addLayout(inputLayout);

  // Command history dropdown
  QHBoxLayout* historyLayout = new QHBoxLayout();
  QLabel* historyLabel = new QLabel(tr("History:"));
  commandHistory_ = new QComboBox();
  commandHistory_->setMinimumWidth(300);
  commandHistory_->addItem(tr("(recent commands)"));
  connect(commandHistory_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int index) {
    if (index > 0) {
      inputEdit_->setText(commandHistory_->itemText(index));
      commandHistory_->setCurrentIndex(0);
    }
  });

  historyLayout->addWidget(historyLabel);
  historyLayout->addWidget(commandHistory_);
  historyLayout->addStretch();

  layout->addLayout(historyLayout);

  // Initialize process
  process_ = new QProcess(this);
  connect(process_, &QProcess::readyReadStandardOutput,
          this, &TerminalWidget::onReadyReadStdout);
  connect(process_, &QProcess::readyReadStandardError,
          this, &TerminalWidget::onReadyReadStderr);
  connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &TerminalWidget::onProcessFinished);
  connect(process_, &QProcess::errorOccurred,
          this, &TerminalWidget::onProcessError);
}

void TerminalWidget::runCommand(const QString& command, const QString& workingDir) {
  if (command.isEmpty()) return;
  if (isRunning()) {
    appendError(tr("A command is already running. Stop it first."));
    return;
  }

  currentCommand_ = command;

  // Add to history
  if (!commandHistoryList_.contains(command)) {
    commandHistoryList_.prepend(command);
    if (commandHistoryList_.size() > 20) {
      commandHistoryList_.removeLast();
    }

    // Update combo box
    commandHistory_->clear();
    commandHistory_->addItem(tr("(recent commands)"));
    commandHistory_->addItems(commandHistoryList_);
  }

  // Show command in output
  appendOutput(QString("$ %1\n").arg(command), QColor("#4ec9b0"));

  // Set working directory if provided
  if (!workingDir.isEmpty()) {
    process_->setWorkingDirectory(workingDir);
  }

  // Start command using bash
  process_->start("/bin/bash", QStringList() << "-c" << command);

  runButton_->setEnabled(false);
  stopButton_->setEnabled(true);
  inputEdit_->setEnabled(false);

  emit commandStarted(command);
}

void TerminalWidget::stopCommand() {
  if (process_ && process_->state() != QProcess::NotRunning) {
    process_->terminate();
    if (!process_->waitForFinished(3000)) {
      process_->kill();
    }
  }
}

void TerminalWidget::clear() {
  outputView_->clear();
}

bool TerminalWidget::isRunning() const {
  return process_ && process_->state() != QProcess::NotRunning;
}

void TerminalWidget::onReadyReadStdout() {
  QString output = QString::fromLocal8Bit(process_->readAllStandardOutput());
  appendOutput(output);
  emit outputReceived(output);
}

void TerminalWidget::onReadyReadStderr() {
  QString output = QString::fromLocal8Bit(process_->readAllStandardError());
  appendError(output);
  emit outputReceived(output);
}

void TerminalWidget::onProcessFinished(int exitCode, QProcess::ExitStatus status) {
  Q_UNUSED(status)

  if (exitCode == 0) {
    appendOutput(tr("\n[Process completed successfully]\n"), QColor("#4ec9b0"));
  } else {
    appendOutput(tr("\n[Process exited with code %1]\n").arg(exitCode), QColor("#f14c4c"));
  }

  runButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  inputEdit_->setEnabled(true);
  inputEdit_->setFocus();

  emit commandFinished(exitCode);
}

void TerminalWidget::onProcessError(QProcess::ProcessError error) {
  QString errorMsg;
  switch (error) {
    case QProcess::FailedToStart:
      errorMsg = tr("Failed to start process");
      break;
    case QProcess::Crashed:
      errorMsg = tr("Process crashed");
      break;
    case QProcess::Timedout:
      errorMsg = tr("Process timed out");
      break;
    default:
      errorMsg = tr("Unknown error");
      break;
  }
  appendError(tr("[Error: %1]\n").arg(errorMsg));

  runButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  inputEdit_->setEnabled(true);
}

void TerminalWidget::onInputEntered() {
  QString command = inputEdit_->text().trimmed();
  if (!command.isEmpty()) {
    inputEdit_->clear();
    runCommand(command);
  }
}

void TerminalWidget::appendOutput(const QString& text, const QColor& color) {
  QTextCursor cursor = outputView_->textCursor();
  cursor.movePosition(QTextCursor::End);

  QTextCharFormat format;
  if (color.isValid()) {
    format.setForeground(color);
  } else {
    format.setForeground(QColor("#d4d4d4"));
  }

  cursor.insertText(text, format);
  outputView_->setTextCursor(cursor);
  outputView_->ensureCursorVisible();
}

void TerminalWidget::appendError(const QString& text) {
  appendOutput(text, QColor("#f14c4c"));
}

// =============================================================================
// OutputPanel
// =============================================================================

OutputPanel::OutputPanel(QWidget* parent)
  : QWidget(parent)
{
  setupUi();
}

OutputPanel::~OutputPanel() = default;

void OutputPanel::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  tabWidget_ = new QTabWidget();
  tabWidget_->setTabPosition(QTabWidget::South);

  // Build/Generation Output tab
  QWidget* buildTab = new QWidget();
  QVBoxLayout* buildLayout = new QVBoxLayout(buildTab);
  buildLayout->setContentsMargins(4, 4, 4, 4);
  buildLayout->setSpacing(4);

  buildOutput_ = new QTextEdit();
  buildOutput_->setReadOnly(true);
  buildOutput_->setPlaceholderText(tr("Build and generation output will appear here..."));
  buildOutput_->setFont(QFont("Monospace", 10));
  buildLayout->addWidget(buildOutput_);

  progressBar_ = new QProgressBar();
  progressBar_->setVisible(false);
  buildLayout->addWidget(progressBar_);

  // Use Unicode icons for tab labels for better visual clarity
  tabWidget_->addTab(buildTab, QString::fromUtf8("\xF0\x9F\x94\xA7 ") + tr("Output"));  // Wrench

  // ROS Log tab
  rosLogViewer_ = new RosLogViewer();
  tabWidget_->addTab(rosLogViewer_, QString::fromUtf8("\xF0\x9F\x93\x9C ") + tr("ROS Logs"));  // Scroll

  // Terminal tab
  terminalWidget_ = new TerminalWidget();
  tabWidget_->addTab(terminalWidget_, QString::fromUtf8("\xF0\x9F\x92\xBB ") + tr("Terminal"));  // Computer

  // LLM Chat tab
  llmChatWidget_ = new LLMChatWidget();
  tabWidget_->addTab(llmChatWidget_, QString::fromUtf8("\xF0\x9F\xA4\x96 ") + tr("LocalLLM"));  // Robot

  layout->addWidget(tabWidget_);

  // Connect tab change signal
  connect(tabWidget_, &QTabWidget::currentChanged, this, &OutputPanel::tabChanged);

  // Connect analyze errors signal
  connect(rosLogViewer_, &RosLogViewer::analyzeErrorsRequested,
          this, &OutputPanel::onAnalyzeErrorsRequested);
}

void OutputPanel::onAnalyzeErrorsRequested(const QString& errorLogs) {
  // Build the analysis request prompt
  QString prompt = QString(
      "Please analyze the following ROS2 error logs and explain what might be going wrong. "
      "Provide:\n"
      "1. A summary of the errors\n"
      "2. Likely causes for each error\n"
      "3. Suggested solutions or troubleshooting steps\n\n"
      "Error Logs:\n```\n%1\n```").arg(errorLogs);

  // Switch to the LLM chat tab
  showLLMChatTab();

  // Send the analysis request to the LLM
  llmChatWidget_->sendMessage(prompt);
}

void OutputPanel::showProgress(bool show) {
  progressBar_->setVisible(show);
  if (show) {
    showBuildTab();
  }
}

void OutputPanel::appendBuildOutput(const QString& text) {
  buildOutput_->append(text);
}

void OutputPanel::appendBuildError(const QString& text) {
  QTextCursor cursor = buildOutput_->textCursor();
  cursor.movePosition(QTextCursor::End);

  QTextCharFormat format;
  format.setForeground(QColor("#f14c4c"));
  cursor.insertText(text + "\n", format);

  buildOutput_->setTextCursor(cursor);
  buildOutput_->ensureCursorVisible();
}

void OutputPanel::clearBuildOutput() {
  buildOutput_->clear();
}

void OutputPanel::showBuildTab() {
  tabWidget_->setCurrentIndex(0);
}

void OutputPanel::showRosLogTab() {
  tabWidget_->setCurrentIndex(1);
}

void OutputPanel::showTerminalTab() {
  tabWidget_->setCurrentIndex(2);
}

void OutputPanel::showLLMChatTab() {
  tabWidget_->setCurrentIndex(3);
}

}  // namespace ros_weaver
