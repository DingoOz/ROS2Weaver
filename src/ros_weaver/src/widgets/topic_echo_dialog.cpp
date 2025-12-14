#include "ros_weaver/widgets/topic_echo_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCloseEvent>
#include <QKeyEvent>
#include <QScrollBar>
#include <QFont>
#include <QApplication>
#include <QStyle>

namespace ros_weaver {

TopicEchoDialog::TopicEchoDialog(const QString& topicName, QWidget* parent)
  : QDialog(parent)
  , topicName_(topicName)
  , echoProcess_(nullptr)
  , isPaused_(false)
  , messageCount_(0)
  , messageCountTimer_(nullptr)
{
  setupUi();
  setupConnections();

  // Start echoing immediately
  startEcho();
}

TopicEchoDialog::~TopicEchoDialog() {
  stopEcho();
}

void TopicEchoDialog::setupUi() {
  setWindowTitle(tr("Topic Echo: %1").arg(topicName_));
  setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint | Qt::WindowMinMaxButtonsHint);
  setAttribute(Qt::WA_DeleteOnClose);
  resize(600, 400);

  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(6);

  // Header with topic info and status
  QHBoxLayout* headerLayout = new QHBoxLayout();

  topicLabel_ = new QLabel(tr("<b>Topic:</b> %1").arg(topicName_));
  headerLayout->addWidget(topicLabel_);

  headerLayout->addStretch();

  messageCountLabel_ = new QLabel(tr("Messages: 0"));
  headerLayout->addWidget(messageCountLabel_);

  statusLabel_ = new QLabel(tr("Starting..."));
  statusLabel_->setStyleSheet("QLabel { color: #999; }");
  headerLayout->addWidget(statusLabel_);

  mainLayout->addLayout(headerLayout);

  // Output text area
  outputTextEdit_ = new QTextEdit();
  outputTextEdit_->setReadOnly(true);
  outputTextEdit_->setFont(QFont("Monospace", 9));
  outputTextEdit_->setStyleSheet(
    "QTextEdit {"
    "  background-color: #1e1e1e;"
    "  color: #d4d4d4;"
    "  border: 1px solid #3c3c3c;"
    "  border-radius: 4px;"
    "}"
  );
  mainLayout->addWidget(outputTextEdit_, 1);

  // Button row
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  pauseButton_ = new QPushButton(tr("Pause"));
  pauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
  buttonLayout->addWidget(pauseButton_);

  clearButton_ = new QPushButton(tr("Clear"));
  clearButton_->setIcon(style()->standardIcon(QStyle::SP_DialogResetButton));
  buttonLayout->addWidget(clearButton_);

  buttonLayout->addStretch();

  closeButton_ = new QPushButton(tr("Close"));
  closeButton_->setIcon(style()->standardIcon(QStyle::SP_DialogCloseButton));
  closeButton_->setDefault(true);
  buttonLayout->addWidget(closeButton_);

  mainLayout->addLayout(buttonLayout);

  // Timer for updating message count display
  messageCountTimer_ = new QTimer(this);
  messageCountTimer_->setInterval(250);
}

void TopicEchoDialog::setupConnections() {
  connect(closeButton_, &QPushButton::clicked, this, &QDialog::close);
  connect(clearButton_, &QPushButton::clicked, this, &TopicEchoDialog::clearOutput);
  connect(pauseButton_, &QPushButton::clicked, this, &TopicEchoDialog::togglePause);
  connect(messageCountTimer_, &QTimer::timeout, this, &TopicEchoDialog::onMessageCountUpdate);
}

bool TopicEchoDialog::isEchoRunning() const {
  return echoProcess_ && echoProcess_->state() == QProcess::Running;
}

void TopicEchoDialog::startEcho() {
  if (echoProcess_) {
    stopEcho();
  }

  echoProcess_ = new QProcess(this);

  connect(echoProcess_, &QProcess::readyReadStandardOutput,
          this, &TopicEchoDialog::onProcessReadyRead);
  connect(echoProcess_, &QProcess::readyReadStandardError,
          this, &TopicEchoDialog::onProcessReadyRead);
  connect(echoProcess_, &QProcess::errorOccurred,
          this, &TopicEchoDialog::onProcessErrorOccurred);
  connect(echoProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &TopicEchoDialog::onProcessFinished);

  // Start ros2 topic echo
  QStringList args;
  args << "topic" << "echo" << topicName_;

  echoProcess_->start("ros2", args);

  statusLabel_->setText(tr("Running"));
  statusLabel_->setStyleSheet("QLabel { color: #4ec9b0; }");

  messageCount_ = 0;
  messageCountTimer_->start();
}

void TopicEchoDialog::stopEcho() {
  if (messageCountTimer_) {
    messageCountTimer_->stop();
  }

  if (echoProcess_) {
    if (echoProcess_->state() != QProcess::NotRunning) {
      // Disconnect signals to prevent callbacks after termination
      echoProcess_->disconnect();

      // Terminate gracefully first
      echoProcess_->terminate();

      // Wait briefly for graceful termination
      if (!echoProcess_->waitForFinished(500)) {
        // Force kill if not terminated
        echoProcess_->kill();
        echoProcess_->waitForFinished(500);
      }
    }

    delete echoProcess_;
    echoProcess_ = nullptr;
  }

  statusLabel_->setText(tr("Stopped"));
  statusLabel_->setStyleSheet("QLabel { color: #999; }");
}

void TopicEchoDialog::clearOutput() {
  outputTextEdit_->clear();
  messageCount_ = 0;
  messageCountLabel_->setText(tr("Messages: 0"));
}

void TopicEchoDialog::togglePause() {
  isPaused_ = !isPaused_;

  if (isPaused_) {
    pauseButton_->setText(tr("Resume"));
    pauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    statusLabel_->setText(tr("Paused"));
    statusLabel_->setStyleSheet("QLabel { color: #dcdcaa; }");
  } else {
    pauseButton_->setText(tr("Pause"));
    pauseButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    statusLabel_->setText(tr("Running"));
    statusLabel_->setStyleSheet("QLabel { color: #4ec9b0; }");
  }
}

void TopicEchoDialog::closeEvent(QCloseEvent* event) {
  stopEcho();
  event->accept();
}

void TopicEchoDialog::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    close();
    return;
  }
  QDialog::keyPressEvent(event);
}

void TopicEchoDialog::onProcessReadyRead() {
  if (!echoProcess_ || isPaused_) {
    return;
  }

  // Read both stdout and stderr
  QByteArray data = echoProcess_->readAllStandardOutput();
  data += echoProcess_->readAllStandardError();

  if (data.isEmpty()) {
    return;
  }

  QString text = QString::fromUtf8(data);
  appendOutput(text);
}

void TopicEchoDialog::onProcessErrorOccurred(QProcess::ProcessError error) {
  QString errorMsg;
  switch (error) {
    case QProcess::FailedToStart:
      errorMsg = tr("Failed to start ros2 topic echo. Is ROS 2 installed and sourced?");
      break;
    case QProcess::Crashed:
      errorMsg = tr("The ros2 process crashed.");
      break;
    case QProcess::Timedout:
      errorMsg = tr("The ros2 process timed out.");
      break;
    case QProcess::WriteError:
      errorMsg = tr("Write error occurred.");
      break;
    case QProcess::ReadError:
      errorMsg = tr("Read error occurred.");
      break;
    default:
      errorMsg = tr("Unknown error occurred.");
      break;
  }

  appendOutput(tr("\n[ERROR] %1\n").arg(errorMsg));
  statusLabel_->setText(tr("Error"));
  statusLabel_->setStyleSheet("QLabel { color: #f14c4c; }");
}

void TopicEchoDialog::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  Q_UNUSED(exitCode)

  if (exitStatus == QProcess::CrashExit) {
    appendOutput(tr("\n[Process crashed]\n"));
  } else {
    appendOutput(tr("\n[Process finished]\n"));
  }

  statusLabel_->setText(tr("Finished"));
  statusLabel_->setStyleSheet("QLabel { color: #999; }");
  messageCountTimer_->stop();
}

void TopicEchoDialog::onMessageCountUpdate() {
  messageCountLabel_->setText(tr("Messages: %1").arg(messageCount_));
}

void TopicEchoDialog::appendOutput(const QString& text) {
  // Add to line buffer
  lineBuffer_ += text;

  // Process complete lines
  int lastNewline = lineBuffer_.lastIndexOf('\n');
  if (lastNewline == -1) {
    return;  // No complete lines yet
  }

  QString completeLines = lineBuffer_.left(lastNewline + 1);
  lineBuffer_ = lineBuffer_.mid(lastNewline + 1);

  // Count messages (each "---" separator indicates a new message)
  messageCount_ += completeLines.count("---");

  // Auto-scroll if at bottom
  QScrollBar* scrollBar = outputTextEdit_->verticalScrollBar();
  bool wasAtBottom = scrollBar->value() >= scrollBar->maximum() - 10;

  // Append text
  QTextCursor cursor = outputTextEdit_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(completeLines);

  // Trim if too many lines
  int blockCount = outputTextEdit_->document()->blockCount();
  if (blockCount > MAX_OUTPUT_LINES) {
    int linesToRemove = blockCount - MAX_OUTPUT_LINES;
    cursor.movePosition(QTextCursor::Start);
    for (int i = 0; i < linesToRemove; ++i) {
      cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor);
    }
    cursor.removeSelectedText();
  }

  // Auto-scroll if was at bottom
  if (wasAtBottom) {
    scrollBar->setValue(scrollBar->maximum());
  }
}

}  // namespace ros_weaver
