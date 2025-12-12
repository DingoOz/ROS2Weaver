#ifndef ROS_WEAVER_WIDGETS_OUTPUT_PANEL_HPP
#define ROS_WEAVER_WIDGETS_OUTPUT_PANEL_HPP

#include <QWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QTreeWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QProgressBar>
#include <QProcess>
#include <QMutex>
#include <QTimer>
#include <QDateTime>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <queue>

namespace ros_weaver {

// Log entry structure
struct LogEntry {
  QDateTime timestamp;
  QString level;
  QString nodeName;
  QString message;
  QString file;
  QString function;
  int line;
};

// ROS Log Viewer Widget
class RosLogViewer : public QWidget {
  Q_OBJECT

public:
  explicit RosLogViewer(QWidget* parent = nullptr);
  ~RosLogViewer() override;

  // Start/stop log subscription
  void startListening();
  void stopListening();
  bool isListening() const { return isListening_; }

  // Clear logs
  void clear();

  // Export logs
  QString exportToText() const;

signals:
  void logReceived(const LogEntry& entry);
  void connectionStatusChanged(bool connected);

private slots:
  void onLogReceived(const LogEntry& entry);
  void onFilterChanged();
  void onClearClicked();
  void onAutoScrollToggled(bool checked);
  void onExportClicked();
  void processLogQueue();

private:
  void setupUi();
  void addLogEntry(const LogEntry& entry);
  QColor getColorForLevel(const QString& level) const;
  QIcon getIconForLevel(const QString& level) const;
  bool passesFilter(const LogEntry& entry) const;

  // ROS2 node and subscription (run in separate thread)
  void rosSpinThread();
  std::shared_ptr<rclcpp::Node> rosNode_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr logSubscription_;
  std::thread spinThread_;
  std::atomic<bool> isListening_;
  std::atomic<bool> shouldStop_;

  // Thread-safe log queue
  std::queue<LogEntry> logQueue_;
  QMutex queueMutex_;
  QTimer* queueTimer_;

  // UI components
  QTreeWidget* logTree_;
  QComboBox* levelFilter_;
  QLineEdit* nodeFilter_;
  QLineEdit* messageFilter_;
  QCheckBox* autoScrollCheck_;
  QPushButton* startStopButton_;
  QPushButton* clearButton_;
  QPushButton* exportButton_;
  QLabel* statusLabel_;

  // Settings
  int maxLogEntries_ = 10000;
};

// Terminal Widget for running commands
class TerminalWidget : public QWidget {
  Q_OBJECT

public:
  explicit TerminalWidget(QWidget* parent = nullptr);
  ~TerminalWidget() override;

  // Run a command
  void runCommand(const QString& command, const QString& workingDir = QString());

  // Stop current command
  void stopCommand();

  // Clear terminal
  void clear();

  // Check if command is running
  bool isRunning() const;

signals:
  void commandStarted(const QString& command);
  void commandFinished(int exitCode);
  void outputReceived(const QString& output);

private slots:
  void onReadyReadStdout();
  void onReadyReadStderr();
  void onProcessFinished(int exitCode, QProcess::ExitStatus status);
  void onProcessError(QProcess::ProcessError error);
  void onInputEntered();

private:
  void setupUi();
  void appendOutput(const QString& text, const QColor& color = QColor());
  void appendError(const QString& text);

  QTextEdit* outputView_;
  QLineEdit* inputEdit_;
  QPushButton* runButton_;
  QPushButton* stopButton_;
  QPushButton* clearButton_;
  QComboBox* commandHistory_;

  QProcess* process_;
  QString currentCommand_;
  QStringList commandHistoryList_;
};

// Main Output Panel with tabs
class OutputPanel : public QWidget {
  Q_OBJECT

public:
  explicit OutputPanel(QWidget* parent = nullptr);
  ~OutputPanel() override;

  // Access to individual widgets
  QTextEdit* buildOutput() const { return buildOutput_; }
  RosLogViewer* rosLogViewer() const { return rosLogViewer_; }
  TerminalWidget* terminalWidget() const { return terminalWidget_; }

  // Progress bar access
  QProgressBar* progressBar() const { return progressBar_; }
  void showProgress(bool show);

  // Convenience methods for build output
  void appendBuildOutput(const QString& text);
  void appendBuildError(const QString& text);
  void clearBuildOutput();

  // Switch to specific tab
  void showBuildTab();
  void showRosLogTab();
  void showTerminalTab();

signals:
  void tabChanged(int index);

private:
  void setupUi();

  QTabWidget* tabWidget_;

  // Build/Generation output tab
  QTextEdit* buildOutput_;
  QProgressBar* progressBar_;

  // ROS Log viewer tab
  RosLogViewer* rosLogViewer_;

  // Terminal tab
  TerminalWidget* terminalWidget_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_OUTPUT_PANEL_HPP
