#ifndef ROS_WEAVER_WIDGETS_TOPIC_ECHO_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_TOPIC_ECHO_DIALOG_HPP

#include <QDialog>
#include <QTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

namespace ros_weaver {

/**
 * @brief Floating dialog that shows live ros2 topic echo output
 *
 * This dialog spawns a ros2 topic echo process and displays the output
 * in real-time. It provides an easy way to close (X button, Escape key,
 * or Close button) which also kills the running process.
 */
class TopicEchoDialog : public QDialog {
  Q_OBJECT

public:
  explicit TopicEchoDialog(const QString& topicName, QWidget* parent = nullptr);
  ~TopicEchoDialog() override;

  // Get the topic name being echoed
  QString topicName() const { return topicName_; }

  // Check if the echo process is running
  bool isEchoRunning() const;

public slots:
  // Start echoing the topic
  void startEcho();

  // Stop echoing the topic
  void stopEcho();

  // Clear the output display
  void clearOutput();

  // Toggle pause state
  void togglePause();

protected:
  void closeEvent(QCloseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

private slots:
  void onProcessReadyRead();
  void onProcessErrorOccurred(QProcess::ProcessError error);
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onMessageCountUpdate();

private:
  void setupUi();
  void setupConnections();
  void appendOutput(const QString& text);

  // The topic name to echo
  QString topicName_;

  // The ros2 topic echo process
  QProcess* echoProcess_;

  // UI components
  QLabel* topicLabel_;
  QLabel* statusLabel_;
  QLabel* messageCountLabel_;
  QTextEdit* outputTextEdit_;
  QPushButton* closeButton_;
  QPushButton* clearButton_;
  QPushButton* pauseButton_;

  // State
  bool isPaused_;
  int messageCount_;
  QTimer* messageCountTimer_;

  // Buffer for partial lines
  QString lineBuffer_;

  // Maximum lines to keep in the output (to prevent memory issues)
  static constexpr int MAX_OUTPUT_LINES = 1000;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_TOPIC_ECHO_DIALOG_HPP
