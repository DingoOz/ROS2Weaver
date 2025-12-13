#ifndef ROS_WEAVER_CONTEXT_HELP_HPP
#define ROS_WEAVER_CONTEXT_HELP_HPP

#include <QWidget>
#include <QString>
#include <QMap>
#include <QEvent>

namespace ros_weaver {

class HelpBrowser;

// Context-sensitive help system
// Provides F1 help for widgets and manages help topic associations
class ContextHelp : public QObject {
  Q_OBJECT

public:
  static ContextHelp* instance();

  // Register a widget with a help topic
  static void registerWidget(QWidget* widget, const QString& topicId);

  // Unregister a widget
  static void unregisterWidget(QWidget* widget);

  // Get the help topic for a widget (searches up the parent chain)
  static QString getHelpTopic(QWidget* widget);

  // Show context help for a widget
  static void showHelp(QWidget* widget);

  // Show help for a specific topic
  static void showTopic(const QString& topicId);

  // Show the getting started page
  static void showGettingStarted();

  // Show keyboard shortcuts
  static void showKeyboardShortcuts();

protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

private:
  ContextHelp();
  ~ContextHelp() override;

  void installHelpFilter(QWidget* widget);

  static ContextHelp* instance_;
  QMap<QWidget*, QString> widgetTopics_;
};

// Convenience macro for registering widgets
#define REGISTER_HELP_TOPIC(widget, topic) \
  ros_weaver::ContextHelp::registerWidget(widget, topic)

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CONTEXT_HELP_HPP
