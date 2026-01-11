#include "ros_weaver/core/context_help.hpp"
#include "ros_weaver/widgets/help_browser.hpp"

#include <QApplication>
#include <QKeyEvent>

namespace ros_weaver {

ContextHelp* ContextHelp::instance_ = nullptr;

ContextHelp* ContextHelp::instance() {
  if (!instance_) {
    instance_ = new ContextHelp();
  }
  return instance_;
}

ContextHelp::ContextHelp()
  : QObject(nullptr)
{
  // Install event filter on all top-level widgets
  qApp->installEventFilter(this);
}

ContextHelp::~ContextHelp() {
  if (instance_ == this) {
    instance_ = nullptr;
  }
}

void ContextHelp::registerWidget(QWidget* widget, const QString& topicId) {
  if (!widget || topicId.isEmpty()) {
    return;
  }

  instance()->widgetTopics_[widget] = topicId;

  // Clean up when widget is destroyed
  connect(widget, &QWidget::destroyed, instance(), [widget]() {
    instance()->widgetTopics_.remove(widget);
  });
}

void ContextHelp::unregisterWidget(QWidget* widget) {
  if (widget) {
    instance()->widgetTopics_.remove(widget);
  }
}

QString ContextHelp::getHelpTopic(QWidget* widget) {
  if (!widget) {
    return QString();
  }

  // Check this widget
  if (instance()->widgetTopics_.contains(widget)) {
    return instance()->widgetTopics_[widget];
  }

  // Check widget's class name for automatic topic mapping
  QString className = widget->metaObject()->className();

  // Map common widget types to help topics
  static const QMap<QString, QString> classTopicMap = {
    {"ros_weaver::WeaverCanvas", "canvas-editor"},
    {"ros_weaver::ParamDashboard", "properties-panel"},
    {"ros_weaver::TopicViewerPanel", "ros2-integration"},
    {"ros_weaver::TFTreePanel", "ros2-integration"},
    {"ros_weaver::PlotPanel", "plot-panel"},
    {"ros_weaver::SystemMappingPanel", "ros2-integration"},
    {"ros_weaver::LLMChatWidget", "local-ai"},
    {"ros_weaver::OllamaSettingsWidget", "local-ai"},
    {"ros_weaver::LocalAIStatusWidget", "local-ai"},
    {"ros_weaver::OutputPanel", "ros2-integration"},
    {"ros_weaver::PackageWizard", "code-generation"},
    {"ros_weaver::LineageDialog", "data-lineage"},
    {"ros_weaver::GuidedTour", "guided-tour"},
    {"ros_weaver::TourTooltip", "guided-tour"},
    {"ros_weaver::KeyboardShortcutsDialog", "keyboard-shortcuts"},
    {"ros_weaver::HelpBrowser", "getting-started"},
    {"ros_weaver::NodeHealthDashboard", "node-health-dashboard"},
    {"ros_weaver::WorkspaceBrowserPanel", "workspace-browser"},
    {"ros_weaver::NodeTemplatesPanel", "node-templates"},
    {"ros_weaver::RosbagWorkbenchPanel", "rosbag-workbench"},
    {"ros_weaver::NetworkTopologyPanel", "network-topology"},
  };

  if (classTopicMap.contains(className)) {
    return classTopicMap[className];
  }

  // Search up parent chain
  QWidget* parent = widget->parentWidget();
  if (parent) {
    return getHelpTopic(parent);
  }

  // Default to getting started
  return "getting-started";
}

void ContextHelp::showHelp(QWidget* widget) {
  QString topic = getHelpTopic(widget);
  showTopic(topic);
}

void ContextHelp::showTopic(const QString& topicId) {
  QWidget* mainWindow = nullptr;
  for (QWidget* w : qApp->topLevelWidgets()) {
    if (w->isWindow() && w->isVisible()) {
      mainWindow = w;
      break;
    }
  }

  HelpBrowser* browser = HelpBrowser::instance(mainWindow);
  browser->showTopic(topicId.isEmpty() ? "getting-started" : topicId);
  browser->show();
  browser->raise();
  browser->activateWindow();
}

void ContextHelp::showGettingStarted() {
  showTopic("getting-started");
}

void ContextHelp::showKeyboardShortcuts() {
  showTopic("keyboard-shortcuts");
}

bool ContextHelp::eventFilter(QObject* watched, QEvent* event) {
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);

    // F1 for context help
    if (keyEvent->key() == Qt::Key_F1) {
      QWidget* focusWidget = QApplication::focusWidget();

      if (keyEvent->modifiers() == Qt::ControlModifier) {
        // Ctrl+F1 always goes to Getting Started
        showGettingStarted();
      } else if (keyEvent->modifiers() == Qt::NoModifier) {
        // F1 shows context help for focused widget
        if (focusWidget) {
          showHelp(focusWidget);
        } else {
          showGettingStarted();
        }
      }

      return true;  // Event handled
    }

    // Ctrl+/ for keyboard shortcuts
    if (keyEvent->key() == Qt::Key_Slash && keyEvent->modifiers() == Qt::ControlModifier) {
      showKeyboardShortcuts();
      return true;
    }
  }

  return QObject::eventFilter(watched, event);
}

}  // namespace ros_weaver
