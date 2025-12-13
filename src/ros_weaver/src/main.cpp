#include <QApplication>
#include <QStyleFactory>
#include "ros_weaver/main_window.hpp"
#include "ros_weaver/core/theme_manager.hpp"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  // Set application info
  app.setApplicationName("ROS Weaver");
  app.setApplicationVersion("0.1.0");
  app.setOrganizationName("ROS Weaver");

  // Use Fusion style for consistent cross-platform look
  app.setStyle(QStyleFactory::create("Fusion"));

  // Apply the saved theme (or default to dark)
  ros_weaver::ThemeManager& themeManager = ros_weaver::ThemeManager::instance();
  themeManager.applyTheme();

  ros_weaver::MainWindow mainWindow;
  mainWindow.show();

  return app.exec();
}
