#include <QApplication>
#include <QStyleFactory>
#include <QDebug>
#include "ros_weaver/main_window.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // Disable native menu bar before creating QApplication (fixes issues on some Linux desktops)
  QApplication::setAttribute(Qt::AA_DontUseNativeMenuBar);

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

  {
    ros_weaver::MainWindow mainWindow;
    mainWindow.show();
    app.exec();
  }

  // MainWindow and all child widgets are now destroyed.
  // All panel destructors have stopped their spin threads and cleared ROS2 nodes.
  // Now it's safe to call rclcpp::shutdown() without hanging.
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
