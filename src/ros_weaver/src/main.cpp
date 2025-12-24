#include <QApplication>
#include <QStyleFactory>
#include <QDebug>
#include <iostream>
#include "ros_weaver/main_window.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include <rclcpp/rclcpp.hpp>

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

  // Shutdown ROS2 before Qt destroys widgets (prevents hanging)
  // This is called after event loop exits but before widget destruction
  QObject::connect(&app, &QApplication::aboutToQuit, []() {
    std::cerr << "aboutToQuit: rclcpp::ok() = " << rclcpp::ok() << std::endl;
    if (rclcpp::ok()) {
      std::cerr << "aboutToQuit: calling rclcpp::shutdown()" << std::endl;
      rclcpp::shutdown();
      std::cerr << "aboutToQuit: rclcpp::shutdown() completed" << std::endl;
    }
  });

  std::cerr << "Creating MainWindow..." << std::endl;
  {
    ros_weaver::MainWindow mainWindow;
    mainWindow.show();

    std::cerr << "Starting event loop" << std::endl;
    int result = app.exec();
    std::cerr << "Event loop exited with result: " << result << std::endl;
    std::cerr << "MainWindow going out of scope..." << std::endl;
  }
  std::cerr << "MainWindow destroyed, returning from main()" << std::endl;

  return 0;
}
