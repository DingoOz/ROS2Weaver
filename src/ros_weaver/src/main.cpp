#include <QApplication>
#include <QStyleFactory>
#include "ros_weaver/main_window.hpp"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  // Set application info
  app.setApplicationName("ROS Weaver");
  app.setApplicationVersion("0.1.0");
  app.setOrganizationName("ROS Weaver");

  // Use Fusion style for consistent cross-platform look
  app.setStyle(QStyleFactory::create("Fusion"));

  // Create dark palette for modern look
  QPalette darkPalette;
  darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::WindowText, Qt::white);
  darkPalette.setColor(QPalette::Base, QColor(42, 42, 42));
  darkPalette.setColor(QPalette::AlternateBase, QColor(66, 66, 66));
  darkPalette.setColor(QPalette::ToolTipBase, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::ToolTipText, Qt::white);
  darkPalette.setColor(QPalette::Text, Qt::white);
  darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::ButtonText, Qt::white);
  darkPalette.setColor(QPalette::BrightText, Qt::red);
  darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::HighlightedText, Qt::black);
  darkPalette.setColor(QPalette::Disabled, QPalette::Text, QColor(127, 127, 127));
  darkPalette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(127, 127, 127));

  app.setPalette(darkPalette);

  // Set stylesheet for additional styling
  app.setStyleSheet(
    "QToolTip { color: #ffffff; background-color: #2a2a2a; border: 1px solid #3a3a3a; }"
    "QMenu { background-color: #353535; border: 1px solid #3a3a3a; }"
    "QMenu::item:selected { background-color: #2a82da; }"
    "QMenuBar { background-color: #353535; }"
    "QMenuBar::item:selected { background-color: #2a82da; }"
  );

  ros_weaver::MainWindow mainWindow;
  mainWindow.show();

  return app.exec();
}
