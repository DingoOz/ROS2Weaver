#include "ros_weaver/core/theme_manager.hpp"
#include <QApplication>
#include <QSettings>
#include <QStyleFactory>

namespace ros_weaver {

ThemeManager& ThemeManager::instance() {
  static ThemeManager instance;
  return instance;
}

ThemeManager::ThemeManager() {
  loadSettings();
}

QString ThemeManager::themeName(Theme theme) {
  switch (theme) {
    case Theme::Dark: return QObject::tr("Dark");
    case Theme::Light: return QObject::tr("Light");
    default: return QObject::tr("Unknown");
  }
}

void ThemeManager::setTheme(Theme theme) {
  if (currentTheme_ != theme) {
    currentTheme_ = theme;
    saveSettings();
    applyTheme();
    emit themeChanged(theme);
  }
}

QPalette ThemeManager::darkPalette() const {
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(53, 53, 53));
  palette.setColor(QPalette::WindowText, Qt::white);
  palette.setColor(QPalette::Base, QColor(42, 42, 42));
  palette.setColor(QPalette::AlternateBase, QColor(66, 66, 66));
  palette.setColor(QPalette::ToolTipBase, QColor(53, 53, 53));
  palette.setColor(QPalette::ToolTipText, Qt::white);
  palette.setColor(QPalette::Text, Qt::white);
  palette.setColor(QPalette::Button, QColor(53, 53, 53));
  palette.setColor(QPalette::ButtonText, Qt::white);
  palette.setColor(QPalette::BrightText, Qt::red);
  palette.setColor(QPalette::Link, QColor(42, 130, 218));
  palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  palette.setColor(QPalette::HighlightedText, Qt::black);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(127, 127, 127));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(127, 127, 127));
  return palette;
}

QPalette ThemeManager::lightPalette() const {
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(240, 240, 240));
  palette.setColor(QPalette::WindowText, QColor(30, 30, 30));
  palette.setColor(QPalette::Base, QColor(255, 255, 255));
  palette.setColor(QPalette::AlternateBase, QColor(245, 245, 245));
  palette.setColor(QPalette::ToolTipBase, QColor(255, 255, 220));
  palette.setColor(QPalette::ToolTipText, QColor(30, 30, 30));
  palette.setColor(QPalette::Text, QColor(30, 30, 30));
  palette.setColor(QPalette::Button, QColor(240, 240, 240));
  palette.setColor(QPalette::ButtonText, QColor(30, 30, 30));
  palette.setColor(QPalette::BrightText, Qt::red);
  palette.setColor(QPalette::Link, QColor(0, 100, 200));
  palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(160, 160, 160));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(160, 160, 160));
  return palette;
}

QPalette ThemeManager::currentPalette() const {
  return (currentTheme_ == Theme::Dark) ? darkPalette() : lightPalette();
}

QString ThemeManager::currentStyleSheet() const {
  if (currentTheme_ == Theme::Dark) {
    return
      "QToolTip { color: #ffffff; background-color: #2a2a2a; border: 1px solid #3a3a3a; }"
      "QMenu { background-color: #353535; border: 1px solid #3a3a3a; }"
      "QMenu::item:selected { background-color: #2a82da; }"
      "QMenuBar { background-color: #353535; }"
      "QMenuBar::item:selected { background-color: #2a82da; }";
  } else {
    return
      "QToolTip { color: #1e1e1e; background-color: #ffffdc; border: 1px solid #c0c0c0; }"
      "QMenu { background-color: #ffffff; border: 1px solid #c0c0c0; }"
      "QMenu::item:selected { background-color: #2a82da; color: white; }"
      "QMenuBar { background-color: #f0f0f0; }"
      "QMenuBar::item:selected { background-color: #2a82da; color: white; }";
  }
}

void ThemeManager::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("Appearance");
  int themeValue = settings.value("theme", static_cast<int>(Theme::Dark)).toInt();
  currentTheme_ = static_cast<Theme>(themeValue);
  settings.endGroup();
}

void ThemeManager::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("Appearance");
  settings.setValue("theme", static_cast<int>(currentTheme_));
  settings.endGroup();
}

void ThemeManager::applyTheme() {
  QApplication* app = qobject_cast<QApplication*>(QApplication::instance());
  if (app) {
    app->setPalette(currentPalette());
    app->setStyleSheet(currentStyleSheet());
  }
}

}  // namespace ros_weaver
