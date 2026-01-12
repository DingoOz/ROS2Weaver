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
    case Theme::HighContrast: return QObject::tr("High Contrast");
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

QPalette ThemeManager::highContrastPalette() const {
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(0, 0, 0));
  palette.setColor(QPalette::WindowText, Qt::white);
  palette.setColor(QPalette::Base, QColor(0, 0, 0));
  palette.setColor(QPalette::AlternateBase, QColor(20, 20, 20));
  palette.setColor(QPalette::ToolTipBase, QColor(0, 0, 0));
  palette.setColor(QPalette::ToolTipText, Qt::white);
  palette.setColor(QPalette::Text, Qt::white);
  palette.setColor(QPalette::Button, QColor(30, 30, 30));
  palette.setColor(QPalette::ButtonText, Qt::white);
  palette.setColor(QPalette::BrightText, QColor(255, 255, 0));
  palette.setColor(QPalette::Link, QColor(0, 200, 255));
  palette.setColor(QPalette::Highlight, QColor(255, 255, 0));
  palette.setColor(QPalette::HighlightedText, Qt::black);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(100, 100, 100));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(100, 100, 100));
  return palette;
}

QPalette ThemeManager::currentPalette() const {
  switch (currentTheme_) {
    case Theme::Light: return lightPalette();
    case Theme::HighContrast: return highContrastPalette();
    case Theme::Dark:
    default: return darkPalette();
  }
}

QString ThemeManager::currentStyleSheet() const {
  if (currentTheme_ == Theme::Light) {
    return
      "QToolTip { color: #1e1e1e; background-color: #ffffdc; border: 1px solid #c0c0c0; }"
      "QMenu { background-color: #ffffff; border: 1px solid #c0c0c0; }"
      "QMenu::item:selected { background-color: #2a82da; color: white; }"
      "QMenuBar { background-color: #f0f0f0; }"
      "QMenuBar::item:selected { background-color: #2a82da; color: white; }"
      // VS Code-like dock widget styling (light theme)
      "QDockWidget { font-weight: bold; }"
      "QDockWidget::title { background: #e8e8e8; padding: 6px; border-bottom: 1px solid #c0c0c0; }"
      "QDockWidget::close-button, QDockWidget::float-button { background: transparent; border: none; padding: 2px; }"
      "QDockWidget::close-button:hover, QDockWidget::float-button:hover { background: #d0d0d0; border-radius: 2px; }"
      "QMainWindow::separator { background: #c0c0c0; width: 4px; height: 4px; }"
      "QMainWindow::separator:hover { background: #2a82da; }"
      "QTabBar::tab { padding: 6px 12px; background: #f0f0f0; border: 1px solid #c0c0c0; border-bottom: none; }"
      "QTabBar::tab:selected { background: #ffffff; border-bottom: 2px solid #2a82da; }"
      "QTabBar::tab:hover:!selected { background: #e0e0e0; }";
  } else if (currentTheme_ == Theme::HighContrast) {
    return
      "QToolTip { color: #ffffff; background-color: #000000; border: 2px solid #ffff00; }"
      "QMenu { background-color: #000000; border: 2px solid #ffffff; }"
      "QMenu::item:selected { background-color: #ffff00; color: #000000; }"
      "QMenuBar { background-color: #000000; }"
      "QMenuBar::item:selected { background-color: #ffff00; color: #000000; }"
      // VS Code-like dock widget styling (high contrast)
      "QDockWidget { font-weight: bold; }"
      "QDockWidget::title { background: #1a1a1a; padding: 6px; border-bottom: 2px solid #ffff00; }"
      "QDockWidget::close-button, QDockWidget::float-button { background: transparent; border: none; padding: 2px; }"
      "QDockWidget::close-button:hover, QDockWidget::float-button:hover { background: #ffff00; }"
      "QMainWindow::separator { background: #ffffff; width: 4px; height: 4px; }"
      "QMainWindow::separator:hover { background: #ffff00; }"
      "QTabBar::tab { padding: 6px 12px; background: #000000; border: 2px solid #ffffff; }"
      "QTabBar::tab:selected { background: #1a1a1a; border-bottom: 2px solid #ffff00; }"
      "QTabBar::tab:hover:!selected { background: #333333; }";
  } else {
    return
      "QToolTip { color: #ffffff; background-color: #2a2a2a; border: 1px solid #3a3a3a; }"
      "QMenu { background-color: #353535; border: 1px solid #3a3a3a; }"
      "QMenu::item:selected { background-color: #2a82da; }"
      "QMenuBar { background-color: #353535; }"
      "QMenuBar::item:selected { background-color: #2a82da; }"
      // VS Code-like dock widget styling (dark theme)
      "QDockWidget { font-weight: bold; }"
      "QDockWidget::title { background: #2d2d2d; padding: 6px; border-bottom: 1px solid #3a3a3a; }"
      "QDockWidget::close-button, QDockWidget::float-button { background: transparent; border: none; padding: 2px; }"
      "QDockWidget::close-button:hover, QDockWidget::float-button:hover { background: #404040; border-radius: 2px; }"
      "QMainWindow::separator { background: #3a3a3a; width: 4px; height: 4px; }"
      "QMainWindow::separator:hover { background: #2a82da; }"
      "QTabBar::tab { padding: 6px 12px; background: #2d2d2d; border: 1px solid #3a3a3a; border-bottom: none; }"
      "QTabBar::tab:selected { background: #353535; border-bottom: 2px solid #2a82da; }"
      "QTabBar::tab:hover:!selected { background: #404040; }";
  }
}

// =============================================================================
// Semantic Colors
// =============================================================================

QColor ThemeManager::primaryColor() const {
  if (customAccentColor_.isValid()) return customAccentColor_;
  if (currentTheme_ == Theme::HighContrast) return QColor(255, 255, 0);
  return QColor(42, 130, 218);  // Blue
}

QColor ThemeManager::primaryHoverColor() const {
  return primaryColor().lighter(115);
}

QColor ThemeManager::primaryPressedColor() const {
  return primaryColor().darker(110);
}

QColor ThemeManager::backgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(240, 240, 240);
    case Theme::HighContrast: return QColor(0, 0, 0);
    case Theme::Dark:
    default: return QColor(53, 53, 53);
  }
}

QColor ThemeManager::surfaceColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(255, 255, 255);
    case Theme::HighContrast: return QColor(20, 20, 20);
    case Theme::Dark:
    default: return QColor(42, 42, 42);
  }
}

QColor ThemeManager::surfaceHoverColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(245, 245, 245);
    case Theme::HighContrast: return QColor(40, 40, 40);
    case Theme::Dark:
    default: return QColor(55, 55, 55);
  }
}

QColor ThemeManager::elevatedSurfaceColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(255, 255, 255);
    case Theme::HighContrast: return QColor(30, 30, 30);
    case Theme::Dark:
    default: return QColor(58, 58, 58);
  }
}

QColor ThemeManager::textPrimaryColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(30, 30, 30);
    case Theme::HighContrast: return Qt::white;
    case Theme::Dark:
    default: return Qt::white;
  }
}

QColor ThemeManager::textSecondaryColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(100, 100, 100);
    case Theme::HighContrast: return QColor(200, 200, 200);
    case Theme::Dark:
    default: return QColor(160, 160, 160);
  }
}

QColor ThemeManager::textDisabledColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(160, 160, 160);
    case Theme::HighContrast: return QColor(100, 100, 100);
    case Theme::Dark:
    default: return QColor(100, 100, 100);
  }
}

QColor ThemeManager::textOnPrimaryColor() const {
  if (currentTheme_ == Theme::HighContrast) return Qt::black;
  return Qt::white;
}

QColor ThemeManager::successColor() const {
  if (currentTheme_ == Theme::HighContrast) return QColor(0, 255, 0);
  return QColor(76, 175, 80);  // Green
}

QColor ThemeManager::successBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(232, 245, 233);
    case Theme::HighContrast: return QColor(0, 40, 0);
    case Theme::Dark:
    default: return QColor(45, 90, 61);
  }
}

QColor ThemeManager::warningColor() const {
  if (currentTheme_ == Theme::HighContrast) return QColor(255, 200, 0);
  return QColor(255, 152, 0);  // Orange
}

QColor ThemeManager::warningBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(255, 243, 224);
    case Theme::HighContrast: return QColor(60, 40, 0);
    case Theme::Dark:
    default: return QColor(90, 74, 45);
  }
}

QColor ThemeManager::errorColor() const {
  if (currentTheme_ == Theme::HighContrast) return QColor(255, 0, 0);
  return QColor(244, 67, 54);  // Red
}

QColor ThemeManager::errorBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(255, 235, 238);
    case Theme::HighContrast: return QColor(60, 0, 0);
    case Theme::Dark:
    default: return QColor(90, 45, 45);
  }
}

QColor ThemeManager::infoColor() const {
  if (currentTheme_ == Theme::HighContrast) return QColor(0, 200, 255);
  return QColor(33, 150, 243);  // Blue
}

QColor ThemeManager::infoBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(227, 242, 253);
    case Theme::HighContrast: return QColor(0, 30, 50);
    case Theme::Dark:
    default: return QColor(45, 58, 90);
  }
}

QColor ThemeManager::borderColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(200, 200, 200);
    case Theme::HighContrast: return Qt::white;
    case Theme::Dark:
    default: return QColor(80, 80, 80);
  }
}

QColor ThemeManager::borderHoverColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(150, 150, 150);
    case Theme::HighContrast: return QColor(255, 255, 0);
    case Theme::Dark:
    default: return QColor(100, 100, 100);
  }
}

QColor ThemeManager::borderFocusColor() const {
  return primaryColor();
}

QColor ThemeManager::canvasBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(245, 245, 245);
    case Theme::HighContrast: return QColor(10, 10, 10);
    case Theme::Dark:
    default: return QColor(35, 35, 35);
  }
}

QColor ThemeManager::canvasGridMinorColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(220, 220, 220);
    case Theme::HighContrast: return QColor(30, 30, 30);
    case Theme::Dark:
    default: return QColor(50, 50, 50);
  }
}

QColor ThemeManager::canvasGridMajorColor() const {
  switch (currentTheme_) {
    case Theme::Light: return QColor(200, 200, 200);
    case Theme::HighContrast: return QColor(50, 50, 50);
    case Theme::Dark:
    default: return QColor(60, 60, 60);
  }
}

QColor ThemeManager::inputBackgroundColor() const {
  switch (currentTheme_) {
    case Theme::Light: return Qt::white;
    case Theme::HighContrast: return QColor(20, 20, 20);
    case Theme::Dark:
    default: return QColor(58, 58, 58);
  }
}

QColor ThemeManager::inputBorderColor() const {
  return borderColor();
}

QColor ThemeManager::inputFocusBorderColor() const {
  return primaryColor();
}

// =============================================================================
// Component Stylesheets
// =============================================================================

QString ThemeManager::primaryButtonStyle() const {
  return QString(
      "QPushButton {"
      "  background-color: %1;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: %4;"
      "  font-weight: bold;"
      "  padding: 6px 16px;"
      "  min-height: 24px;"
      "}"
      "QPushButton:hover {"
      "  background-color: %2;"
      "}"
      "QPushButton:pressed {"
      "  background-color: %3;"
      "}"
      "QPushButton:disabled {"
      "  background-color: %5;"
      "  color: %6;"
      "}")
      .arg(primaryColor().name())
      .arg(primaryHoverColor().name())
      .arg(primaryPressedColor().name())
      .arg(textOnPrimaryColor().name())
      .arg(surfaceColor().name())
      .arg(textDisabledColor().name());
}

QString ThemeManager::secondaryButtonStyle() const {
  return QString(
      "QPushButton {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 4px;"
      "  color: %3;"
      "  padding: 6px 16px;"
      "  min-height: 24px;"
      "}"
      "QPushButton:hover {"
      "  background-color: %4;"
      "  border-color: %5;"
      "}"
      "QPushButton:pressed {"
      "  background-color: %6;"
      "}"
      "QPushButton:disabled {"
      "  background-color: %7;"
      "  color: %8;"
      "  border-color: %7;"
      "}")
      .arg(surfaceColor().name())
      .arg(borderColor().name())
      .arg(textPrimaryColor().name())
      .arg(surfaceHoverColor().name())
      .arg(borderHoverColor().name())
      .arg(backgroundColor().name())
      .arg(surfaceColor().name())
      .arg(textDisabledColor().name());
}

QString ThemeManager::destructiveButtonStyle() const {
  return QString(
      "QPushButton {"
      "  background-color: %1;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: white;"
      "  font-weight: bold;"
      "  padding: 6px 16px;"
      "  min-height: 24px;"
      "}"
      "QPushButton:hover {"
      "  background-color: %2;"
      "}"
      "QPushButton:pressed {"
      "  background-color: %3;"
      "}"
      "QPushButton:disabled {"
      "  background-color: %4;"
      "  color: %5;"
      "}")
      .arg(errorColor().name())
      .arg(errorColor().lighter(115).name())
      .arg(errorColor().darker(110).name())
      .arg(surfaceColor().name())
      .arg(textDisabledColor().name());
}

QString ThemeManager::ghostButtonStyle() const {
  return QString(
      "QPushButton {"
      "  background-color: transparent;"
      "  border: none;"
      "  border-radius: 4px;"
      "  color: %1;"
      "  padding: 6px 16px;"
      "  min-height: 24px;"
      "}"
      "QPushButton:hover {"
      "  background-color: %2;"
      "}"
      "QPushButton:pressed {"
      "  background-color: %3;"
      "}"
      "QPushButton:disabled {"
      "  color: %4;"
      "}")
      .arg(primaryColor().name())
      .arg(surfaceHoverColor().name())
      .arg(surfaceColor().name())
      .arg(textDisabledColor().name());
}

QString ThemeManager::iconButtonStyle() const {
  return QString(
      "QPushButton {"
      "  background-color: transparent;"
      "  border: none;"
      "  border-radius: 4px;"
      "  padding: 4px;"
      "}"
      "QPushButton:hover {"
      "  background-color: %1;"
      "}"
      "QPushButton:pressed {"
      "  background-color: %2;"
      "}")
      .arg(surfaceHoverColor().name())
      .arg(surfaceColor().name());
}

QString ThemeManager::lineEditStyle() const {
  return QString(
      "QLineEdit {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 4px;"
      "  padding: 6px 10px;"
      "  color: %3;"
      "}"
      "QLineEdit:focus {"
      "  border-color: %4;"
      "}"
      "QLineEdit:disabled {"
      "  background-color: %5;"
      "  color: %6;"
      "}")
      .arg(inputBackgroundColor().name())
      .arg(inputBorderColor().name())
      .arg(textPrimaryColor().name())
      .arg(inputFocusBorderColor().name())
      .arg(surfaceColor().name())
      .arg(textDisabledColor().name());
}

QString ThemeManager::comboBoxStyle() const {
  return QString(
      "QComboBox {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 4px;"
      "  padding: 6px 10px;"
      "  color: %3;"
      "}"
      "QComboBox:hover {"
      "  border-color: %4;"
      "}"
      "QComboBox:focus {"
      "  border-color: %5;"
      "}"
      "QComboBox::drop-down {"
      "  border: none;"
      "  width: 20px;"
      "}"
      "QComboBox QAbstractItemView {"
      "  background-color: %1;"
      "  color: %3;"
      "  selection-background-color: %5;"
      "}")
      .arg(inputBackgroundColor().name())
      .arg(inputBorderColor().name())
      .arg(textPrimaryColor().name())
      .arg(borderHoverColor().name())
      .arg(primaryColor().name());
}

QString ThemeManager::spinBoxStyle() const {
  return QString(
      "QSpinBox, QDoubleSpinBox {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 4px;"
      "  padding: 4px 8px;"
      "  color: %3;"
      "}"
      "QSpinBox:focus, QDoubleSpinBox:focus {"
      "  border-color: %4;"
      "}")
      .arg(inputBackgroundColor().name())
      .arg(inputBorderColor().name())
      .arg(textPrimaryColor().name())
      .arg(inputFocusBorderColor().name());
}

QString ThemeManager::panelStyle() const {
  return QString(
      "background-color: %1;"
      "border: 1px solid %2;")
      .arg(surfaceColor().name())
      .arg(borderColor().name());
}

QString ThemeManager::cardStyle() const {
  return QString(
      "background-color: %1;"
      "border: 1px solid %2;"
      "border-radius: 8px;")
      .arg(elevatedSurfaceColor().name())
      .arg(borderColor().name());
}

QString ThemeManager::tooltipStyle() const {
  return QString(
      "QToolTip {"
      "  color: %1;"
      "  background-color: %2;"
      "  border: 1px solid %3;"
      "  border-radius: 4px;"
      "  padding: 4px 8px;"
      "}")
      .arg(textPrimaryColor().name())
      .arg(elevatedSurfaceColor().name())
      .arg(borderColor().name());
}

// =============================================================================
// Settings
// =============================================================================

void ThemeManager::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("Appearance");
  int themeValue = settings.value("theme", static_cast<int>(Theme::Dark)).toInt();
  currentTheme_ = static_cast<Theme>(themeValue);

  // Load custom accent color
  QString accentColorStr = settings.value("accentColor", "").toString();
  if (!accentColorStr.isEmpty()) {
    customAccentColor_ = QColor(accentColorStr);
  }

  settings.endGroup();
}

void ThemeManager::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("Appearance");
  settings.setValue("theme", static_cast<int>(currentTheme_));

  if (customAccentColor_.isValid()) {
    settings.setValue("accentColor", customAccentColor_.name());
  } else {
    settings.remove("accentColor");
  }

  settings.endGroup();
}

void ThemeManager::applyTheme() {
  QApplication* app = qobject_cast<QApplication*>(QApplication::instance());
  if (app) {
    app->setPalette(currentPalette());
    app->setStyleSheet(currentStyleSheet());
  }
}

void ThemeManager::setAccentColor(const QColor& color) {
  if (customAccentColor_ != color) {
    customAccentColor_ = color;
    saveSettings();
    emit accentColorChanged(color);
  }
}

void ThemeManager::resetAccentColor() {
  if (customAccentColor_.isValid()) {
    customAccentColor_ = QColor();  // Invalid = use default
    saveSettings();
    emit accentColorChanged(primaryColor());
  }
}

}  // namespace ros_weaver
