#ifndef ROS_WEAVER_THEME_MANAGER_HPP
#define ROS_WEAVER_THEME_MANAGER_HPP

#include <QObject>
#include <QPalette>
#include <QString>
#include <QColor>

namespace ros_weaver {

enum class Theme {
  Dark,
  Light,
  HighContrast
};

/**
 * @brief Extended theme manager with comprehensive color support
 *
 * Provides semantic colors for all UI components to ensure
 * consistency and proper theming throughout the application.
 */
class ThemeManager : public QObject {
  Q_OBJECT

public:
  static ThemeManager& instance();

  // Get/set current theme
  Theme currentTheme() const { return currentTheme_; }
  void setTheme(Theme theme);
  bool isDarkTheme() const { return currentTheme_ == Theme::Dark || currentTheme_ == Theme::HighContrast; }

  // Get theme name for display
  static QString themeName(Theme theme);

  // Get palettes
  QPalette darkPalette() const;
  QPalette lightPalette() const;
  QPalette highContrastPalette() const;
  QPalette currentPalette() const;

  // Get stylesheet for current theme
  QString currentStyleSheet() const;

  // ==========================================================================
  // Semantic Colors - use these instead of hardcoded values
  // ==========================================================================

  // Primary brand colors
  QColor primaryColor() const;
  QColor primaryHoverColor() const;
  QColor primaryPressedColor() const;

  // Surface colors
  QColor backgroundColor() const;
  QColor surfaceColor() const;
  QColor surfaceHoverColor() const;
  QColor elevatedSurfaceColor() const;

  // Text colors
  QColor textPrimaryColor() const;
  QColor textSecondaryColor() const;
  QColor textDisabledColor() const;
  QColor textOnPrimaryColor() const;

  // Status colors
  QColor successColor() const;
  QColor successBackgroundColor() const;
  QColor warningColor() const;
  QColor warningBackgroundColor() const;
  QColor errorColor() const;
  QColor errorBackgroundColor() const;
  QColor infoColor() const;
  QColor infoBackgroundColor() const;

  // Border colors
  QColor borderColor() const;
  QColor borderHoverColor() const;
  QColor borderFocusColor() const;

  // Canvas-specific colors
  QColor canvasBackgroundColor() const;
  QColor canvasGridMinorColor() const;
  QColor canvasGridMajorColor() const;

  // Input field colors
  QColor inputBackgroundColor() const;
  QColor inputBorderColor() const;
  QColor inputFocusBorderColor() const;

  // ==========================================================================
  // Component Stylesheets - pre-built styles for common components
  // ==========================================================================

  // Button styles
  QString primaryButtonStyle() const;
  QString secondaryButtonStyle() const;
  QString destructiveButtonStyle() const;
  QString ghostButtonStyle() const;
  QString iconButtonStyle() const;

  // Input styles
  QString lineEditStyle() const;
  QString comboBoxStyle() const;
  QString spinBoxStyle() const;

  // Container styles
  QString panelStyle() const;
  QString cardStyle() const;
  QString tooltipStyle() const;

  // Load/save theme preference
  void loadSettings();
  void saveSettings();

  // Apply theme to application
  void applyTheme();

  // Custom accent color
  QColor accentColor() const { return customAccentColor_.isValid() ? customAccentColor_ : primaryColor(); }
  void setAccentColor(const QColor& color);
  void resetAccentColor();

  // Dock button scale
  double dockButtonScale() const { return dockButtonScale_; }
  void setDockButtonScale(double scale);

signals:
  void themeChanged(Theme newTheme);
  void accentColorChanged(const QColor& color);
  void dockButtonScaleChanged(double scale);

private:
  ThemeManager();
  ~ThemeManager() = default;
  ThemeManager(const ThemeManager&) = delete;
  ThemeManager& operator=(const ThemeManager&) = delete;

  Theme currentTheme_ = Theme::Dark;
  QColor customAccentColor_;
  double dockButtonScale_ = 1.5;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_THEME_MANAGER_HPP
