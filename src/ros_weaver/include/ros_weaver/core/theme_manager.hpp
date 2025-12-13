#ifndef ROS_WEAVER_THEME_MANAGER_HPP
#define ROS_WEAVER_THEME_MANAGER_HPP

#include <QObject>
#include <QPalette>
#include <QString>

namespace ros_weaver {

enum class Theme {
  Dark,
  Light
};

class ThemeManager : public QObject {
  Q_OBJECT

public:
  static ThemeManager& instance();

  // Get/set current theme
  Theme currentTheme() const { return currentTheme_; }
  void setTheme(Theme theme);

  // Get theme name for display
  static QString themeName(Theme theme);

  // Get palettes
  QPalette darkPalette() const;
  QPalette lightPalette() const;
  QPalette currentPalette() const;

  // Get stylesheet for current theme
  QString currentStyleSheet() const;

  // Load/save theme preference
  void loadSettings();
  void saveSettings();

  // Apply theme to application
  void applyTheme();

signals:
  void themeChanged(Theme newTheme);

private:
  ThemeManager();
  ~ThemeManager() = default;
  ThemeManager(const ThemeManager&) = delete;
  ThemeManager& operator=(const ThemeManager&) = delete;

  Theme currentTheme_ = Theme::Dark;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_THEME_MANAGER_HPP
