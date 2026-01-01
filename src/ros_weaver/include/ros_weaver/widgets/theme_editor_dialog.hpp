#ifndef ROS_WEAVER_WIDGETS_THEME_EDITOR_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_THEME_EDITOR_DIALOG_HPP

#include <QDialog>
#include <QListWidget>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QFrame>
#include <QMap>
#include <QColor>

namespace ros_weaver {

class ThemeManager;

// Widget for editing a single color
class ColorEditWidget : public QWidget {
  Q_OBJECT

public:
  ColorEditWidget(const QString& name, const QColor& color, QWidget* parent = nullptr);

  QString colorName() const { return name_; }
  QColor color() const { return color_; }
  void setColor(const QColor& color);

signals:
  void colorChanged(const QString& name, const QColor& color);

private slots:
  void onPickColor();

private:
  QString name_;
  QColor color_;
  QFrame* colorPreview_;
  QLabel* colorLabel_;
  QPushButton* pickButton_;
};

// Theme editor dialog
class ThemeEditorDialog : public QDialog {
  Q_OBJECT

public:
  explicit ThemeEditorDialog(QWidget* parent = nullptr);
  ~ThemeEditorDialog() override = default;

  // Get/set theme name
  QString themeName() const;
  void setThemeName(const QString& name);

  // Load a theme for editing
  void loadTheme(const QString& themeName);

  // Create a new theme
  void createNewTheme();

signals:
  void themeApplied(const QString& themeName);

private slots:
  void onThemeSelected(QListWidgetItem* item);
  void onNewTheme();
  void onDuplicateTheme();
  void onDeleteTheme();
  void onColorChanged(const QString& name, const QColor& color);
  void onApply();
  void onSave();
  void onRevert();
  void updatePreview();

private:
  void setupUI();
  void refreshThemeList();
  void loadThemeColors(const QString& themeName);
  void saveCurrentTheme();

  // UI elements
  QListWidget* themeList_;
  QLineEdit* themeNameEdit_;
  QWidget* colorsPanel_;
  QList<ColorEditWidget*> colorEditors_;
  QFrame* previewFrame_;
  QLabel* previewLabel_;

  QPushButton* newButton_;
  QPushButton* duplicateButton_;
  QPushButton* deleteButton_;
  QPushButton* applyButton_;
  QPushButton* saveButton_;
  QPushButton* revertButton_;

  // Current theme data
  QString currentThemeName_;
  QMap<QString, QColor> currentColors_;
  QMap<QString, QColor> originalColors_;
  bool hasUnsavedChanges_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_THEME_EDITOR_DIALOG_HPP
