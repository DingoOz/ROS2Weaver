#ifndef ROS_WEAVER_KEYBOARD_SHORTCUTS_DIALOG_HPP
#define ROS_WEAVER_KEYBOARD_SHORTCUTS_DIALOG_HPP

#include <QDialog>
#include <QTreeWidget>
#include <QLineEdit>
#include <QLabel>

namespace ros_weaver {

// Structure representing a keyboard shortcut
struct ShortcutInfo {
  QString category;
  QString action;
  QString shortcut;
  QString description;
};

class KeyboardShortcutsDialog : public QDialog {
  Q_OBJECT

public:
  explicit KeyboardShortcutsDialog(QWidget* parent = nullptr);
  ~KeyboardShortcutsDialog() override;

private slots:
  void onSearchTextChanged(const QString& text);

private:
  void setupUi();
  void loadShortcuts();
  void filterShortcuts(const QString& filter);

  QLineEdit* searchEdit_;
  QTreeWidget* shortcutsTree_;
  QLabel* countLabel_;

  QList<ShortcutInfo> shortcuts_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_KEYBOARD_SHORTCUTS_DIALOG_HPP
