#ifndef ROS_WEAVER_COMMAND_PALETTE_HPP
#define ROS_WEAVER_COMMAND_PALETTE_HPP

#include <QDialog>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>
#include <QKeyEvent>
#include <QAction>
#include <QList>
#include <QMap>

namespace ros_weaver {

/**
 * @brief Command entry for the palette
 */
struct CommandEntry {
  QString id;           // Unique identifier
  QString name;         // Display name
  QString category;     // Category for grouping
  QString shortcut;     // Keyboard shortcut display text
  QString description;  // Optional description
  QAction* action;      // The action to trigger (can be nullptr for custom handling)

  bool matches(const QString& query) const;
};

/**
 * @brief Command palette for quick access to all application commands
 *
 * Similar to VS Code's Ctrl+Shift+P command palette.
 * Provides fuzzy search over all available commands.
 */
class CommandPalette : public QDialog {
  Q_OBJECT

public:
  explicit CommandPalette(QWidget* parent = nullptr);
  ~CommandPalette() override;

  // Register commands
  void addCommand(const CommandEntry& command);
  void addCommand(const QString& id, const QString& name, const QString& category,
                  QAction* action, const QString& description = QString());
  void addCommands(const QList<CommandEntry>& commands);
  void removeCommand(const QString& id);
  void clearCommands();

  // Show the palette
  void showPalette();

signals:
  void commandExecuted(const QString& commandId);

protected:
  void keyPressEvent(QKeyEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
  void onSearchTextChanged(const QString& text);
  void onItemActivated(QListWidgetItem* item);
  void onItemClicked(QListWidgetItem* item);

private:
  void setupUi();
  void updateResults(const QString& query);
  void executeSelectedCommand();
  void selectNext();
  void selectPrevious();
  QListWidgetItem* createItemForCommand(const CommandEntry& cmd);
  void addToRecentCommands(const QString& commandId);

  QLineEdit* searchEdit_;
  QListWidget* resultsList_;
  QList<CommandEntry> allCommands_;
  QList<CommandEntry> filteredCommands_;
  QStringList recentCommands_;

  static constexpr int MAX_VISIBLE_RESULTS = 10;
  static constexpr int MAX_RECENT_COMMANDS = 5;
};

/**
 * @brief Custom list item for command display
 */
class CommandListItem : public QListWidgetItem {
public:
  explicit CommandListItem(const CommandEntry& command);

  QString commandId() const { return commandId_; }

private:
  QString commandId_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_COMMAND_PALETTE_HPP
