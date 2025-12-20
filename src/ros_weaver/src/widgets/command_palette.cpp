#include "ros_weaver/widgets/command_palette.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QApplication>
#include <QLabel>
#include <QScreen>
#include <QSettings>
#include <algorithm>

namespace ros_weaver {

// =============================================================================
// CommandEntry
// =============================================================================

bool CommandEntry::matches(const QString& query) const {
  if (query.isEmpty()) return true;

  QString lowerQuery = query.toLower();
  QString lowerName = name.toLower();
  QString lowerCategory = category.toLower();
  QString lowerDesc = description.toLower();

  // Simple substring matching (could be improved with fuzzy matching)
  if (lowerName.contains(lowerQuery)) return true;
  if (lowerCategory.contains(lowerQuery)) return true;
  if (lowerDesc.contains(lowerQuery)) return true;

  // Match against words
  QStringList queryWords = lowerQuery.split(' ', Qt::SkipEmptyParts);
  for (const QString& word : queryWords) {
    if (!lowerName.contains(word) && !lowerCategory.contains(word)) {
      return false;
    }
  }

  return true;
}

// =============================================================================
// CommandListItem
// =============================================================================

CommandListItem::CommandListItem(const CommandEntry& command)
    : QListWidgetItem()
    , commandId_(command.id) {
  // Format: "Category: Name    Shortcut"
  QString displayText = command.name;
  setText(displayText);

  // Store command info in user role
  setData(Qt::UserRole, command.id);
  setData(Qt::UserRole + 1, command.shortcut);
  setData(Qt::UserRole + 2, command.category);
  setData(Qt::UserRole + 3, command.description);
}

// =============================================================================
// CommandPalette
// =============================================================================

CommandPalette::CommandPalette(QWidget* parent)
    : QDialog(parent, Qt::Popup | Qt::FramelessWindowHint)
    , searchEdit_(nullptr)
    , resultsList_(nullptr) {
  setAttribute(Qt::WA_TranslucentBackground);
  setModal(true);

  setupUi();

  // Load recent commands from settings
  QSettings settings("ROS Weaver", "ROS Weaver");
  recentCommands_ = settings.value("CommandPalette/recent").toStringList();
}

CommandPalette::~CommandPalette() {
  // Save recent commands
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.setValue("CommandPalette/recent", recentCommands_);
}

void CommandPalette::setupUi() {
  setFixedWidth(600);
  setMaximumHeight(450);

  auto& theme = ThemeManager::instance();

  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  // Container with styling
  QWidget* container = new QWidget(this);
  container->setObjectName("paletteContainer");
  container->setStyleSheet(QString(
      "#paletteContainer {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 8px;"
      "}")
      .arg(theme.elevatedSurfaceColor().name())
      .arg(theme.borderColor().name()));

  QVBoxLayout* containerLayout = new QVBoxLayout(container);
  containerLayout->setContentsMargins(8, 8, 8, 8);
  containerLayout->setSpacing(4);

  // Search input
  searchEdit_ = new QLineEdit(container);
  searchEdit_->setPlaceholderText(tr("Type a command or search..."));
  searchEdit_->setStyleSheet(QString(
      "QLineEdit {"
      "  background-color: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 6px;"
      "  padding: 10px 14px;"
      "  font-size: 14px;"
      "  color: %3;"
      "}"
      "QLineEdit:focus {"
      "  border-color: %4;"
      "}")
      .arg(theme.inputBackgroundColor().name())
      .arg(theme.borderColor().name())
      .arg(theme.textPrimaryColor().name())
      .arg(theme.primaryColor().name()));
  searchEdit_->installEventFilter(this);
  connect(searchEdit_, &QLineEdit::textChanged, this, &CommandPalette::onSearchTextChanged);
  containerLayout->addWidget(searchEdit_);

  // Results list
  resultsList_ = new QListWidget(container);
  resultsList_->setFrameShape(QFrame::NoFrame);
  resultsList_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  resultsList_->setStyleSheet(QString(
      "QListWidget {"
      "  background-color: transparent;"
      "  border: none;"
      "  outline: none;"
      "}"
      "QListWidget::item {"
      "  padding: 10px 14px;"
      "  border-radius: 4px;"
      "  color: %1;"
      "}"
      "QListWidget::item:selected {"
      "  background-color: %2;"
      "}"
      "QListWidget::item:hover {"
      "  background-color: %3;"
      "}")
      .arg(theme.textPrimaryColor().name())
      .arg(theme.primaryColor().name())
      .arg(theme.surfaceHoverColor().name()));

  connect(resultsList_, &QListWidget::itemActivated, this, &CommandPalette::onItemActivated);
  connect(resultsList_, &QListWidget::itemClicked, this, &CommandPalette::onItemClicked);
  containerLayout->addWidget(resultsList_);

  // Hint label
  QLabel* hintLabel = new QLabel(tr("Use arrow keys to navigate, Enter to execute, Esc to close"), container);
  hintLabel->setStyleSheet(QString(
      "color: %1;"
      "font-size: 11px;"
      "padding: 6px;")
      .arg(theme.textSecondaryColor().name()));
  hintLabel->setAlignment(Qt::AlignCenter);
  containerLayout->addWidget(hintLabel);

  mainLayout->addWidget(container);
}

void CommandPalette::addCommand(const CommandEntry& command) {
  // Remove if already exists
  allCommands_.erase(
      std::remove_if(allCommands_.begin(), allCommands_.end(),
                     [&](const CommandEntry& c) { return c.id == command.id; }),
      allCommands_.end());
  allCommands_.append(command);
}

void CommandPalette::addCommand(const QString& id, const QString& name,
                                 const QString& category, QAction* action,
                                 const QString& description) {
  CommandEntry cmd;
  cmd.id = id;
  cmd.name = name;
  cmd.category = category;
  cmd.action = action;
  cmd.description = description;

  if (action && !action->shortcut().isEmpty()) {
    cmd.shortcut = action->shortcut().toString(QKeySequence::NativeText);
  }

  addCommand(cmd);
}

void CommandPalette::addCommands(const QList<CommandEntry>& commands) {
  for (const CommandEntry& cmd : commands) {
    addCommand(cmd);
  }
}

void CommandPalette::removeCommand(const QString& id) {
  allCommands_.erase(
      std::remove_if(allCommands_.begin(), allCommands_.end(),
                     [&](const CommandEntry& c) { return c.id == id; }),
      allCommands_.end());
}

void CommandPalette::clearCommands() {
  allCommands_.clear();
}

void CommandPalette::showPalette() {
  // Position at top center of parent window
  if (parentWidget()) {
    QRect parentRect = parentWidget()->geometry();
    int x = parentRect.x() + (parentRect.width() - width()) / 2;
    int y = parentRect.y() + 80;  // Some offset from top
    move(x, y);
  } else {
    // Center on screen
    QScreen* screen = QApplication::primaryScreen();
    if (screen) {
      QRect screenRect = screen->availableGeometry();
      int x = screenRect.x() + (screenRect.width() - width()) / 2;
      int y = screenRect.y() + 80;
      move(x, y);
    }
  }

  searchEdit_->clear();
  updateResults("");
  show();
  searchEdit_->setFocus();
}

void CommandPalette::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Escape:
      hide();
      break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      executeSelectedCommand();
      break;
    case Qt::Key_Up:
      selectPrevious();
      break;
    case Qt::Key_Down:
      selectNext();
      break;
    default:
      QDialog::keyPressEvent(event);
  }
}

bool CommandPalette::eventFilter(QObject* watched, QEvent* event) {
  if (watched == searchEdit_ && event->type() == QEvent::KeyPress) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
    if (keyEvent->key() == Qt::Key_Up) {
      selectPrevious();
      return true;
    } else if (keyEvent->key() == Qt::Key_Down) {
      selectNext();
      return true;
    } else if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
      executeSelectedCommand();
      return true;
    }
  }
  return QDialog::eventFilter(watched, event);
}

void CommandPalette::onSearchTextChanged(const QString& text) {
  updateResults(text);
}

void CommandPalette::onItemActivated(QListWidgetItem* item) {
  Q_UNUSED(item)
  executeSelectedCommand();
}

void CommandPalette::onItemClicked(QListWidgetItem* item) {
  Q_UNUSED(item)
  executeSelectedCommand();
}

void CommandPalette::updateResults(const QString& query) {
  resultsList_->clear();
  filteredCommands_.clear();

  auto& theme = ThemeManager::instance();

  // If query is empty, show recent commands first
  if (query.isEmpty() && !recentCommands_.isEmpty()) {
    // Add separator for recent
    QListWidgetItem* recentHeader = new QListWidgetItem(tr("Recent"));
    recentHeader->setFlags(Qt::NoItemFlags);
    recentHeader->setForeground(theme.textSecondaryColor());
    QFont headerFont = recentHeader->font();
    headerFont.setBold(true);
    headerFont.setPointSize(9);
    recentHeader->setFont(headerFont);
    resultsList_->addItem(recentHeader);

    for (const QString& recentId : recentCommands_) {
      for (const CommandEntry& cmd : allCommands_) {
        if (cmd.id == recentId) {
          filteredCommands_.append(cmd);
          resultsList_->addItem(createItemForCommand(cmd));
          break;
        }
      }
    }

    // Add separator for all commands
    QListWidgetItem* allHeader = new QListWidgetItem(tr("All Commands"));
    allHeader->setFlags(Qt::NoItemFlags);
    allHeader->setForeground(theme.textSecondaryColor());
    allHeader->setFont(headerFont);
    resultsList_->addItem(allHeader);
  }

  // Filter and sort commands
  QList<CommandEntry> matching;
  for (const CommandEntry& cmd : allCommands_) {
    if (cmd.matches(query)) {
      // Don't duplicate recent items in all commands section
      bool isRecent = query.isEmpty() && recentCommands_.contains(cmd.id);
      if (!isRecent) {
        matching.append(cmd);
      }
    }
  }

  // Sort by category then name
  std::sort(matching.begin(), matching.end(), [](const CommandEntry& a, const CommandEntry& b) {
    if (a.category != b.category) return a.category < b.category;
    return a.name < b.name;
  });

  // Add filtered results
  QString currentCategory;
  int count = 0;
  for (const CommandEntry& cmd : matching) {
    if (count >= MAX_VISIBLE_RESULTS) break;

    // Category header
    if (cmd.category != currentCategory) {
      currentCategory = cmd.category;
      QListWidgetItem* categoryHeader = new QListWidgetItem(currentCategory);
      categoryHeader->setFlags(Qt::NoItemFlags);
      categoryHeader->setForeground(theme.textSecondaryColor());
      QFont headerFont = categoryHeader->font();
      headerFont.setBold(true);
      headerFont.setPointSize(9);
      categoryHeader->setFont(headerFont);
      resultsList_->addItem(categoryHeader);
    }

    filteredCommands_.append(cmd);
    resultsList_->addItem(createItemForCommand(cmd));
    count++;
  }

  // Select first selectable item
  for (int i = 0; i < resultsList_->count(); ++i) {
    QListWidgetItem* item = resultsList_->item(i);
    if (item->flags() & Qt::ItemIsSelectable) {
      resultsList_->setCurrentRow(i);
      break;
    }
  }

  // Adjust height based on content
  int itemHeight = 36;  // Approximate
  int contentHeight = resultsList_->count() * itemHeight + 100;  // +100 for padding and hint
  setFixedHeight(qMin(contentHeight, 450));
}

void CommandPalette::executeSelectedCommand() {
  QListWidgetItem* current = resultsList_->currentItem();
  if (!current || !(current->flags() & Qt::ItemIsSelectable)) {
    hide();
    return;
  }

  QString commandId = current->data(Qt::UserRole).toString();

  // Find and execute the command
  for (const CommandEntry& cmd : allCommands_) {
    if (cmd.id == commandId) {
      addToRecentCommands(commandId);

      if (cmd.action) {
        cmd.action->trigger();
      }
      emit commandExecuted(commandId);
      break;
    }
  }

  hide();
}

void CommandPalette::selectNext() {
  int current = resultsList_->currentRow();
  for (int i = current + 1; i < resultsList_->count(); ++i) {
    QListWidgetItem* item = resultsList_->item(i);
    if (item->flags() & Qt::ItemIsSelectable) {
      resultsList_->setCurrentRow(i);
      return;
    }
  }
}

void CommandPalette::selectPrevious() {
  int current = resultsList_->currentRow();
  for (int i = current - 1; i >= 0; --i) {
    QListWidgetItem* item = resultsList_->item(i);
    if (item->flags() & Qt::ItemIsSelectable) {
      resultsList_->setCurrentRow(i);
      return;
    }
  }
}

QListWidgetItem* CommandPalette::createItemForCommand(const CommandEntry& cmd) {
  QListWidgetItem* item = new CommandListItem(cmd);

  // Create rich text with name and shortcut
  QString displayText = cmd.name;
  if (!cmd.shortcut.isEmpty()) {
    displayText += QString("  ");  // Will be handled with custom delegate ideally
  }
  item->setText(displayText);

  // Set tooltip with description
  if (!cmd.description.isEmpty()) {
    item->setToolTip(cmd.description);
  }

  // Store shortcut for potential custom rendering
  item->setData(Qt::UserRole + 1, cmd.shortcut);

  return item;
}

void CommandPalette::addToRecentCommands(const QString& commandId) {
  recentCommands_.removeAll(commandId);
  recentCommands_.prepend(commandId);
  while (recentCommands_.size() > MAX_RECENT_COMMANDS) {
    recentCommands_.removeLast();
  }
}

}  // namespace ros_weaver
