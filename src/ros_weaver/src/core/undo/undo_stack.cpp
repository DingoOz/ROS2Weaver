#include "ros_weaver/core/undo/undo_stack.hpp"
#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/undo/commands/macro_command.hpp"

namespace ros_weaver {

UndoStack::UndoStack(QObject* parent)
    : QObject(parent) {
}

UndoStack::~UndoStack() {
  clear();
}

void UndoStack::push(UndoCommand* command) {
  if (!command) {
    return;
  }

  // If in macro, add to macro instead
  if (inMacro_) {
    macroCommands_.append(command);
    return;
  }

  // Check for merging with previous command
  if (currentIndex_ > 0 && command->id() != CommandId::None) {
    UndoCommand* prev = commands_[currentIndex_ - 1];
    if (prev->id() == command->id() && prev->mergeWith(command)) {
      // Merge successful, delete the new command
      delete command;
      emitSignals();
      return;
    }
  }

  // Remove any commands after current position (clear redo stack)
  while (commands_.size() > currentIndex_) {
    delete commands_.takeLast();
  }

  // Add the new command
  commands_.append(command);
  currentIndex_++;

  // Execute the command (redo)
  command->redo();

  // Enforce limit
  enforceLimit();

  emitSignals();
}

void UndoStack::undo() {
  if (!canUndo()) {
    return;
  }

  currentIndex_--;
  commands_[currentIndex_]->undo();

  emitSignals();
}

void UndoStack::redo() {
  if (!canRedo()) {
    return;
  }

  commands_[currentIndex_]->redo();
  currentIndex_++;

  emitSignals();
}

bool UndoStack::canUndo() const {
  return currentIndex_ > 0;
}

bool UndoStack::canRedo() const {
  return currentIndex_ < commands_.size();
}

QString UndoStack::undoText() const {
  if (canUndo()) {
    return commands_[currentIndex_ - 1]->text();
  }
  return QString();
}

QString UndoStack::redoText() const {
  if (canRedo()) {
    return commands_[currentIndex_]->text();
  }
  return QString();
}

int UndoStack::count() const {
  return commands_.size();
}

int UndoStack::index() const {
  return currentIndex_;
}

void UndoStack::setUndoLimit(int limit) {
  if (limit >= 0) {
    undoLimit_ = limit;
    enforceLimit();
  }
}

int UndoStack::undoLimit() const {
  return undoLimit_;
}

void UndoStack::clear() {
  qDeleteAll(commands_);
  commands_.clear();
  currentIndex_ = 0;
  cleanIndex_ = 0;

  // Clear macro state
  qDeleteAll(macroCommands_);
  macroCommands_.clear();
  macroText_.clear();
  inMacro_ = false;

  emitSignals();
}

void UndoStack::beginMacro(const QString& text) {
  if (inMacro_) {
    qWarning("UndoStack::beginMacro: Already in a macro");
    return;
  }

  inMacro_ = true;
  macroText_ = text;
  macroCommands_.clear();
}

void UndoStack::endMacro() {
  if (!inMacro_) {
    qWarning("UndoStack::endMacro: Not in a macro");
    return;
  }

  inMacro_ = false;

  if (macroCommands_.isEmpty()) {
    // Empty macro, nothing to do
    macroText_.clear();
    return;
  }

  // Create a macro command containing all the collected commands
  MacroCommand* macro = new MacroCommand(
      macroCommands_.first()->canvas(), macroText_);

  for (UndoCommand* cmd : macroCommands_) {
    macro->addCommand(cmd);
  }
  macroCommands_.clear();
  macroText_.clear();

  // Remove any commands after current position
  while (commands_.size() > currentIndex_) {
    delete commands_.takeLast();
  }

  // Add macro command (don't execute redo - commands already executed)
  commands_.append(macro);
  currentIndex_++;

  enforceLimit();
  emitSignals();
}

bool UndoStack::isInMacro() const {
  return inMacro_;
}

void UndoStack::setClean() {
  cleanIndex_ = currentIndex_;
  emitSignals();
}

bool UndoStack::isClean() const {
  return currentIndex_ == cleanIndex_;
}

int UndoStack::cleanIndex() const {
  return cleanIndex_;
}

void UndoStack::enforceLimit() {
  while (commands_.size() > undoLimit_ && currentIndex_ > 0) {
    delete commands_.takeFirst();
    currentIndex_--;
    if (cleanIndex_ > 0) {
      cleanIndex_--;
    } else {
      // Clean index is now invalid (before history)
      cleanIndex_ = -1;
    }
  }
}

void UndoStack::emitSignals() {
  bool newCanUndo = canUndo();
  bool newCanRedo = canRedo();
  QString newUndoText = undoText();
  QString newRedoText = redoText();
  bool newClean = isClean();

  if (newCanUndo != lastCanUndo_) {
    lastCanUndo_ = newCanUndo;
    emit canUndoChanged(newCanUndo);
  }

  if (newCanRedo != lastCanRedo_) {
    lastCanRedo_ = newCanRedo;
    emit canRedoChanged(newCanRedo);
  }

  if (newUndoText != lastUndoText_) {
    lastUndoText_ = newUndoText;
    emit undoTextChanged(newUndoText);
  }

  if (newRedoText != lastRedoText_) {
    lastRedoText_ = newRedoText;
    emit redoTextChanged(newRedoText);
  }

  if (newClean != lastClean_) {
    lastClean_ = newClean;
    emit cleanChanged(newClean);
  }

  emit indexChanged(currentIndex_);
}

}  // namespace ros_weaver
