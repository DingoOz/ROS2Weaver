#include "ros_weaver/core/undo/scenario_undo_stack.hpp"
#include "ros_weaver/core/undo/scenario_undo_commands.hpp"

namespace ros_weaver {

ScenarioUndoStack::ScenarioUndoStack(QObject* parent)
    : QObject(parent) {
}

ScenarioUndoStack::~ScenarioUndoStack() {
  clear();
}

void ScenarioUndoStack::push(ScenarioUndoCommand* command) {
  if (inMacro_) {
    macroCommands_.append(command);
    return;
  }

  // Check if we can merge with the previous command
  if (!commands_.isEmpty() && currentIndex_ > 0) {
    ScenarioUndoCommand* lastCommand = commands_[currentIndex_ - 1];
    if (command->id() != CommandId::None &&
        command->id() == lastCommand->id() &&
        lastCommand->mergeWith(command)) {
      // Merged successfully, delete the new command
      delete command;
      emitSignals();
      return;
    }
  }

  // Clear redo stack (commands after current index)
  while (commands_.size() > currentIndex_) {
    delete commands_.takeLast();
  }

  // Add and execute the new command
  commands_.append(command);
  command->redo();
  currentIndex_++;

  enforceLimit();
  emitSignals();
}

void ScenarioUndoStack::undo() {
  if (!canUndo()) return;

  currentIndex_--;
  commands_[currentIndex_]->undo();
  emitSignals();
}

void ScenarioUndoStack::redo() {
  if (!canRedo()) return;

  commands_[currentIndex_]->redo();
  currentIndex_++;
  emitSignals();
}

bool ScenarioUndoStack::canUndo() const {
  return currentIndex_ > 0;
}

bool ScenarioUndoStack::canRedo() const {
  return currentIndex_ < commands_.size();
}

QString ScenarioUndoStack::undoText() const {
  if (!canUndo()) return QString();
  return commands_[currentIndex_ - 1]->text();
}

QString ScenarioUndoStack::redoText() const {
  if (!canRedo()) return QString();
  return commands_[currentIndex_]->text();
}

int ScenarioUndoStack::count() const {
  return commands_.size();
}

int ScenarioUndoStack::index() const {
  return currentIndex_;
}

void ScenarioUndoStack::setUndoLimit(int limit) {
  undoLimit_ = limit;
  enforceLimit();
}

int ScenarioUndoStack::undoLimit() const {
  return undoLimit_;
}

void ScenarioUndoStack::clear() {
  qDeleteAll(commands_);
  commands_.clear();
  qDeleteAll(macroCommands_);
  macroCommands_.clear();
  currentIndex_ = 0;
  cleanIndex_ = 0;
  inMacro_ = false;
  emitSignals();
}

void ScenarioUndoStack::beginMacro(const QString& text) {
  if (inMacro_) return;
  inMacro_ = true;
  macroText_ = text;
  macroCommands_.clear();
}

void ScenarioUndoStack::endMacro() {
  if (!inMacro_) return;
  inMacro_ = false;

  if (macroCommands_.isEmpty()) return;

  // Execute all macro commands as one unit
  // Clear redo stack first
  while (commands_.size() > currentIndex_) {
    delete commands_.takeLast();
  }

  // Store the macro commands
  for (ScenarioUndoCommand* cmd : macroCommands_) {
    commands_.append(cmd);
  }
  macroCommands_.clear();

  currentIndex_ = commands_.size();
  enforceLimit();
  emitSignals();
}

bool ScenarioUndoStack::isInMacro() const {
  return inMacro_;
}

void ScenarioUndoStack::setClean() {
  cleanIndex_ = currentIndex_;
  emitSignals();
}

bool ScenarioUndoStack::isClean() const {
  return currentIndex_ == cleanIndex_;
}

int ScenarioUndoStack::cleanIndex() const {
  return cleanIndex_;
}

void ScenarioUndoStack::enforceLimit() {
  while (commands_.size() > undoLimit_ && currentIndex_ > 0) {
    delete commands_.takeFirst();
    currentIndex_--;
    cleanIndex_--;
  }
  if (cleanIndex_ < 0) cleanIndex_ = -1;
}

void ScenarioUndoStack::emitSignals() {
  bool canUndoNow = canUndo();
  bool canRedoNow = canRedo();
  QString undoTextNow = undoText();
  QString redoTextNow = redoText();
  bool cleanNow = isClean();

  if (canUndoNow != lastCanUndo_) {
    lastCanUndo_ = canUndoNow;
    emit canUndoChanged(canUndoNow);
  }
  if (canRedoNow != lastCanRedo_) {
    lastCanRedo_ = canRedoNow;
    emit canRedoChanged(canRedoNow);
  }
  if (undoTextNow != lastUndoText_) {
    lastUndoText_ = undoTextNow;
    emit undoTextChanged(undoTextNow);
  }
  if (redoTextNow != lastRedoText_) {
    lastRedoText_ = redoTextNow;
    emit redoTextChanged(redoTextNow);
  }
  if (cleanNow != lastClean_) {
    lastClean_ = cleanNow;
    emit cleanChanged(cleanNow);
  }

  emit indexChanged(currentIndex_);
}

}  // namespace ros_weaver
