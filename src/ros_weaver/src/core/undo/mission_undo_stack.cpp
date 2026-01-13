#include "ros_weaver/core/undo/mission_undo_stack.hpp"
#include "ros_weaver/core/undo/mission_undo_commands.hpp"

namespace ros_weaver {

MissionUndoStack::MissionUndoStack(QObject* parent)
    : QObject(parent) {
}

MissionUndoStack::~MissionUndoStack() {
  clear();
}

void MissionUndoStack::push(MissionUndoCommand* command) {
  if (!command) return;

  if (inMacro_) {
    macroCommands_.append(command);
    return;
  }

  // Clear redo history
  while (commands_.size() > currentIndex_) {
    delete commands_.takeLast();
  }

  // Try to merge with previous command
  if (!commands_.isEmpty()) {
    MissionUndoCommand* top = commands_.last();
    if (command->id() == top->id() && command->id() != CommandId::None) {
      if (top->mergeWith(command)) {
        delete command;
        emitSignals();
        return;
      }
    }
  }

  commands_.append(command);
  currentIndex_ = commands_.size();

  enforceLimit();
  emitSignals();
}

void MissionUndoStack::undo() {
  if (!canUndo()) return;

  --currentIndex_;
  commands_[currentIndex_]->undo();
  emitSignals();
}

void MissionUndoStack::redo() {
  if (!canRedo()) return;

  commands_[currentIndex_]->redo();
  ++currentIndex_;
  emitSignals();
}

bool MissionUndoStack::canUndo() const {
  return currentIndex_ > 0;
}

bool MissionUndoStack::canRedo() const {
  return currentIndex_ < commands_.size();
}

QString MissionUndoStack::undoText() const {
  if (!canUndo()) return QString();
  return commands_[currentIndex_ - 1]->text();
}

QString MissionUndoStack::redoText() const {
  if (!canRedo()) return QString();
  return commands_[currentIndex_]->text();
}

int MissionUndoStack::count() const {
  return commands_.size();
}

int MissionUndoStack::index() const {
  return currentIndex_;
}

void MissionUndoStack::setUndoLimit(int limit) {
  undoLimit_ = limit;
  enforceLimit();
}

int MissionUndoStack::undoLimit() const {
  return undoLimit_;
}

void MissionUndoStack::clear() {
  qDeleteAll(commands_);
  commands_.clear();
  qDeleteAll(macroCommands_);
  macroCommands_.clear();

  currentIndex_ = 0;
  cleanIndex_ = 0;
  inMacro_ = false;

  emitSignals();
}

void MissionUndoStack::beginMacro(const QString& text) {
  if (inMacro_) return;
  inMacro_ = true;
  macroText_ = text;
  macroCommands_.clear();
}

void MissionUndoStack::endMacro() {
  if (!inMacro_) return;
  inMacro_ = false;

  if (macroCommands_.isEmpty()) return;

  // For simplicity, just push all macro commands individually
  // A more sophisticated implementation would wrap them in a composite command
  for (MissionUndoCommand* cmd : macroCommands_) {
    // Clear redo history
    while (commands_.size() > currentIndex_) {
      delete commands_.takeLast();
    }
    commands_.append(cmd);
    currentIndex_ = commands_.size();
  }
  macroCommands_.clear();

  enforceLimit();
  emitSignals();
}

bool MissionUndoStack::isInMacro() const {
  return inMacro_;
}

void MissionUndoStack::setClean() {
  cleanIndex_ = currentIndex_;
  emitSignals();
}

bool MissionUndoStack::isClean() const {
  return currentIndex_ == cleanIndex_;
}

int MissionUndoStack::cleanIndex() const {
  return cleanIndex_;
}

void MissionUndoStack::enforceLimit() {
  if (undoLimit_ <= 0) return;

  while (commands_.size() > undoLimit_) {
    delete commands_.takeFirst();
    --currentIndex_;
    if (cleanIndex_ > 0) --cleanIndex_;
  }
}

void MissionUndoStack::emitSignals() {
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
