#include "ros_weaver/core/undo/scenario_undo_commands.hpp"
#include "ros_weaver/widgets/scenario_editor_widget.hpp"

namespace ros_weaver {

// Base class implementation
ScenarioUndoCommand::ScenarioUndoCommand(ScenarioEditorWidget* editor)
    : editor_(editor)
    , timestamp_(QDateTime::currentDateTime()) {
}

// AddScenarioStepCommand implementation
AddScenarioStepCommand::AddScenarioStepCommand(ScenarioEditorWidget* editor,
                                               const ScenarioStep& step, int index)
    : ScenarioUndoCommand(editor)
    , step_(step)
    , index_(index) {
}

void AddScenarioStepCommand::undo() {
  if (!editor_) return;
  // Remove the step that was added
  editor_->removeStepAt(index_);
}

void AddScenarioStepCommand::redo() {
  if (!editor_) return;
  // Add the step back
  editor_->insertStepAt(index_, step_);
}

QString AddScenarioStepCommand::text() const {
  return QObject::tr("Add Step");
}

// RemoveScenarioStepCommand implementation
RemoveScenarioStepCommand::RemoveScenarioStepCommand(ScenarioEditorWidget* editor,
                                                     const ScenarioStep& step, int index)
    : ScenarioUndoCommand(editor)
    , step_(step)
    , index_(index) {
}

void RemoveScenarioStepCommand::undo() {
  if (!editor_) return;
  // Re-insert the removed step
  editor_->insertStepAt(index_, step_);
}

void RemoveScenarioStepCommand::redo() {
  if (!editor_) return;
  // Remove the step
  editor_->removeStepAt(index_);
}

QString RemoveScenarioStepCommand::text() const {
  return QObject::tr("Remove Step");
}

// MoveScenarioStepCommand implementation
MoveScenarioStepCommand::MoveScenarioStepCommand(ScenarioEditorWidget* editor,
                                                 int fromIndex, int toIndex)
    : ScenarioUndoCommand(editor)
    , fromIndex_(fromIndex)
    , toIndex_(toIndex) {
}

void MoveScenarioStepCommand::undo() {
  if (!editor_) return;
  // Move back to original position
  editor_->moveStep(toIndex_, fromIndex_);
}

void MoveScenarioStepCommand::redo() {
  if (!editor_) return;
  // Move to new position
  editor_->moveStep(fromIndex_, toIndex_);
}

QString MoveScenarioStepCommand::text() const {
  return QObject::tr("Move Step");
}

// ModifyScenarioStepCommand implementation
ModifyScenarioStepCommand::ModifyScenarioStepCommand(ScenarioEditorWidget* editor, int index,
                                                     const ScenarioStep& oldStep,
                                                     const ScenarioStep& newStep)
    : ScenarioUndoCommand(editor)
    , index_(index)
    , oldStep_(oldStep)
    , newStep_(newStep) {
}

void ModifyScenarioStepCommand::undo() {
  if (!editor_) return;
  editor_->setStepAt(index_, oldStep_);
}

void ModifyScenarioStepCommand::redo() {
  if (!editor_) return;
  editor_->setStepAt(index_, newStep_);
}

QString ModifyScenarioStepCommand::text() const {
  return QObject::tr("Modify Step");
}

bool ModifyScenarioStepCommand::mergeWith(const ScenarioUndoCommand* other) {
  const ModifyScenarioStepCommand* modifyCmd =
      dynamic_cast<const ModifyScenarioStepCommand*>(other);
  if (!modifyCmd) return false;

  // Only merge if modifying the same step
  if (modifyCmd->index_ != index_) return false;

  // Only merge if commands are recent (within 500ms)
  qint64 timeDiff = timestamp_.msecsTo(modifyCmd->timestamp_);
  if (timeDiff > 500) return false;

  // Merge: keep our old values, take the new command's new values
  newStep_ = modifyCmd->newStep_;
  return true;
}

// DuplicateScenarioStepCommand implementation
DuplicateScenarioStepCommand::DuplicateScenarioStepCommand(ScenarioEditorWidget* editor,
                                                           const ScenarioStep& step,
                                                           int sourceIndex, int destIndex)
    : ScenarioUndoCommand(editor)
    , step_(step)
    , sourceIndex_(sourceIndex)
    , destIndex_(destIndex) {
}

void DuplicateScenarioStepCommand::undo() {
  if (!editor_) return;
  // Remove the duplicated step
  editor_->removeStepAt(destIndex_);
}

void DuplicateScenarioStepCommand::redo() {
  if (!editor_) return;
  // Insert the duplicate
  editor_->insertStepAt(destIndex_, step_);
}

QString DuplicateScenarioStepCommand::text() const {
  return QObject::tr("Duplicate Step");
}

}  // namespace ros_weaver
