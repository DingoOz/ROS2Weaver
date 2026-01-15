#ifndef ROS_WEAVER_SCENARIO_UNDO_COMMANDS_HPP
#define ROS_WEAVER_SCENARIO_UNDO_COMMANDS_HPP

#include <QObject>
#include <QString>
#include <QDateTime>
#include "ros_weaver/core/undo/command_ids.hpp"
#include "ros_weaver/core/scenario_player.hpp"

namespace ros_weaver {

// Forward declarations
class ScenarioEditorWidget;

/**
 * @brief Base class for scenario editor undo commands
 */
class ScenarioUndoCommand {
public:
  explicit ScenarioUndoCommand(ScenarioEditorWidget* editor);
  virtual ~ScenarioUndoCommand() = default;

  virtual void undo() = 0;
  virtual void redo() = 0;
  virtual QString text() const = 0;
  virtual CommandId id() const { return CommandId::None; }
  virtual bool mergeWith(const ScenarioUndoCommand* other) {
    Q_UNUSED(other);
    return false;
  }

  QDateTime timestamp() const { return timestamp_; }
  virtual bool isObsolete() const { return false; }
  ScenarioEditorWidget* editor() const { return editor_; }

protected:
  ScenarioEditorWidget* editor_;
  QDateTime timestamp_;
};

/**
 * @brief Command for adding a step
 */
class AddScenarioStepCommand : public ScenarioUndoCommand {
public:
  AddScenarioStepCommand(ScenarioEditorWidget* editor, const ScenarioStep& step, int index);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  ScenarioStep step_;
  int index_;
};

/**
 * @brief Command for removing a step
 */
class RemoveScenarioStepCommand : public ScenarioUndoCommand {
public:
  RemoveScenarioStepCommand(ScenarioEditorWidget* editor, const ScenarioStep& step, int index);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  ScenarioStep step_;
  int index_;
};

/**
 * @brief Command for moving a step (reordering)
 */
class MoveScenarioStepCommand : public ScenarioUndoCommand {
public:
  MoveScenarioStepCommand(ScenarioEditorWidget* editor, int fromIndex, int toIndex);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  int fromIndex_;
  int toIndex_;
};

/**
 * @brief Command for modifying step properties
 */
class ModifyScenarioStepCommand : public ScenarioUndoCommand {
public:
  ModifyScenarioStepCommand(ScenarioEditorWidget* editor, int index,
                            const ScenarioStep& oldStep, const ScenarioStep& newStep);

  void undo() override;
  void redo() override;
  QString text() const override;
  CommandId id() const override { return CommandId::ModifyScenarioStep; }
  bool mergeWith(const ScenarioUndoCommand* other) override;

private:
  int index_;
  ScenarioStep oldStep_;
  ScenarioStep newStep_;
};

/**
 * @brief Command for duplicating a step
 */
class DuplicateScenarioStepCommand : public ScenarioUndoCommand {
public:
  DuplicateScenarioStepCommand(ScenarioEditorWidget* editor, const ScenarioStep& step,
                               int sourceIndex, int destIndex);

  void undo() override;
  void redo() override;
  QString text() const override;

private:
  ScenarioStep step_;
  int sourceIndex_;
  int destIndex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_SCENARIO_UNDO_COMMANDS_HPP
