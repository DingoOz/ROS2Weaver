#ifndef ROS_WEAVER_SCENARIO_UNDO_STACK_HPP
#define ROS_WEAVER_SCENARIO_UNDO_STACK_HPP

#include <QObject>
#include <QList>
#include <QString>

namespace ros_weaver {

class ScenarioUndoCommand;

/**
 * @brief Undo stack for scenario editor commands
 *
 * Similar to MissionUndoStack but manages ScenarioUndoCommand instead.
 * Supports command merging for continuous operations.
 */
class ScenarioUndoStack : public QObject {
  Q_OBJECT

public:
  explicit ScenarioUndoStack(QObject* parent = nullptr);
  ~ScenarioUndoStack() override;

  void push(ScenarioUndoCommand* command);
  void undo();
  void redo();

  bool canUndo() const;
  bool canRedo() const;

  QString undoText() const;
  QString redoText() const;

  int count() const;
  int index() const;

  void setUndoLimit(int limit);
  int undoLimit() const;

  void clear();

  void beginMacro(const QString& text);
  void endMacro();
  bool isInMacro() const;

  void setClean();
  bool isClean() const;
  int cleanIndex() const;

signals:
  void canUndoChanged(bool canUndo);
  void canRedoChanged(bool canRedo);
  void undoTextChanged(const QString& text);
  void redoTextChanged(const QString& text);
  void cleanChanged(bool clean);
  void indexChanged(int index);

private:
  void enforceLimit();
  void emitSignals();

  QList<ScenarioUndoCommand*> commands_;
  int currentIndex_ = 0;
  int cleanIndex_ = 0;
  int undoLimit_ = 50;

  QList<ScenarioUndoCommand*> macroCommands_;
  QString macroText_;
  bool inMacro_ = false;

  bool lastCanUndo_ = false;
  bool lastCanRedo_ = false;
  QString lastUndoText_;
  QString lastRedoText_;
  bool lastClean_ = true;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_SCENARIO_UNDO_STACK_HPP
