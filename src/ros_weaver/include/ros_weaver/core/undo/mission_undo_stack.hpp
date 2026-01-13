#ifndef ROS_WEAVER_MISSION_UNDO_STACK_HPP
#define ROS_WEAVER_MISSION_UNDO_STACK_HPP

#include <QObject>
#include <QList>
#include <QString>
#include <memory>

namespace ros_weaver {

class MissionUndoCommand;

/**
 * @brief Undo stack for mission planner commands
 *
 * Similar to UndoStack but manages MissionUndoCommand instead of UndoCommand.
 * Supports command merging for continuous operations like dragging.
 */
class MissionUndoStack : public QObject {
  Q_OBJECT

public:
  explicit MissionUndoStack(QObject* parent = nullptr);
  ~MissionUndoStack() override;

  void push(MissionUndoCommand* command);
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

  QList<MissionUndoCommand*> commands_;
  int currentIndex_ = 0;
  int cleanIndex_ = 0;
  int undoLimit_ = 50;

  QList<MissionUndoCommand*> macroCommands_;
  QString macroText_;
  bool inMacro_ = false;

  bool lastCanUndo_ = false;
  bool lastCanRedo_ = false;
  QString lastUndoText_;
  QString lastRedoText_;
  bool lastClean_ = true;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MISSION_UNDO_STACK_HPP
