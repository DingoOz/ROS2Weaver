// Unit tests for UndoStack
// Tests the undo/redo infrastructure without requiring canvas

#include <gtest/gtest.h>
#include <QCoreApplication>
#include "ros_weaver/core/undo/undo_stack.hpp"
#include "ros_weaver/core/undo/undo_command.hpp"
#include "ros_weaver/core/undo/command_ids.hpp"

using namespace ros_weaver;

// Simple test command that tracks execution count
class TestCommand : public UndoCommand {
public:
  TestCommand(int& undoCounter, int& redoCounter, const QString& text = "Test")
      : UndoCommand(nullptr)
      , undoCounter_(undoCounter)
      , redoCounter_(redoCounter)
      , text_(text)
      , firstRedo_(true) {}

  void undo() override { undoCounter_++; }
  void redo() override {
    if (firstRedo_) {
      firstRedo_ = false;
      return;
    }
    redoCounter_++;
  }
  QString text() const override { return text_; }

private:
  int& undoCounter_;
  int& redoCounter_;
  QString text_;
  bool firstRedo_;
};

// Mergeable test command for testing command merging
class MergeableCommand : public UndoCommand {
public:
  MergeableCommand(int& value, int delta)
      : UndoCommand(nullptr)
      , value_(value)
      , delta_(delta) {}

  void undo() override { value_ -= delta_; }
  void redo() override { value_ += delta_; }
  QString text() const override { return QString("Change by %1").arg(delta_); }

  CommandId id() const override { return CommandId::MoveBlock; }
  bool mergeWith(const UndoCommand* other) override {
    auto* cmd = dynamic_cast<const MergeableCommand*>(other);
    if (!cmd) return false;
    delta_ += cmd->delta_;
    return true;
  }

private:
  int& value_;
  int delta_;
};

class UndoStackTest : public ::testing::Test {
protected:
  void SetUp() override {
    stack = new UndoStack();
    undoCount = 0;
    redoCount = 0;
  }

  void TearDown() override {
    delete stack;
  }

  UndoStack* stack;
  int undoCount;
  int redoCount;
};

// Basic functionality tests
TEST_F(UndoStackTest, InitialState) {
  EXPECT_FALSE(stack->canUndo());
  EXPECT_FALSE(stack->canRedo());
  EXPECT_EQ(stack->count(), 0);
  EXPECT_EQ(stack->index(), 0);
  EXPECT_TRUE(stack->isClean());
}

TEST_F(UndoStackTest, PushCommand) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));

  EXPECT_TRUE(stack->canUndo());
  EXPECT_FALSE(stack->canRedo());
  EXPECT_EQ(stack->count(), 1);
  EXPECT_EQ(stack->index(), 1);
  EXPECT_EQ(stack->undoText(), "Test 1");
}

TEST_F(UndoStackTest, UndoSingleCommand) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));

  stack->undo();

  EXPECT_EQ(undoCount, 1);
  EXPECT_FALSE(stack->canUndo());
  EXPECT_TRUE(stack->canRedo());
  EXPECT_EQ(stack->index(), 0);
}

TEST_F(UndoStackTest, RedoAfterUndo) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  stack->undo();

  stack->redo();

  EXPECT_EQ(redoCount, 1);
  EXPECT_TRUE(stack->canUndo());
  EXPECT_FALSE(stack->canRedo());
  EXPECT_EQ(stack->index(), 1);
}

TEST_F(UndoStackTest, MultipleCommands) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 2"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 3"));

  EXPECT_EQ(stack->count(), 3);
  EXPECT_EQ(stack->index(), 3);
  EXPECT_EQ(stack->undoText(), "Test 3");

  stack->undo();
  EXPECT_EQ(stack->undoText(), "Test 2");

  stack->undo();
  EXPECT_EQ(stack->undoText(), "Test 1");
}

TEST_F(UndoStackTest, PushClearsRedoStack) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 2"));
  stack->undo();

  EXPECT_TRUE(stack->canRedo());

  stack->push(new TestCommand(undoCount, redoCount, "Test 3"));

  EXPECT_FALSE(stack->canRedo());
  EXPECT_EQ(stack->count(), 2);  // Test 1, Test 3 (Test 2 was removed)
  EXPECT_EQ(stack->undoText(), "Test 3");
}

TEST_F(UndoStackTest, Clear) {
  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 2"));

  stack->clear();

  EXPECT_EQ(stack->count(), 0);
  EXPECT_EQ(stack->index(), 0);
  EXPECT_FALSE(stack->canUndo());
  EXPECT_FALSE(stack->canRedo());
}

TEST_F(UndoStackTest, CleanState) {
  EXPECT_TRUE(stack->isClean());

  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  EXPECT_FALSE(stack->isClean());

  stack->setClean();
  EXPECT_TRUE(stack->isClean());

  stack->push(new TestCommand(undoCount, redoCount, "Test 2"));
  EXPECT_FALSE(stack->isClean());

  stack->undo();
  EXPECT_TRUE(stack->isClean());
}

TEST_F(UndoStackTest, UndoLimit) {
  stack->setUndoLimit(3);

  stack->push(new TestCommand(undoCount, redoCount, "Test 1"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 2"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 3"));
  stack->push(new TestCommand(undoCount, redoCount, "Test 4"));

  EXPECT_EQ(stack->count(), 3);
  EXPECT_EQ(stack->undoText(), "Test 4");

  stack->undo();
  EXPECT_EQ(stack->undoText(), "Test 3");

  stack->undo();
  EXPECT_EQ(stack->undoText(), "Test 2");

  stack->undo();
  EXPECT_FALSE(stack->canUndo());  // Test 1 was removed
}

TEST_F(UndoStackTest, CommandMerging) {
  int value = 0;

  // Push a mergeable command
  auto* cmd1 = new MergeableCommand(value, 10);
  stack->push(cmd1);
  value = 10;  // Simulate the redo effect

  // Push another command that should merge
  auto* cmd2 = new MergeableCommand(value, 5);
  stack->push(cmd2);
  value = 15;

  // Should have merged into single command
  EXPECT_EQ(stack->count(), 1);

  // Undo should undo the combined change
  stack->undo();
  EXPECT_EQ(value, 0);
}

TEST_F(UndoStackTest, NullCommandIgnored) {
  stack->push(nullptr);

  EXPECT_EQ(stack->count(), 0);
  EXPECT_FALSE(stack->canUndo());
}

// Macro tests
TEST_F(UndoStackTest, MacroEmpty) {
  stack->beginMacro("Empty Macro");
  stack->endMacro();

  // Empty macro should not be added
  EXPECT_EQ(stack->count(), 0);
}

TEST_F(UndoStackTest, MacroSingleCommand) {
  stack->beginMacro("Single Command Macro");
  stack->push(new TestCommand(undoCount, redoCount, "Inner"));
  stack->endMacro();

  EXPECT_EQ(stack->count(), 1);
  EXPECT_EQ(stack->undoText(), "Single Command Macro");

  stack->undo();
  EXPECT_EQ(undoCount, 1);
}

TEST_F(UndoStackTest, MacroMultipleCommands) {
  stack->beginMacro("Multi Macro");
  stack->push(new TestCommand(undoCount, redoCount, "Inner 1"));
  stack->push(new TestCommand(undoCount, redoCount, "Inner 2"));
  stack->push(new TestCommand(undoCount, redoCount, "Inner 3"));
  stack->endMacro();

  EXPECT_EQ(stack->count(), 1);
  EXPECT_EQ(stack->undoText(), "Multi Macro");

  // Single undo should undo all inner commands
  stack->undo();
  EXPECT_EQ(undoCount, 3);
}

// Signal tests
TEST_F(UndoStackTest, CanUndoChangedSignal) {
  bool canUndoChanged = false;
  QObject::connect(stack, &UndoStack::canUndoChanged,
                   [&canUndoChanged](bool canUndo) { canUndoChanged = canUndo; });

  stack->push(new TestCommand(undoCount, redoCount, "Test"));
  EXPECT_TRUE(canUndoChanged);

  stack->undo();
  EXPECT_FALSE(canUndoChanged);

  stack->disconnect();  // Prevent dangling reference in TearDown
}

TEST_F(UndoStackTest, CanRedoChangedSignal) {
  bool canRedoChanged = false;
  QObject::connect(stack, &UndoStack::canRedoChanged,
                   [&canRedoChanged](bool canRedo) { canRedoChanged = canRedo; });

  stack->push(new TestCommand(undoCount, redoCount, "Test"));
  EXPECT_FALSE(canRedoChanged);

  stack->undo();
  EXPECT_TRUE(canRedoChanged);

  stack->redo();
  EXPECT_FALSE(canRedoChanged);

  stack->disconnect();  // Prevent dangling reference in TearDown
}

TEST_F(UndoStackTest, UndoTextChangedSignal) {
  QString lastUndoText;
  QObject::connect(stack, &UndoStack::undoTextChanged,
                   [&lastUndoText](const QString& text) { lastUndoText = text; });

  stack->push(new TestCommand(undoCount, redoCount, "First"));
  EXPECT_EQ(lastUndoText, "First");

  stack->push(new TestCommand(undoCount, redoCount, "Second"));
  EXPECT_EQ(lastUndoText, "Second");

  stack->undo();
  EXPECT_EQ(lastUndoText, "First");

  stack->disconnect();  // Prevent dangling reference in TearDown
}

TEST_F(UndoStackTest, CleanChangedSignal) {
  bool cleanChanged = false;
  bool lastCleanState = true;
  QObject::connect(stack, &UndoStack::cleanChanged,
                   [&cleanChanged, &lastCleanState](bool clean) {
                     cleanChanged = true;
                     lastCleanState = clean;
                   });

  stack->push(new TestCommand(undoCount, redoCount, "Test"));
  EXPECT_TRUE(cleanChanged);
  EXPECT_FALSE(lastCleanState);

  cleanChanged = false;
  stack->setClean();
  EXPECT_TRUE(cleanChanged);
  EXPECT_TRUE(lastCleanState);

  stack->disconnect();  // Prevent dangling reference in TearDown
}

int main(int argc, char** argv) {
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
