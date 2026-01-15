# Add Test Skill

Generate test scaffolding for a ROS2Weaver class following established patterns.

## Arguments

The argument should be the class name to test (e.g., `BehaviorTreeParser`, `NetworkTopologyManager`).

## Instructions

When this skill is invoked with a class name argument ($ARGUMENTS):

### Step 1: Locate the Class

1. Search for the header file in `src/ros_weaver/include/ros_weaver/` matching the class name
2. Read the header file to understand:
   - The class structure and public methods
   - Whether it uses Q_OBJECT (requires MOC)
   - Required dependencies (Qt, ROS2, etc.)
   - The namespace (should be `ros_weaver`)

### Step 2: Determine Test Type

Based on the class characteristics:

**Standalone Tests** (no source dependencies needed):
- Pure data structures
- Classes with header-only implementations
- Use pattern from `test_dot_importer.cpp`

**Source-Linked Tests** (need source files):
- Classes with signals/slots (Q_OBJECT)
- Classes with complex implementations
- Use pattern from `test_theme_manager.cpp` or `test_undo_stack.cpp`

**ROS2-Dependent Tests**:
- Classes using rclcpp, topics, services
- Use pattern from `test_topic_monitor.cpp`

### Step 3: Generate Test File

Create `src/ros_weaver/test/test_<snake_case_name>.cpp` with:

```cpp
// Unit tests for <ClassName>
// Tests <brief description of what's being tested>

#include <gtest/gtest.h>
#include "ros_weaver/<path>/<header>.hpp"
// Add Qt includes if needed: <QCoreApplication>, <QApplication>
// Add other required includes

using namespace ros_weaver;

// Test fixture class (if needed for setup/teardown)
class <ClassName>Test : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize test resources
  }

  void TearDown() override {
    // Clean up test resources
  }

  // Member variables for tests
};

// ============================================================================
// <Test Category 1>
// ============================================================================

TEST_F(<ClassName>Test, <TestName>) {
  // Arrange

  // Act

  // Assert
  EXPECT_TRUE(/* condition */);
}

// Add more test categories and tests...

int main(int argc, char** argv) {
  // Use QCoreApplication for non-GUI Qt classes
  // Use QApplication for GUI Qt classes (QPalette, QWidget, etc.)
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### Step 4: Update CMakeLists.txt

Add a new test target in the `if(BUILD_TESTING)` section of `src/ros_weaver/CMakeLists.txt`:

**For standalone tests:**
```cmake
  # Unit tests - <ClassName>
  ament_add_gtest(test_<snake_case>
    test/test_<snake_case>.cpp
  )
  target_include_directories(test_<snake_case> PRIVATE include)
  target_link_libraries(test_<snake_case> Qt5::Core Qt5::Gui)
```

**For source-linked tests with Q_OBJECT:**
```cmake
  # Unit tests - <ClassName>
  ament_add_gtest(test_<snake_case>
    test/test_<snake_case>.cpp
    src/<path>/<source>.cpp
    # Add any dependency sources
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_weaver/<path>/<header>.hpp
  )
  target_include_directories(test_<snake_case> PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_<snake_case> Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Test)
  set_target_properties(test_<snake_case> PROPERTIES AUTOMOC ON)
```

**For ROS2-dependent tests:**
```cmake
  # Unit tests - <ClassName>
  ament_add_gtest(test_<snake_case>
    test/test_<snake_case>.cpp
    src/<path>/<source>.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_weaver/<path>/<header>.hpp
  )
  target_include_directories(test_<snake_case> PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_<snake_case> Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Test)
  ament_target_dependencies(test_<snake_case> rclcpp <other_ros_deps>)
  set_target_properties(test_<snake_case> PROPERTIES AUTOMOC ON)
```

### Step 5: Generate Initial Tests

Based on the class's public API, generate test cases for:
1. **Construction** - Can the object be created?
2. **Basic functionality** - Do the primary methods work?
3. **Edge cases** - Empty inputs, null values, boundary conditions
4. **Error handling** - Invalid inputs, error states
5. **Signal/slot behavior** (if applicable) - Do signals emit correctly?

### Step 6: Verify Build

Run `colcon build --packages-select ros_weaver` to verify the test compiles.
Run `colcon test --packages-select ros_weaver --ctest-args -R test_<snake_case>` to run just the new test.

### Output

Provide a summary:
- Test file created: `src/ros_weaver/test/test_<name>.cpp`
- Number of test cases generated
- CMakeLists.txt updated
- Build verification result
- Any additional test cases the user might want to add

## Example Usage

```
/add-test BehaviorTreeParser
```

This will:
1. Read `include/ros_weaver/core/behavior_tree_parser.hpp`
2. Generate `test/test_behavior_tree_parser.cpp` with appropriate tests
3. Update CMakeLists.txt with the test target
4. Verify the build succeeds
